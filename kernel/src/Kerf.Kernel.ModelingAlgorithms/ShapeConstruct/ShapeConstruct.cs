// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Healing.Construct;

/// <summary>
/// Constructions of geometry and topology for shape healing: conversion of curves and surfaces to B-splines, and
/// joining of curves and of the pcurves of edges.
/// </summary>
/// <remarks>
/// <para>
/// The conversions and joins hand out frozen geometry (docs/phase3-design.md, section 4.2); ported code (ShapeFix,
/// ShapeUpgrade) calls the internal forms (<c>InternalConvertCurveToBSpline</c>, <c>InternalJoinCurves</c>, ...),
/// whose new curves it may go on editing. A conversion of a curve that is already a B-spline returns that curve.
/// </para>
/// <para>
/// <see cref="JoinPCurves"/> writes the joined pcurves into the given edge, so it is internal and throws on a frozen
/// edge; its public functional twin is <see cref="PCurvesJoined"/>.
/// </para>
/// </remarks>
internal static partial class ShapeConstruct
{
    /// <summary>
    /// The 3D curve on [<paramref name="first"/>, <paramref name="last"/>] as a B-spline (frozen): a B-spline is
    /// returned as it is, other curves are approximated (GeomConvert_ApproxCurve; degree at most 6 for conics), or
    /// converted by GeomConvert if the approximation fails.
    /// </summary>
    public static BSplineCurve3D? ConvertCurveToBSpline(Curve3D c3D, double first, double last, double tol3d,
                                                           Continuity continuity, int maxSegments, int maxDegree)
    {
        BSplineCurve3D? result = InternalConvertCurveToBSpline(c3D, first, last, tol3d, continuity, maxSegments, maxDegree);
        result?.Freeze();
        return result;
    }

    /// <summary>The 3D <see cref="ConvertCurveToBSpline(Curve3D, double, double, double, Continuity, int, int)"/> without freezing (ported code).</summary>
    internal static BSplineCurve3D? InternalConvertCurveToBSpline(Curve3D c3D, double first, double last, double tol3d,
                                                                     Continuity continuity, int maxSegments, int maxDegree)
    {
        int maxDeg = maxDegree;
        BSplineCurve3D? bSpline;
        if (c3D is BSplineCurve3D bSplineC3D)
        {
            bSpline = bSplineC3D;
        }
        else
        {
            if (c3D is ConicCurve3D)
            {
                maxDeg = Min(maxDeg, 6);
            }

            Curve3D tcurve = new TrimmedCurve3D(c3D, first, last); //protection against parabols ets
            try
            {
                ApproxCurve approx = new(tcurve, tol3d, continuity, maxSegments, maxDeg);
                if (approx.HasResult)
                {
                    bSpline = approx.InternalCurve();
                }
                else
                {
                    bSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(c3D, QuasiAngular);
                }
            }
            catch (FailureException)
            {
                bSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(c3D, QuasiAngular);
            }
        }
        return bSpline;
    }

    /// <summary>
    /// The 2D curve as a B-spline (frozen): a conic on [<paramref name="first"/>, <paramref name="last"/>] is
    /// approximated (Geom2dConvert_ApproxCurve, or converted if that fails), other curves converted whole
    /// (Geom2dConvert), a B-spline returned as it is.
    /// </summary>
    public static BSplineCurve2D? ConvertCurveToBSpline(Curve2D c2D, double first, double last, double tol2d,
                                                             Continuity continuity, int maxSegments, int maxDegree)
    {
        BSplineCurve2D? result = InternalConvertCurveToBSpline(c2D, first, last, tol2d, continuity, maxSegments, maxDegree);
        result?.Freeze();
        return result;
    }

    /// <summary>The 2D <c>ConvertCurveToBSpline</c> without freezing (ported code).</summary>
    internal static BSplineCurve2D? InternalConvertCurveToBSpline(Curve2D c2D, double first, double last,
                                                                       double tol2d, Continuity continuity,
                                                                       int maxSegments, int maxDegree)
    {
        BSplineCurve2D? bSpline2d;
        if (c2D is ConicCurve2D)
        {
            Curve2D tcurve = new TrimmedCurve2D(c2D, first, last); //protection against parabols ets
            ApproxCurve2D approx = new(tcurve, tol2d, continuity, maxSegments, maxDegree);
            if (approx.HasResult)
            {
                bSpline2d = approx.InternalCurve();
            }
            else
            {
                bSpline2d = Geom2dConvert2D.CurveToBSplineCurve(tcurve, QuasiAngular);
            }
        }
        else if (c2D is not BSplineCurve2D)
        {
            bSpline2d = Geom2dConvert2D.CurveToBSplineCurve(c2D, QuasiAngular);
        }
        else
        {
            bSpline2d = (BSplineCurve2D)c2D;
        }

        return bSpline2d;
    }

    /// <summary>
    /// The surface on [UF, UL] x [VF, VL] as a B-spline surface (frozen): elementary surfaces converted exactly
    /// (GeomConvert), extrusions built from their converted basis curve, other surfaces approximated
    /// (GeomConvert_ApproxSurface, lowering the continuity on failure; a revolution of an offset curve has its basis
    /// converted first). Null if no approximation within <paramref name="tol3d"/> is found; then the last
    /// approximation found, if any.
    /// </summary>
    /// <remarks>
    /// This has the purpose of GeomConvert::SurfaceToBSplineSurface, but treats offset surfaces more correctly and
    /// takes the UV limits and degree as arguments.
    /// </remarks>
    public static BSplineSurface? ConvertSurfaceToBSpline(Surface surf, double uf, double ul, double vf, double vl,
                                                               double tol3d, Continuity continuity, int maxSegments,
                                                               int maxDegree)
    {
        BSplineSurface? result = InternalConvertSurfaceToBSpline(surf, uf, ul, vf, vl, tol3d, continuity, maxSegments,
                                                                       maxDegree);
        result?.Freeze();
        return result;
    }

    /// <summary><see cref="ConvertSurfaceToBSpline"/> without freezing (ported code).</summary>
    internal static BSplineSurface? InternalConvertSurfaceToBSpline(Surface surf, double uf, double ul, double vf,
                                                                         double vl, double tol3d, Continuity continuity,
                                                                         int maxSegments, int maxDegree)
    {
        Surface s = surf;
        if (surf is TrimmedSurface RTS)
        {
            s = RTS.BasisSurface;
        }

        // use GeomConvert for direct conversion of analytic surfaces
        if (s is ElementarySurface)
        {
            TrimmedSurface rts = new(s, uf, ul, vf, vl);
            return global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(rts);
        }

        if (s is ExtrusionSurface extr)
        {
            Curve3D basis = extr.BasisCurve;
            // gp_Dir direction = extr->Direction(); // direction not used (skl)

            Continuity cnt = (continuity > C2 ? C2 : continuity);
            BSplineCurve3D bspl = InternalConvertCurveToBSpline(basis, uf, ul, tol3d, cnt, maxSegments, maxDegree)!;

            Transform3D shiftF = new(), shiftL = new();
            shiftF = shiftF.WithTranslation(extr.Value(uf, 0), extr.Value(uf, vf));
            shiftL = shiftL.WithTranslation(extr.Value(uf, 0), extr.Value(uf, vl));

            int nbPoles = bspl.PoleCount;
            NCollection_Array1<Point3D> poles = bspl.InternalPoles();
            NCollection_Array1<double> weights = bspl.InternalWeightsArray();
            NCollection_Array1<double> knots = bspl.InternalKnots();
            NCollection_Array1<int> mults = bspl.InternalMultiplicities();

            NCollection_Array2<Point3D> resPoles = new(1, nbPoles, 1, 2);
            NCollection_Array2<double> resWeigth = new(1, nbPoles, 1, 2);
            for (int j = 1; j <= nbPoles; j++)
            {
                resPoles[j, 1] = poles[j].Transformed(shiftF);
                resPoles[j, 2] = poles[j].Transformed(shiftL);
                resWeigth[j, 1] = weights[j];
                resWeigth[j, 2] = weights[j];
            }

            NCollection_Array1<double> vknots = new(1, 2);
            NCollection_Array1<int> vmults = new(1, 2);
            vknots[1] = vf;
            vknots[2] = vl;
            vmults[1] = vmults[2] = 2;

            BSplineSurface bspline = new(resPoles, resWeigth, knots, vknots, mults, vmults, bspl.Degree, 1,
                                              bspl.IsPeriodic, false);
            return bspline;
        }

        if (s is RevolutionSurface revol)
        {
            Curve3D basis = revol.BasisCurve;
            if (basis is OffsetCurve3D)
            {
                Continuity cnt = basis.Continuity;
                cnt = (cnt > C2 ? C2 : cnt);
                BSplineCurve3D bspl = InternalConvertCurveToBSpline(basis, vf, vl, tol3d, cnt, maxSegments, maxDegree)!;
                Axis3D axis = revol.Axis;
                RevolutionSurface newRevol = new(bspl, axis);
                s = newRevol;
            }
        }

        Surface surface = new TrimmedSurface(s, uf, ul, vf, vl);
        BSplineSurface? errSpl = null;
        for (int cnt = (int)(continuity > C3 ? C3 : continuity); cnt >= 0;)
        {
            try
            {
                Continuity cont = (Continuity)cnt;
                ApproxSurface approx = new(surface, tol3d / 2, cont, cont, maxDegree, maxDegree, maxSegments, 0);
                bool done = approx.IsDone;
                if (approx.MaxError() <= tol3d && done)
                {
                    return approx.Surface();
                }
                else
                {
                    if (approx.HasResult)
                    {
                        errSpl = approx.Surface();
                    }
                    break;
                }
            }
            catch (FailureException)
            {
                // a lower continuity is tried; a failure at C0 gives up, where OCCT retried C0 without end (decision
                // 16505)
                if (cnt == 0)
                {
                    break;
                }
                cnt--;
            }
        }

        return errSpl;
    }

    /// <summary>
    /// Joins the pcurves on <paramref name="face"/> of the edges <paramref name="edges"/> (in order, all seams or
    /// none) into the pcurve of <paramref name="edge"/> on that face, which is updated in place (its range on the
    /// face set, SameRange and SameParameter cleared). As OCCT, true on a plane (nothing written) and when a pcurve
    /// could not be joined, false when all were (occt-issues.md).
    /// Internal: it throws on a frozen edge; the public form is <see cref="PCurvesJoined"/>.
    /// </summary>
    internal static bool JoinPCurves(NCollection_HSequence<Shape> edges, in Face face, in Edge edge)
    {
        EdgeAnalysis sae = new();
        BrepBuilder b = new();

        try
        {
            // check if current face is plane.
            Surface? geomSurf = Tool.Surface(face);
            while (geomSurf is TrimmedSurface trimmed)
            {
                geomSurf = trimmed.BasisSurface;
            }
            if (geomSurf is PlaneSurface)
            {
                return true;
            }

            bool isEdgeSeam = false;
            Curve2D? crvRes1 = null, crvRes2 = null;
            Orientation resOrient;
            double newf = 0.0, newl = 0.0;
            // iterates on edges
            int i = 1;
            for (; i <= edges.Length(); i++)
            {
                Edge Edge = ShapeCasts.Edge(edges.Value(i));
                if (i == 1)
                {
                    isEdgeSeam = sae.IsSeam(Edge, face);
                }
                else if (isEdgeSeam && (!sae.IsSeam(Edge, face)))
                {
                    break; // different cases
                }
                else if (!isEdgeSeam && (sae.IsSeam(Edge, face)))
                {
                    break; // different cases
                }

                resOrient = Forward;
                Curve2D? c2d2 = null;
                double first2 = 0.0, last2 = 0.0;
                if (!sae.PCurve(Edge, face, out Curve2D? c2d, out double first, out double last, false))
                {
                    break;
                }

                if (isEdgeSeam)
                {
                    Edge tmpE1 = ShapeCasts.Edge(Edge.Reversed());
                    sae.PCurve(tmpE1, face, out c2d2, out first2, out last2, false);
                }

                if (i == 1)
                {
                    crvRes1 = c2d;
                    if (isEdgeSeam)
                    {
                        crvRes2 = c2d2;
                    }
                    newf = first;
                    newl = last;
                    resOrient = Edge.Orientation;
                }
                else
                {
                    if (!InternalJoinCurves(crvRes1!, c2d!, resOrient, Edge.Orientation, ref newf, ref newl, ref first,
                                            ref last, out Curve2D? newCrv, out _, out _))
                    {
                        break;
                    }

                    if (isEdgeSeam)
                    {
                        double newf2 = newf, newl2 = newl;

                        if (!InternalJoinCurves(crvRes2!, c2d2!, resOrient, Edge.Orientation, ref newf2, ref newl2,
                                                ref first2, ref last2, out Curve2D? newCrv2, out _, out _))
                        {
                            break;
                        }
                        crvRes2 = newCrv2;
                    }
                    crvRes1 = newCrv;
                    double fp2d = newCrv!.FirstParameter;
                    double lp2d = newCrv.LastParameter;
                    newl += (last - first);
                    if (fp2d > newf)
                    {
                        newf = fp2d;
                    }
                    if (lp2d < newl)
                    {
                        newl = lp2d;
                    }
                }
            }
            if (isEdgeSeam)
            {
                b.UpdateEdge(edge, crvRes1, crvRes2, face, 0);
            }
            else
            {
                b.UpdateEdge(edge, crvRes1, face, 0);
            }
            b.Range(edge, face, newf, newl);
            b.SameRange(edge, false);
            b.SameParameter(edge, false);
            // OCCT: true only when a pcurve could not be joined (the loop broke early); all joined gives false, against
            // the documentation. No caller in OCCT reads it.
            return (i <= edges.Length());
        }
        catch (FailureException) when (!edge.IsNull && !WorkingOverlay.IsReadOnly(edge.TShape()!))
        {
            // OCCT.NET: a frozen edge's exception goes through (decision 423's pattern)
        }
        return false;
    }

    // OCCT's GetCurveCopy template: a copy of the curve, reversed (with its range) for a REVERSED edge.
    private static Curve3D GetCurveCopy(Curve3D curve, ref double first, ref double last, Orientation orient)
    {
        if (orient == Reversed)
        {
            double cf = first;
            first = curve.ReversedParameter(last);
            last = curve.ReversedParameter(cf);
            return curve.Reversed();
        }
        return (Curve3D)curve.Copy();
    }

    private static Curve2D GetCurveCopy(Curve2D curve, ref double first, ref double last, Orientation orient)
    {
        if (orient == Reversed)
        {
            double cf = first;
            first = curve.ReversedParameter(last);
            last = curve.ReversedParameter(cf);
            return curve.Reversed();
        }
        return (Curve2D)curve.Copy();
    }

    // OCCT's SegmentCurve template: segments the (unfrozen) B-spline to [first, last] when its range is wider.
    private static void SegmentCurve(BSplineCurve3D curve, double first, double last)
    {
        if (curve.FirstParameter < first - global::Kerf.Engine.Foundation.Precision.PConfusion() || curve.LastParameter > last + global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (curve.IsPeriodic)
            {
                curve.Segment(first, last);
            }
            else
            {
                curve.Segment(Max(curve.FirstParameter, first), Min(curve.LastParameter, last));
            }
        }
    }

    private static void SegmentCurve(BSplineCurve2D curve, double first, double last)
    {
        if (curve.FirstParameter < first - global::Kerf.Engine.Foundation.Precision.PConfusion() || curve.LastParameter > last + global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (curve.IsPeriodic)
            {
                curve.Segment(first, last);
            }
            else
            {
                curve.Segment(Max(curve.FirstParameter, first), Min(curve.LastParameter, last));
            }
        }
    }

    // OCCT's GetReversedParameters template, for the 3D and 2D points.
    private static void GetReversedParameters(in Point3D p11, in Point3D p12, in Point3D p21, in Point3D p22, out bool isRev1,
                                              out bool isRev2)
    {
        GetReversedParameters(p11.DistanceTo(p21), p12.DistanceTo(p21), p11.DistanceTo(p22), p22.DistanceTo(p12), out isRev1,
                              out isRev2);
    }

    private static void GetReversedParameters(in Point2D p11, in Point2D p12, in Point2D p21, in Point2D p22,
                                              out bool isRev1, out bool isRev2)
    {
        GetReversedParameters(p11.DistanceTo(p21), p12.DistanceTo(p21), p11.DistanceTo(p22), p22.DistanceTo(p12), out isRev1,
                              out isRev2);
    }

    private static void GetReversedParameters(double d11, double d21, double d12, double d22, out bool isRev1, out bool isRev2)
    {
        isRev1 = false;
        isRev2 = false;
        // gka protection against crossing seem on second face

        double dmin1 = Min(d11, d21);
        double dmin2 = Min(d12, d22);
        if (Math.Abs(dmin1 - dmin2) <= global::Kerf.Engine.Foundation.Precision.Confusion() || dmin2 > dmin1)
        {
            isRev1 = (d11 < d21);
        }
        else if (dmin2 < dmin1)
        {
            isRev1 = (d12 < d22);
            isRev2 = true;
        }
    }

    /// <summary>
    /// Joins two 3D curves, the ranges [first1, last1] and [first2, last2] of edges oriented Orient1 and Orient2
    /// (the ranges are recomputed for reversed edges), into one B-spline <paramref name="c3dOut"/> (frozen; null when
    /// false). <paramref name="isRev1"/> and <paramref name="isRev2"/> tell which curve had to be reversed. True if
    /// the curves were joined.
    /// </summary>
    public static bool JoinCurves(Curve3D c3d1, Curve3D ac3d2, Orientation orient1, Orientation orient2,
                                  ref double first1, ref double last1, ref double first2, ref double last2,
                                  out Curve3D? c3dOut, out bool isRev1, out bool isRev2)
    {
        bool result = InternalJoinCurves(c3d1, ac3d2, orient1, orient2, ref first1, ref last1, ref first2, ref last2,
                                          out c3dOut, out isRev1, out isRev2);
        c3dOut?.Freeze();
        return result;
    }

    /// <summary>The 3D <c>JoinCurves</c> without freezing the joined curve (ported code).</summary>
    internal static bool InternalJoinCurves(Curve3D ac3d1, Curve3D ac3d2, Orientation orient1,
                                            Orientation orient2, ref double first1, ref double last1,
                                            ref double first2, ref double last2, out Curve3D? c3dOut, out bool isRev1,
                                            out bool isRev2)
    {
        // OCCT leaves the outputs unchanged when it returns false early; the port gives null and false
        c3dOut = null;
        isRev1 = false;
        isRev2 = false;

        Curve3D c3d1, c3d2;

        c3d1 = GetCurveCopy(ac3d1, ref first1, ref last1, orient1);
        c3d2 = GetCurveCopy(ac3d2, ref first2, ref last2, orient2);
        ConstructCurve scc = new();
        bool after = true;
        // the copies (or new curves) are unfrozen, so the segmenting and pole moves below edit only them
        BSplineCurve3D? bsplc1 = scc.InternalConvertToBSpline(c3d1, first1, last1, global::Kerf.Engine.Foundation.Precision.Confusion());
        BSplineCurve3D? bsplc2 = scc.InternalConvertToBSpline(c3d2, first2, last2, global::Kerf.Engine.Foundation.Precision.Confusion());
        //  newf = first1;
        //  newl = last1 + last2 - first2;

        if (bsplc1 is null || bsplc2 is null)
        {
            return false;
        }

        SegmentCurve(bsplc1, first1, last1);
        SegmentCurve(bsplc2, first2, last2);

        // regression on file 866026_M-f276-f311.brep bug OCC482
        Point3D pp11 = bsplc1.Pole(1);
        Point3D pp12 = bsplc1.Pole(bsplc1.PoleCount);

        Point3D pp21 = bsplc2.Pole(1);
        Point3D pp22 = bsplc2.Pole(bsplc2.PoleCount);

        GetReversedParameters(pp11, pp12, pp21, pp22, out isRev1, out isRev2);

        if (isRev1)
        {
            bsplc1.Reverse();
        }
        if (isRev2)
        {
            bsplc2.Reverse();
        }

        Point3D pmid = new(0.5 * (bsplc1.Pole(bsplc1.PoleCount).XYZ() + bsplc2.Pole(1).XYZ()));
        bsplc1.SetPole(bsplc1.PoleCount, pmid);
        bsplc2.SetPole(1, pmid);
        CompCurveToBSplineCurve connect3d = new(bsplc1);
        if (!connect3d.Add(bsplc2, global::Kerf.Engine.Foundation.Precision.Confusion(), after, false))
        {
            return false;
        }
        c3dOut = connect3d.InternalBSplineCurve();
        return true;
    }

    /// <summary>
    /// Joins two 2D curves (as the 3D <see cref="JoinCurves(Curve3D, Curve3D, Orientation, Orientation, ref double, ref double, ref double, ref double, out Curve3D?, out bool, out bool)"/>),
    /// through 3D curves on the XOY plane so that the second isn't reparametrised. With <paramref name="isError"/>,
    /// refuses a join that the topology makes suspicious (one edge with one vertex). The result is frozen (null when
    /// false).
    /// </summary>
    public static bool JoinCurves(Curve2D aC2d1, Curve2D aC2d2, Orientation orient1,
                                  Orientation orient2, ref double first1, ref double last1, ref double first2,
                                  ref double last2, out Curve2D? c2dOut, out bool isRev1, out bool isRev2,
                                  bool isError = false)
    {
        bool result = InternalJoinCurves(aC2d1, aC2d2, orient1, orient2, ref first1, ref last1, ref first2, ref last2,
                                          out c2dOut, out isRev1, out isRev2, isError);
        c2dOut?.Freeze();
        return result;
    }

    /// <summary>The 2D <c>JoinCurves</c> without freezing the joined curve (ported code).</summary>
    internal static bool InternalJoinCurves(Curve2D aC2d1, Curve2D aC2d2, Orientation orient1,
                                            Orientation orient2, ref double first1, ref double last1,
                                            ref double first2, ref double last2, out Curve2D? c2dOut,
                                            out bool isRev1, out bool isRev2, bool isError = false)
    {
        // OCCT leaves the outputs unchanged when it returns false early; the port gives null and false
        c2dOut = null;
        isRev1 = false;
        isRev2 = false;

        Curve2D c2d1, c2d2;
        c2d1 = GetCurveCopy(aC2d1, ref first1, ref last1, orient1);
        c2d2 = GetCurveCopy(aC2d2, ref first2, ref last2, orient2);
        ConstructCurve scc = new();
        bool after = true;

        BSplineCurve2D? bsplc12d = scc.InternalConvertToBSpline(c2d1, first1, last1, global::Kerf.Engine.Foundation.Precision.Confusion());
        BSplineCurve2D? bsplc22d = scc.InternalConvertToBSpline(c2d2, first2, last2, global::Kerf.Engine.Foundation.Precision.Confusion());

        if (bsplc12d is null || bsplc22d is null)
        {
            return false;
        }

        SegmentCurve(bsplc12d, first1, last1);
        SegmentCurve(bsplc22d, first2, last2);
        // gka protection against crossing seem on second face
        Point2D pp112d = new(bsplc12d.Pole(1).XY());
        Point2D pp122d = new(bsplc12d.Pole(bsplc12d.PoleCount).XY());

        Point2D pp212d = new(bsplc22d.Pole(1).XY());
        Point2D pp222d = new(bsplc22d.Pole(bsplc22d.PoleCount).XY());

        GetReversedParameters(pp112d, pp122d, pp212d, pp222d, out isRev1, out isRev2);

        // regression on file 866026_M-f276-f311.brep bug OCC482
        // if(isRev1 || isRev2)
        //   return newedge1;
        if (isRev1)
        {
            bsplc12d.Reverse();
        }
        if (isRev2)
        {
            bsplc22d.Reverse();
        }

        //---------------------------------------------------------
        // protection against invalid topology Housing(sam1296.brep(face 707) - bugmergeedges4.brep)
        if (isError)
        {
            Point2D pp1 = bsplc12d.Value(bsplc12d.FirstParameter);
            Point2D pp2 = bsplc12d.Value(bsplc12d.LastParameter);
            Point2D pp3 = bsplc12d.Value((bsplc12d.FirstParameter + bsplc12d.LastParameter) * 0.5);

            double leng = pp1.DistanceTo(pp2);
            bool isCircle = (leng < pp1.DistanceTo(pp3) + global::Kerf.Engine.Foundation.Precision.PConfusion());
            if ((pp1.DistanceTo(bsplc22d.Pole(1)) < leng) && !isCircle)
            {
                return false;
            }
        }
        //-------------------------------------------------------
        Point2D pmid1 = new(0.5 * (bsplc12d.Pole(bsplc12d.PoleCount).XY() + bsplc22d.Pole(1).XY()));
        bsplc12d.SetPole(bsplc12d.PoleCount, pmid1);
        bsplc22d.SetPole(1, pmid1);

        // abv 01 Sep 99: Geom2dConvert ALWAYS performs reparametrisation of the
        // second curve before merging; this is quite not suitable
        // Use 3d tool instead
        //      Geom2dConvert_CompCurveToBSplineCurve connect2d(bsplc12d);
        Point3D vPnt = new(0, 0, 0);
        Vector3D vDir = new(0, 0, 1);
        Plane3D vPln = new(vPnt, new Direction3D(vDir));
        BSplineCurve3D? bspl1 = Projections.To3d(bsplc12d, vPln) as BSplineCurve3D;
        BSplineCurve3D? bspl2 = Projections.To3d(bsplc22d, vPln) as BSplineCurve3D;
        CompCurveToBSplineCurve connect2d = new(bspl1!);
        if (!connect2d.Add(bspl2!, global::Kerf.Engine.Foundation.Precision.PConfusion(), after, false))
        {
            return false;
        }
        c2dOut = Projections.To2d(connect2d.InternalBSplineCurve()!, vPln);

        return true;
    }

    //=================================================================================================
    // The functional twin of JoinPCurves (OCCT.NET)
    //=================================================================================================

    /// <summary>
    /// The functional twin of <c>JoinPCurves</c>: <paramref name="edge"/> with the pcurves of
    /// <paramref name="edges"/> on <paramref name="face"/> joined into its pcurve on that face, run on the edge in a
    /// working overlay (Kerf unit 10H4): the result is frozen and is the edge itself when nothing was written (a
    /// plane). <paramref name="isDone"/> is JoinPCurves' result, inverted as in OCCT (false when all were joined).
    /// </summary>
    public static Edge PCurvesJoined(NCollection_HSequence<Shape> edges, in Face face,
                                            in Edge edge, out bool isDone)
    {
        if (edge.IsNull)
        {
            isDone = false;
            return edge;
        }

        Edge work = ShapeCasts.Edge(WorkingOverlay.WithUnfrozenCopied(edge, out Dictionary<TShape, TShape>? copies));
        using var overlay = WorkingOverlay.Begin(work);
        isDone = JoinPCurves(edges, face, work);
        Shape result = WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies);
        result.TShape()!.FreezeGraph();
        return ShapeCasts.Edge(result);
    }
}
