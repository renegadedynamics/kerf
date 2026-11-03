// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_Orientation;

namespace Occt;

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
public static partial class ShapeConstruct
{
    /// <summary>
    /// The 3D curve on [<paramref name="First"/>, <paramref name="Last"/>] as a B-spline (frozen): a B-spline is
    /// returned as it is, other curves are approximated (GeomConvert_ApproxCurve; degree at most 6 for conics), or
    /// converted by GeomConvert if the approximation fails.
    /// </summary>
    public static Geom_BSplineCurve? ConvertCurveToBSpline(Geom_Curve C3D, double First, double Last, double Tol3d,
                                                           Continuity Continuity, int MaxSegments, int MaxDegree)
    {
        Geom_BSplineCurve? aResult = InternalConvertCurveToBSpline(C3D, First, Last, Tol3d, Continuity, MaxSegments, MaxDegree);
        aResult?.Freeze();
        return aResult;
    }

    /// <summary>The 3D <see cref="ConvertCurveToBSpline(Geom_Curve, double, double, double, Continuity, int, int)"/> without freezing (ported code).</summary>
    internal static Geom_BSplineCurve? InternalConvertCurveToBSpline(Geom_Curve C3D, double First, double Last, double Tol3d,
                                                                     Continuity Continuity, int MaxSegments, int MaxDegree)
    {
        int MaxDeg = MaxDegree;
        Geom_BSplineCurve? aBSpline;
        if (C3D is Geom_BSplineCurve aBSplineC3D)
        {
            aBSpline = aBSplineC3D;
        }
        else
        {
            if (C3D is Geom_Conic)
            {
                MaxDeg = Min(MaxDeg, 6);
            }

            Geom_Curve tcurve = new Geom_TrimmedCurve(C3D, First, Last); //protection against parabols ets
            try
            {
                GeomConvert_ApproxCurve approx = new(tcurve, Tol3d, Continuity, MaxSegments, MaxDeg);
                if (approx.HasResult())
                {
                    aBSpline = approx.InternalCurve();
                }
                else
                {
                    aBSpline = GeomConvert.CurveToBSplineCurve(C3D, QuasiAngular);
                }
            }
            catch (FailureException)
            {
                aBSpline = GeomConvert.CurveToBSplineCurve(C3D, QuasiAngular);
            }
        }
        return aBSpline;
    }

    /// <summary>
    /// The 2D curve as a B-spline (frozen): a conic on [<paramref name="First"/>, <paramref name="Last"/>] is
    /// approximated (Geom2dConvert_ApproxCurve, or converted if that fails), other curves converted whole
    /// (Geom2dConvert), a B-spline returned as it is.
    /// </summary>
    public static Geom2d_BSplineCurve? ConvertCurveToBSpline(Geom2d_Curve C2D, double First, double Last, double Tol2d,
                                                             Continuity Continuity, int MaxSegments, int MaxDegree)
    {
        Geom2d_BSplineCurve? aResult = InternalConvertCurveToBSpline(C2D, First, Last, Tol2d, Continuity, MaxSegments, MaxDegree);
        aResult?.Freeze();
        return aResult;
    }

    /// <summary>The 2D <c>ConvertCurveToBSpline</c> without freezing (ported code).</summary>
    internal static Geom2d_BSplineCurve? InternalConvertCurveToBSpline(Geom2d_Curve C2D, double First, double Last,
                                                                       double Tol2d, Continuity Continuity,
                                                                       int MaxSegments, int MaxDegree)
    {
        Geom2d_BSplineCurve? aBSpline2d;
        if (C2D is Geom2d_Conic)
        {
            Geom2d_Curve tcurve = new Geom2d_TrimmedCurve(C2D, First, Last); //protection against parabols ets
            Geom2dConvert_ApproxCurve approx = new(tcurve, Tol2d, Continuity, MaxSegments, MaxDegree);
            if (approx.HasResult())
            {
                aBSpline2d = approx.InternalCurve();
            }
            else
            {
                aBSpline2d = Geom2dConvert.CurveToBSplineCurve(tcurve, QuasiAngular);
            }
        }
        else if (C2D is not Geom2d_BSplineCurve)
        {
            aBSpline2d = Geom2dConvert.CurveToBSplineCurve(C2D, QuasiAngular);
        }
        else
        {
            aBSpline2d = (Geom2d_BSplineCurve)C2D;
        }

        return aBSpline2d;
    }

    /// <summary>
    /// The surface on [UF, UL] x [VF, VL] as a B-spline surface (frozen): elementary surfaces converted exactly
    /// (GeomConvert), extrusions built from their converted basis curve, other surfaces approximated
    /// (GeomConvert_ApproxSurface, lowering the continuity on failure; a revolution of an offset curve has its basis
    /// converted first). Null if no approximation within <paramref name="Tol3d"/> is found; then the last
    /// approximation found, if any.
    /// </summary>
    /// <remarks>
    /// This has the purpose of GeomConvert::SurfaceToBSplineSurface, but treats offset surfaces more correctly and
    /// takes the UV limits and degree as arguments.
    /// </remarks>
    public static Geom_BSplineSurface? ConvertSurfaceToBSpline(Geom_Surface surf, double UF, double UL, double VF, double VL,
                                                               double Tol3d, Continuity Continuity, int MaxSegments,
                                                               int MaxDegree)
    {
        Geom_BSplineSurface? aResult = InternalConvertSurfaceToBSpline(surf, UF, UL, VF, VL, Tol3d, Continuity, MaxSegments,
                                                                       MaxDegree);
        aResult?.Freeze();
        return aResult;
    }

    /// <summary><see cref="ConvertSurfaceToBSpline"/> without freezing (ported code).</summary>
    internal static Geom_BSplineSurface? InternalConvertSurfaceToBSpline(Geom_Surface surf, double UF, double UL, double VF,
                                                                         double VL, double Tol3d, Continuity Continuity,
                                                                         int MaxSegments, int MaxDegree)
    {
        Geom_Surface S = surf;
        if (surf is Geom_RectangularTrimmedSurface RTS)
        {
            S = RTS.BasisSurface();
        }

        // use GeomConvert for direct conversion of analytic surfaces
        if (S is Geom_ElementarySurface)
        {
            Geom_RectangularTrimmedSurface aRTS = new(S, UF, UL, VF, VL);
            return GeomConvert.SurfaceToBSplineSurface(aRTS);
        }

        if (S is Geom_SurfaceOfLinearExtrusion extr)
        {
            Geom_Curve basis = extr.BasisCurve();
            // gp_Dir direction = extr->Direction(); // direction not used (skl)

            Continuity cnt = (Continuity > C2 ? C2 : Continuity);
            Geom_BSplineCurve bspl = InternalConvertCurveToBSpline(basis, UF, UL, Tol3d, cnt, MaxSegments, MaxDegree)!;

            Transform3D shiftF = new(), shiftL = new();
            shiftF.SetTranslation(extr.Value(UF, 0), extr.Value(UF, VF));
            shiftL.SetTranslation(extr.Value(UF, 0), extr.Value(UF, VL));

            int nbPoles = bspl.NbPoles();
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
            vknots[1] = VF;
            vknots[2] = VL;
            vmults[1] = vmults[2] = 2;

            Geom_BSplineSurface bspline = new(resPoles, resWeigth, knots, vknots, mults, vmults, bspl.Degree(), 1,
                                              bspl.IsPeriodic(), false);
            return bspline;
        }

        if (S is Geom_SurfaceOfRevolution revol)
        {
            Geom_Curve basis = revol.BasisCurve();
            if (basis is Geom_OffsetCurve)
            {
                Continuity cnt = basis.Continuity();
                cnt = (cnt > C2 ? C2 : cnt);
                Geom_BSplineCurve bspl = InternalConvertCurveToBSpline(basis, VF, VL, Tol3d, cnt, MaxSegments, MaxDegree)!;
                Axis3D axis = revol.Axis();
                Geom_SurfaceOfRevolution newRevol = new(bspl, axis);
                S = newRevol;
            }
        }

        Geom_Surface aSurface = new Geom_RectangularTrimmedSurface(S, UF, UL, VF, VL);
        Geom_BSplineSurface? errSpl = null;
        for (int cnt = (int)(Continuity > C3 ? C3 : Continuity); cnt >= 0;)
        {
            try
            {
                Continuity aCont = (Continuity)cnt;
                GeomConvert_ApproxSurface anApprox = new(aSurface, Tol3d / 2, aCont, aCont, MaxDegree, MaxDegree, MaxSegments, 0);
                bool Done = anApprox.IsDone();
                if (anApprox.MaxError() <= Tol3d && Done)
                {
                    return anApprox.Surface();
                }
                else
                {
                    if (anApprox.HasResult())
                    {
                        errSpl = anApprox.Surface();
                    }
                    break;
                }
            }
            catch (FailureException)
            {
                // OCCT: once the continuity is C0, a failure retries C0 without end
                if (cnt > 0)
                {
                    cnt--;
                }
                continue;
            }
        }

        return errSpl;
    }

    /// <summary>
    /// Joins the pcurves on <paramref name="theFace"/> of the edges <paramref name="edges"/> (in order, all seams or
    /// none) into the pcurve of <paramref name="theEdge"/> on that face, which is updated in place (its range on the
    /// face set, SameRange and SameParameter cleared). As OCCT, true on a plane (nothing written) and when a pcurve
    /// could not be joined, false when all were (occt-issues.md).
    /// Internal: it throws on a frozen edge; the public form is <see cref="PCurvesJoined"/>.
    /// </summary>
    internal static bool JoinPCurves(NCollection_HSequence<TopoDS_Shape> edges, in TopoDS_Face theFace, in TopoDS_Edge theEdge)
    {
        ShapeAnalysis_Edge sae = new();
        BRep_Builder B = new();

        try
        {
            // check if current face is plane.
            Geom_Surface? aGeomSurf = BRep_Tool.Surface(theFace);
            while (aGeomSurf is Geom_RectangularTrimmedSurface aTrimmed)
            {
                aGeomSurf = aTrimmed.BasisSurface();
            }
            if (aGeomSurf is Geom_Plane)
            {
                return true;
            }

            bool IsEdgeSeam = false;
            Geom2d_Curve? aCrvRes1 = null, aCrvRes2 = null;
            TopAbs_Orientation resOrient;
            double newf = 0.0, newl = 0.0;
            // iterates on edges
            int i = 1;
            for (; i <= edges.Length(); i++)
            {
                TopoDS_Edge Edge = TopoDS.Edge(edges.Value(i));
                if (i == 1)
                {
                    IsEdgeSeam = sae.IsSeam(Edge, theFace);
                }
                else if (IsEdgeSeam && (!sae.IsSeam(Edge, theFace)))
                {
                    break; // different cases
                }
                else if (!IsEdgeSeam && (sae.IsSeam(Edge, theFace)))
                {
                    break; // different cases
                }

                resOrient = TopAbs_FORWARD;
                Geom2d_Curve? c2d2 = null;
                double first2 = 0.0, last2 = 0.0;
                if (!sae.PCurve(Edge, theFace, out Geom2d_Curve? c2d, out double first, out double last, false))
                {
                    break;
                }

                if (IsEdgeSeam)
                {
                    TopoDS_Edge tmpE1 = TopoDS.Edge(Edge.Reversed());
                    sae.PCurve(tmpE1, theFace, out c2d2, out first2, out last2, false);
                }

                if (i == 1)
                {
                    aCrvRes1 = c2d;
                    if (IsEdgeSeam)
                    {
                        aCrvRes2 = c2d2;
                    }
                    newf = first;
                    newl = last;
                    resOrient = Edge.Orientation();
                }
                else
                {
                    if (!InternalJoinCurves(aCrvRes1!, c2d!, resOrient, Edge.Orientation(), ref newf, ref newl, ref first,
                                            ref last, out Geom2d_Curve? newCrv, out _, out _))
                    {
                        break;
                    }

                    if (IsEdgeSeam)
                    {
                        double newf2 = newf, newl2 = newl;

                        if (!InternalJoinCurves(aCrvRes2!, c2d2!, resOrient, Edge.Orientation(), ref newf2, ref newl2,
                                                ref first2, ref last2, out Geom2d_Curve? newCrv2, out _, out _))
                        {
                            break;
                        }
                        aCrvRes2 = newCrv2;
                    }
                    aCrvRes1 = newCrv;
                    double fp2d = newCrv!.FirstParameter();
                    double lp2d = newCrv.LastParameter();
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
            if (IsEdgeSeam)
            {
                B.UpdateEdge(theEdge, aCrvRes1, aCrvRes2, theFace, 0);
            }
            else
            {
                B.UpdateEdge(theEdge, aCrvRes1, theFace, 0);
            }
            B.Range(theEdge, theFace, newf, newl);
            B.SameRange(theEdge, false);
            B.SameParameter(theEdge, false);
            // OCCT: true only when a pcurve could not be joined (the loop broke early); all joined gives false, against
            // the documentation. No caller in OCCT reads it.
            return (i <= edges.Length());
        }
        catch (FailureException) when (!theEdge.IsNull() && !theEdge.TShape()!.IsFrozen)
        {
            // OCCT.NET: a frozen edge's exception goes through (decision 423's pattern)
        }
        return false;
    }

    // OCCT's GetCurveCopy template: a copy of the curve, reversed (with its range) for a REVERSED edge.
    private static Geom_Curve GetCurveCopy(Geom_Curve curve, ref double first, ref double last, TopAbs_Orientation orient)
    {
        if (orient == TopAbs_REVERSED)
        {
            double cf = first;
            first = curve.ReversedParameter(last);
            last = curve.ReversedParameter(cf);
            return curve.Reversed();
        }
        return (Geom_Curve)curve.Copy();
    }

    private static Geom2d_Curve GetCurveCopy(Geom2d_Curve curve, ref double first, ref double last, TopAbs_Orientation orient)
    {
        if (orient == TopAbs_REVERSED)
        {
            double cf = first;
            first = curve.ReversedParameter(last);
            last = curve.ReversedParameter(cf);
            return curve.Reversed();
        }
        return (Geom2d_Curve)curve.Copy();
    }

    // OCCT's SegmentCurve template: segments the (unfrozen) B-spline to [first, last] when its range is wider.
    private static void SegmentCurve(Geom_BSplineCurve curve, double first, double last)
    {
        if (curve.FirstParameter() < first - global::Kerf.Engine.Foundation.Precision.PConfusion() || curve.LastParameter() > last + global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (curve.IsPeriodic())
            {
                curve.Segment(first, last);
            }
            else
            {
                curve.Segment(Max(curve.FirstParameter(), first), Min(curve.LastParameter(), last));
            }
        }
    }

    private static void SegmentCurve(Geom2d_BSplineCurve curve, double first, double last)
    {
        if (curve.FirstParameter() < first - global::Kerf.Engine.Foundation.Precision.PConfusion() || curve.LastParameter() > last + global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (curve.IsPeriodic())
            {
                curve.Segment(first, last);
            }
            else
            {
                curve.Segment(Max(curve.FirstParameter(), first), Min(curve.LastParameter(), last));
            }
        }
    }

    // OCCT's GetReversedParameters template, for the 3D and 2D points.
    private static void GetReversedParameters(in Point3D p11, in Point3D p12, in Point3D p21, in Point3D p22, out bool isRev1,
                                              out bool isRev2)
    {
        GetReversedParameters(p11.Distance(p21), p12.Distance(p21), p11.Distance(p22), p22.Distance(p12), out isRev1,
                              out isRev2);
    }

    private static void GetReversedParameters(in Point2D p11, in Point2D p12, in Point2D p21, in Point2D p22,
                                              out bool isRev1, out bool isRev2)
    {
        GetReversedParameters(p11.Distance(p21), p12.Distance(p21), p11.Distance(p22), p22.Distance(p12), out isRev1,
                              out isRev2);
    }

    private static void GetReversedParameters(double d11, double d21, double d12, double d22, out bool isRev1, out bool isRev2)
    {
        isRev1 = false;
        isRev2 = false;
        // gka protection against crossing seem on second face

        double Dmin1 = Min(d11, d21);
        double Dmin2 = Min(d12, d22);
        if (Math.Abs(Dmin1 - Dmin2) <= global::Kerf.Engine.Foundation.Precision.Confusion() || Dmin2 > Dmin1)
        {
            isRev1 = (d11 < d21);
        }
        else if (Dmin2 < Dmin1)
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
    public static bool JoinCurves(Geom_Curve c3d1, Geom_Curve ac3d2, TopAbs_Orientation Orient1, TopAbs_Orientation Orient2,
                                  ref double first1, ref double last1, ref double first2, ref double last2,
                                  out Geom_Curve? c3dOut, out bool isRev1, out bool isRev2)
    {
        bool aResult = InternalJoinCurves(c3d1, ac3d2, Orient1, Orient2, ref first1, ref last1, ref first2, ref last2,
                                          out c3dOut, out isRev1, out isRev2);
        c3dOut?.Freeze();
        return aResult;
    }

    /// <summary>The 3D <c>JoinCurves</c> without freezing the joined curve (ported code).</summary>
    internal static bool InternalJoinCurves(Geom_Curve ac3d1, Geom_Curve ac3d2, TopAbs_Orientation Orient1,
                                            TopAbs_Orientation Orient2, ref double first1, ref double last1,
                                            ref double first2, ref double last2, out Geom_Curve? c3dOut, out bool isRev1,
                                            out bool isRev2)
    {
        // OCCT leaves the outputs unchanged when it returns false early; the port gives null and false
        c3dOut = null;
        isRev1 = false;
        isRev2 = false;

        Geom_Curve c3d1, c3d2;

        c3d1 = GetCurveCopy(ac3d1, ref first1, ref last1, Orient1);
        c3d2 = GetCurveCopy(ac3d2, ref first2, ref last2, Orient2);
        ShapeConstruct_Curve scc = new();
        bool After = true;
        // the copies (or new curves) are unfrozen, so the segmenting and pole moves below edit only them
        Geom_BSplineCurve? bsplc1 = scc.InternalConvertToBSpline(c3d1, first1, last1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Geom_BSplineCurve? bsplc2 = scc.InternalConvertToBSpline(c3d2, first2, last2, global::Kerf.Engine.Foundation.Precision.Confusion());
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
        Point3D pp12 = bsplc1.Pole(bsplc1.NbPoles());

        Point3D pp21 = bsplc2.Pole(1);
        Point3D pp22 = bsplc2.Pole(bsplc2.NbPoles());

        GetReversedParameters(pp11, pp12, pp21, pp22, out isRev1, out isRev2);

        if (isRev1)
        {
            bsplc1.Reverse();
        }
        if (isRev2)
        {
            bsplc2.Reverse();
        }

        Point3D pmid = new(0.5 * (bsplc1.Pole(bsplc1.NbPoles()).XYZ() + bsplc2.Pole(1).XYZ()));
        bsplc1.SetPole(bsplc1.NbPoles(), pmid);
        bsplc2.SetPole(1, pmid);
        GeomConvert_CompCurveToBSplineCurve connect3d = new(bsplc1);
        if (!connect3d.Add(bsplc2, global::Kerf.Engine.Foundation.Precision.Confusion(), After, false))
        {
            return false;
        }
        c3dOut = connect3d.InternalBSplineCurve();
        return true;
    }

    /// <summary>
    /// Joins two 2D curves (as the 3D <see cref="JoinCurves(Geom_Curve, Geom_Curve, TopAbs_Orientation, TopAbs_Orientation, ref double, ref double, ref double, ref double, out Geom_Curve?, out bool, out bool)"/>),
    /// through 3D curves on the XOY plane so that the second isn't reparametrised. With <paramref name="isError"/>,
    /// refuses a join that the topology makes suspicious (one edge with one vertex). The result is frozen (null when
    /// false).
    /// </summary>
    public static bool JoinCurves(Geom2d_Curve aC2d1, Geom2d_Curve aC2d2, TopAbs_Orientation Orient1,
                                  TopAbs_Orientation Orient2, ref double first1, ref double last1, ref double first2,
                                  ref double last2, out Geom2d_Curve? C2dOut, out bool isRev1, out bool isRev2,
                                  bool isError = false)
    {
        bool aResult = InternalJoinCurves(aC2d1, aC2d2, Orient1, Orient2, ref first1, ref last1, ref first2, ref last2,
                                          out C2dOut, out isRev1, out isRev2, isError);
        C2dOut?.Freeze();
        return aResult;
    }

    /// <summary>The 2D <c>JoinCurves</c> without freezing the joined curve (ported code).</summary>
    internal static bool InternalJoinCurves(Geom2d_Curve aC2d1, Geom2d_Curve aC2d2, TopAbs_Orientation Orient1,
                                            TopAbs_Orientation Orient2, ref double first1, ref double last1,
                                            ref double first2, ref double last2, out Geom2d_Curve? C2dOut,
                                            out bool isRev1, out bool isRev2, bool isError = false)
    {
        // OCCT leaves the outputs unchanged when it returns false early; the port gives null and false
        C2dOut = null;
        isRev1 = false;
        isRev2 = false;

        Geom2d_Curve c2d1, c2d2;
        c2d1 = GetCurveCopy(aC2d1, ref first1, ref last1, Orient1);
        c2d2 = GetCurveCopy(aC2d2, ref first2, ref last2, Orient2);
        ShapeConstruct_Curve scc = new();
        bool After = true;

        Geom2d_BSplineCurve? bsplc12d = scc.InternalConvertToBSpline(c2d1, first1, last1, global::Kerf.Engine.Foundation.Precision.Confusion());
        Geom2d_BSplineCurve? bsplc22d = scc.InternalConvertToBSpline(c2d2, first2, last2, global::Kerf.Engine.Foundation.Precision.Confusion());

        if (bsplc12d is null || bsplc22d is null)
        {
            return false;
        }

        SegmentCurve(bsplc12d, first1, last1);
        SegmentCurve(bsplc22d, first2, last2);
        // gka protection against crossing seem on second face
        Point2D pp112d = new(bsplc12d.Pole(1).XY());
        Point2D pp122d = new(bsplc12d.Pole(bsplc12d.NbPoles()).XY());

        Point2D pp212d = new(bsplc22d.Pole(1).XY());
        Point2D pp222d = new(bsplc22d.Pole(bsplc22d.NbPoles()).XY());

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
            Point2D pp1 = bsplc12d.Value(bsplc12d.FirstParameter());
            Point2D pp2 = bsplc12d.Value(bsplc12d.LastParameter());
            Point2D pp3 = bsplc12d.Value((bsplc12d.FirstParameter() + bsplc12d.LastParameter()) * 0.5);

            double leng = pp1.Distance(pp2);
            bool isCircle = (leng < pp1.Distance(pp3) + global::Kerf.Engine.Foundation.Precision.PConfusion());
            if ((pp1.Distance(bsplc22d.Pole(1)) < leng) && !isCircle)
            {
                return false;
            }
        }
        //-------------------------------------------------------
        Point2D pmid1 = new(0.5 * (bsplc12d.Pole(bsplc12d.NbPoles()).XY() + bsplc22d.Pole(1).XY()));
        bsplc12d.SetPole(bsplc12d.NbPoles(), pmid1);
        bsplc22d.SetPole(1, pmid1);

        // abv 01 Sep 99: Geom2dConvert ALWAYS performs reparametrisation of the
        // second curve before merging; this is quite not suitable
        // Use 3d tool instead
        //      Geom2dConvert_CompCurveToBSplineCurve connect2d(bsplc12d);
        Point3D vPnt = new(0, 0, 0);
        Vector3D vDir = new(0, 0, 1);
        Plane3D vPln = new(vPnt, new Direction3D(vDir));
        Geom_BSplineCurve? bspl1 = GeomAPI.To3d(bsplc12d, vPln) as Geom_BSplineCurve;
        Geom_BSplineCurve? bspl2 = GeomAPI.To3d(bsplc22d, vPln) as Geom_BSplineCurve;
        GeomConvert_CompCurveToBSplineCurve connect2d = new(bspl1!);
        if (!connect2d.Add(bspl2!, global::Kerf.Engine.Foundation.Precision.PConfusion(), After, false))
        {
            return false;
        }
        C2dOut = GeomAPI.To2d(connect2d.InternalBSplineCurve()!, vPln);

        return true;
    }

    //=================================================================================================
    // The functional twin of JoinPCurves (OCCT.NET)
    //=================================================================================================

    /// <summary>
    /// The functional twin of <c>JoinPCurves</c>: <paramref name="theEdge"/> with the pcurves of
    /// <paramref name="theEdges"/> on <paramref name="theFace"/> joined into its pcurve on that face, run on an exact
    /// copy for write (decision 420's pattern): the result is frozen and is the edge itself when nothing was written
    /// (a plane). <paramref name="isDone"/> is JoinPCurves' result, inverted as in OCCT (false when all were joined).
    /// </summary>
    public static TopoDS_Edge PCurvesJoined(NCollection_HSequence<TopoDS_Shape> theEdges, in TopoDS_Face theFace,
                                            in TopoDS_Edge theEdge, out bool isDone)
    {
        if (theEdge.IsNull())
        {
            isDone = false;
            return theEdge;
        }

        TopoDS_Edge aCopy = TopoDS.Edge(BRepTools.ExactCopyForWrite(theEdge, out _));
        isDone = JoinPCurves(theEdges, theFace, aCopy);
        TopoDS_Shape aResult = BRepTools.KeepUnchanged(theEdge, aCopy, out _);
        aResult.TShape()!.FreezeGraph();
        return TopoDS.Edge(aResult);
    }
}
