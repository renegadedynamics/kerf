// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_DistanceSS.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_DistanceSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Measurement.Distance.SupportType;
using static Kerf.Engine.Extrema.ExtAlgo;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// The minimum distance between two elementary shapes (vertex, edge or face), used by
/// <see cref="DistanceShapeShape"/>.
/// </summary>
/// <remarks>
/// A working object computed in its constructor. The supports of its solutions are the input shapes, or, for an
/// infinite edge or face facing a finite one, a trimmed copy built here and frozen (it is handed out as a support).
/// </remarks>
internal sealed class DistanceSS
{
    private readonly NCollection_Sequence<SolutionElem> _seqSolShape1 = new(); //!< Solutions on the first shape
    private readonly NCollection_Sequence<SolutionElem> _seqSolShape2 = new(); //!< Solutions on the second shape
    private double _dstRef; //!< The minimal distance found
    private bool _modif; //!< Flag indicating whether the solution was improved or not
    private readonly double _eps; //!< Deflection
    private readonly ExtFlag _flag; //!< Extrema flag indicating what solutions to look for
    private readonly ExtAlgo _algo; //!< Extrema algo to be used to look for solutions

    /// <summary>
    /// Computes the distance between two shapes (face, edge or vertex), starting from the distance
    /// <paramref name="dstRef"/>; solutions within <paramref name="deflection"/> of the minimum are kept. The
    /// flag and the algorithm apply only to point-face extrema.
    /// </summary>
    public DistanceSS(in Shape s1,
                                  in Shape s2,
                                  in Box box1,
                                  in Box box2,
                                  double dstRef,
                                  double deflection = 1.0e-7, // Precision::Confusion()
                                  ExtFlag extFlag = ExtFlag_MINMAX,
                                  ExtAlgo extAlgo = ExtAlgo_Grad)
    {
        _dstRef = dstRef;
        _modif = false;
        _eps = deflection;
        _flag = extFlag;
        _algo = extAlgo;
        Perform(s1, s2, box1, box2);
    }

    /// <summary>True if the distance has been computed (improved), false otherwise.</summary>
    public bool IsDone => _modif;

    /// <summary>The distance value.</summary>
    public double DistValue() => _dstRef;

    /// <summary>The solutions on the first shape (a copy).</summary>
    public ISchematiqList<SolutionElem> Seq1Value() => PackedList.Own([.. _seqSolShape1]);

    /// <summary>The solutions on the second shape (a copy).</summary>
    public ISchematiqList<SolutionElem> Seq2Value() => PackedList.Own([.. _seqSolShape2]);

    /// <summary>The solutions on the first shape, the object's own sequence (OCCT's const reference).</summary>
    internal NCollection_Sequence<SolutionElem> InternalSeq1Value() => _seqSolShape1;

    /// <summary>The solutions on the second shape, the object's own sequence (OCCT's const reference).</summary>
    internal NCollection_Sequence<SolutionElem> InternalSeq2Value() => _seqSolShape2;

    //=================================================================================================

    private static bool TRI_SOLUTION(NCollection_Sequence<SolutionElem> seqSol, in Point3D pt)
    {
        foreach (SolutionElem it in seqSol)
        {
            double dst = it.Point().DistanceTo(pt);
            if (dst <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    private static void MIN_SOLUTION(NCollection_Sequence<SolutionElem> SeqSol1,
                                     NCollection_Sequence<SolutionElem> SeqSol2,
                                     double dstRef,
                                     double eps,
                                     NCollection_Sequence<SolutionElem> seqSol1,
                                     NCollection_Sequence<SolutionElem> seqSol2)
    {
        for (int it = 1; it <= SeqSol1.Length(); it++)
        {
            double dst1 = SeqSol1.Value(it).Dist();
            if (Math.Abs(dst1 - dstRef) < eps)
            {
                seqSol1.Append(SeqSol1.Value(it));
                seqSol2.Append(SeqSol2.Value(it));
            }
        }
    }

    //=================================================================================================

    private static void TRIM_INFINIT_EDGE(in Edge s1,
                                          in Edge s2,
                                          ref Edge aResEdge,
                                          ref bool bIsTrim1,
                                          ref bool bIsTrim2)
    {
        if (Tool.Degenerated(s1) || Tool.Degenerated(s2))
        {
            return;
        }

        aResEdge = s2;
        double first1, last1, first2, last2;
        Curve3D? pCurv1 = Tool.Curve(s1, out first1, out last1);
        Curve3D? pCurv2 = Tool.Curve(s2, out first2, out last2);

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(first1) && global::Kerf.Engine.Foundation.Precision.IsInfinite(last1)
            && global::Kerf.Engine.Foundation.Precision.IsInfinite(first2) && global::Kerf.Engine.Foundation.Precision.IsInfinite(last2))
        {
            return;
        }

        double umin = 0.0, umax = 0.0;
        bool bUmin, bUmax;
        bUmin = bUmax = false;

        Curve3D? pCurv = null;
        if (pCurv1 is not null && (global::Kerf.Engine.Foundation.Precision.IsInfinite(first1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(last1)))
        {
            pCurv = pCurv1;
            bIsTrim1 = true;
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(first1))
            {
                bUmin = true;
                umin = first1;
            }
            else if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(last1))
            {
                bUmax = true;
                umax = last1;
            }
        }
        else if (pCurv2 is not null && (global::Kerf.Engine.Foundation.Precision.IsInfinite(first2) || global::Kerf.Engine.Foundation.Precision.IsInfinite(last2)))
        {
            pCurv = pCurv2;
            bIsTrim2 = true;
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(first2))
            {
                bUmin = true;
                umin = first2;
            }
            else if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(last2))
            {
                bUmax = true;
                umax = last2;
            }
        }
        if (bIsTrim1 || bIsTrim2)
        {
            var edgeBox = new Box();
            if (bIsTrim1)
            {
                Bounds.Add(s2, ref edgeBox);
            }
            if (bIsTrim2)
            {
                Bounds.Add(s1, ref edgeBox);
            }
            edgeBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);

            var pnt0 = new Point3D(xmin, ymin, zmin);
            var pnt1 = new Point3D(xmin, ymax, zmin);
            var pnt2 = new Point3D(xmin, ymax, zmax);
            var pnt3 = new Point3D(xmin, ymin, zmax);
            var pnt4 = new Point3D(xmax, ymax, zmin);
            var pnt5 = new Point3D(xmax, ymax, zmax);
            var pnt6 = new Point3D(xmax, ymin, zmax);
            var pnt7 = new Point3D(xmax, ymin, zmin);

            Span<double> arrU = stackalloc double[8];
            var proj = new ProjectPointOnCurve(pnt0, pCurv!);
            /*szv:aProj.Perform(aPnt0);*/
            arrU[0] = proj.LowerDistanceParameter();
            proj.Perform(pnt1);
            arrU[1] = proj.LowerDistanceParameter();
            proj.Perform(pnt2);
            arrU[2] = proj.LowerDistanceParameter();
            proj.Perform(pnt3);
            arrU[3] = proj.LowerDistanceParameter();
            proj.Perform(pnt4);
            arrU[4] = proj.LowerDistanceParameter();
            proj.Perform(pnt5);
            arrU[5] = proj.LowerDistanceParameter();
            proj.Perform(pnt6);
            arrU[6] = proj.LowerDistanceParameter();
            proj.Perform(pnt7);
            arrU[7] = proj.LowerDistanceParameter();

            if (!bUmin)
            {
                umin = arrU[0];
            }

            if (!bUmax)
            {
                umax = arrU[0];
            }

            int i = 0;
            while (i < 8)
            {
                double u = arrU[i++];
                if (u < umin)
                {
                    umin = u;
                }
                else if (u > umax)
                {
                    umax = u;
                }
            }

            double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
            if (bIsTrim1)
            {
                tol = Tool.Tolerance(s1);
            }
            else if (bIsTrim2)
            {
                tol = Tool.Tolerance(s2);
            }

            double epsU = new CurveAdaptor(pCurv!).Resolution(3.0 * tol);
            if (Math.Abs(umin - umax) < epsU)
            {
                umin -= epsU;
                umax += epsU;
            }

            Curve3D result = new TrimmedCurve3D(pCurv!, umin, umax);
            aResEdge = new MakeEdge(result).Edge();
        }
    }

    //=================================================================================================

    private static void TRIM_INFINIT_FACE(in Shape s1,
                                          in Shape s2,
                                          ref Face aResFace,
                                          out bool bIsInfinit)
    {
        bIsInfinit = false;

        ShapeType type1 = s1.Type;
        ShapeType type2 = s2.Type;

        Edge e;
        Face f;

        if (type1 == ShapeType.Edge && type2 == ShapeType.Face)
        {
            e = ShapeCasts.Edge(s1);
            if (Tool.Degenerated(e))
            {
                return;
            }
            f = ShapeCasts.Face(s2);
        }
        else if (type2 == ShapeType.Edge && type1 == ShapeType.Face)
        {
            e = ShapeCasts.Edge(s2);
            if (Tool.Degenerated(e))
            {
                return;
            }
            f = ShapeCasts.Face(s1);
        }
        else
        {
            bIsInfinit = false;
            return;
        }

        aResFace = f;
        Surface pSurf = Tool.Surface(f)!;

        bool bRestrict = Tool.NaturalRestriction(f);

        double u1, v1, u2, v2;
        double umin = RealLast(), umax = RealFirst(), vmin = RealLast(), vmax = RealFirst();
        bool bUmin, bUmax, bVmin, bVmax;
        bUmin = bUmax = bVmin = bVmax = false;
        bool bIsTrim = false;

        if (bRestrict)
        {
            pSurf.ParameterBounds(out u1, out u2, out v1, out v2);
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1))
            {
                bIsTrim = true;
            }
            else
            {
                umin = u1;
                bUmin = true;
            }

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
            {
                bIsTrim = true;
            }
            else
            {
                umax = u2;
                bUmax = true;
            }

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1))
            {
                bIsTrim = true;
            }
            else
            {
                vmin = v1;
                bVmin = true;
            }

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
            {
                bIsTrim = true;
            }
            else
            {
                vmax = v2;
                bVmax = true;
            }
        }
        else
        {
            BrepTools.UVBounds(f, out u1, out u2, out v1, out v2);
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) && global::Kerf.Engine.Foundation.Precision.IsInfinite(u2) && global::Kerf.Engine.Foundation.Precision.IsInfinite(v1)
                && global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
            {
                bIsTrim = true;
            }
        }

        if (bIsTrim)
        {
            var edgeBox = new Box();
            Bounds.Add(e, ref edgeBox);

            if (edgeBox.IsWhole)
            {
                return;
            }

            edgeBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);

            var pnt0 = new Point3D(xmin, ymin, zmin);
            var pnt1 = new Point3D(xmin, ymax, zmin);
            var pnt2 = new Point3D(xmin, ymax, zmax);
            var pnt3 = new Point3D(xmin, ymin, zmax);
            var pnt4 = new Point3D(xmax, ymax, zmin);
            var pnt5 = new Point3D(xmax, ymax, zmax);
            var pnt6 = new Point3D(xmax, ymin, zmax);
            var pnt7 = new Point3D(xmax, ymin, zmin);

            // OCCT reads arrU and arrV uninitialised where a projection fails; the port's are zero (occt-issues.md).
            Span<double> arrU = stackalloc double[8];
            Span<double> arrV = stackalloc double[8];
            var proj = new ProjectPointOnSurface(pnt0, pSurf);
            /*szv:aProj.Perform(aPnt0);*/
            if (proj.IsDone)
            {
                proj.LowerDistanceParameters(out arrU[0], out arrV[0]);
            }
            proj.Perform(pnt1);
            if (proj.IsDone)
            {
                proj.LowerDistanceParameters(out arrU[1], out arrV[1]);
            }
            proj.Perform(pnt2);
            if (proj.IsDone)
            {
                proj.LowerDistanceParameters(out arrU[2], out arrV[2]);
            }
            proj.Perform(pnt3);
            if (proj.IsDone)
            {
                proj.LowerDistanceParameters(out arrU[3], out arrV[3]);
            }
            proj.Perform(pnt4);
            if (proj.IsDone)
            {
                proj.LowerDistanceParameters(out arrU[4], out arrV[4]);
            }
            proj.Perform(pnt5);
            if (proj.IsDone)
            {
                proj.LowerDistanceParameters(out arrU[5], out arrV[5]);
            }
            proj.Perform(pnt6);
            if (proj.IsDone)
            {
                proj.LowerDistanceParameters(out arrU[6], out arrV[6]);
            }
            proj.Perform(pnt7);
            if (proj.IsDone)
            {
                proj.LowerDistanceParameters(out arrU[7], out arrV[7]);
            }

            if (!bUmin)
            {
                umin = arrU[0];
            }
            if (!bUmax)
            {
                umax = arrU[0];
            }
            if (!bVmin)
            {
                vmin = arrV[0];
            }
            if (!bVmax)
            {
                vmax = arrV[0];
            }

            int i = 0;
            while (i < 8)
            {
                double u = arrU[i];
                if (u < umin)
                {
                    umin = u;
                }
                else if (u > umax)
                {
                    umax = u;
                }

                double v = arrV[i];
                if (v < vmin)
                {
                    vmin = v;
                }
                else if (v > vmax)
                {
                    vmax = v;
                }

                i++;
            }

            var gas = new SurfaceAdaptor(pSurf);
            double tol = Tool.Tolerance(f);

            double epsU = gas.UResolution(3.0 * tol);
            if (Math.Abs(umin - umax) < epsU)
            {
                umin -= epsU;
                umax += epsU;
            }

            double epsV = gas.VResolution(3.0 * tol);
            if (Math.Abs(vmin - vmax) < epsV)
            {
                vmin -= epsV;
                vmax += epsV;
            }

            Surface result = new TrimmedSurface(pSurf, umin, umax, vmin, vmax);
            aResFace = new MakeFace(result, global::Kerf.Engine.Foundation.Precision.Confusion()).Face();

            bIsInfinit = true;
        }
        else
        {
            bIsInfinit = false;
        }
    }

    //=================================================================================================

    private static void PERFORM_C0(in Edge s1,
                                   in Edge s2,
                                   NCollection_Sequence<SolutionElem> seqSol1,
                                   NCollection_Sequence<SolutionElem> seqSol2,
                                   double dstRef,
                                   ref double mDstRef,
                                   double eps)
    {
        if (Tool.Degenerated(s1) || Tool.Degenerated(s2))
        {
            return;
        }

        int iE;
        for (iE = 0; iE < 2; iE++)
        {
            Edge e, eother;
            if (iE == 0)
            {
                e = s1;
                eother = s2;
            }
            else
            {
                e = s2;
                eother = s1;
            }

            double first, last;
            Curve3D? pCurv = Tool.Curve(e, out first, out last);

            double fOther, lOther;
            Curve3D? pCurvOther = Tool.Curve(eother, out fOther, out lOther);

            if (pCurv is null)
            {
                return;
            }

            if (pCurv.Continuity == C0)
            {
                double epsP = global::Kerf.Engine.Foundation.Precision.PConfusion();

                var adaptorCurve = new CurveAdaptor(pCurv, first, last);
                int nbIntervals = adaptorCurve.NbIntervals(C1);

                var arrInter = new NCollection_Array1<double>(1, 1 + nbIntervals);
                adaptorCurve.Intervals(arrInter, C1);

                var adaptorCurveOther = new CurveAdaptor(pCurvOther!, fOther, lOther);
                int nbIntervalsOther = adaptorCurveOther.NbIntervals(C1);
                var arrInterOther = new NCollection_Array1<double>(1, 1 + nbIntervalsOther);
                adaptorCurveOther.Intervals(arrInterOther, C1);

                double udeb, ufin;
                Tool.Range(eother, out udeb, out ufin);

                Point3D pt;
                int i, ii;
                for (i = 1; i <= arrInter.Length(); i++)
                {
                    double parameter = arrInter[i];
                    Point3D pnt = adaptorCurve.Value(parameter);
                    // a local vertex, never handed out: not frozen
                    Vertex v1 = new MakeVertex(pnt).InternalVertex();

                    var ext = new ExtPC(v1, eother);
                    int nbExtrema = ext.IsDone ? ext.ExtCount : 0;
                    if (nbExtrema > 0)
                    {
                        // Search minimum distance dstmin
                        double dstmin = ext.SquareDistance(1);
                        for (ii = 2; ii <= nbExtrema; ii++)
                        {
                            double sDst = ext.SquareDistance(ii);
                            if (sDst < dstmin)
                            {
                                dstmin = sDst;
                            }
                        }
                        dstmin = Math.Sqrt(dstmin);

                        if ((dstmin < dstRef - eps) || (Math.Abs(dstmin - dstRef) < eps))
                        {
                            for (ii = 1; ii <= nbExtrema; ii++)
                            {
                                if (Math.Abs(dstmin - Math.Sqrt(ext.SquareDistance(ii))) < eps)
                                {
                                    pt = ext.Point(ii);
                                    // Pt - point on the curve pCurvOther/Eother, but
                                    // if iE == 0 -> Eother correspond to edge S2
                                    // and to edge S1 in the opposite case.
                                    // Therefore we should search Pt through previous solution points on Other curve
                                    // (edge): if iE == 0 - on edge S2, namely through SeqSol2, else       - on edge S1,
                                    // namely through SeqSol1.
                                    bool triSolutionResult =
                                        (iE == 0) ? TRI_SOLUTION(seqSol2, pt) : TRI_SOLUTION(seqSol1, pt);
                                    if (triSolutionResult)
                                    {
                                        // Check if the parameter does not correspond to a vertex
                                        double t = ext.Parameter(ii);
                                        if ((Math.Abs(t - udeb) >= epsP) && (Math.Abs(t - ufin) > epsP))
                                        {
                                            if (mDstRef > dstmin)
                                            {
                                                mDstRef = dstmin;
                                            }
                                            var sol1 = new SolutionElem(dstmin, pnt, SupportType.IsOnEdge, e, parameter);
                                            var sol2 = new SolutionElem(dstmin, pt, SupportType.IsOnEdge, eother, t);
                                            seqSol1.Append(iE == 0 ? sol1 : sol2);
                                            seqSol2.Append(iE == 0 ? sol2 : sol1);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    for (int i2 = 1; i2 <= arrInterOther.Length(); i2++)
                    {
                        double parameterOther = arrInterOther[i2];
                        Point3D pntOther = adaptorCurveOther.Value(parameterOther);
                        double dst = pnt.DistanceTo(pntOther);
                        if ((dst < dstRef - eps) || (Math.Abs(dst - dstRef) < eps))
                        {
                            if (mDstRef > dst)
                            {
                                mDstRef = dst;
                            }
                            var sol1 = new SolutionElem(dst, pnt, SupportType.IsOnEdge, e, parameter);
                            var sol2 = new SolutionElem(dst, pntOther, SupportType.IsOnEdge, eother, parameterOther);
                            seqSol1.Append(iE == 0 ? sol1 : sol2);
                            seqSol2.Append(iE == 0 ? sol2 : sol1);
                        }
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : isOnBoundary
    // purpose  : Checks in 3d if the extrema point belongs to edge boundary
    //=======================================================================
    private static bool IsOnBoundary(in Edge edge, in Point3D sol, double param, double pTol)
    {
        for (var it = new Iterator(edge); it.More(); it.Next())
        {
            Vertex v = ShapeCasts.Vertex(it.Value());
            double vParam = Tool.Parameter(v, edge);
            if (Math.Abs(vParam - param) < pTol
                && Tool.Pnt(v).DistanceTo(sol) < Tool.Tolerance(v))
            {
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    private void Perform(in Shape s1, in Shape s2, in Box box1, in Box box2)
    {
        _seqSolShape1.Clear();
        _seqSolShape2.Clear();
        _modif = false;

        double bbDist = box1.Distance(box2);
        if (bbDist - _dstRef > _eps)
        {
            // The Box-Box distance is greater than the start distance.
            // The solution cannot be improved further.
            return;
        }

        switch (s1.Type)
        {
            case ShapeType.Vertex:
            {
                Vertex v1 = ShapeCasts.Vertex(s1);
                switch (s2.Type)
                {
                    case ShapeType.Vertex:
                    {
                        Vertex v2 = ShapeCasts.Vertex(s2);
                        Perform(v1, v2, _seqSolShape1, _seqSolShape2);
                        break;
                    }
                    case ShapeType.Edge:
                    {
                        Edge e2 = ShapeCasts.Edge(s2);
                        Perform(v1, e2, _seqSolShape1, _seqSolShape2);
                        break;
                    }
                    case ShapeType.Face:
                    {
                        Face f2 = ShapeCasts.Face(s2);
                        Perform(v1, f2, _seqSolShape1, _seqSolShape2);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case ShapeType.Edge:
            {
                Edge e1 = ShapeCasts.Edge(s1);
                switch (s2.Type)
                {
                    case ShapeType.Vertex:
                    {
                        Vertex v2 = ShapeCasts.Vertex(s2);
                        Perform(v2, e1, _seqSolShape2, _seqSolShape1);
                        break;
                    }
                    case ShapeType.Edge:
                    {
                        Edge e2 = ShapeCasts.Edge(s2);
                        var trimEdge = new Edge();
                        bool bIsTrim1 = false;
                        bool bIsTrim2 = false;
                        TRIM_INFINIT_EDGE(e1, e2, ref trimEdge, ref bIsTrim1, ref bIsTrim2);
                        if (bIsTrim1)
                        {
                            e1 = trimEdge;
                        }
                        if (bIsTrim2)
                        {
                            e2 = trimEdge;
                        }
                        Perform(e1, e2, _seqSolShape1, _seqSolShape2);
                        break;
                    }
                    case ShapeType.Face:
                    {
                        Face f2 = ShapeCasts.Face(s2);
                        var trimFace = new Face();
                        TRIM_INFINIT_FACE(e1, f2, ref trimFace, out bool bIsInfinit);
                        if (bIsInfinit)
                        {
                            f2 = trimFace;
                        }
                        Perform(e1, f2, _seqSolShape1, _seqSolShape2);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case ShapeType.Face:
            {
                Face f1 = ShapeCasts.Face(s1);
                switch (s2.Type)
                {
                    case ShapeType.Vertex:
                    {
                        Vertex v2 = ShapeCasts.Vertex(s2);
                        Perform(v2, f1, _seqSolShape2, _seqSolShape1);
                        break;
                    }
                    case ShapeType.Edge:
                    {
                        Edge e2 = ShapeCasts.Edge(s2);
                        var trimFace = new Face();
                        TRIM_INFINIT_FACE(f1, e2, ref trimFace, out bool bIsInfinit);
                        if (bIsInfinit)
                        {
                            f1 = trimFace;
                        }
                        Perform(e2, f1, _seqSolShape2, _seqSolShape1);
                        break;
                    }
                    case ShapeType.Face:
                    {
                        Face f2 = ShapeCasts.Face(s2);
                        Perform(f1, f2, _seqSolShape1, _seqSolShape2);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            default:
                break;
        }
    }

    //=================================================================================================

    /// <summary>The distance between two vertices.</summary>
    private void Perform(in Vertex s1,
                         in Vertex s2,
                         NCollection_Sequence<SolutionElem> seqSolShape1,
                         NCollection_Sequence<SolutionElem> seqSolShape2)
    {
        Point3D p1 = Tool.Pnt(s1);
        Point3D p2 = Tool.Pnt(s2);

        double dst = p1.DistanceTo(p2);
        if ((dst < _dstRef - _eps) || (Math.Abs(dst - _dstRef) < _eps))
        {
            if (_dstRef > dst)
            {
                _dstRef = dst;
            }
            _modif = true;
            var sol1 = new SolutionElem(dst, p1, SupportType.IsVertex, s1);
            var sol2 = new SolutionElem(dst, p2, SupportType.IsVertex, s2);
            seqSolShape1.Append(sol1);
            seqSolShape2.Append(sol2);
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between a vertex and an edge.</summary>
    private void Perform(in Vertex s1,
                         in Edge s2,
                         NCollection_Sequence<SolutionElem> seqSolShape1,
                         NCollection_Sequence<SolutionElem> seqSolShape2)
    {
        if (Tool.Degenerated(s2))
        {
            return;
        }

        var ext = new ExtPC(s1, s2);
        int nbExtrema = ext.IsDone ? ext.ExtCount : 0;
        if (nbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double dstmin = ext.SquareDistance(1);
            for (i = 2; i <= nbExtrema; i++)
            {
                double sDst = ext.SquareDistance(i);
                if (sDst < dstmin)
                {
                    dstmin = sDst;
                }
            }
            dstmin = Math.Sqrt(dstmin);
            if ((dstmin < _dstRef - _eps) || (Math.Abs(dstmin - _dstRef) < _eps))
            {
                Point3D pt, p1 = Tool.Pnt(s1);
                double epsP = global::Kerf.Engine.Foundation.Precision.PConfusion();

                for (i = 1; i <= nbExtrema; i++)
                {
                    if (Math.Abs(dstmin - Math.Sqrt(ext.SquareDistance(i))) < _eps)
                    {
                        pt = ext.Point(i);
                        if (TRI_SOLUTION(seqSolShape2, pt))
                        {
                            // Check if the parameter does not correspond to a vertex
                            double t = ext.Parameter(i);
                            if (!IsOnBoundary(s2, pt, t, epsP))
                            {
                                if (_dstRef > dstmin)
                                {
                                    _dstRef = dstmin;
                                }
                                _modif = true;
                                var sol1 = new SolutionElem(dstmin, p1, SupportType.IsVertex, s1);
                                var sol2 = new SolutionElem(dstmin, pt, SupportType.IsOnEdge, s2, t);
                                seqSolShape1.Append(sol1);
                                seqSolShape2.Append(sol2);
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between a vertex and a face.</summary>
    private void Perform(in Vertex s1,
                         in Face s2,
                         NCollection_Sequence<SolutionElem> seqSolShape1,
                         NCollection_Sequence<SolutionElem> seqSolShape2)
    {
        var ext = new ExtPF(s1, s2, _flag, _algo);
        int nbExtrema = ext.IsDone ? ext.ExtCount : 0;
        if (nbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double dstmin = ext.SquareDistance(1);
            for (i = 2; i <= nbExtrema; i++)
            {
                double sDst = ext.SquareDistance(i);
                if (sDst < dstmin)
                {
                    dstmin = sDst;
                }
            }
            dstmin = Math.Sqrt(dstmin);
            if ((dstmin < _dstRef - _eps) || (Math.Abs(dstmin - _dstRef) < _eps))
            {
                double u, v;
                Point3D pt, p1 = Tool.Pnt(s1);
                var classifier = new FaceClassifier();
                double tol = Tool.Tolerance(s2);

                for (i = 1; i <= nbExtrema; i++)
                {
                    if (Math.Abs(dstmin - Math.Sqrt(ext.SquareDistance(i))) < _eps)
                    {
                        pt = ext.Point(i);
                        if (TRI_SOLUTION(seqSolShape2, pt))
                        {
                            // Check if the parameter does not correspond to a vertex
                            ext.Parameter(i, out u, out v);
                            var puv = new Point2D(u, v);
                            classifier.Perform(s2, puv, tol);
                            if (classifier.State() == In)
                            {
                                if (_dstRef > dstmin)
                                {
                                    _dstRef = dstmin;
                                }
                                _modif = true;
                                var sol1 = new SolutionElem(dstmin, p1, SupportType.IsVertex, s1);
                                var sol2 = new SolutionElem(dstmin, pt, SupportType.IsInFace, s2, u, v);
                                seqSolShape1.Append(sol1);
                                seqSolShape2.Append(sol2);
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between two edges.</summary>
    private void Perform(in Edge s1,
                         in Edge s2,
                         NCollection_Sequence<SolutionElem> seqSolShape1,
                         NCollection_Sequence<SolutionElem> seqSolShape2)
    {
        if (Tool.Degenerated(s1) || Tool.Degenerated(s2))
        {
            return;
        }

        double dstRef = _dstRef;

        var ext = new DistanceExtCC(s1, s2);
        int nbExtrema = ext.IsDone ? (ext.IsParallel ? 0 : ext.ExtCount) : 0;
        if (nbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double dstmin = ext.SquareDistance(1);
            for (i = 2; i <= nbExtrema; i++)
            {
                double sDst = ext.SquareDistance(i);
                if (sDst < dstmin)
                {
                    dstmin = sDst;
                }
            }
            dstmin = Math.Sqrt(dstmin);
            if ((dstmin < _dstRef - _eps) || (Math.Abs(dstmin - _dstRef) < _eps))
            {
                Point3D pt1, pt2;
                double epsP = global::Kerf.Engine.Foundation.Precision.PConfusion();

                for (i = 1; i <= nbExtrema; i++)
                {
                    if (Math.Abs(dstmin - Math.Sqrt(ext.SquareDistance(i))) < _eps)
                    {
                        pt1 = ext.PointOnE1(i);
                        pt2 = ext.PointOnE2(i);
                        if (TRI_SOLUTION(seqSolShape1, pt1) || TRI_SOLUTION(seqSolShape2, pt2))
                        {
                            // Check if the parameters do not correspond to a vertex
                            double t1 = ext.ParameterOnE1(i);
                            double t2 = ext.ParameterOnE2(i);

                            if (!IsOnBoundary(s1, pt1, t1, epsP) && !IsOnBoundary(s2, pt2, t2, epsP))
                            {
                                if (_dstRef > dstmin)
                                {
                                    _dstRef = dstmin;
                                }
                                _modif = true;
                                var sol1 = new SolutionElem(dstmin, pt1, SupportType.IsOnEdge, s1, t1);
                                var sol2 = new SolutionElem(dstmin, pt2, SupportType.IsOnEdge, s2, t2);
                                seqSolShape1.Append(sol1);
                                seqSolShape2.Append(sol2);
                            }
                        }
                    }
                }
            }
        }

        var seqSolution1 = new NCollection_Sequence<SolutionElem>();
        var seqSolution2 = new NCollection_Sequence<SolutionElem>();

        PERFORM_C0(s1, s2, seqSolution1, seqSolution2, dstRef, ref _dstRef, _eps);

        var seqSol1 = new NCollection_Sequence<SolutionElem>();
        var seqSol2 = new NCollection_Sequence<SolutionElem>();

        if (seqSolution1.Length() > 0 && seqSolution2.Length() > 0)
        {
            MIN_SOLUTION(seqSolution1, seqSolution2, _dstRef, _eps, seqSol1, seqSol2);
        }

        if (!seqSol1.IsEmpty() && !seqSol2.IsEmpty())
        {
            for (int it = 1; it <= seqSol1.Length() && it <= seqSol2.Length(); it++)
            {
                Point3D pt1 = seqSol1.Value(it).Point();
                Point3D pt2 = seqSol2.Value(it).Point();
                if (TRI_SOLUTION(seqSolShape1, pt1) || TRI_SOLUTION(seqSolShape2, pt2))
                {
                    seqSolShape1.Append(seqSol1.Value(it));
                    seqSolShape2.Append(seqSol2.Value(it));
                    _modif = true;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between an edge and a face.</summary>
    private void Perform(in Edge s1,
                         in Face s2,
                         NCollection_Sequence<SolutionElem> seqSolShape1,
                         NCollection_Sequence<SolutionElem> seqSolShape2)
    {
        if (Tool.Degenerated(s1))
        {
            return;
        }

        var classifier = new FaceClassifier();

        var ext = new ExtCF(s1, s2);
        int nbExtrema = ext.IsDone ? (ext.IsParallel ? 0 : ext.ExtCount) : 0;
        if (nbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double dstmin = ext.SquareDistance(1);
            for (i = 2; i <= nbExtrema; i++)
            {
                double sDst = ext.SquareDistance(i);
                if (sDst < dstmin)
                {
                    dstmin = sDst;
                }
            }
            dstmin = Math.Sqrt(dstmin);
            if ((dstmin < _dstRef - _eps) || (Math.Abs(dstmin - _dstRef) < _eps))
            {
                double u, v;
                double tol = Tool.Tolerance(s2);

                Point3D pt1, pt2;
                double epsP = global::Kerf.Engine.Foundation.Precision.PConfusion();

                for (i = 1; i <= nbExtrema; i++)
                {
                    if (Math.Abs(dstmin - Math.Sqrt(ext.SquareDistance(i))) < _eps)
                    {
                        pt1 = ext.PointOnEdge(i);
                        pt2 = ext.PointOnFace(i);
                        if (TRI_SOLUTION(seqSolShape1, pt1) || TRI_SOLUTION(seqSolShape2, pt2))
                        {
                            // Check if the parameter does not correspond to a vertex
                            double t1 = ext.ParameterOnEdge(i);
                            if (!IsOnBoundary(s1, pt1, t1, epsP))
                            {
                                ext.ParameterOnFace(i, out u, out v);
                                var puv = new Point2D(u, v);
                                classifier.Perform(s2, puv, tol);
                                if (classifier.State() == In)
                                {
                                    if (_dstRef > dstmin)
                                    {
                                        _dstRef = dstmin;
                                    }
                                    _modif = true;
                                    var sol1 = new SolutionElem(dstmin, pt1, SupportType.IsOnEdge, s1, t1);
                                    var sol2 = new SolutionElem(dstmin, pt2, SupportType.IsInFace, s2, u, v);
                                    seqSolShape1.Append(sol1);
                                    seqSolShape2.Append(sol2);
                                }
                            }
                        }
                    }
                }
            }
        }

        double first, last;
        Curve3D? pCurv = Tool.Curve(s1, out first, out last);

        if (pCurv is null)
        {
            return;
        }

        if (pCurv.Continuity == C0)
        {
            var seqSolution1 = new NCollection_Sequence<SolutionElem>();
            var seqSolution2 = new NCollection_Sequence<SolutionElem>();

            var adaptorCurve = new CurveAdaptor(pCurv, first, last);
            int nbIntervals = adaptorCurve.NbIntervals(C1);

            var arrInter = new NCollection_Array1<double>(1, 1 + nbIntervals);
            adaptorCurve.Intervals(arrInter, C1);

            double u, v;
            double tol = Tool.Tolerance(s2);

            int i;
            for (i = 1; i <= arrInter.Length(); i++)
            {
                double parameter = arrInter[i];
                Point3D pnt = adaptorCurve.Value(parameter);
                // a local vertex, never handed out: not frozen
                Vertex v1 = new MakeVertex(pnt).InternalVertex();

                var extPF = new ExtPF(v1, s2);
                int nbExtremaPF = extPF.IsDone ? extPF.ExtCount : 0;
                if (nbExtremaPF > 0)
                {
                    // Search minimum distance Dstmin
                    int ii;
                    double dstmin = extPF.SquareDistance(1);
                    for (ii = 2; ii <= nbExtremaPF; ii++)
                    {
                        double sDst = extPF.SquareDistance(ii);
                        if (sDst < dstmin)
                        {
                            dstmin = sDst;
                        }
                    }
                    dstmin = Math.Sqrt(dstmin);

                    if ((dstmin < _dstRef - _eps) || (Math.Abs(dstmin - _dstRef) < _eps))
                    {
                        for (ii = 1; ii <= nbExtremaPF; ii++)
                        {
                            if (Math.Abs(dstmin - Math.Sqrt(extPF.SquareDistance(ii))) < _eps)
                            {
                                // Check if the parameter does not correspond to a vertex
                                extPF.Parameter(ii, out u, out v);
                                var puv = new Point2D(u, v);
                                classifier.Perform(s2, puv, tol);
                                if (classifier.State() == In)
                                {
                                    if (_dstRef > dstmin)
                                    {
                                        _dstRef = dstmin;
                                    }
                                    _modif = true;
                                    var sol1 = new SolutionElem(dstmin, pnt, SupportType.IsOnEdge, s1, parameter);
                                    var sol2 = new SolutionElem(dstmin, extPF.Point(ii), SupportType.IsInFace, s2, u, v);
                                    seqSolution1.Append(sol1);
                                    seqSolution2.Append(sol2);
                                }
                            }
                        }
                    }
                }
            }

            var seqSol1 = new NCollection_Sequence<SolutionElem>();
            var seqSol2 = new NCollection_Sequence<SolutionElem>();
            if (seqSolution1.Length() > 0 && seqSolution2.Length() > 0)
            {
                MIN_SOLUTION(seqSolution1, seqSolution2, _dstRef, _eps, seqSol1, seqSol2);
            }

            if (!seqSol1.IsEmpty() && !seqSol2.IsEmpty())
            {
                seqSolShape1.Append(seqSol1);
                seqSolShape2.Append(seqSol2);
            }
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between two faces.</summary>
    private void Perform(in Face s1,
                         in Face s2,
                         NCollection_Sequence<SolutionElem> seqSolShape1,
                         NCollection_Sequence<SolutionElem> seqSolShape2)
    {
        var ext = new ExtFF(s1, s2);
        int nbExtrema = ext.IsDone ? (ext.IsParallel ? 0 : ext.ExtCount) : 0;
        if (nbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double dstmin = ext.SquareDistance(1);
            for (i = 2; i <= nbExtrema; i++)
            {
                double sDst = ext.SquareDistance(i);
                if (sDst < dstmin)
                {
                    dstmin = sDst;
                }
            }
            dstmin = Math.Sqrt(dstmin);
            if ((dstmin < _dstRef - _eps) || (Math.Abs(dstmin - _dstRef) < _eps))
            {
                double tol1 = Tool.Tolerance(s1);
                double tol2 = Tool.Tolerance(s2);

                Point3D pt1, pt2;
                Point2D puv;
                double u1, v1, u2, v2;
                var classifier = new FaceClassifier();

                for (i = 1; i <= nbExtrema; i++)
                {
                    if (Math.Abs(dstmin - Math.Sqrt(ext.SquareDistance(i))) < _eps)
                    {
                        pt1 = ext.PointOnFace1(i);
                        pt2 = ext.PointOnFace2(i);
                        if (TRI_SOLUTION(seqSolShape1, pt1) || TRI_SOLUTION(seqSolShape2, pt2))
                        {
                            // Check if the parameter does not correspond to a vertex
                            ext.ParameterOnFace1(i, out u1, out v1);
                            puv = new Point2D(u1, v1);
                            classifier.Perform(s1, puv, tol1);
                            if (classifier.State() == In)
                            {
                                ext.ParameterOnFace2(i, out u2, out v2);
                                puv = new Point2D(u2, v2);
                                classifier.Perform(s2, puv, tol2);
                                if (classifier.State() == In)
                                {
                                    if (_dstRef > dstmin)
                                    {
                                        _dstRef = dstmin;
                                    }
                                    _modif = true;
                                    var sol1 = new SolutionElem(dstmin, pt1, SupportType.IsInFace, s1, u1, v1);
                                    var sol2 = new SolutionElem(dstmin, pt2, SupportType.IsInFace, s2, u2, v2);
                                    seqSolShape1.Append(sol1);
                                    seqSolShape2.Append(sol2);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
