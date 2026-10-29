// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_DistanceSS.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_DistanceSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.BRepExtrema_SupportType;
using static Occt.Extrema_ExtAlgo;
using static Occt.Extrema_ExtFlag;
using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The minimum distance between two elementary shapes (vertex, edge or face), used by
/// <see cref="BRepExtrema_DistShapeShape"/>.
/// </summary>
/// <remarks>
/// A working object computed in its constructor. The supports of its solutions are the input shapes, or, for an
/// infinite edge or face facing a finite one, a trimmed copy built here and frozen (it is handed out as a support).
/// </remarks>
public sealed class BRepExtrema_DistanceSS
{
    private readonly NCollection_Sequence<BRepExtrema_SolutionElem> mySeqSolShape1 = new(); //!< Solutions on the first shape
    private readonly NCollection_Sequence<BRepExtrema_SolutionElem> mySeqSolShape2 = new(); //!< Solutions on the second shape
    private double myDstRef; //!< The minimal distance found
    private bool myModif; //!< Flag indicating whether the solution was improved or not
    private readonly double myEps; //!< Deflection
    private readonly Extrema_ExtFlag myFlag; //!< Extrema flag indicating what solutions to look for
    private readonly Extrema_ExtAlgo myAlgo; //!< Extrema algo to be used to look for solutions

    /// <summary>
    /// Computes the distance between two shapes (face, edge or vertex), starting from the distance
    /// <paramref name="theDstRef"/>; solutions within <paramref name="theDeflection"/> of the minimum are kept. The
    /// flag and the algorithm apply only to point-face extrema.
    /// </summary>
    public BRepExtrema_DistanceSS(in TopoDS_Shape theS1,
                                  in TopoDS_Shape theS2,
                                  in Bnd_Box theBox1,
                                  in Bnd_Box theBox2,
                                  double theDstRef,
                                  double theDeflection = 1.0e-7, // Precision::Confusion()
                                  Extrema_ExtFlag theExtFlag = Extrema_ExtFlag_MINMAX,
                                  Extrema_ExtAlgo theExtAlgo = Extrema_ExtAlgo_Grad)
    {
        myDstRef = theDstRef;
        myModif = false;
        myEps = theDeflection;
        myFlag = theExtFlag;
        myAlgo = theExtAlgo;
        Perform(theS1, theS2, theBox1, theBox2);
    }

    /// <summary>True if the distance has been computed (improved), false otherwise.</summary>
    public bool IsDone() => myModif;

    /// <summary>The distance value.</summary>
    public double DistValue() => myDstRef;

    /// <summary>The solutions on the first shape (a copy).</summary>
    public IBentoList<BRepExtrema_SolutionElem> Seq1Value() => PackedList.Own([.. mySeqSolShape1]);

    /// <summary>The solutions on the second shape (a copy).</summary>
    public IBentoList<BRepExtrema_SolutionElem> Seq2Value() => PackedList.Own([.. mySeqSolShape2]);

    /// <summary>The solutions on the first shape, the object's own sequence (OCCT's const reference).</summary>
    internal NCollection_Sequence<BRepExtrema_SolutionElem> InternalSeq1Value() => mySeqSolShape1;

    /// <summary>The solutions on the second shape, the object's own sequence (OCCT's const reference).</summary>
    internal NCollection_Sequence<BRepExtrema_SolutionElem> InternalSeq2Value() => mySeqSolShape2;

    //=================================================================================================

    private static bool TRI_SOLUTION(NCollection_Sequence<BRepExtrema_SolutionElem> SeqSol, in gp_Pnt Pt)
    {
        foreach (BRepExtrema_SolutionElem anIt in SeqSol)
        {
            double dst = anIt.Point().Distance(Pt);
            if (dst <= Precision.Confusion())
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    private static void MIN_SOLUTION(NCollection_Sequence<BRepExtrema_SolutionElem> SeqSol1,
                                     NCollection_Sequence<BRepExtrema_SolutionElem> SeqSol2,
                                     double DstRef,
                                     double Eps,
                                     NCollection_Sequence<BRepExtrema_SolutionElem> seqSol1,
                                     NCollection_Sequence<BRepExtrema_SolutionElem> seqSol2)
    {
        for (int anIt = 1; anIt <= SeqSol1.Length(); anIt++)
        {
            double dst1 = SeqSol1.Value(anIt).Dist();
            if (Math.Abs(dst1 - DstRef) < Eps)
            {
                seqSol1.Append(SeqSol1.Value(anIt));
                seqSol2.Append(SeqSol2.Value(anIt));
            }
        }
    }

    //=================================================================================================

    private static void TRIM_INFINIT_EDGE(in TopoDS_Edge S1,
                                          in TopoDS_Edge S2,
                                          ref TopoDS_Edge aResEdge,
                                          ref bool bIsTrim1,
                                          ref bool bIsTrim2)
    {
        if (BRep_Tool.Degenerated(S1) || BRep_Tool.Degenerated(S2))
        {
            return;
        }

        aResEdge = S2;
        double aFirst1, aLast1, aFirst2, aLast2;
        Geom_Curve? pCurv1 = BRep_Tool.Curve(S1, out aFirst1, out aLast1);
        Geom_Curve? pCurv2 = BRep_Tool.Curve(S2, out aFirst2, out aLast2);

        if (Precision.IsInfinite(aFirst1) && Precision.IsInfinite(aLast1)
            && Precision.IsInfinite(aFirst2) && Precision.IsInfinite(aLast2))
        {
            return;
        }

        double Umin = 0.0, Umax = 0.0;
        bool bUmin, bUmax;
        bUmin = bUmax = false;

        Geom_Curve? pCurv = null;
        if (pCurv1 is not null && (Precision.IsInfinite(aFirst1) || Precision.IsInfinite(aLast1)))
        {
            pCurv = pCurv1;
            bIsTrim1 = true;
            if (!Precision.IsInfinite(aFirst1))
            {
                bUmin = true;
                Umin = aFirst1;
            }
            else if (!Precision.IsInfinite(aLast1))
            {
                bUmax = true;
                Umax = aLast1;
            }
        }
        else if (pCurv2 is not null && (Precision.IsInfinite(aFirst2) || Precision.IsInfinite(aLast2)))
        {
            pCurv = pCurv2;
            bIsTrim2 = true;
            if (!Precision.IsInfinite(aFirst2))
            {
                bUmin = true;
                Umin = aFirst2;
            }
            else if (!Precision.IsInfinite(aLast2))
            {
                bUmax = true;
                Umax = aLast2;
            }
        }
        if (bIsTrim1 || bIsTrim2)
        {
            var aEdgeBox = new Bnd_Box();
            if (bIsTrim1)
            {
                BRepBndLib.Add(S2, ref aEdgeBox);
            }
            if (bIsTrim2)
            {
                BRepBndLib.Add(S1, ref aEdgeBox);
            }
            aEdgeBox.Get(out double Xmin, out double Ymin, out double Zmin, out double Xmax, out double Ymax, out double Zmax);

            var aPnt0 = new gp_Pnt(Xmin, Ymin, Zmin);
            var aPnt1 = new gp_Pnt(Xmin, Ymax, Zmin);
            var aPnt2 = new gp_Pnt(Xmin, Ymax, Zmax);
            var aPnt3 = new gp_Pnt(Xmin, Ymin, Zmax);
            var aPnt4 = new gp_Pnt(Xmax, Ymax, Zmin);
            var aPnt5 = new gp_Pnt(Xmax, Ymax, Zmax);
            var aPnt6 = new gp_Pnt(Xmax, Ymin, Zmax);
            var aPnt7 = new gp_Pnt(Xmax, Ymin, Zmin);

            Span<double> arrU = stackalloc double[8];
            var aProj = new GeomAPI_ProjectPointOnCurve(aPnt0, pCurv!);
            /*szv:aProj.Perform(aPnt0);*/
            arrU[0] = aProj.LowerDistanceParameter();
            aProj.Perform(aPnt1);
            arrU[1] = aProj.LowerDistanceParameter();
            aProj.Perform(aPnt2);
            arrU[2] = aProj.LowerDistanceParameter();
            aProj.Perform(aPnt3);
            arrU[3] = aProj.LowerDistanceParameter();
            aProj.Perform(aPnt4);
            arrU[4] = aProj.LowerDistanceParameter();
            aProj.Perform(aPnt5);
            arrU[5] = aProj.LowerDistanceParameter();
            aProj.Perform(aPnt6);
            arrU[6] = aProj.LowerDistanceParameter();
            aProj.Perform(aPnt7);
            arrU[7] = aProj.LowerDistanceParameter();

            if (!bUmin)
            {
                Umin = arrU[0];
            }

            if (!bUmax)
            {
                Umax = arrU[0];
            }

            int i = 0;
            while (i < 8)
            {
                double aU = arrU[i++];
                if (aU < Umin)
                {
                    Umin = aU;
                }
                else if (aU > Umax)
                {
                    Umax = aU;
                }
            }

            double tol = Precision.Confusion();
            if (bIsTrim1)
            {
                tol = BRep_Tool.Tolerance(S1);
            }
            else if (bIsTrim2)
            {
                tol = BRep_Tool.Tolerance(S2);
            }

            double EpsU = new GeomAdaptor_Curve(pCurv!).Resolution(3.0 * tol);
            if (Math.Abs(Umin - Umax) < EpsU)
            {
                Umin -= EpsU;
                Umax += EpsU;
            }

            Geom_Curve result = new Geom_TrimmedCurve(pCurv!, Umin, Umax);
            // TODO(3K-merge): BRepBuilderAPI_MakeEdge
            aResEdge = new BRepLib_MakeEdge(result).Edge();
        }
    }

    //=================================================================================================

    private static void TRIM_INFINIT_FACE(in TopoDS_Shape S1,
                                          in TopoDS_Shape S2,
                                          ref TopoDS_Face aResFace,
                                          out bool bIsInfinit)
    {
        bIsInfinit = false;

        TopAbs_ShapeEnum Type1 = S1.ShapeType();
        TopAbs_ShapeEnum Type2 = S2.ShapeType();

        TopoDS_Edge aE;
        TopoDS_Face aF;

        if (Type1 == TopAbs_EDGE && Type2 == TopAbs_FACE)
        {
            aE = TopoDS.Edge(S1);
            if (BRep_Tool.Degenerated(aE))
            {
                return;
            }
            aF = TopoDS.Face(S2);
        }
        else if (Type2 == TopAbs_EDGE && Type1 == TopAbs_FACE)
        {
            aE = TopoDS.Edge(S2);
            if (BRep_Tool.Degenerated(aE))
            {
                return;
            }
            aF = TopoDS.Face(S1);
        }
        else
        {
            bIsInfinit = false;
            return;
        }

        aResFace = aF;
        Geom_Surface pSurf = BRep_Tool.Surface(aF)!;

        bool bRestrict = BRep_Tool.NaturalRestriction(aF);

        double U1, V1, U2, V2;
        double Umin = RealLast(), Umax = RealFirst(), Vmin = RealLast(), Vmax = RealFirst();
        bool bUmin, bUmax, bVmin, bVmax;
        bUmin = bUmax = bVmin = bVmax = false;
        bool bIsTrim = false;

        if (bRestrict)
        {
            pSurf.Bounds(out U1, out U2, out V1, out V2);
            if (Precision.IsInfinite(U1))
            {
                bIsTrim = true;
            }
            else
            {
                Umin = U1;
                bUmin = true;
            }

            if (Precision.IsInfinite(U2))
            {
                bIsTrim = true;
            }
            else
            {
                Umax = U2;
                bUmax = true;
            }

            if (Precision.IsInfinite(V1))
            {
                bIsTrim = true;
            }
            else
            {
                Vmin = V1;
                bVmin = true;
            }

            if (Precision.IsInfinite(V2))
            {
                bIsTrim = true;
            }
            else
            {
                Vmax = V2;
                bVmax = true;
            }
        }
        else
        {
            BRepTools.UVBounds(aF, out U1, out U2, out V1, out V2);
            if (Precision.IsInfinite(U1) && Precision.IsInfinite(U2) && Precision.IsInfinite(V1)
                && Precision.IsInfinite(V2))
            {
                bIsTrim = true;
            }
        }

        if (bIsTrim)
        {
            var aEdgeBox = new Bnd_Box();
            BRepBndLib.Add(aE, ref aEdgeBox);

            if (aEdgeBox.IsWhole())
            {
                return;
            }

            aEdgeBox.Get(out double Xmin, out double Ymin, out double Zmin, out double Xmax, out double Ymax, out double Zmax);

            var aPnt0 = new gp_Pnt(Xmin, Ymin, Zmin);
            var aPnt1 = new gp_Pnt(Xmin, Ymax, Zmin);
            var aPnt2 = new gp_Pnt(Xmin, Ymax, Zmax);
            var aPnt3 = new gp_Pnt(Xmin, Ymin, Zmax);
            var aPnt4 = new gp_Pnt(Xmax, Ymax, Zmin);
            var aPnt5 = new gp_Pnt(Xmax, Ymax, Zmax);
            var aPnt6 = new gp_Pnt(Xmax, Ymin, Zmax);
            var aPnt7 = new gp_Pnt(Xmax, Ymin, Zmin);

            // OCCT reads arrU and arrV uninitialised where a projection fails; the port's are zero (occt-issues.md).
            Span<double> arrU = stackalloc double[8];
            Span<double> arrV = stackalloc double[8];
            var aProj = new GeomAPI_ProjectPointOnSurf(aPnt0, pSurf);
            /*szv:aProj.Perform(aPnt0);*/
            if (aProj.IsDone())
            {
                aProj.LowerDistanceParameters(out arrU[0], out arrV[0]);
            }
            aProj.Perform(aPnt1);
            if (aProj.IsDone())
            {
                aProj.LowerDistanceParameters(out arrU[1], out arrV[1]);
            }
            aProj.Perform(aPnt2);
            if (aProj.IsDone())
            {
                aProj.LowerDistanceParameters(out arrU[2], out arrV[2]);
            }
            aProj.Perform(aPnt3);
            if (aProj.IsDone())
            {
                aProj.LowerDistanceParameters(out arrU[3], out arrV[3]);
            }
            aProj.Perform(aPnt4);
            if (aProj.IsDone())
            {
                aProj.LowerDistanceParameters(out arrU[4], out arrV[4]);
            }
            aProj.Perform(aPnt5);
            if (aProj.IsDone())
            {
                aProj.LowerDistanceParameters(out arrU[5], out arrV[5]);
            }
            aProj.Perform(aPnt6);
            if (aProj.IsDone())
            {
                aProj.LowerDistanceParameters(out arrU[6], out arrV[6]);
            }
            aProj.Perform(aPnt7);
            if (aProj.IsDone())
            {
                aProj.LowerDistanceParameters(out arrU[7], out arrV[7]);
            }

            if (!bUmin)
            {
                Umin = arrU[0];
            }
            if (!bUmax)
            {
                Umax = arrU[0];
            }
            if (!bVmin)
            {
                Vmin = arrV[0];
            }
            if (!bVmax)
            {
                Vmax = arrV[0];
            }

            int i = 0;
            while (i < 8)
            {
                double aU = arrU[i];
                if (aU < Umin)
                {
                    Umin = aU;
                }
                else if (aU > Umax)
                {
                    Umax = aU;
                }

                double aV = arrV[i];
                if (aV < Vmin)
                {
                    Vmin = aV;
                }
                else if (aV > Vmax)
                {
                    Vmax = aV;
                }

                i++;
            }

            var aGAS = new GeomAdaptor_Surface(pSurf);
            double tol = BRep_Tool.Tolerance(aF);

            double EpsU = aGAS.UResolution(3.0 * tol);
            if (Math.Abs(Umin - Umax) < EpsU)
            {
                Umin -= EpsU;
                Umax += EpsU;
            }

            double EpsV = aGAS.VResolution(3.0 * tol);
            if (Math.Abs(Vmin - Vmax) < EpsV)
            {
                Vmin -= EpsV;
                Vmax += EpsV;
            }

            Geom_Surface result = new Geom_RectangularTrimmedSurface(pSurf, Umin, Umax, Vmin, Vmax);
            // TODO(3K-merge): BRepBuilderAPI_MakeFace
            aResFace = new BRepLib_MakeFace(result, Precision.Confusion()).Face();

            bIsInfinit = true;
        }
        else
        {
            bIsInfinit = false;
        }
    }

    //=================================================================================================

    private static void PERFORM_C0(in TopoDS_Edge S1,
                                   in TopoDS_Edge S2,
                                   NCollection_Sequence<BRepExtrema_SolutionElem> SeqSol1,
                                   NCollection_Sequence<BRepExtrema_SolutionElem> SeqSol2,
                                   double DstRef,
                                   ref double mDstRef,
                                   double Eps)
    {
        if (BRep_Tool.Degenerated(S1) || BRep_Tool.Degenerated(S2))
        {
            return;
        }

        int iE;
        for (iE = 0; iE < 2; iE++)
        {
            TopoDS_Edge E, Eother;
            if (iE == 0)
            {
                E = S1;
                Eother = S2;
            }
            else
            {
                E = S2;
                Eother = S1;
            }

            double aFirst, aLast;
            Geom_Curve? pCurv = BRep_Tool.Curve(E, out aFirst, out aLast);

            double aFOther, aLOther;
            Geom_Curve? pCurvOther = BRep_Tool.Curve(Eother, out aFOther, out aLOther);

            if (pCurv is null)
            {
                return;
            }

            if (pCurv.Continuity() == GeomAbs_C0)
            {
                double epsP = Precision.PConfusion();

                var aAdaptorCurve = new GeomAdaptor_Curve(pCurv, aFirst, aLast);
                int nbIntervals = aAdaptorCurve.NbIntervals(GeomAbs_C1);

                var arrInter = new NCollection_Array1<double>(1, 1 + nbIntervals);
                aAdaptorCurve.Intervals(arrInter, GeomAbs_C1);

                var aAdaptorCurveOther = new GeomAdaptor_Curve(pCurvOther!, aFOther, aLOther);
                int nbIntervalsOther = aAdaptorCurveOther.NbIntervals(GeomAbs_C1);
                var arrInterOther = new NCollection_Array1<double>(1, 1 + nbIntervalsOther);
                aAdaptorCurveOther.Intervals(arrInterOther, GeomAbs_C1);

                double Udeb, Ufin;
                BRep_Tool.Range(Eother, out Udeb, out Ufin);

                gp_Pnt Pt;
                int i, ii;
                for (i = 1; i <= arrInter.Length(); i++)
                {
                    double aParameter = arrInter[i];
                    gp_Pnt aPnt = aAdaptorCurve.Value(aParameter);
                    // TODO(3K-merge): BRepBuilderAPI_MakeVertex (a local vertex, never handed out)
                    TopoDS_Vertex V1 = new BRepLib_MakeVertex(aPnt).Vertex();

                    var Ext = new BRepExtrema_ExtPC(V1, Eother);
                    int NbExtrema = Ext.IsDone() ? Ext.NbExt() : 0;
                    if (NbExtrema > 0)
                    {
                        // Search minimum distance dstmin
                        double Dstmin = Ext.SquareDistance(1);
                        for (ii = 2; ii <= NbExtrema; ii++)
                        {
                            double sDst = Ext.SquareDistance(ii);
                            if (sDst < Dstmin)
                            {
                                Dstmin = sDst;
                            }
                        }
                        Dstmin = Math.Sqrt(Dstmin);

                        if ((Dstmin < DstRef - Eps) || (Math.Abs(Dstmin - DstRef) < Eps))
                        {
                            for (ii = 1; ii <= NbExtrema; ii++)
                            {
                                if (Math.Abs(Dstmin - Math.Sqrt(Ext.SquareDistance(ii))) < Eps)
                                {
                                    Pt = Ext.Point(ii);
                                    // Pt - point on the curve pCurvOther/Eother, but
                                    // if iE == 0 -> Eother correspond to edge S2
                                    // and to edge S1 in the opposite case.
                                    // Therefore we should search Pt through previous solution points on Other curve
                                    // (edge): if iE == 0 - on edge S2, namely through SeqSol2, else       - on edge S1,
                                    // namely through SeqSol1.
                                    bool triSolutionResult =
                                        (iE == 0) ? TRI_SOLUTION(SeqSol2, Pt) : TRI_SOLUTION(SeqSol1, Pt);
                                    if (triSolutionResult)
                                    {
                                        // Check if the parameter does not correspond to a vertex
                                        double t = Ext.Parameter(ii);
                                        if ((Math.Abs(t - Udeb) >= epsP) && (Math.Abs(t - Ufin) > epsP))
                                        {
                                            if (mDstRef > Dstmin)
                                            {
                                                mDstRef = Dstmin;
                                            }
                                            var Sol1 = new BRepExtrema_SolutionElem(Dstmin, aPnt, BRepExtrema_IsOnEdge, E, aParameter);
                                            var Sol2 = new BRepExtrema_SolutionElem(Dstmin, Pt, BRepExtrema_IsOnEdge, Eother, t);
                                            SeqSol1.Append(iE == 0 ? Sol1 : Sol2);
                                            SeqSol2.Append(iE == 0 ? Sol2 : Sol1);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    for (int i2 = 1; i2 <= arrInterOther.Length(); i2++)
                    {
                        double aParameterOther = arrInterOther[i2];
                        gp_Pnt aPntOther = aAdaptorCurveOther.Value(aParameterOther);
                        double Dst = aPnt.Distance(aPntOther);
                        if ((Dst < DstRef - Eps) || (Math.Abs(Dst - DstRef) < Eps))
                        {
                            if (mDstRef > Dst)
                            {
                                mDstRef = Dst;
                            }
                            var Sol1 = new BRepExtrema_SolutionElem(Dst, aPnt, BRepExtrema_IsOnEdge, E, aParameter);
                            var Sol2 = new BRepExtrema_SolutionElem(Dst, aPntOther, BRepExtrema_IsOnEdge, Eother, aParameterOther);
                            SeqSol1.Append(iE == 0 ? Sol1 : Sol2);
                            SeqSol2.Append(iE == 0 ? Sol2 : Sol1);
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
    private static bool isOnBoundary(in TopoDS_Edge theEdge, in gp_Pnt theSol, double theParam, double thePTol)
    {
        for (var it = new TopoDS_Iterator(theEdge); it.More(); it.Next())
        {
            TopoDS_Vertex aV = TopoDS.Vertex(it.Value());
            double aVParam = BRep_Tool.Parameter(aV, theEdge);
            if (Math.Abs(aVParam - theParam) < thePTol
                && BRep_Tool.Pnt(aV).Distance(theSol) < BRep_Tool.Tolerance(aV))
            {
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    private void Perform(in TopoDS_Shape theS1, in TopoDS_Shape theS2, in Bnd_Box theBox1, in Bnd_Box theBox2)
    {
        mySeqSolShape1.Clear();
        mySeqSolShape2.Clear();
        myModif = false;

        double aBBDist = theBox1.Distance(theBox2);
        if (aBBDist - myDstRef > myEps)
        {
            // The Box-Box distance is greater than the start distance.
            // The solution cannot be improved further.
            return;
        }

        switch (theS1.ShapeType())
        {
            case TopAbs_VERTEX:
            {
                TopoDS_Vertex aV1 = TopoDS.Vertex(theS1);
                switch (theS2.ShapeType())
                {
                    case TopAbs_VERTEX:
                    {
                        TopoDS_Vertex aV2 = TopoDS.Vertex(theS2);
                        Perform(aV1, aV2, mySeqSolShape1, mySeqSolShape2);
                        break;
                    }
                    case TopAbs_EDGE:
                    {
                        TopoDS_Edge aE2 = TopoDS.Edge(theS2);
                        Perform(aV1, aE2, mySeqSolShape1, mySeqSolShape2);
                        break;
                    }
                    case TopAbs_FACE:
                    {
                        TopoDS_Face aF2 = TopoDS.Face(theS2);
                        Perform(aV1, aF2, mySeqSolShape1, mySeqSolShape2);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case TopAbs_EDGE:
            {
                TopoDS_Edge aE1 = TopoDS.Edge(theS1);
                switch (theS2.ShapeType())
                {
                    case TopAbs_VERTEX:
                    {
                        TopoDS_Vertex aV2 = TopoDS.Vertex(theS2);
                        Perform(aV2, aE1, mySeqSolShape2, mySeqSolShape1);
                        break;
                    }
                    case TopAbs_EDGE:
                    {
                        TopoDS_Edge aE2 = TopoDS.Edge(theS2);
                        var aTrimEdge = new TopoDS_Edge();
                        bool bIsTrim1 = false;
                        bool bIsTrim2 = false;
                        TRIM_INFINIT_EDGE(aE1, aE2, ref aTrimEdge, ref bIsTrim1, ref bIsTrim2);
                        if (bIsTrim1)
                        {
                            aE1 = aTrimEdge;
                        }
                        if (bIsTrim2)
                        {
                            aE2 = aTrimEdge;
                        }
                        Perform(aE1, aE2, mySeqSolShape1, mySeqSolShape2);
                        break;
                    }
                    case TopAbs_FACE:
                    {
                        TopoDS_Face aF2 = TopoDS.Face(theS2);
                        var aTrimFace = new TopoDS_Face();
                        TRIM_INFINIT_FACE(aE1, aF2, ref aTrimFace, out bool bIsInfinit);
                        if (bIsInfinit)
                        {
                            aF2 = aTrimFace;
                        }
                        Perform(aE1, aF2, mySeqSolShape1, mySeqSolShape2);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case TopAbs_FACE:
            {
                TopoDS_Face aF1 = TopoDS.Face(theS1);
                switch (theS2.ShapeType())
                {
                    case TopAbs_VERTEX:
                    {
                        TopoDS_Vertex aV2 = TopoDS.Vertex(theS2);
                        Perform(aV2, aF1, mySeqSolShape2, mySeqSolShape1);
                        break;
                    }
                    case TopAbs_EDGE:
                    {
                        TopoDS_Edge aE2 = TopoDS.Edge(theS2);
                        var aTrimFace = new TopoDS_Face();
                        TRIM_INFINIT_FACE(aF1, aE2, ref aTrimFace, out bool bIsInfinit);
                        if (bIsInfinit)
                        {
                            aF1 = aTrimFace;
                        }
                        Perform(aE2, aF1, mySeqSolShape2, mySeqSolShape1);
                        break;
                    }
                    case TopAbs_FACE:
                    {
                        TopoDS_Face aF2 = TopoDS.Face(theS2);
                        Perform(aF1, aF2, mySeqSolShape1, mySeqSolShape2);
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
    private void Perform(in TopoDS_Vertex theS1,
                         in TopoDS_Vertex theS2,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape1,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape2)
    {
        gp_Pnt aP1 = BRep_Tool.Pnt(theS1);
        gp_Pnt aP2 = BRep_Tool.Pnt(theS2);

        double Dst = aP1.Distance(aP2);
        if ((Dst < myDstRef - myEps) || (Math.Abs(Dst - myDstRef) < myEps))
        {
            if (myDstRef > Dst)
            {
                myDstRef = Dst;
            }
            myModif = true;
            var Sol1 = new BRepExtrema_SolutionElem(Dst, aP1, BRepExtrema_IsVertex, theS1);
            var Sol2 = new BRepExtrema_SolutionElem(Dst, aP2, BRepExtrema_IsVertex, theS2);
            theSeqSolShape1.Append(Sol1);
            theSeqSolShape2.Append(Sol2);
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between a vertex and an edge.</summary>
    private void Perform(in TopoDS_Vertex theS1,
                         in TopoDS_Edge theS2,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape1,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape2)
    {
        if (BRep_Tool.Degenerated(theS2))
        {
            return;
        }

        var Ext = new BRepExtrema_ExtPC(theS1, theS2);
        int NbExtrema = Ext.IsDone() ? Ext.NbExt() : 0;
        if (NbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double Dstmin = Ext.SquareDistance(1);
            for (i = 2; i <= NbExtrema; i++)
            {
                double sDst = Ext.SquareDistance(i);
                if (sDst < Dstmin)
                {
                    Dstmin = sDst;
                }
            }
            Dstmin = Math.Sqrt(Dstmin);
            if ((Dstmin < myDstRef - myEps) || (Math.Abs(Dstmin - myDstRef) < myEps))
            {
                gp_Pnt Pt, P1 = BRep_Tool.Pnt(theS1);
                double epsP = Precision.PConfusion();

                for (i = 1; i <= NbExtrema; i++)
                {
                    if (Math.Abs(Dstmin - Math.Sqrt(Ext.SquareDistance(i))) < myEps)
                    {
                        Pt = Ext.Point(i);
                        if (TRI_SOLUTION(theSeqSolShape2, Pt))
                        {
                            // Check if the parameter does not correspond to a vertex
                            double t = Ext.Parameter(i);
                            if (!isOnBoundary(theS2, Pt, t, epsP))
                            {
                                if (myDstRef > Dstmin)
                                {
                                    myDstRef = Dstmin;
                                }
                                myModif = true;
                                var Sol1 = new BRepExtrema_SolutionElem(Dstmin, P1, BRepExtrema_IsVertex, theS1);
                                var Sol2 = new BRepExtrema_SolutionElem(Dstmin, Pt, BRepExtrema_IsOnEdge, theS2, t);
                                theSeqSolShape1.Append(Sol1);
                                theSeqSolShape2.Append(Sol2);
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between a vertex and a face.</summary>
    private void Perform(in TopoDS_Vertex theS1,
                         in TopoDS_Face theS2,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape1,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape2)
    {
        var Ext = new BRepExtrema_ExtPF(theS1, theS2, myFlag, myAlgo);
        int NbExtrema = Ext.IsDone() ? Ext.NbExt() : 0;
        if (NbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double Dstmin = Ext.SquareDistance(1);
            for (i = 2; i <= NbExtrema; i++)
            {
                double sDst = Ext.SquareDistance(i);
                if (sDst < Dstmin)
                {
                    Dstmin = sDst;
                }
            }
            Dstmin = Math.Sqrt(Dstmin);
            if ((Dstmin < myDstRef - myEps) || (Math.Abs(Dstmin - myDstRef) < myEps))
            {
                double U, V;
                gp_Pnt Pt, P1 = BRep_Tool.Pnt(theS1);
                var classifier = new BRepClass_FaceClassifier();
                double tol = BRep_Tool.Tolerance(theS2);

                for (i = 1; i <= NbExtrema; i++)
                {
                    if (Math.Abs(Dstmin - Math.Sqrt(Ext.SquareDistance(i))) < myEps)
                    {
                        Pt = Ext.Point(i);
                        if (TRI_SOLUTION(theSeqSolShape2, Pt))
                        {
                            // Check if the parameter does not correspond to a vertex
                            Ext.Parameter(i, out U, out V);
                            var PUV = new gp_Pnt2d(U, V);
                            classifier.Perform(theS2, PUV, tol);
                            if (classifier.State() == TopAbs_IN)
                            {
                                if (myDstRef > Dstmin)
                                {
                                    myDstRef = Dstmin;
                                }
                                myModif = true;
                                var Sol1 = new BRepExtrema_SolutionElem(Dstmin, P1, BRepExtrema_IsVertex, theS1);
                                var Sol2 = new BRepExtrema_SolutionElem(Dstmin, Pt, BRepExtrema_IsInFace, theS2, U, V);
                                theSeqSolShape1.Append(Sol1);
                                theSeqSolShape2.Append(Sol2);
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between two edges.</summary>
    private void Perform(in TopoDS_Edge theS1,
                         in TopoDS_Edge theS2,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape1,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape2)
    {
        if (BRep_Tool.Degenerated(theS1) || BRep_Tool.Degenerated(theS2))
        {
            return;
        }

        double DstRef = myDstRef;

        var Ext = new BRepExtrema_ExtCC(theS1, theS2);
        int NbExtrema = Ext.IsDone() ? (Ext.IsParallel() ? 0 : Ext.NbExt()) : 0;
        if (NbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double Dstmin = Ext.SquareDistance(1);
            for (i = 2; i <= NbExtrema; i++)
            {
                double sDst = Ext.SquareDistance(i);
                if (sDst < Dstmin)
                {
                    Dstmin = sDst;
                }
            }
            Dstmin = Math.Sqrt(Dstmin);
            if ((Dstmin < myDstRef - myEps) || (Math.Abs(Dstmin - myDstRef) < myEps))
            {
                gp_Pnt Pt1, Pt2;
                double epsP = Precision.PConfusion();

                for (i = 1; i <= NbExtrema; i++)
                {
                    if (Math.Abs(Dstmin - Math.Sqrt(Ext.SquareDistance(i))) < myEps)
                    {
                        Pt1 = Ext.PointOnE1(i);
                        Pt2 = Ext.PointOnE2(i);
                        if (TRI_SOLUTION(theSeqSolShape1, Pt1) || TRI_SOLUTION(theSeqSolShape2, Pt2))
                        {
                            // Check if the parameters do not correspond to a vertex
                            double t1 = Ext.ParameterOnE1(i);
                            double t2 = Ext.ParameterOnE2(i);

                            if (!isOnBoundary(theS1, Pt1, t1, epsP) && !isOnBoundary(theS2, Pt2, t2, epsP))
                            {
                                if (myDstRef > Dstmin)
                                {
                                    myDstRef = Dstmin;
                                }
                                myModif = true;
                                var Sol1 = new BRepExtrema_SolutionElem(Dstmin, Pt1, BRepExtrema_IsOnEdge, theS1, t1);
                                var Sol2 = new BRepExtrema_SolutionElem(Dstmin, Pt2, BRepExtrema_IsOnEdge, theS2, t2);
                                theSeqSolShape1.Append(Sol1);
                                theSeqSolShape2.Append(Sol2);
                            }
                        }
                    }
                }
            }
        }

        var SeqSolution1 = new NCollection_Sequence<BRepExtrema_SolutionElem>();
        var SeqSolution2 = new NCollection_Sequence<BRepExtrema_SolutionElem>();

        PERFORM_C0(theS1, theS2, SeqSolution1, SeqSolution2, DstRef, ref myDstRef, myEps);

        var seqSol1 = new NCollection_Sequence<BRepExtrema_SolutionElem>();
        var seqSol2 = new NCollection_Sequence<BRepExtrema_SolutionElem>();

        if (SeqSolution1.Length() > 0 && SeqSolution2.Length() > 0)
        {
            MIN_SOLUTION(SeqSolution1, SeqSolution2, myDstRef, myEps, seqSol1, seqSol2);
        }

        if (!seqSol1.IsEmpty() && !seqSol2.IsEmpty())
        {
            for (int anIt = 1; anIt <= seqSol1.Length() && anIt <= seqSol2.Length(); anIt++)
            {
                gp_Pnt Pt1 = seqSol1.Value(anIt).Point();
                gp_Pnt Pt2 = seqSol2.Value(anIt).Point();
                if (TRI_SOLUTION(theSeqSolShape1, Pt1) || TRI_SOLUTION(theSeqSolShape2, Pt2))
                {
                    theSeqSolShape1.Append(seqSol1.Value(anIt));
                    theSeqSolShape2.Append(seqSol2.Value(anIt));
                    myModif = true;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between an edge and a face.</summary>
    private void Perform(in TopoDS_Edge theS1,
                         in TopoDS_Face theS2,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape1,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape2)
    {
        if (BRep_Tool.Degenerated(theS1))
        {
            return;
        }

        var classifier = new BRepClass_FaceClassifier();

        var Ext = new BRepExtrema_ExtCF(theS1, theS2);
        int NbExtrema = Ext.IsDone() ? (Ext.IsParallel() ? 0 : Ext.NbExt()) : 0;
        if (NbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double Dstmin = Ext.SquareDistance(1);
            for (i = 2; i <= NbExtrema; i++)
            {
                double sDst = Ext.SquareDistance(i);
                if (sDst < Dstmin)
                {
                    Dstmin = sDst;
                }
            }
            Dstmin = Math.Sqrt(Dstmin);
            if ((Dstmin < myDstRef - myEps) || (Math.Abs(Dstmin - myDstRef) < myEps))
            {
                double U, V;
                double tol = BRep_Tool.Tolerance(theS2);

                gp_Pnt Pt1, Pt2;
                double epsP = Precision.PConfusion();

                for (i = 1; i <= NbExtrema; i++)
                {
                    if (Math.Abs(Dstmin - Math.Sqrt(Ext.SquareDistance(i))) < myEps)
                    {
                        Pt1 = Ext.PointOnEdge(i);
                        Pt2 = Ext.PointOnFace(i);
                        if (TRI_SOLUTION(theSeqSolShape1, Pt1) || TRI_SOLUTION(theSeqSolShape2, Pt2))
                        {
                            // Check if the parameter does not correspond to a vertex
                            double t1 = Ext.ParameterOnEdge(i);
                            if (!isOnBoundary(theS1, Pt1, t1, epsP))
                            {
                                Ext.ParameterOnFace(i, out U, out V);
                                var PUV = new gp_Pnt2d(U, V);
                                classifier.Perform(theS2, PUV, tol);
                                if (classifier.State() == TopAbs_IN)
                                {
                                    if (myDstRef > Dstmin)
                                    {
                                        myDstRef = Dstmin;
                                    }
                                    myModif = true;
                                    var Sol1 = new BRepExtrema_SolutionElem(Dstmin, Pt1, BRepExtrema_IsOnEdge, theS1, t1);
                                    var Sol2 = new BRepExtrema_SolutionElem(Dstmin, Pt2, BRepExtrema_IsInFace, theS2, U, V);
                                    theSeqSolShape1.Append(Sol1);
                                    theSeqSolShape2.Append(Sol2);
                                }
                            }
                        }
                    }
                }
            }
        }

        double aFirst, aLast;
        Geom_Curve? pCurv = BRep_Tool.Curve(theS1, out aFirst, out aLast);

        if (pCurv is null)
        {
            return;
        }

        if (pCurv.Continuity() == GeomAbs_C0)
        {
            var SeqSolution1 = new NCollection_Sequence<BRepExtrema_SolutionElem>();
            var SeqSolution2 = new NCollection_Sequence<BRepExtrema_SolutionElem>();

            var aAdaptorCurve = new GeomAdaptor_Curve(pCurv, aFirst, aLast);
            int nbIntervals = aAdaptorCurve.NbIntervals(GeomAbs_C1);

            var arrInter = new NCollection_Array1<double>(1, 1 + nbIntervals);
            aAdaptorCurve.Intervals(arrInter, GeomAbs_C1);

            double U, V;
            double tol = BRep_Tool.Tolerance(theS2);

            int i;
            for (i = 1; i <= arrInter.Length(); i++)
            {
                double aParameter = arrInter[i];
                gp_Pnt aPnt = aAdaptorCurve.Value(aParameter);
                // TODO(3K-merge): BRepBuilderAPI_MakeVertex (a local vertex, never handed out)
                TopoDS_Vertex V1 = new BRepLib_MakeVertex(aPnt).Vertex();

                var ExtPF = new BRepExtrema_ExtPF(V1, theS2);
                int NbExtremaPF = ExtPF.IsDone() ? ExtPF.NbExt() : 0;
                if (NbExtremaPF > 0)
                {
                    // Search minimum distance Dstmin
                    int ii;
                    double Dstmin = ExtPF.SquareDistance(1);
                    for (ii = 2; ii <= NbExtremaPF; ii++)
                    {
                        double sDst = ExtPF.SquareDistance(ii);
                        if (sDst < Dstmin)
                        {
                            Dstmin = sDst;
                        }
                    }
                    Dstmin = Math.Sqrt(Dstmin);

                    if ((Dstmin < myDstRef - myEps) || (Math.Abs(Dstmin - myDstRef) < myEps))
                    {
                        for (ii = 1; ii <= NbExtremaPF; ii++)
                        {
                            if (Math.Abs(Dstmin - Math.Sqrt(ExtPF.SquareDistance(ii))) < myEps)
                            {
                                // Check if the parameter does not correspond to a vertex
                                ExtPF.Parameter(ii, out U, out V);
                                var PUV = new gp_Pnt2d(U, V);
                                classifier.Perform(theS2, PUV, tol);
                                if (classifier.State() == TopAbs_IN)
                                {
                                    if (myDstRef > Dstmin)
                                    {
                                        myDstRef = Dstmin;
                                    }
                                    myModif = true;
                                    var Sol1 = new BRepExtrema_SolutionElem(Dstmin, aPnt, BRepExtrema_IsOnEdge, theS1, aParameter);
                                    var Sol2 = new BRepExtrema_SolutionElem(Dstmin, ExtPF.Point(ii), BRepExtrema_IsInFace, theS2, U, V);
                                    SeqSolution1.Append(Sol1);
                                    SeqSolution2.Append(Sol2);
                                }
                            }
                        }
                    }
                }
            }

            var seqSol1 = new NCollection_Sequence<BRepExtrema_SolutionElem>();
            var seqSol2 = new NCollection_Sequence<BRepExtrema_SolutionElem>();
            if (SeqSolution1.Length() > 0 && SeqSolution2.Length() > 0)
            {
                MIN_SOLUTION(SeqSolution1, SeqSolution2, myDstRef, myEps, seqSol1, seqSol2);
            }

            if (!seqSol1.IsEmpty() && !seqSol2.IsEmpty())
            {
                theSeqSolShape1.Append(seqSol1);
                theSeqSolShape2.Append(seqSol2);
            }
        }
    }

    //=================================================================================================

    /// <summary>The minimum distance between two faces.</summary>
    private void Perform(in TopoDS_Face theS1,
                         in TopoDS_Face theS2,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape1,
                         NCollection_Sequence<BRepExtrema_SolutionElem> theSeqSolShape2)
    {
        var Ext = new BRepExtrema_ExtFF(theS1, theS2);
        int NbExtrema = Ext.IsDone() ? (Ext.IsParallel() ? 0 : Ext.NbExt()) : 0;
        if (NbExtrema > 0)
        {
            // Search minimum distance Dstmin
            int i;
            double Dstmin = Ext.SquareDistance(1);
            for (i = 2; i <= NbExtrema; i++)
            {
                double sDst = Ext.SquareDistance(i);
                if (sDst < Dstmin)
                {
                    Dstmin = sDst;
                }
            }
            Dstmin = Math.Sqrt(Dstmin);
            if ((Dstmin < myDstRef - myEps) || (Math.Abs(Dstmin - myDstRef) < myEps))
            {
                double tol1 = BRep_Tool.Tolerance(theS1);
                double tol2 = BRep_Tool.Tolerance(theS2);

                gp_Pnt Pt1, Pt2;
                gp_Pnt2d PUV;
                double U1, V1, U2, V2;
                var classifier = new BRepClass_FaceClassifier();

                for (i = 1; i <= NbExtrema; i++)
                {
                    if (Math.Abs(Dstmin - Math.Sqrt(Ext.SquareDistance(i))) < myEps)
                    {
                        Pt1 = Ext.PointOnFace1(i);
                        Pt2 = Ext.PointOnFace2(i);
                        if (TRI_SOLUTION(theSeqSolShape1, Pt1) || TRI_SOLUTION(theSeqSolShape2, Pt2))
                        {
                            // Check if the parameter does not correspond to a vertex
                            Ext.ParameterOnFace1(i, out U1, out V1);
                            PUV = new gp_Pnt2d(U1, V1);
                            classifier.Perform(theS1, PUV, tol1);
                            if (classifier.State() == TopAbs_IN)
                            {
                                Ext.ParameterOnFace2(i, out U2, out V2);
                                PUV = new gp_Pnt2d(U2, V2);
                                classifier.Perform(theS2, PUV, tol2);
                                if (classifier.State() == TopAbs_IN)
                                {
                                    if (myDstRef > Dstmin)
                                    {
                                        myDstRef = Dstmin;
                                    }
                                    myModif = true;
                                    var Sol1 = new BRepExtrema_SolutionElem(Dstmin, Pt1, BRepExtrema_IsInFace, theS1, U1, V1);
                                    var Sol2 = new BRepExtrema_SolutionElem(Dstmin, Pt2, BRepExtrema_IsInFace, theS2, U2, V2);
                                    theSeqSolShape1.Append(Sol1);
                                    theSeqSolShape2.Append(Sol2);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
