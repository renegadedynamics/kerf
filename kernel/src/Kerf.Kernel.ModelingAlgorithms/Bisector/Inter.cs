// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Inter.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Inter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>
/// The intersection of two bisectors (Bisector_Bisec) on their domains. Each bisector is cut into its intervals of
/// continuity, extensions intersected as segments; two bisectors of curves that share an element (ComunElement) are
/// intersected by a root of the difference of their distances to it.
/// </summary>
/// <remarks>
/// A working object computed by its constructor or Perform. OCCT's bugs in Perform are kept (occt-issues.md):
/// the pieces are stored at their interval's index but read from 1 to their count, and the second bisector's last
/// interval is tested against the first's interval count.
/// </remarks>
public sealed class Inter : Intersection2D
{
    /// <summary>An empty intersection.</summary>
    public Inter()
    {
    }

    /// <summary>
    /// The intersection of C1 (which separates the elements A and B) on D1 and C2 (which separates C and D) on D2.
    /// ComunElement must be true if B and C are the same element: it allows a faster computation.
    /// </summary>
    public Inter(Bisec c1, in TwoDDomain d1, Bisec c2, in TwoDDomain d2,
                          double tolConf, double tol, bool comunElement)
    {
        Perform(c1, d1, c2, d2, tolConf, tol, comunElement);
    }

    //===================================================================================
    // Construction of a segment of line from PMin to PMax, parameterised from UMin.
    //===================================================================================
    private static LineCurve2D ConstructSegment(in Point2D pMin, in Point2D pMax, double uMin, double uMax)
    {
        Direction2D dir = new(pMax.X() - pMin.X(), pMax.Y() - pMin.Y());
        LineCurve2D l = new(new Point2D(pMin.X() - uMin * dir.X(), pMin.Y() - uMin * dir.Y()), dir);
        return l;
    }

    /// <summary>
    /// Computes the intersection of C1 (which separates the elements A and B) on D1 and C2 (which separates C and D)
    /// on D2. ComunElement must be true if B and C are the same element.
    /// </summary>
    public void Perform(Bisec c1, in TwoDDomain d1, Bisec c2, in TwoDDomain d2,
                        double tolConf, double tol, bool comunElement)
    {
        // OCCT.NET: the bisectors are read without freezing them (MAT2d re-trims them in place afterwards).
        BisectorCurve bis1 = (BisectorCurve)c1.Value().ChangeBasisCurve();
        BisectorCurve bis2 = (BisectorCurve)c2.Value().ChangeBasisCurve();

        Curve2D?[] sBis1 = new Curve2D?[bis1.IntervalCount + 1];
        Curve2D?[] sBis2 = new Curve2D?[bis2.IntervalCount + 1];
        TwoDDomain[] sd1 = new TwoDDomain[bis1.IntervalCount + 1];
        TwoDDomain[] sd2 = new TwoDDomain[bis2.IntervalCount + 1];
        for (int i = 0; i < sd1.Length; i++)
        {
            sd1[i] = new TwoDDomain();
        }
        for (int i = 0; i < sd2.Length; i++)
        {
            sd2[i] = new TwoDDomain();
        }

        int nb1 = 0;
        int nb2 = 0;
        double minDomain, maxDomain;
        double uMin, uMax;
        Point2D pMin, pMax;

        //------------------------------------------------------
        // Return Min Max domain1.
        //------------------------------------------------------
        if (d1.HasFirstPoint)
        {
            minDomain = d1.FirstParameter();
        }
        else
        {
            minDomain = RealFirst();
        }

        if (d1.HasLastPoint)
        {
            maxDomain = d1.LastParameter();
        }
        else
        {
            maxDomain = RealLast();
        }

        //----------------------------------------------------------
        // Cutting the first curve by the intervals of
        // continuity taking account of D1
        //----------------------------------------------------------
        int ib1;
        for (ib1 = 1; ib1 <= bis1.IntervalCount; ib1++)
        {
            uMin = bis1.IntervalFirst(ib1);
            uMax = bis1.IntervalLast(ib1);
            if (uMax > minDomain && uMin < maxDomain)
            {
                uMin = Max(uMin, minDomain);
                uMax = Min(uMax, maxDomain);
                pMin = bis1.Value(uMin);
                pMax = bis1.Value(uMax);
                sd1[ib1].SetValues(pMin, uMin, d1.FirstTolerance(), pMax, uMax, d1.LastTolerance());

                if ((ib1 == 1 && bis1.IsExtendAtStart) || (ib1 == bis1.IntervalCount && bis1.IsExtendAtEnd))
                {
                    //--------------------------------------------------------
                    // Part corresponding to an extension is a segment.
                    //--------------------------------------------------------
                    sBis1[ib1] = ConstructSegment(pMin, pMax, uMin, uMax);
                }
                else
                {
                    sBis1[ib1] = bis1;
                }
                nb1++; // OCCT: the pieces are stored at IB1 but read from 1 to NB1 (occt-issues.md).
            }
        }

        //------------------------------------------------------
        // Return Min Max domain2.
        //------------------------------------------------------
        if (d2.HasFirstPoint)
        {
            minDomain = d2.FirstParameter();
        }
        else
        {
            minDomain = RealFirst();
        }

        if (d2.HasLastPoint)
        {
            maxDomain = d2.LastParameter();
        }
        else
        {
            maxDomain = RealLast();
        }

        //----------------------------------------------------------
        // Cut the second curve following the intervals of
        // continuity taking account of D2
        //----------------------------------------------------------
        int ib2;
        for (ib2 = 1; ib2 <= bis2.IntervalCount; ib2++)
        {
            uMin = bis2.IntervalFirst(ib2);
            uMax = bis2.IntervalLast(ib2);
            if (uMax > minDomain && uMin < maxDomain)
            {
                uMin = Max(uMin, minDomain);
                uMax = Min(uMax, maxDomain);
                pMin = bis2.Value(uMin);
                pMax = bis2.Value(uMax);
                sd2[ib2].SetValues(pMin, uMin, d2.FirstTolerance(), pMax, uMax, d2.LastTolerance());

                // OCCT: the last interval of Bis2 is tested against Bis1's count (occt-issues.md).
                if ((ib2 == 1 && bis2.IsExtendAtStart) || (ib2 == bis1.IntervalCount && bis2.IsExtendAtEnd))
                {
                    //--------------------------------------------------------
                    // Part corresponding to an extension is a segment.
                    //--------------------------------------------------------
                    sBis2[ib2] = ConstructSegment(pMin, pMax, uMin, uMax);
                }
                else
                {
                    sBis2[ib2] = bis2;
                }
                nb2++;
            }
        }

        //--------------------------------------------------------------
        // Loop on the intersections of parts of each curve.
        //--------------------------------------------------------------
        for (ib1 = 1; ib1 <= nb1; ib1++)
        {
            for (ib2 = 1; ib2 <= nb2; ib2++)
            {
                SinglePerform(sBis1[ib1]!, sd1[ib1], sBis2[ib2]!, sd2[ib2], tolConf, tol, comunElement);
            }
        }
    }

    private void SinglePerform(Curve2D cBis1, in TwoDDomain d1, Curve2D cBis2, in TwoDDomain d2,
                               double tolConf, double tol, bool comunElement)
    {
        Curve2D bis1 = cBis1;
        Curve2D bis2 = cBis2;

        Type type1 = bis1.GetType();
        Type type2 = bis2.GetType();

        if (type1 == typeof(BisecAna) || type2 == typeof(BisecAna))
        {
            Curve2D c2Bis1, c2Bis2;
            if (type1 == typeof(BisecAna))
            {
                c2Bis1 = ((BisecAna)bis1).Geom2dCurve();
            }
            else
            {
                c2Bis1 = bis1;
            }
            if (type2 == typeof(BisecAna))
            {
                c2Bis2 = ((BisecAna)bis2).Geom2dCurve();
            }
            else
            {
                c2Bis2 = bis2;
            }
            type1 = c2Bis1.GetType();
            type2 = c2Bis2.GetType();
            if (type1 == typeof(LineCurve2D) && type2 != typeof(LineCurve2D))
            {
                TestBound((LineCurve2D)c2Bis1, d1, c2Bis2, d2, tolConf, false);
            }
            else if (type2 == typeof(LineCurve2D) && type1 != typeof(LineCurve2D))
            {
                TestBound((LineCurve2D)c2Bis2, d2, c2Bis1, d1, tolConf, true);
            }
            GInter intersect = new();
            CurveAdaptor2D ac2Bis1 = new(c2Bis1);
            CurveAdaptor2D ac2Bis2 = new(c2Bis2);
            intersect.Perform(ac2Bis1, d1, ac2Bis2, d2, tolConf, tol);
            Append(intersect, d1.FirstParameter(), d1.LastParameter(), d2.FirstParameter(), d2.LastParameter());
        }
        else if (type1 == typeof(BisecPC) || type2 == typeof(BisecPC))
        {
            GInter intersect = new();
            CurveAdaptor2D aBis1 = new(bis1);
            CurveAdaptor2D aBis2 = new(bis2);
            intersect.Perform(aBis1, d1, aBis2, d2, tolConf, tol);
            Append(intersect, d1.FirstParameter(), d1.LastParameter(), d2.FirstParameter(), d2.LastParameter());
        }
        else if (comunElement && type1 == typeof(BisecCC) && type2 == typeof(BisecCC))
        {
            NeighbourPerform((BisecCC)bis1, d1, (BisecCC)bis2, d2, tol);
        }
        else
        {
            // If we are here one of two bissectrices is a segment.
            // If one of bissectrices is not a segment, it is tested if
            // its extremities are on the straight line.

            if (type1 == typeof(LineCurve2D) && type2 != typeof(LineCurve2D))
            {
                TestBound((LineCurve2D)bis1, d1, bis2, d2, tolConf, false);
            }
            else if (type2 == typeof(LineCurve2D) && type1 != typeof(LineCurve2D))
            {
                TestBound((LineCurve2D)bis2, d2, bis1, d1, tolConf, true);
            }
            GInter intersect = new();
            CurveAdaptor2D aBis1 = new(bis1);
            CurveAdaptor2D aBis2 = new(bis2);
            intersect.Perform(aBis1, d1, aBis2, d2, tolConf, tol);
            Append(intersect, d1.FirstParameter(), d1.LastParameter(), d2.FirstParameter(), d2.LastParameter());
        }
    }

    //===========================================================================
    //            Intersection of two bissectrices that share a curve
    //            (ie Bis1 separates A and B and Bis2 separates B and C).
    //            Bis1 is parameterized by B and Bis2 by C.
    //
    //            Method : Bis2 is parameterized by B
    //            2 bissectrices are thus parameterized by the same curve.
    //            Let D1(u) = d(Bis1(u),B(u)) and D2(U) = d(Bis2(u),B(U))
    //            Parameter U0 for which D1(U0)-D2(U0) = 0 is found.
    //===========================================================================
    private void NeighbourPerform(BisecCC bis1, in TwoDDomain d1, BisecCC bis2,
                                  in TwoDDomain d2, double tol)
    {
        double uSol, u1 = 0.0, u2 = 0.0, dist = 0.0;
        double uMin = 0.0, uMax = 0.0;
        double eps = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Point2D pSol;

        Curve2D guide;
        BisecCC bisTemp;

        // Change guiedline on Bis2.
        bisTemp = bis2.ChangeGuide();
        guide = bis2.Curve(2);
        // note: returned points are not used in the code, but can be useful for consulting in debugger
        // OCCT: ValueAndDist may leave UMax and UMin unset (they are 0 here; occt-issues.md).
        bis2.ValueAndDist(d2.FirstParameter(), ref u1, ref uMax, ref dist);
        bis2.ValueAndDist(d2.LastParameter(), ref u1, ref uMin, ref dist);

        // Calculate the domain of intersection on the guideline.
        uMin = Max(d1.FirstParameter(), uMin);
        uMax = Min(d1.LastParameter(), uMax);

        _done = true;

        if (uMin - eps > uMax + eps)
        {
            return;
        }

        // Solution F = 0 to find the common point.
        FunctionInter fint = new(guide, bis1, bisTemp);
        BissecNewton solution = new(tol);
        solution.Perform(fint, uMin, uMax, 20);

        if (solution.IsDone)
        {
            uSol = solution.Root();
        }
        else
        {
            return;
        }

        pSol = bisTemp.ValueAndDist(uSol, ref u1, ref u2, ref dist);

        Transition trans1 = new(), trans2 = new();
        IntersectionPoint2D pointInterSol = new(pSol, uSol, u2, trans1, trans2, false);
        Append(pointInterSol);
    }

    private void TestBound(LineCurve2D bis1, in TwoDDomain d1, Curve2D bis2, in TwoDDomain d2,
                           double tolConf, bool reverse)
    {
        Transition trans1 = new(), trans2 = new();
        IntersectionPoint2D pointInterSol = new();

        Line2D l1 = bis1.Lin2d();
        Point2D pf = bis2.Value(d2.FirstParameter());
        Point2D pl = bis2.Value(d2.LastParameter());
        //  Modified by skv - Mon May  5 14:43:28 2003 OCC616 Begin
        double tol = tolConf;
        //  Modified by skv - Mon May  5 14:43:30 2003 OCC616 End

        bool bisecAlgo = false;
        if (bis2.GetType() == typeof(BisecCC))
        {
            bisecAlgo = true;
        }

        if (l1.Distance(pf) < tol)
        {
            double u1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, pf);
            //  Modified by skv - Mon May  5 14:48:12 2003 OCC616 Begin
            if (d1.FirstParameter() - d1.FirstTolerance() < u1 && d1.LastParameter() + d1.LastTolerance() > u1)
            {
                //  Modified by skv - Mon May  5 14:48:14 2003 OCC616 End
                // PF est sur L1
                if (bisecAlgo)
                {
                    pf = global::Kerf.Engine.Geometry.ElCLib.Value(u1, l1);
                }
                pointInterSol.SetValues(pf, u1, d2.FirstParameter(), trans1, trans2, reverse);
                Append(pointInterSol);
            }
        }

        if (l1.Distance(pl) < tol)
        {
            double u1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, pl);
            //  Modified by skv - Mon May  5 15:05:48 2003 OCC616 Begin
            if (d1.FirstParameter() - d1.FirstTolerance() < u1 && d1.LastParameter() + d1.LastTolerance() > u1)
            {
                //  Modified by skv - Mon May  5 15:05:49 2003 OCC616 End
                if (bisecAlgo)
                {
                    pl = global::Kerf.Engine.Geometry.ElCLib.Value(u1, l1);
                }
                pointInterSol.SetValues(pl, u1, d2.LastParameter(), trans1, trans2, reverse);
                Append(pointInterSol);
            }
        }
    }
}
