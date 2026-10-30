// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Inter.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Inter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public sealed class Bisector_Inter : IntRes2d_Intersection
{
    /// <summary>An empty intersection.</summary>
    public Bisector_Inter()
    {
    }

    /// <summary>
    /// The intersection of C1 (which separates the elements A and B) on D1 and C2 (which separates C and D) on D2.
    /// ComunElement must be true if B and C are the same element: it allows a faster computation.
    /// </summary>
    public Bisector_Inter(Bisector_Bisec C1, in IntRes2d_Domain D1, Bisector_Bisec C2, in IntRes2d_Domain D2,
                          double TolConf, double Tol, bool ComunElement)
    {
        Perform(C1, D1, C2, D2, TolConf, Tol, ComunElement);
    }

    //===================================================================================
    // Construction of a segment of line from PMin to PMax, parameterised from UMin.
    //===================================================================================
    private static Geom2d_Line ConstructSegment(in gp_Pnt2d PMin, in gp_Pnt2d PMax, double UMin, double UMax)
    {
        gp_Dir2d Dir = new(PMax.X() - PMin.X(), PMax.Y() - PMin.Y());
        Geom2d_Line L = new(new gp_Pnt2d(PMin.X() - UMin * Dir.X(), PMin.Y() - UMin * Dir.Y()), Dir);
        return L;
    }

    /// <summary>
    /// Computes the intersection of C1 (which separates the elements A and B) on D1 and C2 (which separates C and D)
    /// on D2. ComunElement must be true if B and C are the same element.
    /// </summary>
    public void Perform(Bisector_Bisec C1, in IntRes2d_Domain D1, Bisector_Bisec C2, in IntRes2d_Domain D2,
                        double TolConf, double Tol, bool ComunElement)
    {
        // OCCT.NET: the bisectors are read without freezing them (MAT2d re-trims them in place afterwards).
        Bisector_Curve Bis1 = (Bisector_Curve)C1.Value().ChangeBasisCurve();
        Bisector_Curve Bis2 = (Bisector_Curve)C2.Value().ChangeBasisCurve();

        Geom2d_Curve?[] SBis1 = new Geom2d_Curve?[Bis1.NbIntervals() + 1];
        Geom2d_Curve?[] SBis2 = new Geom2d_Curve?[Bis2.NbIntervals() + 1];
        IntRes2d_Domain[] SD1 = new IntRes2d_Domain[Bis1.NbIntervals() + 1];
        IntRes2d_Domain[] SD2 = new IntRes2d_Domain[Bis2.NbIntervals() + 1];
        for (int i = 0; i < SD1.Length; i++)
        {
            SD1[i] = new IntRes2d_Domain();
        }
        for (int i = 0; i < SD2.Length; i++)
        {
            SD2[i] = new IntRes2d_Domain();
        }

        int NB1 = 0;
        int NB2 = 0;
        double MinDomain, MaxDomain;
        double UMin, UMax;
        gp_Pnt2d PMin, PMax;

        //------------------------------------------------------
        // Return Min Max domain1.
        //------------------------------------------------------
        if (D1.HasFirstPoint())
        {
            MinDomain = D1.FirstParameter();
        }
        else
        {
            MinDomain = RealFirst();
        }

        if (D1.HasLastPoint())
        {
            MaxDomain = D1.LastParameter();
        }
        else
        {
            MaxDomain = RealLast();
        }

        //----------------------------------------------------------
        // Cutting the first curve by the intervals of
        // continuity taking account of D1
        //----------------------------------------------------------
        int IB1;
        for (IB1 = 1; IB1 <= Bis1.NbIntervals(); IB1++)
        {
            UMin = Bis1.IntervalFirst(IB1);
            UMax = Bis1.IntervalLast(IB1);
            if (UMax > MinDomain && UMin < MaxDomain)
            {
                UMin = Max(UMin, MinDomain);
                UMax = Min(UMax, MaxDomain);
                PMin = Bis1.Value(UMin);
                PMax = Bis1.Value(UMax);
                SD1[IB1].SetValues(PMin, UMin, D1.FirstTolerance(), PMax, UMax, D1.LastTolerance());

                if ((IB1 == 1 && Bis1.IsExtendAtStart()) || (IB1 == Bis1.NbIntervals() && Bis1.IsExtendAtEnd()))
                {
                    //--------------------------------------------------------
                    // Part corresponding to an extension is a segment.
                    //--------------------------------------------------------
                    SBis1[IB1] = ConstructSegment(PMin, PMax, UMin, UMax);
                }
                else
                {
                    SBis1[IB1] = Bis1;
                }
                NB1++; // OCCT: the pieces are stored at IB1 but read from 1 to NB1 (occt-issues.md).
            }
        }

        //------------------------------------------------------
        // Return Min Max domain2.
        //------------------------------------------------------
        if (D2.HasFirstPoint())
        {
            MinDomain = D2.FirstParameter();
        }
        else
        {
            MinDomain = RealFirst();
        }

        if (D2.HasLastPoint())
        {
            MaxDomain = D2.LastParameter();
        }
        else
        {
            MaxDomain = RealLast();
        }

        //----------------------------------------------------------
        // Cut the second curve following the intervals of
        // continuity taking account of D2
        //----------------------------------------------------------
        int IB2;
        for (IB2 = 1; IB2 <= Bis2.NbIntervals(); IB2++)
        {
            UMin = Bis2.IntervalFirst(IB2);
            UMax = Bis2.IntervalLast(IB2);
            if (UMax > MinDomain && UMin < MaxDomain)
            {
                UMin = Max(UMin, MinDomain);
                UMax = Min(UMax, MaxDomain);
                PMin = Bis2.Value(UMin);
                PMax = Bis2.Value(UMax);
                SD2[IB2].SetValues(PMin, UMin, D2.FirstTolerance(), PMax, UMax, D2.LastTolerance());

                // OCCT: the last interval of Bis2 is tested against Bis1's count (occt-issues.md).
                if ((IB2 == 1 && Bis2.IsExtendAtStart()) || (IB2 == Bis1.NbIntervals() && Bis2.IsExtendAtEnd()))
                {
                    //--------------------------------------------------------
                    // Part corresponding to an extension is a segment.
                    //--------------------------------------------------------
                    SBis2[IB2] = ConstructSegment(PMin, PMax, UMin, UMax);
                }
                else
                {
                    SBis2[IB2] = Bis2;
                }
                NB2++;
            }
        }

        //--------------------------------------------------------------
        // Loop on the intersections of parts of each curve.
        //--------------------------------------------------------------
        for (IB1 = 1; IB1 <= NB1; IB1++)
        {
            for (IB2 = 1; IB2 <= NB2; IB2++)
            {
                SinglePerform(SBis1[IB1]!, SD1[IB1], SBis2[IB2]!, SD2[IB2], TolConf, Tol, ComunElement);
            }
        }
    }

    private void SinglePerform(Geom2d_Curve CBis1, in IntRes2d_Domain D1, Geom2d_Curve CBis2, in IntRes2d_Domain D2,
                               double TolConf, double Tol, bool ComunElement)
    {
        Geom2d_Curve Bis1 = CBis1;
        Geom2d_Curve Bis2 = CBis2;

        Type Type1 = Bis1.GetType();
        Type Type2 = Bis2.GetType();

        if (Type1 == typeof(Bisector_BisecAna) || Type2 == typeof(Bisector_BisecAna))
        {
            Geom2d_Curve C2Bis1, C2Bis2;
            if (Type1 == typeof(Bisector_BisecAna))
            {
                C2Bis1 = ((Bisector_BisecAna)Bis1).Geom2dCurve();
            }
            else
            {
                C2Bis1 = Bis1;
            }
            if (Type2 == typeof(Bisector_BisecAna))
            {
                C2Bis2 = ((Bisector_BisecAna)Bis2).Geom2dCurve();
            }
            else
            {
                C2Bis2 = Bis2;
            }
            Type1 = C2Bis1.GetType();
            Type2 = C2Bis2.GetType();
            if (Type1 == typeof(Geom2d_Line) && Type2 != typeof(Geom2d_Line))
            {
                TestBound((Geom2d_Line)C2Bis1, D1, C2Bis2, D2, TolConf, false);
            }
            else if (Type2 == typeof(Geom2d_Line) && Type1 != typeof(Geom2d_Line))
            {
                TestBound((Geom2d_Line)C2Bis2, D2, C2Bis1, D1, TolConf, true);
            }
            Geom2dInt_GInter Intersect = new();
            Geom2dAdaptor_Curve AC2Bis1 = new(C2Bis1);
            Geom2dAdaptor_Curve AC2Bis2 = new(C2Bis2);
            Intersect.Perform(AC2Bis1, D1, AC2Bis2, D2, TolConf, Tol);
            Append(Intersect, D1.FirstParameter(), D1.LastParameter(), D2.FirstParameter(), D2.LastParameter());
        }
        else if (Type1 == typeof(Bisector_BisecPC) || Type2 == typeof(Bisector_BisecPC))
        {
            Geom2dInt_GInter Intersect = new();
            Geom2dAdaptor_Curve ABis1 = new(Bis1);
            Geom2dAdaptor_Curve ABis2 = new(Bis2);
            Intersect.Perform(ABis1, D1, ABis2, D2, TolConf, Tol);
            Append(Intersect, D1.FirstParameter(), D1.LastParameter(), D2.FirstParameter(), D2.LastParameter());
        }
        else if (ComunElement && Type1 == typeof(Bisector_BisecCC) && Type2 == typeof(Bisector_BisecCC))
        {
            NeighbourPerform((Bisector_BisecCC)Bis1, D1, (Bisector_BisecCC)Bis2, D2, Tol);
        }
        else
        {
            // If we are here one of two bissectrices is a segment.
            // If one of bissectrices is not a segment, it is tested if
            // its extremities are on the straight line.

            if (Type1 == typeof(Geom2d_Line) && Type2 != typeof(Geom2d_Line))
            {
                TestBound((Geom2d_Line)Bis1, D1, Bis2, D2, TolConf, false);
            }
            else if (Type2 == typeof(Geom2d_Line) && Type1 != typeof(Geom2d_Line))
            {
                TestBound((Geom2d_Line)Bis2, D2, Bis1, D1, TolConf, true);
            }
            Geom2dInt_GInter Intersect = new();
            Geom2dAdaptor_Curve ABis1 = new(Bis1);
            Geom2dAdaptor_Curve ABis2 = new(Bis2);
            Intersect.Perform(ABis1, D1, ABis2, D2, TolConf, Tol);
            Append(Intersect, D1.FirstParameter(), D1.LastParameter(), D2.FirstParameter(), D2.LastParameter());
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
    private void NeighbourPerform(Bisector_BisecCC Bis1, in IntRes2d_Domain D1, Bisector_BisecCC Bis2,
                                  in IntRes2d_Domain D2, double Tol)
    {
        double USol, U1 = 0.0, U2 = 0.0, Dist = 0.0;
        double UMin = 0.0, UMax = 0.0;
        double Eps = Precision.PConfusion();
        gp_Pnt2d PSol;

        Geom2d_Curve Guide;
        Bisector_BisecCC BisTemp;

        // Change guiedline on Bis2.
        BisTemp = Bis2.ChangeGuide();
        Guide = Bis2.Curve(2);
        // note: returned points are not used in the code, but can be useful for consulting in debugger
        Bis2.ValueAndDist(D2.FirstParameter(), ref U1, ref UMax, ref Dist);
        Bis2.ValueAndDist(D2.LastParameter(), ref U1, ref UMin, ref Dist);

        // Calculate the domain of intersection on the guideline.
        UMin = Max(D1.FirstParameter(), UMin);
        UMax = Min(D1.LastParameter(), UMax);

        done = true;

        if (UMin - Eps > UMax + Eps)
        {
            return;
        }

        // Solution F = 0 to find the common point.
        Bisector_FunctionInter Fint = new(Guide, Bis1, BisTemp);
        math_BissecNewton aSolution = new(Tol);
        aSolution.Perform(Fint, UMin, UMax, 20);

        if (aSolution.IsDone())
        {
            USol = aSolution.Root();
        }
        else
        {
            return;
        }

        PSol = BisTemp.ValueAndDist(USol, ref U1, ref U2, ref Dist);

        IntRes2d_Transition Trans1 = new(), Trans2 = new();
        IntRes2d_IntersectionPoint PointInterSol = new(PSol, USol, U2, Trans1, Trans2, false);
        Append(PointInterSol);
    }

    private void TestBound(Geom2d_Line Bis1, in IntRes2d_Domain D1, Geom2d_Curve Bis2, in IntRes2d_Domain D2,
                           double TolConf, bool Reverse)
    {
        IntRes2d_Transition Trans1 = new(), Trans2 = new();
        IntRes2d_IntersectionPoint PointInterSol = new();

        gp_Lin2d L1 = Bis1.Lin2d();
        gp_Pnt2d PF = Bis2.Value(D2.FirstParameter());
        gp_Pnt2d PL = Bis2.Value(D2.LastParameter());
        //  Modified by skv - Mon May  5 14:43:28 2003 OCC616 Begin
        double Tol = TolConf;
        //  Modified by skv - Mon May  5 14:43:30 2003 OCC616 End

        bool BisecAlgo = false;
        if (Bis2.GetType() == typeof(Bisector_BisecCC))
        {
            BisecAlgo = true;
        }

        if (L1.Distance(PF) < Tol)
        {
            double U1 = ElCLib.Parameter(L1, PF);
            //  Modified by skv - Mon May  5 14:48:12 2003 OCC616 Begin
            if (D1.FirstParameter() - D1.FirstTolerance() < U1 && D1.LastParameter() + D1.LastTolerance() > U1)
            {
                //  Modified by skv - Mon May  5 14:48:14 2003 OCC616 End
                // PF est sur L1
                if (BisecAlgo)
                {
                    PF = ElCLib.Value(U1, L1);
                }
                PointInterSol.SetValues(PF, U1, D2.FirstParameter(), Trans1, Trans2, Reverse);
                Append(PointInterSol);
            }
        }

        if (L1.Distance(PL) < Tol)
        {
            double U1 = ElCLib.Parameter(L1, PL);
            //  Modified by skv - Mon May  5 15:05:48 2003 OCC616 Begin
            if (D1.FirstParameter() - D1.FirstTolerance() < U1 && D1.LastParameter() + D1.LastTolerance() > U1)
            {
                //  Modified by skv - Mon May  5 15:05:49 2003 OCC616 End
                if (BisecAlgo)
                {
                    PL = ElCLib.Value(U1, L1);
                }
                PointInterSol.SetValues(PL, U1, D2.LastParameter(), Trans1, Trans2, Reverse);
                Append(PointInterSol);
            }
        }
    }
}
