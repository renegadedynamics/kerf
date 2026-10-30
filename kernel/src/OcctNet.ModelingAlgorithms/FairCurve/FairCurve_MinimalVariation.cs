// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_MinimalVariation.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_MinimalVariation.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_MinimalVariation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.FairCurve_AnalysisCode;

namespace Occt;

/// <summary>
/// Computes 2D curves of minimal variation of curvature: a batten whose energy mixes jerk (variation of curvature)
/// and sagging by a physical ratio, with optional curvature constraints at the ends.
/// </summary>
/// <remarks>A working object, as <see cref="FairCurve_Batten"/> (decision 859).</remarks>
public class FairCurve_MinimalVariation : FairCurve_Batten
{
    private double OldCurvature1;
    private double OldCurvature2;
    private double OldPhysicalRatio;
    private double NewCurvature1;
    private double NewCurvature2;
    private double NewPhysicalRatio;

    /// <summary>
    /// A curve through <paramref name="P1"/> and <paramref name="P2"/> of height <paramref name="Heigth"/>, slope
    /// <paramref name="Slope"/> and physical ratio <paramref name="PhysicalRatio"/> (0: pure variation of curvature;
    /// 1: a physical batten). Curvatures start at 0, the other parameters as FairCurve_Batten's.
    /// </summary>
    /// <exception cref="Standard_NullValue">P1 and P2 are confused (within Precision::Confusion).</exception>
    /// <exception cref="Standard_NegativeValue"><paramref name="Heigth"/> is not positive.</exception>
    public FairCurve_MinimalVariation(in gp_Pnt2d P1,
                                      in gp_Pnt2d P2,
                                      double Heigth,
                                      double Slope = 0,
                                      double PhysicalRatio = 0)
        : base(P1, P2, Heigth, Slope)
    {
        OldCurvature1 = 0;
        OldCurvature2 = 0;
        OldPhysicalRatio = PhysicalRatio;
        NewCurvature1 = 0;
        NewCurvature2 = 0;
        NewPhysicalRatio = PhysicalRatio;
    }

    /// <summary>Sets the curvature at the first point (used with constraint order 2).</summary>
    public void SetCurvature1(double Curvature)
    {
        NewCurvature1 = Curvature;
    }

    /// <summary>Sets the curvature at the second point (used with constraint order 2).</summary>
    public void SetCurvature2(double Curvature)
    {
        NewCurvature2 = Curvature;
    }

    /// <summary>Sets the physical ratio: 0 minimises the variation of curvature only, 1 is a physical batten.</summary>
    /// <exception cref="Standard_DomainError"><paramref name="Ratio"/> is not in [0, 1].</exception>
    public void SetPhysicalRatio(double Ratio)
    {
        Standard_DomainError_Raise_if(Ratio < 0 || Ratio > 1,
                                      "FairCurve_MinimalVariation::SetPhysicalRatio");
        NewPhysicalRatio = Ratio;
    }

    /// <summary>The curvature at the first point of the latest computation.</summary>
    public double GetCurvature1()
    {
        return OldCurvature1;
    }

    /// <summary>The curvature at the second point of the latest computation.</summary>
    public double GetCurvature2()
    {
        return OldCurvature2;
    }

    /// <summary>The physical ratio of the latest computation.</summary>
    public double GetPhysicalRatio()
    {
        return OldPhysicalRatio;
    }

    /// <summary>
    /// Computes the curve with respect to the constraints. <paramref name="ACode"/> is OK, NotConverged,
    /// InfiniteSliding or NullHeight.
    /// </summary>
    public override bool Compute(out FairCurve_AnalysisCode ACode, int NbIterations = 50, double Tolerance = 1.0e-3)
    {
        bool Ok = true, End = false;
        double AngleMax = 0.7; // parameter regulating the function of increment ( 40 degrees )
        double AngleMin = 2 * M_PI / 100; // parameter regulating the function of increment
                                          // full passage should not contain more than 100 steps.
        double DAngle1, DAngle2, DRho1, DRho2, Ratio, Fraction, Toler;
        double OldDist, NewDist;
        // the loop runs at least once, and its first call writes the code (decision 860)
        ACode = FairCurve_OK;

        //  Loop of Homotopy : calculation of the step and optimisation

        while (Ok && !End)
        {
            DAngle1 = NewAngle1 - OldAngle1;
            DAngle2 = NewAngle2 - OldAngle2;
            DRho1 = NewCurvature1 - OldCurvature1;
            DRho2 = NewCurvature2 - OldCurvature2;
            Ratio = 1;

            if (NewConstraintOrder1 > 0)
            {
                Fraction =
                  Math.Abs(DAngle1) / (AngleMax * Math.Exp(-Math.Abs(OldAngle1) / AngleMax) + AngleMin);
                if (Fraction > 1)
                {
                    Ratio = 1 / Fraction;
                }
            }
            if (NewConstraintOrder2 > 0)
            {
                Fraction =
                  Math.Abs(DAngle2) / (AngleMax * Math.Exp(-Math.Abs(OldAngle2) / AngleMax) + AngleMin);
                if (Fraction > 1)
                {
                    Ratio = (Ratio < 1 / Fraction ? Ratio : 1 / Fraction);
                }
            }

            OldDist = OldP1.Distance(OldP2);
            NewDist = NewP1.Distance(NewP2);
            Fraction = Math.Abs(OldDist - NewDist) / (OldDist / 3);
            if (Fraction > 1)
            {
                Ratio = (Ratio < 1 / Fraction ? Ratio : 1 / Fraction);
            }

            if (NewConstraintOrder1 > 1)
            {
                Fraction = Math.Abs(DRho1) * OldDist / (2 + Math.Abs(OldAngle1) + Math.Abs(OldAngle2));
                if (Fraction > 1)
                {
                    Ratio = (Ratio < 1 / Fraction ? Ratio : 1 / Fraction);
                }
            }

            if (NewConstraintOrder2 > 1)
            {
                Fraction = Math.Abs(DRho2) * OldDist / (2 + Math.Abs(OldAngle1) + Math.Abs(OldAngle2));
                if (Fraction > 1)
                {
                    Ratio = (Ratio < 1 / Fraction ? Ratio : 1 / Fraction);
                }
            }

            gp_Vec2d DeltaP1 = new(OldP1, NewP1), DeltaP2 = new(OldP2, NewP2);
            if (Ratio == 1)
            {
                End = true;
                Toler = Tolerance;
            }
            else
            {
                DeltaP1 *= Ratio;
                DeltaP2 *= Ratio;
                DAngle1 *= Ratio;
                DAngle2 *= Ratio;
                DRho1 *= Ratio;
                DRho2 *= Ratio;
                Toler = 10 * Tolerance;
            }

            Ok = Compute(DeltaP1, DeltaP2, DAngle1, DAngle2, DRho1, DRho2, out ACode, NbIterations, Toler);

            if (ACode != FairCurve_OK)
            {
                End = true;
            }
            if (NewFreeSliding)
            {
                NewSlidingFactor = OldSlidingFactor;
            }
            if (NewConstraintOrder1 == 0)
            {
                NewAngle1 = OldAngle1;
            }
            if (NewConstraintOrder1 < 2)
            {
                NewCurvature1 = OldCurvature1;
            }
            if (NewConstraintOrder2 == 0)
            {
                NewAngle2 = OldAngle2;
            }
            if (NewConstraintOrder2 < 2)
            {
                NewCurvature2 = OldCurvature2;
            }
        }
        myCode = ACode;
        return Ok;
    }

    private bool Compute(in gp_Vec2d DeltaP1,
                         in gp_Vec2d DeltaP2,
                         double DeltaAngle1,
                         double DeltaAngle2,
                         double DeltaCurvature1,
                         double DeltaCurvature2,
                         out FairCurve_AnalysisCode ACode,
                         int NbIterations,
                         double Tolerance)
    {
        bool Ok, OkCompute = true;
        ACode = FairCurve_OK;

        // Deformation of the curve by adding a polynom of interpolation
        int L = 2 + NewConstraintOrder1 + NewConstraintOrder2, kk, ii;
        // (OCCT's debug build counts the poles here)
        NCollection_Array1<double> knots = new(1, 2);
        knots[1] = 0;
        knots[2] = 1;
        NCollection_Array1<int> mults = new(1, 2);
        NCollection_Array1<gp_Pnt2d> HermitePoles = new(1, L);
        NCollection_Array1<gp_Pnt2d> Interpolation = new(1, L);
        NCollection_HArray1<gp_Pnt2d> NPoles = new(1, Poles.Length());

        // Polynomes of Hermite
        math_Matrix HermiteCoef = new(1, L, 1, L);
        Ok = PLib.HermiteCoefficients(0, 1, NewConstraintOrder1, NewConstraintOrder2, HermiteCoef);
        if (!Ok)
        {
            return false;
        }

        // Definition of constraints of interpolation
        NCollection_Array1<gp_XY> ADelta = new(1, L);
        gp_Vec2d VOld = new(OldP1, OldP2), VNew = new(-(OldP1.XY() + DeltaP1.XY()) + (OldP2.XY() + DeltaP2.XY()));
        double DAngleRef = VNew.Angle(VOld);
        double DAngle1 = DeltaAngle1 - DAngleRef,
               DAngle2 = DAngleRef - DeltaAngle2; // Correction of Delta by the Delta induced by the points.

        ADelta[1] = DeltaP1.XY();
        kk = 2;
        if (NewConstraintOrder1 > 0)
        {
            // rotation of the derivative premiereDeltaAngle1
            gp_Vec2d OldDerive = new(Poles.Value(Poles.Lower()), Poles.Value(Poles.Lower() + 1));
            double aKnotGapL = Knots.Value(Knots.Lower() + 1) - Knots.Value(Knots.Lower());
            if (Math.Abs(aKnotGapL) > Precision.Computational())
            {
                OldDerive *= Degree / aKnotGapL;
            }
            ADelta[kk] = (OldDerive.Rotated(DAngle1) - OldDerive).XY();
            kk += 1;

            if (NewConstraintOrder1 > 1)
            {
                // rotation of the second derivative + adding
                gp_Vec2d OldSeconde = new(Poles.Value(Poles.Lower()).XY() + Poles.Value(Poles.Lower() + 2).XY()
                                          - 2 * Poles.Value(Poles.Lower() + 1).XY());
                double aKnotGapLSq = aKnotGapL * aKnotGapL;
                if (Math.Abs(aKnotGapLSq) > Precision.SquareComputational())
                {
                    OldSeconde *= Degree * (Degree - 1) / aKnotGapLSq;
                }
                double CPrim = OldDerive.Magnitude();
                ADelta[kk] = (OldSeconde.Rotated(DAngle1) - OldSeconde
                              + DeltaCurvature1 * CPrim * OldDerive.Rotated(M_PI / 2 + DAngle1))
                               .XY();
                kk += 1;
            }
        }
        ADelta[kk] = DeltaP2.XY();
        kk += 1;
        if (NewConstraintOrder2 > 0)
        {
            gp_Vec2d OldDerive = new(Poles.Value(Poles.Upper() - 1), Poles.Value(Poles.Upper()));
            double aKnotGapU = Knots.Value(Knots.Upper()) - Knots.Value(Knots.Upper() - 1);
            if (Math.Abs(aKnotGapU) > Precision.Computational())
            {
                OldDerive *= Degree / aKnotGapU;
            }
            ADelta[kk] = (OldDerive.Rotated(DAngle2) - OldDerive).XY();
            kk += 1;
            if (NewConstraintOrder2 > 1)
            {
                // rotation of the second derivative + adding
                gp_Vec2d OldSeconde = new(Poles.Value(Poles.Upper()).XY() + Poles.Value(Poles.Upper() - 2).XY()
                                          - 2 * Poles.Value(Poles.Upper() - 1).XY());
                double aKnotGapUSq = aKnotGapU * aKnotGapU;
                if (Math.Abs(aKnotGapUSq) > Precision.SquareComputational())
                {
                    OldSeconde *= Degree * (Degree - 1) / aKnotGapUSq;
                }
                double CPrim = OldDerive.Magnitude();
                ADelta[kk] = (OldSeconde.Rotated(DAngle2) - OldSeconde
                              + DeltaCurvature2 * CPrim * OldDerive.Rotated(M_PI / 2 + DAngle2))
                               .XY();
                kk += 1;
            }
        }

        // Interpolation
        gp_XY AuxXY = new(0, 0);
        for (ii = 1; ii <= L; ii++)
        {
            AuxXY.SetCoord(0.0, 0);
            for (kk = 1; kk <= L; kk++)
            {
                AuxXY += HermiteCoef[kk, ii] * ADelta[kk];
            }
            Interpolation[ii].SetXY(AuxXY);
        }
        // Conversion into BSpline of the same structure as the current batten.
        PLib.CoefficientsPoles(Interpolation, PLib.NoWeights(), HermitePoles, PLib.NoWeights());

        mults.Init(L);

        Geom2d_BSplineCurve DeltaCurve = new(HermitePoles, knots, mults, L - 1);

        DeltaCurve.IncreaseDegree(Degree);
        if (Mults.Length() > 2)
        {
            DeltaCurve.InsertKnots(Knots.Array1(), Mults.Array1(), 1.0e-10);
        }

        // Summing
        NPoles.ChangeArray1().Assign(DeltaCurve.InternalPoles());
        for (kk = NPoles.Lower(); kk <= NPoles.Upper(); kk++)
        {
            NPoles.ChangeValue(kk).ChangeCoord() += Poles.Value(kk).Coord();
        }

        // Intermediaires

        double Angle1, Angle2, SlidingLength,
          Alph1 = OldAngle1 + DeltaAngle1, Alph2 = OldAngle2 + DeltaAngle2,
          Rho1 = OldCurvature1 + DeltaCurvature1, Rho2 = OldCurvature2 + DeltaCurvature2,
          Dist = NPoles.Value(NPoles.Upper()).Distance(NPoles.Value(NPoles.Lower())),
          LReference = SlidingOfReference(Dist, Alph1, Alph2);
        gp_Vec2d Ox = new(1, 0),
          P1P2 = new(NPoles.Value(NPoles.Upper()).Coord() - NPoles.Value(NPoles.Lower()).Coord());

        // Angles corresponding to axis ox

        Angle1 = Ox.Angle(P1P2) + Alph1;
        Angle2 = -Ox.Angle(P1P2) + Alph2;

        // Calculation of the length of sliding (imposed or initial);

        if (!NewFreeSliding)
        {
            SlidingLength = NewSlidingFactor * LReference;
        }
        else
        {
            if (OldFreeSliding)
            {
                SlidingLength = OldSlidingFactor * LReference;
            }
            else
            {
                SlidingLength = SlidingOfReference(Dist, Alph1, Alph2);
            }
        }

        // Energy and vectors of initialization
        FairCurve_BattenLaw LBatten = new(NewHeight, NewSlope, SlidingLength);
        FairCurve_EnergyOfMVC EMVC = new(Degree + 1,
                                         Flatknots,
                                         NPoles,
                                         NewConstraintOrder1,
                                         NewConstraintOrder2,
                                         LBatten,
                                         NewPhysicalRatio,
                                         SlidingLength,
                                         NewFreeSliding,
                                         Angle1,
                                         Angle2,
                                         Rho1,
                                         Rho2);
        math_Vector VInit = new(1, EMVC.NbVariables());

        // The value below gives an idea about the smallest value of the criterion of flexion.
        double VConvex = 0.01 * Math.Pow(NewHeight / SlidingLength, 3);
        if (VConvex < 1.0e-12)
        {
            VConvex = 1.0e-12;
        }

        Ok = EMVC.Variable(VInit);

        // Minimisation
        FairCurve_Newton Newton = new(EMVC,
                                      Tolerance * (P1P2.Magnitude() / 10),
                                      Tolerance,
                                      NbIterations,
                                      VConvex);
        Newton.Perform(EMVC, VInit);
        Ok = Newton.IsDone();

        if (Ok)
        {
            gp_Vec2d Tangente = new(), PseudoNormale = new();
            Poles = NPoles;
            Newton.Location(VInit);

            if (NewFreeSliding)
            {
                OldSlidingFactor = VInit[VInit.Upper()] / LReference;
            }
            else
            {
                OldSlidingFactor = NewSlidingFactor;
            }

            if (NewConstraintOrder1 < 2)
            {
                Tangente.SetXY(Poles.Value(Poles.Lower() + 1).XY() - Poles.Value(Poles.Lower()).XY());

                if (NewConstraintOrder1 == 0)
                {
                    OldAngle1 = P1P2.Angle(Tangente);
                }
                else
                {
                    OldAngle1 = Alph1;
                }

                PseudoNormale.SetXY(Poles.Value(Poles.Lower()).XY()
                                    - 2 * Poles.Value(Poles.Lower() + 1).XY()
                                    + Poles.Value(Poles.Lower() + 2).XY());
                OldCurvature1 = (((double)Degree - 1) / Degree) * (Tangente.Normalized() ^ PseudoNormale)
                                / Tangente.SquareMagnitude();
            }
            else
            {
                OldCurvature1 = Rho1;
                OldAngle1 = Alph1;
            }

            if (NewConstraintOrder2 < 2)
            {
                Tangente.SetXY(Poles.Value(Poles.Upper()).XY() - Poles.Value(Poles.Upper() - 1).XY());
                if (NewConstraintOrder2 == 0)
                {
                    OldAngle2 = (-Tangente).Angle(-P1P2);
                }
                else
                {
                    OldAngle2 = Alph2;
                }
                PseudoNormale.SetXY(Poles.Value(Poles.Upper()).XY()
                                    - 2 * Poles.Value(Poles.Upper() - 1).XY()
                                    + Poles.Value(Poles.Upper() - 2).XY());
                OldCurvature2 = (((double)Degree - 1) / Degree) * (Tangente.Normalized() ^ PseudoNormale)
                                / Tangente.SquareMagnitude();
            }
            else
            {
                OldAngle2 = Alph2;
                OldCurvature2 = Rho2;
            }

            OldP1 = Poles.Value(Poles.Lower());
            OldP2 = Poles.Value(Poles.Upper());
            OldConstraintOrder1 = NewConstraintOrder1;
            OldConstraintOrder2 = NewConstraintOrder2;
            OldFreeSliding = NewFreeSliding;
            OldSlope = NewSlope;
            OldHeight = NewHeight;
            OldPhysicalRatio = NewPhysicalRatio;
        }
        else
        {
            double V;
            ACode = EMVC.Status();
            if (!LBatten.Value(0, out V) || !LBatten.Value(1, out V))
            {
                ACode = FairCurve_NullHeight;
            }
            else
            {
                OkCompute = false;
            }
            return OkCompute;
        }

        Ok = EMVC.Variable(VInit);

        // Processing of non convergence
        if (!Newton.IsConverged())
        {
            ACode = FairCurve_NotConverged;
        }

        // Prevention of infinite sliding
        if (NewFreeSliding && VInit[VInit.Upper()] > 2 * LReference)
        {
            ACode = FairCurve_InfiniteSliding;
        }

        // Eventual insertion of Nodes
        bool NewKnots = false;
        int NbKnots = Knots.Length();
        double ValAngles =
          (Math.Abs(OldAngle1) + Math.Abs(OldAngle2) + 2 * Math.Abs(OldAngle2 - OldAngle1));
        while (ValAngles > (2 * (NbKnots - 2) + 1) * (1 + 2 * NbKnots))
        {
            NewKnots = true;
            NbKnots += NbKnots - 1;
        }

        if (NewKnots)
        {
            Geom2d_BSplineCurve NewBS = new(NPoles.Array1(), Knots.Array1(), Mults.Array1(), Degree);

            NCollection_HArray1<int> NMults = new(1, NbKnots);
            NMults.Init(Degree - 3);

            NCollection_HArray1<double> NKnots = new(1, NbKnots);
            for (ii = 1; ii <= NbKnots; ii++)
            {
                NKnots.ChangeValue(ii) = (double)(ii - 1) / (NbKnots - 1);
            }

            NewBS.InsertKnots(NKnots.Array1(), NMults.Array1(), 1.0e-10);
            NCollection_HArray1<gp_Pnt2d> NewNPoles = new(NewBS.InternalPoles());
            NMults = new NCollection_HArray1<int>(NewBS.InternalMultiplicities());
            NKnots = new NCollection_HArray1<double>(NewBS.InternalKnots());
            NCollection_HArray1<double> FKnots = new(NewBS.InternalKnotSequence());

            Poles = NewNPoles;
            Mults = NMults;
            Knots = NKnots;
            Flatknots = FKnots;
        }

        // For eventual debug Newton.Dump(std::cout);

        return OkCompute;
    }

    /// <summary>Writes the current state (old and new constraints, the analysis code) to <paramref name="o"/>.</summary>
    public override void Dump(TextWriter o)
    {

        o.Write("  MVCurve      |");
        o.Write(Width7("Old  |   New"));
        o.Write('\n');
        o.Write("  P1    X      |");
        DumpPair(o, OldP1.X(), NewP1.X());
        o.Write("        Y      |");
        DumpPair(o, OldP1.Y(), NewP1.Y());
        o.Write("  P2    X      |");
        DumpPair(o, OldP2.X(), NewP2.X());
        o.Write("        Y      |");
        DumpPair(o, OldP2.Y(), NewP2.Y());
        o.Write("      Angle1   |");
        DumpPair(o, OldAngle1, NewAngle1);
        o.Write("      Angle2   |");
        DumpPair(o, OldAngle2, NewAngle2);
        o.Write(" Curvature1    |");
        DumpPair(o, OldCurvature1, NewCurvature1);
        o.Write(" Curvature2    |");
        DumpPair(o, OldCurvature2, NewCurvature2);
        o.Write("      Height   |");
        DumpPair(o, OldHeight, NewHeight);
        o.Write("      Slope    |");
        DumpPair(o, OldSlope, NewSlope);
        o.Write(" PhysicalRatio |");
        DumpPair(o, OldPhysicalRatio, NewPhysicalRatio);
        o.Write(" SlidingFactor |");
        DumpPair(o, OldSlidingFactor, NewSlidingFactor);
        o.Write(" FreeSliding   |");
        DumpPair(o, OldFreeSliding ? 1 : 0, NewFreeSliding ? 1 : 0);
        o.Write(" ConstrOrder1  |");
        DumpPair(o, OldConstraintOrder1, NewConstraintOrder1);
        o.Write(" ConstrOrder2  |");
        DumpPair(o, OldConstraintOrder2, NewConstraintOrder2);
        DumpCode(o);
    }
}
