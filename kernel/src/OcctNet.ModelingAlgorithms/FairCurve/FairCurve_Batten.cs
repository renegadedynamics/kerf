// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Batten.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Batten.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Batten.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.FairCurve_AnalysisCode;

namespace Occt;

/// <summary>
/// Constructs 2D curves that simulate physical splines or battens (wooden or plastic beams of constant or linearly
/// increasing section), from two points and constraints of tangency at each.
/// </summary>
/// <remarks>
/// A working object (decision 859), as OCCT's: the setters record the wanted ("new") constraints, <see
/// cref="Compute(out FairCurve_AnalysisCode, int, double)"/> deforms the current curve toward them by homotopy and
/// records the reached ("old") ones, which the getters return. <see cref="Curve"/> returns a new, frozen curve each
/// time. OCCT's <c>operator&lt;&lt;</c> is <see cref="ToString"/>.
/// </remarks>
public class FairCurve_Batten
{
    protected FairCurve_AnalysisCode myCode;
    protected gp_Pnt2d OldP1;
    protected gp_Pnt2d OldP2;
    protected double OldAngle1;
    protected double OldAngle2;
    protected double OldHeight;
    protected double OldSlope;
    protected double OldSlidingFactor;
    protected bool OldFreeSliding;
    protected int OldConstraintOrder1;
    protected int OldConstraintOrder2;
    protected gp_Pnt2d NewP1;
    protected gp_Pnt2d NewP2;
    protected double NewAngle1;
    protected double NewAngle2;
    protected double NewHeight;
    protected double NewSlope;
    protected double NewSlidingFactor;
    protected bool NewFreeSliding;
    protected int NewConstraintOrder1;
    protected int NewConstraintOrder2;
    protected int Degree;
    protected NCollection_HArray1<gp_Pnt2d> Poles;
    protected NCollection_HArray1<double> Flatknots;
    protected NCollection_HArray1<double> Knots;
    protected NCollection_HArray1<int> Mults;

    /// <summary>
    /// A batten through <paramref name="P1"/> and <paramref name="P2"/> of height <paramref name="Height"/> and slope
    /// <paramref name="Slope"/>. The other parameters start as: FreeSliding false, ConstraintOrder1 and 2 equal to 1,
    /// Angle1 and Angle2 equal to 0, SlidingFactor 1.
    /// </summary>
    /// <exception cref="Standard_NullValue">P1 and P2 are confused (within Precision::Confusion).</exception>
    /// <exception cref="Standard_NegativeValue"><paramref name="Height"/> is not positive.</exception>
    public FairCurve_Batten(in gp_Pnt2d P1, in gp_Pnt2d P2, double Height, double Slope = 0)
    {
        myCode = FairCurve_OK;
        OldP1 = P1;
        OldP2 = P2;
        OldAngle1 = 0;
        OldAngle2 = 0;
        OldHeight = Height;
        OldSlope = Slope;
        OldSlidingFactor = 1;
        OldFreeSliding = false;
        OldConstraintOrder1 = 1;
        OldConstraintOrder2 = 1;
        NewP1 = P1;
        NewP2 = P2;
        NewAngle1 = 0;
        NewAngle2 = 0;
        NewHeight = Height;
        NewSlope = Slope;
        NewSlidingFactor = 1;
        NewFreeSliding = false;
        NewConstraintOrder1 = 1;
        NewConstraintOrder2 = 1;
        Degree = 9;

        if (P1.IsEqual(P2, Precision.Confusion()))
        {
            throw new Standard_NullValue("FairCurve : P1 and P2 are confused");
        }
        if (Height <= 0)
        {
            throw new Standard_NegativeValue("FairCurve : Height is not positive");
        }
        //
        //   Initialize by a straight line (2 poles)
        //
        NCollection_HArray1<double> Iknots = new(1, 2);
        NCollection_HArray1<int> Imults = new(1, 2);
        NCollection_HArray1<gp_Pnt2d> Ipoles = new(1, 2);

        Iknots.SetValue(1, 0);
        Iknots.SetValue(2, 1);

        Imults.SetValue(1, 2);
        Imults.SetValue(2, 2);

        Ipoles.SetValue(1, P1);
        Ipoles.SetValue(2, P2);

        //  Increase the degree

        NCollection_HArray1<gp_Pnt2d> Npoles = new(1, Degree + 1);
        NCollection_HArray1<double> Nweight = new(1, 2);
        NCollection_HArray1<double> Nknots = new(1, 2);
        NCollection_HArray1<int> Nmults = new(1, 2);

        BSplCLib.IncreaseDegree(1,
                                Degree,
                                false,
                                Ipoles.Array1(),
                                BSplCLib.NoWeights(),
                                Iknots.Array1(),
                                Imults.Array1(),
                                Npoles.ChangeArray1(),
                                Nweight.ChangeArray1(),
                                Nknots.ChangeArray1(),
                                Nmults.ChangeArray1());

        // and impact the result in our fields

        Poles = Npoles;
        Knots = Nknots;
        Mults = Nmults;

        // calculate "plane" nodes

        Flatknots =
          new NCollection_HArray1<double>(1,
                                          BSplCLib.KnotSequenceLength(Mults.Array1(), Degree, false));

        BSplCLib.KnotSequence(Knots.Array1(),
                              Mults.Array1(),
                              Degree,
                              false,
                              Flatknots.ChangeArray1());
    }

    /// <summary>
    /// Sets free sliding (false by default). When sliding is free, the sliding factor is computed to satisfy the
    /// equilibrium of the batten.
    /// </summary>
    public void SetFreeSliding(bool FreeSliding)
    {
        NewFreeSliding = FreeSliding;
    }

    /// <summary>
    /// Sets the order of the constraint at the first point (1 by default): 0, the curve passes through the point; 1,
    /// with a given tangent; 2, with a given tangent and curvature (FairCurve_MinimalVariation only).
    /// </summary>
    public void SetConstraintOrder1(int ConstraintOrder)
    {
        NewConstraintOrder1 = ConstraintOrder;
    }

    /// <summary>Sets the order of the constraint at the second point (1 by default), as <see cref="SetConstraintOrder1"/>.</summary>
    public void SetConstraintOrder2(int ConstraintOrder)
    {
        NewConstraintOrder2 = ConstraintOrder;
    }

    /// <summary>Moves the first point (which changes the angles as well).</summary>
    /// <exception cref="Standard_NullValue">P1 and P2 are confused (within Precision::Confusion).</exception>
    public void SetP1(in gp_Pnt2d P1)
    {
        if (P1.IsEqual(NewP2, Precision.Confusion()))
        {
            throw new Standard_NullValue("FairCurve : P1 and P2 are confused");
        }
        Angles(P1, NewP2);
        NewP1 = P1;
    }

    /// <summary>Moves the second point (which changes the angles as well).</summary>
    /// <exception cref="Standard_NullValue">P1 and P2 are confused (within Precision::Confusion).</exception>
    public void SetP2(in gp_Pnt2d P2)
    {
        if (NewP1.IsEqual(P2, Precision.Confusion()))
        {
            throw new Standard_NullValue("FairCurve : P1 and P2 are confused");
        }
        Angles(NewP1, P2);
        NewP2 = P2;
    }

    /// <summary>Sets the angle at the first point (0 by default).</summary>
    public void SetAngle1(double Angle1)
    {
        NewAngle1 = Angle1;
    }

    /// <summary>Sets the angle at the second point (0 by default).</summary>
    public void SetAngle2(double Angle2)
    {
        NewAngle2 = Angle2;
    }

    /// <summary>Sets the height of the deformation.</summary>
    /// <exception cref="Standard_NegativeValue"><paramref name="Height"/> is not positive.</exception>
    public void SetHeight(double Height)
    {
        if (Height <= 0)
        {
            throw new Standard_NegativeValue("FairCurve : Height is no positive");
        }
        NewHeight = Height;
    }

    /// <summary>Sets the slope.</summary>
    public void SetSlope(double Slope)
    {
        NewSlope = Slope;
    }

    /// <summary>
    /// Sets the ratio of the batten's length to the reference length (1 by default): a larger value inflates the
    /// batten, a smaller one flattens it. Used when sliding is imposed.
    /// </summary>
    public void SetSlidingFactor(double SlidingFactor)
    {
        NewSlidingFactor = SlidingFactor;
    }

    /// <summary>
    /// Computes the curve with respect to the constraints. <paramref name="Code"/> is OK, NotConverged,
    /// InfiniteSliding or NullHeight; <paramref name="NbIterations"/> and <paramref name="Tolerance"/> control the
    /// precision and the time of the computation.
    /// </summary>
    public virtual bool Compute(out FairCurve_AnalysisCode Code, int NbIterations = 50, double Tolerance = 1.0e-3)
    {
        bool Ok = true, End = false;
        double AngleMax = 0.7;            // parameter ruling the function of increment ( 40 degrees )
        double AngleMin = 2 * M_PI / 100; // parameter ruling the function of increment
                                          // full passage should not cost more than 100 steps.
        double DAngle1, DAngle2, Ratio, Fraction, Toler;
        double OldDist, NewDist;
        // the loop runs at least once, and its first call writes the code (decision 860)
        Code = FairCurve_OK;

        //  Loop of Homotopy : calculation of the step and optimisation

        while (Ok && !End)
        {
            DAngle1 = NewAngle1 - OldAngle1;
            DAngle2 = NewAngle2 - OldAngle2;
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
                Toler = 10 * Tolerance;
            }

            Ok = Compute(DeltaP1, DeltaP2, DAngle1, DAngle2, out Code, NbIterations, Toler);

            if (Code != FairCurve_OK)
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
            if (NewConstraintOrder2 == 0)
            {
                NewAngle2 = OldAngle2;
            }
        }
        myCode = Code;
        return Ok;
    }

    /// <summary>Computes the curve with respect to the delta-constraints.</summary>
    private bool Compute(in gp_Vec2d DeltaP1,
                         in gp_Vec2d DeltaP2,
                         double DeltaAngle1,
                         double DeltaAngle2,
                         out FairCurve_AnalysisCode ACode,
                         int NbIterations,
                         double Tolerance)
    {
        bool Ok, OkCompute = true;
        ACode = FairCurve_OK;

        // Deformation of the curve by adding a polynom of interpolation
        int L = 2 + NewConstraintOrder1 + NewConstraintOrder2, kk, ii;
        NCollection_Array1<double> knots = new(1, 2);
        knots[1] = 0;
        knots[2] = 1;
        NCollection_Array1<int> mults = new(1, 2);
        NCollection_Array1<gp_Pnt2d> HermitePoles = new(1, L);
        NCollection_Array1<gp_Pnt2d> Interpolation = new(1, L);
        NCollection_HArray1<gp_Pnt2d> NPoles = new(1, Poles.Length());

        // Polynoms of Hermite
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

        ADelta[1] = DeltaP1.XY();
        kk = 2;
        if (NewConstraintOrder1 > 0)
        {
            gp_Vec2d OldDerive = new(Poles.Value(Poles.Lower()), Poles.Value(Poles.Lower() + 1));
            double aKnotGap1 = Knots.Value(2) - Knots.Value(1);
            if (Math.Abs(aKnotGap1) > Precision.Computational())
            {
                OldDerive *= Degree / aKnotGap1;
            }
            ADelta[kk] = (OldDerive.Rotated(DeltaAngle1 - DAngleRef) - OldDerive).XY();
            kk += 1;
        }
        ADelta[kk] = DeltaP2.XY();
        kk += 1;
        if (NewConstraintOrder2 > 0)
        {
            gp_Vec2d OldDerive = new(Poles.Value(Poles.Upper() - 1), Poles.Value(Poles.Upper()));
            double aKnotGap2 = Knots.Value(Knots.Upper()) - Knots.Value(Knots.Upper() - 1);
            if (Math.Abs(aKnotGap2) > Precision.Computational())
            {
                OldDerive *= Degree / aKnotGap2;
            }
            ADelta[kk] = (OldDerive.Rotated(DAngleRef - DeltaAngle2) - OldDerive).XY();
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

        // Intermediary data

        double Angle1, Angle2, SlidingLength,
          Alph1 = OldAngle1 + DeltaAngle1, Alph2 = OldAngle2 + DeltaAngle2,
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

        // Energy and vectors of initialisation
        FairCurve_BattenLaw LBatten = new(NewHeight, NewSlope, SlidingLength);
        FairCurve_EnergyOfBatten EBatten = new(Degree + 1,
                                               Flatknots,
                                               NPoles,
                                               NewConstraintOrder1,
                                               NewConstraintOrder2,
                                               LBatten,
                                               SlidingLength,
                                               NewFreeSliding,
                                               Angle1,
                                               Angle2);
        math_Vector VInit = new(1, EBatten.NbVariables());

        // The valeur below is the smallest value of the criterion of flexion.
        double VConvex = 0.01 * Math.Pow(NewHeight / SlidingLength, 3);
        if (VConvex < 1.0e-12)
        {
            VConvex = 1.0e-12;
        }

        Ok = EBatten.Variable(VInit);

        // Minimisation
        FairCurve_Newton Newton = new(EBatten,
                                      Tolerance * P1P2.Magnitude() / 10,
                                      Tolerance,
                                      NbIterations,
                                      VConvex);
        Newton.Perform(EBatten, VInit);
        Ok = Newton.IsDone();

        if (Ok)
        {
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

            if (NewConstraintOrder1 == 0)
            {
                gp_Vec2d Tangente = new(Poles.Value(Poles.Lower() + 1).Coord()
                                        - Poles.Value(Poles.Lower()).Coord());
                OldAngle1 = P1P2.Angle(Tangente);
            }
            else
            {
                OldAngle1 = Alph1;
            }

            if (NewConstraintOrder2 == 0)
            {
                gp_Vec2d Tangente = new(Poles.Value(Poles.Upper()).Coord()
                                        - Poles.Value(Poles.Upper() - 1).Coord());
                OldAngle2 = (-Tangente).Angle(-P1P2);
            }
            else
            {
                OldAngle2 = Alph2;
            }

            OldP1 = Poles.Value(Poles.Lower());
            OldP2 = Poles.Value(Poles.Upper());
            OldConstraintOrder1 = NewConstraintOrder1;
            OldConstraintOrder2 = NewConstraintOrder2;
            OldFreeSliding = NewFreeSliding;
            OldSlope = NewSlope;
            OldHeight = NewHeight;
        }
        else
        {
            double V;
            ACode = EBatten.Status();
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

        Ok = EBatten.Variable(VInit);

        // Processing of non-convergence
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

        // For eventual debug
        //  Newton.Dump(std::cout);

        return OkCompute;
    }

    /// <summary>
    /// The length of sliding of reference for the new constraints. To give a batten <c>b</c> a length L:
    /// <c>b.SetSlidingFactor(L / b.SlidingOfReference())</c>.
    /// </summary>
    public double SlidingOfReference()
    {
        return SlidingOfReference(NewP1.Distance(NewP2), NewAngle1, NewAngle2);
    }

    protected double SlidingOfReference(double Dist, double Angle1, double Angle2)
    {
        double a1, a2;

        // case of angle without constraints
        if ((NewConstraintOrder1 == 0) && (NewConstraintOrder2 == 0))
        {
            return Dist;
        }

        if (NewConstraintOrder1 == 0)
        {
            a1 = Math.Abs(Math.Abs(NewAngle2) < M_PI ? Angle2 / 2 : M_PI / 2);
        }
        else
        {
            a1 = Math.Abs(Angle1);
        }

        if (NewConstraintOrder2 == 0)
        {
            a2 = Math.Abs(Math.Abs(NewAngle1) < M_PI ? Angle1 / 2 : M_PI / 2);
        }
        else
        {
            a2 = Math.Abs(Angle2);
        }

        // case of angle of the same sign
        if (Angle1 * Angle2 >= 0)
        {
            return Compute(Dist, a1, a2);
        }

        // case of angle of opposite sign
        else
        {
            double aSum = a1 + a2;
            if (Math.Abs(aSum) < Precision.Angular())
            {
                return Compute(Dist, a1, M_PI / 2);
            }
            double Ratio = a1 / aSum;
            double AngleMilieu = Math.Pow(1 - Ratio, 2) * a1 + Math.Pow(Ratio, 2) * a2;
            if (AngleMilieu > M_PI / 2)
            {
                AngleMilieu = M_PI / 2;
            }

            return Ratio * Compute(Dist, a1, AngleMilieu) + (1 - Ratio) * Compute(Dist, a2, AngleMilieu);
        }
    }

    protected double Compute(double Dist, double Angle1, double Angle2)
    {
        double L1 = Compute(Dist, Angle1);
        double L2 = Compute(Dist, Angle2), Aux;
        if (L1 < L2)
        {
            Aux = L2;
            L2 = L1;
            L1 = Aux;
        }
        return 0.3 * L1 + 0.7 * L2;
    }

    /// <summary>
    /// The length of the batten for one end's angle: the segment, the circle arc through P1P2 with that angle, or an
    /// extrapolation beyond.
    /// </summary>
    protected double Compute(double Dist, double Angle)
    {
        if (Angle < Precision.Angular())
        {
            return Dist;
        } // length of segment P1P2
        if (Angle < M_PI / 2)
        {
            return Angle * Dist / Math.Sin(Angle);
        } // length of circle P1P2 respecting ANGLE
        if (Angle > M_PI)
        {
            return Math.Sqrt(Angle * M_PI) * Dist;
        }
        else
        {
            return Angle * Dist;
        } // linear interpolation
    }

    /// <summary>The free sliding of the latest computation (false by default).</summary>
    public bool GetFreeSliding()
    {
        return OldFreeSliding;
    }

    /// <summary>The first constraint order of the latest computation.</summary>
    public int GetConstraintOrder1()
    {
        return OldConstraintOrder1;
    }

    /// <summary>The second constraint order of the latest computation.</summary>
    public int GetConstraintOrder2()
    {
        return OldConstraintOrder2;
    }

    /// <summary>The first point of the latest computation.</summary>
    public gp_Pnt2d GetP1()
    {
        return OldP1;
    }

    /// <summary>The second point of the latest computation.</summary>
    public gp_Pnt2d GetP2()
    {
        return OldP2;
    }

    /// <summary>The first angle of the latest computation.</summary>
    public double GetAngle1()
    {
        return OldAngle1;
    }

    /// <summary>The second angle of the latest computation.</summary>
    public double GetAngle2()
    {
        return OldAngle2;
    }

    /// <summary>The height (thickness) of the batten of the latest computation.</summary>
    public double GetHeight()
    {
        return OldHeight;
    }

    /// <summary>The slope of the latest computation.</summary>
    public double GetSlope()
    {
        return OldSlope;
    }

    /// <summary>The sliding factor of the latest computation.</summary>
    public double GetSlidingFactor()
    {
        return OldSlidingFactor;
    }

    /// <summary>The computed curve: a new 2D B-spline, frozen (decision 859).</summary>
    public Geom2d_BSplineCurve Curve()
    {
        Geom2d_BSplineCurve TheCurve = new(Poles.Array1(), Knots.Array1(), Mults.Array1(), Degree);
        TheCurve.Freeze();
        return TheCurve;
    }

    /// <summary>Writes the current state (old and new constraints, the analysis code) to <paramref name="o"/>.</summary>
    public virtual void Dump(TextWriter o)
    {

        o.Write("  Batten       |");
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
        o.Write("      Height   |");
        DumpPair(o, OldHeight, NewHeight);
        o.Write("      Slope    |");
        DumpPair(o, OldSlope, NewSlope);
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

    /// <summary>OCCT's <c>operator&lt;&lt;</c>: <see cref="Dump"/> as a string.</summary>
    public override string ToString()
    {
        using StringWriter aWriter = new(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    // o.width(7) applies to the next item only (OCCT writes "old | new" lines with it)
    private protected static string Width7(string theText) => Standard_Stream.Setw(7, theText);

    private protected static void DumpPair(TextWriter o, double theOld, double theNew)
    {
        int aPrecision = Standard_Stream.Precision(o);
        o.Write(Width7(Standard_Stream.FormatReal(theOld, aPrecision)));
        o.Write(" | ");
        o.Write(Standard_Stream.FormatReal(theNew, aPrecision));
        o.Write('\n');
    }

    private protected static void DumpPair(TextWriter o, int theOld, int theNew)
    {
        o.Write(Width7(theOld.ToString(CultureInfo.InvariantCulture)));
        o.Write(" | ");
        o.Write(theNew.ToString(CultureInfo.InvariantCulture));
        o.Write('\n');
    }

    private protected void DumpCode(TextWriter o)
    {
        switch (myCode)
        {
            case FairCurve_OK:
                o.Write("AnalysisCode : Ok" + '\n');
                break;
            case FairCurve_NotConverged:
                o.Write("AnalysisCode : NotConverged" + '\n');
                break;
            case FairCurve_InfiniteSliding:
                o.Write("AnalysisCode : InfiniteSliding" + '\n');
                break;
            case FairCurve_NullHeight:
                o.Write("AnalysisCode : NullHeight" + '\n');
                break;
        }
    }

    private void Angles(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        gp_Vec2d VOld = new(NewP1, NewP2), VNew = new(P1, P2);
        double Dangle = VOld.Angle(VNew);
        NewAngle1 -= Dangle;
        NewAngle2 += Dangle;
    }
}
