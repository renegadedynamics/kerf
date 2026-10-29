// Port of: src/FoundationClasses/TKMath/Convert/Convert_ConicToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_ConicToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;

namespace Occt;

/// <summary>
/// Root class for the algorithms converting a 2D conic (gp_Circ2d, gp_Elips2d, gp_Hypr2d, gp_Parab2d) into a
/// B-spline curve: the degree, periodicity, poles with weights, and knots with multiplicities of the equivalent
/// B-spline curve, computed by the constructor of the derived class.
/// </summary>
/// <remarks>
/// The batch accessors (<see cref="Poles"/>, <see cref="Weights"/>, <see cref="Knots"/>,
/// <see cref="Multiplicities"/>) return copies, where OCCT returns references to its fields: the converter's
/// result can't be changed through them. The per-index accessors (<see cref="Pole"/>, ...) are deprecated in OCCT
/// in favour of the batch accessors.
/// </remarks>
public class Convert_ConicToBSplineCurve
{
    protected NCollection_Array1<gp_Pnt2d> myPoles = new();
    protected NCollection_Array1<double> myWeights = new();
    protected NCollection_Array1<double> myKnots = new();
    protected NCollection_Array1<int> myMults = new();
    protected int myDegree = 0;
    protected bool myIsPeriodic = false;

    protected Convert_ConicToBSplineCurve(int theNumberOfPoles, int theNumberOfKnots, int theDegree)
    {
        myDegree = theDegree;
        if (theNumberOfPoles >= 2)
        {
            myPoles = new NCollection_Array1<gp_Pnt2d>(1, theNumberOfPoles);
            myWeights = new NCollection_Array1<double>(1, theNumberOfPoles);
        }
        if (theNumberOfKnots >= 2)
        {
            myKnots = new NCollection_Array1<double>(1, theNumberOfKnots);
            myMults = new NCollection_Array1<int>(1, theNumberOfKnots);
        }
    }

    /// <summary>The degree of the B-spline curve.</summary>
    public int Degree()
    {
        return myDegree;
    }

    /// <summary>The number of poles of the B-spline curve.</summary>
    public int NbPoles()
    {
        return myPoles.Length();
    }

    /// <summary>The number of knots of the B-spline curve.</summary>
    public int NbKnots()
    {
        return myKnots.Length();
    }

    /// <summary>True if the B-spline curve is periodic.</summary>
    public bool IsPeriodic()
    {
        return myIsPeriodic;
    }

    /// <summary>The pole of index <paramref name="theIndex"/> (from 1). Deprecated in OCCT: use <see cref="Poles"/>.</summary>
    public gp_Pnt2d Pole(int theIndex)
    {
        if (theIndex < 1 || theIndex > myPoles.Length())
        {
            throw new Standard_OutOfRange("Convert_ConicToBSplineCurve::Pole: Index out of range");
        }
        return myPoles[theIndex];
    }

    /// <summary>The weight of the pole of index <paramref name="theIndex"/>. Deprecated in OCCT: use <see cref="Weights"/>.</summary>
    public double Weight(int theIndex)
    {
        if (theIndex < 1 || theIndex > myPoles.Length())
        {
            throw new Standard_OutOfRange("Convert_ConicToBSplineCurve::Weight: Index out of range");
        }
        return myWeights[theIndex];
    }

    /// <summary>The knot of index <paramref name="theIndex"/>. Deprecated in OCCT: use <see cref="Knots"/>.</summary>
    public double Knot(int theIndex)
    {
        if (theIndex < 1 || theIndex > myKnots.Length())
        {
            throw new Standard_OutOfRange("Convert_ConicToBSplineCurve::Knot: Index out of range");
        }
        return myKnots[theIndex];
    }

    /// <summary>The multiplicity of the knot of index <paramref name="theIndex"/>. Deprecated in OCCT: use <see cref="Multiplicities"/>.</summary>
    public int Multiplicity(int theIndex)
    {
        if (theIndex < 1 || theIndex > myKnots.Length())
        {
            throw new Standard_OutOfRange("Convert_ConicToBSplineCurve::Multiplicity: Index out of range");
        }
        return myMults[theIndex];
    }

    /// <summary>The poles of the B-spline curve: a copy.</summary>
    public NCollection_Array1<gp_Pnt2d> Poles()
    {
        return new NCollection_Array1<gp_Pnt2d>(myPoles);
    }

    /// <summary>The weights of the B-spline curve: a copy.</summary>
    public NCollection_Array1<double> Weights()
    {
        return new NCollection_Array1<double>(myWeights);
    }

    /// <summary>The knots of the B-spline curve: a copy.</summary>
    public NCollection_Array1<double> Knots()
    {
        return new NCollection_Array1<double>(myKnots);
    }

    /// <summary>The multiplicities of the knots: a copy.</summary>
    public NCollection_Array1<int> Multiplicities()
    {
        return new NCollection_Array1<int>(myMults);
    }

    /// <summary>
    /// The periodic B-spline representation of cos and sin over [0, 2 Pi] (handle form). Deprecated in OCCT: use
    /// the array form.
    /// </summary>
    public void BuildCosAndSin(Convert_ParameterisationType theParametrisation,
                               out NCollection_HArray1<double> theCosNumerator,
                               out NCollection_HArray1<double> theSinNumerator,
                               out NCollection_HArray1<double> theDenominator,
                               out int theDegree,
                               out NCollection_HArray1<double> theKnots,
                               out NCollection_HArray1<int> theMults)
    {
        BuildCosAndSin(theParametrisation,
                       out NCollection_Array1<double> aCosNumerator,
                       out NCollection_Array1<double> aSinNumerator,
                       out NCollection_Array1<double> aDenominator,
                       out theDegree,
                       out NCollection_Array1<double> aKnots,
                       out NCollection_Array1<int> aMults);
        theCosNumerator = new NCollection_HArray1<double>(aCosNumerator);
        theSinNumerator = new NCollection_HArray1<double>(aSinNumerator);
        theDenominator = new NCollection_HArray1<double>(aDenominator);
        theKnots = new NCollection_HArray1<double>(aKnots);
        theMults = new NCollection_HArray1<int>(aMults);
    }

    /// <summary>
    /// The B-spline representation of cos and sin over [<paramref name="theUFirst"/>, <paramref name="theULast"/>]
    /// (handle form). Deprecated in OCCT: use the array form.
    /// </summary>
    public void BuildCosAndSin(Convert_ParameterisationType theParametrisation,
                               double theUFirst,
                               double theULast,
                               out NCollection_HArray1<double> theCosNumerator,
                               out NCollection_HArray1<double> theSinNumerator,
                               out NCollection_HArray1<double> theDenominator,
                               out int theDegree,
                               out NCollection_HArray1<double> theKnots,
                               out NCollection_HArray1<int> theMults)
    {
        BuildCosAndSin(theParametrisation,
                       theUFirst,
                       theULast,
                       out NCollection_Array1<double> aCosNumerator,
                       out NCollection_Array1<double> aSinNumerator,
                       out NCollection_Array1<double> aDenominator,
                       out theDegree,
                       out NCollection_Array1<double> aKnots,
                       out NCollection_Array1<int> aMults);
        theCosNumerator = new NCollection_HArray1<double>(aCosNumerator);
        theSinNumerator = new NCollection_HArray1<double>(aSinNumerator);
        theDenominator = new NCollection_HArray1<double>(aDenominator);
        theKnots = new NCollection_HArray1<double>(aKnots);
        theMults = new NCollection_HArray1<int>(aMults);
    }

    //=======================================================================
    // function : CosAndSinRationalC1
    // purpose  : evaluates U(t) and V(t) such that
    //                      2      2
    //                     U   -  V
    //   cos (theta(t)) = ----------
    //                      2      2
    //                     U   +  V
    //
    //                      2 * U*V
    //   sin (theta(t)) = ----------
    //                      2      2
    //                     U   +  V
    //                                                    2     2
    //  such that the derivative at the domain bounds of U   + V   is 0.0e0
    //  with is helpful when having to make a C1 BSpline by merging two BSpline together
    //=======================================================================
    private static void CosAndSinRationalC1(double Parameter,
                                            int EvalDegree,
                                            NCollection_Array1<gp_Pnt2d> EvalPoles,
                                            NCollection_Array1<double> EvalKnots,
                                            NCollection_Array1<int>? EvalMults,
                                            Span<double> Result)
    {
        BSplCLib.D0(Parameter,
                    0,
                    EvalDegree,
                    false,
                    EvalPoles,
                    BSplCLib.NoWeights(),
                    EvalKnots,
                    EvalMults,
                    out gp_Pnt2d a_point);
        Result[0] = a_point.Coord(1);
        Result[1] = a_point.Coord(2);
    }

    //=======================================================================
    // function : CosAndSinQuasiAngular
    // purpose  : evaluates U(t) and V(t) such that
    //                      2      2
    //                     U   -  V
    //   cos (theta(t)) = ----------
    //                      2      2
    //                     U   +  V
    //
    //                      2 * U*V
    //   sin (theta(t)) = ----------
    //                      2      2
    //                     U   +  V
    //=======================================================================
    private static void CosAndSinQuasiAngular(double Parameter,
                                              int EvalDegree,
                                              NCollection_Array1<gp_Pnt2d> EvalPoles,
                                              NCollection_Array1<double> EvalKnots,
                                              NCollection_Array1<int>? EvalMults,
                                              Span<double> Result)
    {
        _ = EvalKnots;
        _ = EvalMults;
        // Extract X,Y coordinates from all EvalPoles into a flat double array
        // to safely pass to PLib::NoDerivativeEvalPolynomial.
        Standard_OutOfRange_Raise_if(EvalPoles.Length() != EvalDegree + 1,
                                     "CosAndSinQuasiAngular: EvalPoles size mismatch");
        int aNumCoords = (EvalDegree + 1) * 2;
        NCollection_Array1<double> aCoeffs = new(0, aNumCoords - 1);
        for (int i = EvalPoles.Lower(); i <= EvalPoles.Upper(); i++)
        {
            int anIdx = (i - EvalPoles.Lower()) * 2;
            aCoeffs[anIdx] = EvalPoles[i].X();
            aCoeffs[anIdx + 1] = EvalPoles[i].Y();
        }
        //
        //   rational_function_coeff represent a rational approximation
        //   of U ---> cotan( PI * U /2) between [0 1]
        //   rational_function_coeff[i][0] is the denominator
        //   rational_function_coeff[i][1] is the numerator
        //
        double param = Parameter * 0.5e0;
        PLib.NoDerivativeEvalPolynomial(param, EvalDegree, 2, EvalDegree << 1, aCoeffs.AsSpan(), Result);
    }

    //=======================================================================
    // function : AlgorithmicCosAndSin
    // purpose  : Build the Bspline representation of
    //            an algorithmic description of the function cos and sin
    //=======================================================================
    private static void AlgorithmicCosAndSin(int Degree,
                                             NCollection_Array1<double> FlatKnots,
                                             int EvalDegree,
                                             NCollection_Array1<gp_Pnt2d> EvalPoles,
                                             NCollection_Array1<double> EvalKnots,
                                             NCollection_Array1<int>? EvalMults,
                                             Convert_CosAndSinEvalFunction Evaluator,
                                             NCollection_Array1<double> CosNumerator,
                                             NCollection_Array1<double> SinNumerator,
                                             NCollection_Array1<double> Denominator)
    {
        int order, num_poles, ii;

        Span<double> result = stackalloc double[2];
        double inverse;

        order = Degree + 1;
        num_poles = FlatKnots.Length() - order;

        if (num_poles != CosNumerator.Length() || num_poles != SinNumerator.Length()
            || num_poles != Denominator.Length())
        {
            throw new Standard_ConstructionError();
        }
        NCollection_Array1<double> parameters = new(1, num_poles);
        NCollection_Array1<gp_Pnt> poles_array = new(1, num_poles);
        NCollection_Array1<int> contact_order_array = new(1, num_poles);
        BSplCLib.BuildSchoenbergPoints(Degree, FlatKnots, parameters);
        for (ii = parameters.Lower(); ii <= parameters.Upper(); ii++)
        {
            Evaluator(parameters[ii], EvalDegree, EvalPoles, EvalKnots, EvalMults, result);
            contact_order_array[ii] = 0;

            poles_array[ii].SetCoord(1, (result[1] * result[1] - result[0] * result[0]));
            poles_array[ii].SetCoord(2, 2.0e0 * result[1] * result[0]);
            poles_array[ii].SetCoord(3, result[1] * result[1] + result[0] * result[0]);
        }
        BSplCLib.Interpolate(Degree,
                             FlatKnots,
                             parameters,
                             contact_order_array,
                             poles_array,
                             out int _);
        for (ii = 1; ii <= num_poles; ii++)
        {
            inverse = 1.0e0 / poles_array[ii].Coord(3);
            CosNumerator[ii] = poles_array[ii].Coord(1) * inverse;
            SinNumerator[ii] = poles_array[ii].Coord(2) * inverse;
            Denominator[ii] = poles_array[ii].Coord(3);
        }
    }

    /// <summary>
    /// The B-spline representation of cos and sin over [<paramref name="UFirst"/>, <paramref name="ULast"/>] with
    /// the given parameterisation: cos = CosNumerator / Denominator and sin = SinNumerator / Denominator, the
    /// numerators being given as poles of the rational B-spline (already divided by the weights).
    /// </summary>
    /// <remarks>The arrays are outputs, where OCCT assigns new arrays to its reference parameters.</remarks>
    protected void BuildCosAndSin(Convert_ParameterisationType Parameterisation,
                                  double UFirst,
                                  double ULast,
                                  out NCollection_Array1<double> CosNumerator,
                                  out NCollection_Array1<double> SinNumerator,
                                  out NCollection_Array1<double> Denominator,
                                  out int Degree,
                                  out NCollection_Array1<double> Knots,
                                  out NCollection_Array1<int> Mults)
    {
        double delta = ULast - UFirst, direct, inverse, value1, value2, cos_beta, sin_beta, alpha = 0,
               alpha_2, alpha_4, tan_alpha_2, beta, p_param, q_param, param;

        int num_poles = 0, ii, jj, num_knots = 1, num_spans = 1, num_flat_knots, num_temp_knots,
            temp_degree = 0, tgt_theta_flag, num_temp_poles, order = 0;

        Convert_CosAndSinEvalFunction? EvaluatorPtr = null;

        tgt_theta_flag = 0;
        // Always assigned (decision 82), where OCCT leaves Degree unchanged for a parameterisation outside the enumeration
        Degree = 0;

        switch (Parameterisation)
        {
            case Convert_TgtThetaOver2:
                num_spans = (int)Math.Truncate(1.2 * delta / M_PI) + 1;

                tgt_theta_flag = 1;
                break;
            case Convert_TgtThetaOver2_1:
                num_spans = 1;
                if (delta > 0.9999 * M_PI)
                {
                    throw new Standard_ConstructionError();
                }
                tgt_theta_flag = 1;
                break;
            case Convert_TgtThetaOver2_2:
                num_spans = 2;
                if (delta > 1.9999 * M_PI)
                {
                    throw new Standard_ConstructionError();
                }
                tgt_theta_flag = 1;
                break;

            case Convert_TgtThetaOver2_3:
                num_spans = 3;
                tgt_theta_flag = 1;
                break;
            case Convert_TgtThetaOver2_4:
                num_spans = 4;
                tgt_theta_flag = 1;
                break;
            case Convert_QuasiAngular:
                num_poles = 7;
                Degree = 6;
                num_spans = 1;
                num_knots = 2;
                order = Degree + 1;
                break;
            case Convert_RationalC1:
                Degree = 4;
                order = Degree + 1;
                num_poles = 8;
                num_knots = 3;
                num_spans = 2;
                break;
            case Convert_Polynomial:
                Degree = 7;
                num_poles = 8;
                num_knots = 2;
                num_spans = 1;
                break;
            default:
                break;
        }
        if (tgt_theta_flag != 0)
        {
            alpha = delta / (2.0e0 * num_spans);
            Degree = 2;
            num_poles = 2 * num_spans + 1;
        }

        CosNumerator = new NCollection_Array1<double>(1, num_poles);
        SinNumerator = new NCollection_Array1<double>(1, num_poles);
        Denominator = new NCollection_Array1<double>(1, num_poles);
        Knots = new NCollection_Array1<double>(1, num_spans + 1);
        Mults = new NCollection_Array1<int>(1, num_spans + 1);
        if (tgt_theta_flag != 0)
        {
            param = UFirst;
            CosNumerator[1] = Math.Cos(UFirst);
            SinNumerator[1] = Math.Sin(UFirst);
            Denominator[1] = 1.0e0;
            Knots[1] = param;
            Mults[1] = Degree + 1;
            direct = Math.Cos(alpha);
            inverse = 1.0e0 / direct;
            for (ii = 1; ii <= num_spans; ii++)
            {
                CosNumerator[2 * ii] = inverse * Math.Cos(param + alpha);
                SinNumerator[2 * ii] = inverse * Math.Sin(param + alpha);
                Denominator[2 * ii] = direct;
                CosNumerator[2 * ii + 1] = Math.Cos(param + 2 * alpha);
                SinNumerator[2 * ii + 1] = Math.Sin(param + 2 * alpha);
                Denominator[2 * ii + 1] = 1.0e0;
                Knots[ii + 1] = param + 2 * alpha;
                Mults[ii + 1] = 2;
                param += 2 * alpha;
            }
            Mults[num_spans + 1] = Degree + 1;
        }
        else if (Parameterisation != Convert_Polynomial)
        {
            alpha = ULast - UFirst;
            alpha *= 0.5e0;
            beta = ULast + UFirst;
            beta *= 0.5e0;
            cos_beta = Math.Cos(beta);
            sin_beta = Math.Sin(beta);
            num_flat_knots = num_poles + order;

            num_temp_poles = 4;
            num_temp_knots = 3;
            NCollection_Array1<double> flat_knots = new(1, num_flat_knots);

            NCollection_Array1<gp_Pnt2d> temp_poles = new(1, num_temp_poles);
            NCollection_Array1<double> temp_knots = new(1, num_temp_knots);
            NCollection_Array1<int> temp_mults = new(1, num_temp_knots);

            for (ii = 1; ii <= order; ii++)
            {
                flat_knots[ii] = -alpha;
                flat_knots[ii + num_poles] = alpha;
            }
            Knots[1] = UFirst;
            Knots[num_knots] = ULast;
            Mults[1] = order;
            Mults[num_knots] = order;

            switch (Parameterisation)
            {
                case Convert_QuasiAngular:
                    //
                    //    we code here in temp_poles(xx).Coord(1) the following function V(t)
                    //   and in temp_poles(xx).Coord(2) the function U(t)
                    //                     3
                    //       V(t) = t + c t
                    //                     2
                    //       U(t) = 1 + b t
                    //            1
                    //       c = ---  + b   = q_param
                    //            3
                    //                          3
                    //                     gamma
                    //            gamma +  ------  - tang gamma
                    //                      3
                    //       b =------------------------------    = p_param
                    //                 2
                    //            gamma  (tang gamma - gamma)
                    //
                    //     with gamma = alpha / 2
                    //
                    //

                    alpha_2 = alpha * 0.5e0;
                    p_param = -1.0e0 / (alpha_2 * alpha_2);

                    if (alpha_2 < M_PI * 0.5e0)
                    {
                        if (alpha_2 < 1.0e-7)
                        {
                            // Fixed degenerate case, when obtain 0 / 0 uncertainty.
                            // According to Taylor approximation:
                            // b (gamma) = -6.0 / 15.0 + o(gamma^2)
                            p_param = -6.0 / 15.0;
                        }
                        else
                        {
                            tan_alpha_2 = Math.Tan(alpha_2);
                            value1 = 3.0e0 * (tan_alpha_2 - alpha_2);
                            value1 = alpha_2 / value1;
                            p_param += value1;
                        }
                    }
                    q_param = (1.0e0 / 3.0e0) + p_param;

                    temp_degree = 3;
                    temp_poles[1].SetCoord(1, 0.0e0);
                    temp_poles[2].SetCoord(1, 1.0e0);
                    temp_poles[3].SetCoord(1, 0.0e0);
                    temp_poles[4].SetCoord(1, q_param);

                    temp_poles[1].SetCoord(2, 1.0e0);
                    temp_poles[2].SetCoord(2, 0.0e0);
                    temp_poles[3].SetCoord(2, p_param);
                    temp_poles[4].SetCoord(2, 0.0e0);
                    EvaluatorPtr = CosAndSinQuasiAngular;
                    break;
                case Convert_RationalC1:
                    for (ii = order + 1; ii <= num_poles; ii++)
                    {
                        flat_knots[ii] = 0.0e0;
                    }
                    Knots[2] = UFirst + alpha;
                    Mults[2] = Degree - 1;
                    temp_degree = 2;
                    alpha_2 = alpha * 0.5e0;
                    alpha_4 = alpha * 0.25e0;
                    tan_alpha_2 = Math.Tan(alpha_2);
                    jj = 1;
                    for (ii = 1; ii <= 2; ii++)
                    {
                        temp_poles[1 + ii].SetCoord(2, 1.0e0 + alpha_4 * tan_alpha_2);
                        temp_poles[jj].SetCoord(2, 1.0e0);
                        jj += 3;
                    }
                    temp_poles[1].SetCoord(1, -tan_alpha_2);
                    temp_poles[2].SetCoord(1, alpha_4 - tan_alpha_2);
                    temp_poles[3].SetCoord(1, -alpha_4 + tan_alpha_2);
                    temp_poles[4].SetCoord(1, tan_alpha_2);
                    temp_knots[1] = -alpha;
                    temp_knots[2] = 0.0e0;
                    temp_knots[3] = alpha;
                    temp_mults[1] = temp_degree + 1;
                    temp_mults[2] = 1;
                    temp_mults[3] = temp_degree + 1;

                    EvaluatorPtr = CosAndSinRationalC1;
                    break;
                default:
                    break;
            }
            // OCCT calls through a null function pointer for a parameterisation outside the enumeration
            AlgorithmicCosAndSin(Degree,
                                 flat_knots,
                                 temp_degree,
                                 temp_poles,
                                 temp_knots,
                                 temp_mults,
                                 EvaluatorPtr!,
                                 CosNumerator,
                                 SinNumerator,
                                 Denominator);

            for (ii = 1; ii <= num_poles; ii++)
            {
                value1 = cos_beta * CosNumerator[ii] - sin_beta * SinNumerator[ii];
                value2 = sin_beta * CosNumerator[ii] + cos_beta * SinNumerator[ii];
                CosNumerator[ii] = value1;
                SinNumerator[ii] = value2;
            }
        }
        else
        { // Convert_Polynomial

            Knots[1] = 0.0;
            Knots[num_knots] = 1.0;
            Mults[1] = num_poles;
            Mults[num_knots] = num_poles;

            Convert_PolynomialCosAndSin.BuildPolynomialCosAndSin(UFirst, ULast, num_poles, CosNumerator, SinNumerator, Denominator);
        }
    }

    /// <summary>
    /// The periodic B-spline representation of cos and sin over [0, 2 Pi], for <c>Convert_TgtThetaOver2</c> and
    /// <c>Convert_RationalC1</c> only (<see cref="Standard_ConstructionError"/> otherwise).
    /// </summary>
    /// <remarks>The arrays are outputs, where OCCT assigns new arrays to its reference parameters.</remarks>
    protected void BuildCosAndSin(Convert_ParameterisationType Parameterisation,
                                  out NCollection_Array1<double> CosNumerator,
                                  out NCollection_Array1<double> SinNumerator,
                                  out NCollection_Array1<double> Denominator,
                                  out int Degree,
                                  out NCollection_Array1<double> Knots,
                                  out NCollection_Array1<int> Mults)
    {
        double half_pi, param, first_param, last_param, inverse, value1, value2, value3;

        int ii, jj, index, num_poles, num_periodic_poles, temp_degree,
          num_flat_knots, num_knots;

        if (Parameterisation != Convert_TgtThetaOver2 && Parameterisation != Convert_RationalC1)
        {
            throw new Standard_ConstructionError();
        }
        NCollection_Array1<double> temp_cos, temp_sin, temp_denominator, temp_knots;
        NCollection_Array1<int> temp_mults;
        if (Parameterisation == Convert_TgtThetaOver2)
        {
            BuildCosAndSin(Convert_TgtThetaOver2_3,
                           0.0e0,
                           2 * M_PI,
                           out temp_cos,
                           out temp_sin,
                           out temp_denominator,
                           out Degree,
                           out Knots,
                           out Mults);
            CosNumerator = new NCollection_Array1<double>(1, temp_cos.Length() - 1);
            SinNumerator = new NCollection_Array1<double>(1, temp_cos.Length() - 1);
            Denominator = new NCollection_Array1<double>(1, temp_cos.Length() - 1);
            for (ii = temp_cos.Lower(); ii <= temp_cos.Upper() - 1; ii++)
            {
                CosNumerator[ii] = temp_cos[ii];
                SinNumerator[ii] = temp_sin[ii];
                Denominator[ii] = temp_denominator[ii];
            }
            for (ii = Mults.Lower(); ii <= Mults.Upper(); ii++)
            {
                Mults[ii] = Degree;
            }
        }
        else
        {
            // Parameterisation == Convert_RationalC1 (the C++ tests it again)
            first_param = 0.0e0;
            last_param = M_PI;
            BuildCosAndSin(Convert_RationalC1,
                           first_param,
                           last_param,
                           out temp_cos,
                           out temp_sin,
                           out temp_denominator,
                           out temp_degree,
                           out temp_knots,
                           out temp_mults);

            Degree = 4;
            num_knots = 5;
            num_flat_knots = (Degree - 1) * num_knots + 2 * 2;
            num_poles = num_flat_knots - Degree - 1;
            num_periodic_poles = num_poles - 2;
            NCollection_Array1<double> flat_knots = new(1, num_flat_knots);
            CosNumerator = new NCollection_Array1<double>(1, num_periodic_poles);
            SinNumerator = new NCollection_Array1<double>(1, num_periodic_poles);
            Denominator = new NCollection_Array1<double>(1, num_periodic_poles);

            half_pi = M_PI * 0.5e0;
            index = 1;
            for (jj = 1; jj <= 2; jj++)
            {
                flat_knots[index] = -half_pi;
                index += 1;
            }
            for (ii = 1; ii <= num_knots; ii++)
            {
                for (jj = 1; jj <= Degree - 1; jj++)
                {
                    flat_knots[index] = (ii - 1) * half_pi;

                    index += 1;
                }
            }
            for (jj = 1; jj <= 2; jj++)
            {
                flat_knots[index] = 2 * M_PI + half_pi;
                index += 1;
            }
            Knots = new NCollection_Array1<double>(1, num_knots);
            Mults = new NCollection_Array1<int>(1, num_knots);
            for (ii = 1; ii <= num_knots; ii++)
            {
                Knots[ii] = (ii - 1) * half_pi;
                Mults[ii] = Degree - 1;
            }

            NCollection_Array1<double> parameters = new(1, num_poles);
            NCollection_Array1<gp_Pnt> poles_array = new(1, num_poles);
            NCollection_Array1<int> contact_order_array = new(1, num_poles);
            BSplCLib.BuildSchoenbergPoints(Degree, flat_knots, parameters);
            inverse = 1.0e0;
            for (ii = parameters.Lower(); ii <= parameters.Upper(); ii++)
            {
                param = parameters[ii];
                if (param > M_PI)
                {
                    inverse = -1.0e0;
                    param -= M_PI;
                }
                BSplCLib.D0(param,
                            0,
                            temp_degree,
                            false,
                            temp_cos,
                            temp_denominator,
                            temp_knots,
                            temp_mults,
                            out value1);

                BSplCLib.D0(param,
                            0,
                            temp_degree,
                            false,
                            temp_sin,
                            temp_denominator,
                            temp_knots,
                            temp_mults,
                            out value2);
                BSplCLib.D0(param,
                            0,
                            temp_degree,
                            false,
                            temp_denominator,
                            BSplCLib.NoWeights(),
                            temp_knots,
                            temp_mults,
                            out value3);
                contact_order_array[ii] = 0;

                poles_array[ii].SetCoord(1, value1 * value3 * inverse);
                poles_array[ii].SetCoord(2, value2 * value3 * inverse);
                poles_array[ii].SetCoord(3, value3);
            }
            BSplCLib.Interpolate(Degree,
                                 flat_knots,
                                 parameters,
                                 contact_order_array,
                                 poles_array,
                                 out int _);
            for (ii = 1; ii <= num_periodic_poles; ii++)
            {
                inverse = 1.0e0 / poles_array[ii].Coord(3);
                CosNumerator[ii] = poles_array[ii].Coord(1) * inverse;
                SinNumerator[ii] = poles_array[ii].Coord(2) * inverse;
                Denominator[ii] = poles_array[ii].Coord(3);
            }
        }
    }
}
