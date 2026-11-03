// Port of: src/FoundationClasses/TKMath/Convert/Convert_ConicToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_ConicToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Kerf.Engine.Geometry.Conversion;

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
public class ConicToBSplineCurve
{
    protected NCollection_Array1<Point2D> _poles = new();
    protected NCollection_Array1<double> _weights = new();
    protected NCollection_Array1<double> _knots = new();
    protected NCollection_Array1<int> _mults = new();
    protected int _degree = 0;
    protected bool _isPeriodic = false;

    protected ConicToBSplineCurve(int numberOfPoles, int numberOfKnots, int degree)
    {
        _degree = degree;
        if (numberOfPoles >= 2)
        {
            _poles = new NCollection_Array1<Point2D>(1, numberOfPoles);
            _weights = new NCollection_Array1<double>(1, numberOfPoles);
        }
        if (numberOfKnots >= 2)
        {
            _knots = new NCollection_Array1<double>(1, numberOfKnots);
            _mults = new NCollection_Array1<int>(1, numberOfKnots);
        }
    }

    /// <summary>The degree of the B-spline curve.</summary>
    public int Degree()
    {
        return _degree;
    }

    /// <summary>The number of poles of the B-spline curve.</summary>
    public int PoleCount
    {
        get
        {
            return _poles.Length();
        }
    }

    /// <summary>The number of knots of the B-spline curve.</summary>
    public int KnotCount
    {
        get
        {
            return _knots.Length();
        }
    }

    /// <summary>True if the B-spline curve is periodic.</summary>
    public bool IsPeriodic
    {
        get
        {
            return _isPeriodic;
        }
    }

    /// <summary>The pole of index <paramref name="index"/> (from 1). Deprecated in OCCT: use <see cref="Poles"/>.</summary>
    public Point2D Pole(int index)
    {
        if (index < 1 || index > _poles.Length())
        {
            throw new OutOfRangeException("Convert_ConicToBSplineCurve::Pole: Index out of range");
        }
        return _poles[index];
    }

    /// <summary>The weight of the pole of index <paramref name="index"/>. Deprecated in OCCT: use <see cref="Weights"/>.</summary>
    public double Weight(int index)
    {
        if (index < 1 || index > _poles.Length())
        {
            throw new OutOfRangeException("Convert_ConicToBSplineCurve::Weight: Index out of range");
        }
        return _weights[index];
    }

    /// <summary>The knot of index <paramref name="index"/>. Deprecated in OCCT: use <see cref="Knots"/>.</summary>
    public double Knot(int index)
    {
        if (index < 1 || index > _knots.Length())
        {
            throw new OutOfRangeException("Convert_ConicToBSplineCurve::Knot: Index out of range");
        }
        return _knots[index];
    }

    /// <summary>The multiplicity of the knot of index <paramref name="index"/>. Deprecated in OCCT: use <see cref="Multiplicities"/>.</summary>
    public int Multiplicity(int index)
    {
        if (index < 1 || index > _knots.Length())
        {
            throw new OutOfRangeException("Convert_ConicToBSplineCurve::Multiplicity: Index out of range");
        }
        return _mults[index];
    }

    /// <summary>The poles of the B-spline curve: a copy.</summary>
    public NCollection_Array1<Point2D> Poles()
    {
        return new NCollection_Array1<Point2D>(_poles);
    }

    /// <summary>The weights of the B-spline curve: a copy.</summary>
    public NCollection_Array1<double> Weights()
    {
        return new NCollection_Array1<double>(_weights);
    }

    /// <summary>The knots of the B-spline curve: a copy.</summary>
    public NCollection_Array1<double> Knots()
    {
        return new NCollection_Array1<double>(_knots);
    }

    /// <summary>The multiplicities of the knots: a copy.</summary>
    public NCollection_Array1<int> Multiplicities()
    {
        return new NCollection_Array1<int>(_mults);
    }

    /// <summary>
    /// The periodic B-spline representation of cos and sin over [0, 2 Pi] (handle form). Deprecated in OCCT: use
    /// the array form.
    /// </summary>
    public void BuildCosAndSin(ParameterisationType parametrisation,
                               out NCollection_HArray1<double> cosNumerator,
                               out NCollection_HArray1<double> sinNumerator,
                               out NCollection_HArray1<double> denominator,
                               out int degree,
                               out NCollection_HArray1<double> knots,
                               out NCollection_HArray1<int> mults)
    {
        BuildCosAndSin(parametrisation,
                       out NCollection_Array1<double> aCosNumerator,
                       out NCollection_Array1<double> aSinNumerator,
                       out NCollection_Array1<double> aDenominator,
                       out degree,
                       out NCollection_Array1<double> aKnots,
                       out NCollection_Array1<int> aMults);
        cosNumerator = new NCollection_HArray1<double>(aCosNumerator);
        sinNumerator = new NCollection_HArray1<double>(aSinNumerator);
        denominator = new NCollection_HArray1<double>(aDenominator);
        knots = new NCollection_HArray1<double>(aKnots);
        mults = new NCollection_HArray1<int>(aMults);
    }

    /// <summary>
    /// The B-spline representation of cos and sin over [<paramref name="uFirst"/>, <paramref name="uLast"/>]
    /// (handle form). Deprecated in OCCT: use the array form.
    /// </summary>
    public void BuildCosAndSin(ParameterisationType parametrisation,
                               double uFirst,
                               double uLast,
                               out NCollection_HArray1<double> cosNumerator,
                               out NCollection_HArray1<double> sinNumerator,
                               out NCollection_HArray1<double> denominator,
                               out int degree,
                               out NCollection_HArray1<double> knots,
                               out NCollection_HArray1<int> mults)
    {
        BuildCosAndSin(parametrisation,
                       uFirst,
                       uLast,
                       out NCollection_Array1<double> aCosNumerator,
                       out NCollection_Array1<double> aSinNumerator,
                       out NCollection_Array1<double> aDenominator,
                       out degree,
                       out NCollection_Array1<double> aKnots,
                       out NCollection_Array1<int> aMults);
        cosNumerator = new NCollection_HArray1<double>(aCosNumerator);
        sinNumerator = new NCollection_HArray1<double>(aSinNumerator);
        denominator = new NCollection_HArray1<double>(aDenominator);
        knots = new NCollection_HArray1<double>(aKnots);
        mults = new NCollection_HArray1<int>(aMults);
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
    private static void CosAndSinRationalC1(double parameter,
                                            int evalDegree,
                                            NCollection_Array1<Point2D> evalPoles,
                                            NCollection_Array1<double> evalKnots,
                                            NCollection_Array1<int>? evalMults,
                                            Span<double> result)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(parameter,
                    0,
                    evalDegree,
                    false,
                    evalPoles,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                    evalKnots,
                    evalMults,
                    out Point2D a_point);
        result[0] = a_point.Coord(1);
        result[1] = a_point.Coord(2);
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
    private static void CosAndSinQuasiAngular(double parameter,
                                              int evalDegree,
                                              NCollection_Array1<Point2D> evalPoles,
                                              NCollection_Array1<double> evalKnots,
                                              NCollection_Array1<int>? evalMults,
                                              Span<double> result)
    {
        _ = evalKnots;
        _ = evalMults;
        // Extract X,Y coordinates from all EvalPoles into a flat double array
        // to safely pass to PLib::NoDerivativeEvalPolynomial.
        Standard_OutOfRange_Raise_if(evalPoles.Length() != evalDegree + 1,
                                     "CosAndSinQuasiAngular: EvalPoles size mismatch");
        int numCoords = (evalDegree + 1) * 2;
        NCollection_Array1<double> coeffs = new(0, numCoords - 1);
        for (int i = evalPoles.Lower(); i <= evalPoles.Upper(); i++)
        {
            int idx = (i - evalPoles.Lower()) * 2;
            coeffs[idx] = evalPoles[i].X();
            coeffs[idx + 1] = evalPoles[i].Y();
        }
        //
        //   rational_function_coeff represent a rational approximation
        //   of U ---> cotan( PI * U /2) between [0 1]
        //   rational_function_coeff[i][0] is the denominator
        //   rational_function_coeff[i][1] is the numerator
        //
        double param = parameter * 0.5e0;
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(param, evalDegree, 2, evalDegree << 1, coeffs.AsSpan(), result);
    }

    //=======================================================================
    // function : AlgorithmicCosAndSin
    // purpose  : Build the Bspline representation of
    //            an algorithmic description of the function cos and sin
    //=======================================================================
    private static void AlgorithmicCosAndSin(int degree,
                                             NCollection_Array1<double> flatKnots,
                                             int evalDegree,
                                             NCollection_Array1<Point2D> evalPoles,
                                             NCollection_Array1<double> evalKnots,
                                             NCollection_Array1<int>? evalMults,
                                             CosAndSinEvalFunction evaluator,
                                             NCollection_Array1<double> cosNumerator,
                                             NCollection_Array1<double> sinNumerator,
                                             NCollection_Array1<double> denominator)
    {
        int order, num_poles, ii;

        Span<double> result = stackalloc double[2];
        double inverse;

        order = degree + 1;
        num_poles = flatKnots.Length() - order;

        if (num_poles != cosNumerator.Length() || num_poles != sinNumerator.Length()
            || num_poles != denominator.Length())
        {
            throw new ConstructionErrorException();
        }
        NCollection_Array1<double> parameters = new(1, num_poles);
        NCollection_Array1<Point3D> poles_array = new(1, num_poles);
        NCollection_Array1<int> contact_order_array = new(1, num_poles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(degree, flatKnots, parameters);
        for (ii = parameters.Lower(); ii <= parameters.Upper(); ii++)
        {
            evaluator(parameters[ii], evalDegree, evalPoles, evalKnots, evalMults, result);
            contact_order_array[ii] = 0;

            poles_array[ii].SetCoord(1, (result[1] * result[1] - result[0] * result[0]));
            poles_array[ii].SetCoord(2, 2.0e0 * result[1] * result[0]);
            poles_array[ii].SetCoord(3, result[1] * result[1] + result[0] * result[0]);
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree,
                             flatKnots,
                             parameters,
                             contact_order_array,
                             poles_array,
                             out int _);
        for (ii = 1; ii <= num_poles; ii++)
        {
            inverse = 1.0e0 / poles_array[ii].Coord(3);
            cosNumerator[ii] = poles_array[ii].Coord(1) * inverse;
            sinNumerator[ii] = poles_array[ii].Coord(2) * inverse;
            denominator[ii] = poles_array[ii].Coord(3);
        }
    }

    /// <summary>
    /// The B-spline representation of cos and sin over [<paramref name="uFirst"/>, <paramref name="uLast"/>] with
    /// the given parameterisation: cos = CosNumerator / Denominator and sin = SinNumerator / Denominator, the
    /// numerators being given as poles of the rational B-spline (already divided by the weights).
    /// </summary>
    /// <remarks>The arrays are outputs, where OCCT assigns new arrays to its reference parameters.</remarks>
    protected void BuildCosAndSin(ParameterisationType parameterisation,
                                  double uFirst,
                                  double uLast,
                                  out NCollection_Array1<double> cosNumerator,
                                  out NCollection_Array1<double> sinNumerator,
                                  out NCollection_Array1<double> denominator,
                                  out int degree,
                                  out NCollection_Array1<double> knots,
                                  out NCollection_Array1<int> mults)
    {
        double delta = uLast - uFirst, direct, inverse, value1, value2, cos_beta, sin_beta, alpha = 0,
               alpha_2, alpha_4, tan_alpha_2, beta, p_param, q_param, param;

        int num_poles = 0, ii, jj, num_knots = 1, num_spans = 1, num_flat_knots, num_temp_knots,
            temp_degree = 0, tgt_theta_flag, num_temp_poles, order = 0;

        CosAndSinEvalFunction? evaluatorPtr = null;

        tgt_theta_flag = 0;
        // Always assigned (decision 82), where OCCT leaves Degree unchanged for a parameterisation outside the enumeration
        degree = 0;

        switch (parameterisation)
        {
            case TgtThetaOver2:
                num_spans = (int)Math.Truncate(1.2 * delta / M_PI) + 1;

                tgt_theta_flag = 1;
                break;
            case TgtThetaOver2_1:
                num_spans = 1;
                if (delta > 0.9999 * M_PI)
                {
                    throw new ConstructionErrorException();
                }
                tgt_theta_flag = 1;
                break;
            case TgtThetaOver2_2:
                num_spans = 2;
                if (delta > 1.9999 * M_PI)
                {
                    throw new ConstructionErrorException();
                }
                tgt_theta_flag = 1;
                break;

            case TgtThetaOver2_3:
                num_spans = 3;
                tgt_theta_flag = 1;
                break;
            case TgtThetaOver2_4:
                num_spans = 4;
                tgt_theta_flag = 1;
                break;
            case QuasiAngular:
                num_poles = 7;
                degree = 6;
                num_spans = 1;
                num_knots = 2;
                order = degree + 1;
                break;
            case RationalC1:
                degree = 4;
                order = degree + 1;
                num_poles = 8;
                num_knots = 3;
                num_spans = 2;
                break;
            case Polynomial:
                degree = 7;
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
            degree = 2;
            num_poles = 2 * num_spans + 1;
        }

        cosNumerator = new NCollection_Array1<double>(1, num_poles);
        sinNumerator = new NCollection_Array1<double>(1, num_poles);
        denominator = new NCollection_Array1<double>(1, num_poles);
        knots = new NCollection_Array1<double>(1, num_spans + 1);
        mults = new NCollection_Array1<int>(1, num_spans + 1);
        if (tgt_theta_flag != 0)
        {
            param = uFirst;
            cosNumerator[1] = Math.Cos(uFirst);
            sinNumerator[1] = Math.Sin(uFirst);
            denominator[1] = 1.0e0;
            knots[1] = param;
            mults[1] = degree + 1;
            direct = Math.Cos(alpha);
            inverse = 1.0e0 / direct;
            for (ii = 1; ii <= num_spans; ii++)
            {
                cosNumerator[2 * ii] = inverse * Math.Cos(param + alpha);
                sinNumerator[2 * ii] = inverse * Math.Sin(param + alpha);
                denominator[2 * ii] = direct;
                cosNumerator[2 * ii + 1] = Math.Cos(param + 2 * alpha);
                sinNumerator[2 * ii + 1] = Math.Sin(param + 2 * alpha);
                denominator[2 * ii + 1] = 1.0e0;
                knots[ii + 1] = param + 2 * alpha;
                mults[ii + 1] = 2;
                param += 2 * alpha;
            }
            mults[num_spans + 1] = degree + 1;
        }
        else if (parameterisation != Polynomial)
        {
            alpha = uLast - uFirst;
            alpha *= 0.5e0;
            beta = uLast + uFirst;
            beta *= 0.5e0;
            cos_beta = Math.Cos(beta);
            sin_beta = Math.Sin(beta);
            num_flat_knots = num_poles + order;

            num_temp_poles = 4;
            num_temp_knots = 3;
            NCollection_Array1<double> flat_knots = new(1, num_flat_knots);

            NCollection_Array1<Point2D> temp_poles = new(1, num_temp_poles);
            NCollection_Array1<double> temp_knots = new(1, num_temp_knots);
            NCollection_Array1<int> temp_mults = new(1, num_temp_knots);

            for (ii = 1; ii <= order; ii++)
            {
                flat_knots[ii] = -alpha;
                flat_knots[ii + num_poles] = alpha;
            }
            knots[1] = uFirst;
            knots[num_knots] = uLast;
            mults[1] = order;
            mults[num_knots] = order;

            switch (parameterisation)
            {
                case QuasiAngular:
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
                    evaluatorPtr = CosAndSinQuasiAngular;
                    break;
                case RationalC1:
                    for (ii = order + 1; ii <= num_poles; ii++)
                    {
                        flat_knots[ii] = 0.0e0;
                    }
                    knots[2] = uFirst + alpha;
                    mults[2] = degree - 1;
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

                    evaluatorPtr = CosAndSinRationalC1;
                    break;
                default:
                    break;
            }
            // OCCT calls through a null function pointer for a parameterisation outside the enumeration
            AlgorithmicCosAndSin(degree,
                                 flat_knots,
                                 temp_degree,
                                 temp_poles,
                                 temp_knots,
                                 temp_mults,
                                 evaluatorPtr!,
                                 cosNumerator,
                                 sinNumerator,
                                 denominator);

            for (ii = 1; ii <= num_poles; ii++)
            {
                value1 = cos_beta * cosNumerator[ii] - sin_beta * sinNumerator[ii];
                value2 = sin_beta * cosNumerator[ii] + cos_beta * sinNumerator[ii];
                cosNumerator[ii] = value1;
                sinNumerator[ii] = value2;
            }
        }
        else
        { // Convert_Polynomial

            knots[1] = 0.0;
            knots[num_knots] = 1.0;
            mults[1] = num_poles;
            mults[num_knots] = num_poles;

            PolynomialCosAndSin.BuildPolynomialCosAndSin(uFirst, uLast, num_poles, cosNumerator, sinNumerator, denominator);
        }
    }

    /// <summary>
    /// The periodic B-spline representation of cos and sin over [0, 2 Pi], for <c>Convert_TgtThetaOver2</c> and
    /// <c>Convert_RationalC1</c> only (<see cref="ConstructionErrorException"/> otherwise).
    /// </summary>
    /// <remarks>The arrays are outputs, where OCCT assigns new arrays to its reference parameters.</remarks>
    protected void BuildCosAndSin(ParameterisationType parameterisation,
                                  out NCollection_Array1<double> cosNumerator,
                                  out NCollection_Array1<double> sinNumerator,
                                  out NCollection_Array1<double> denominator,
                                  out int degree,
                                  out NCollection_Array1<double> knots,
                                  out NCollection_Array1<int> mults)
    {
        double half_pi, param, first_param, last_param, inverse, value1, value2, value3;

        int ii, jj, index, num_poles, num_periodic_poles, temp_degree,
          num_flat_knots, num_knots;

        if (parameterisation != TgtThetaOver2 && parameterisation != RationalC1)
        {
            throw new ConstructionErrorException();
        }
        NCollection_Array1<double> temp_cos, temp_sin, temp_denominator, temp_knots;
        NCollection_Array1<int> temp_mults;
        if (parameterisation == TgtThetaOver2)
        {
            BuildCosAndSin(TgtThetaOver2_3,
                           0.0e0,
                           2 * M_PI,
                           out temp_cos,
                           out temp_sin,
                           out temp_denominator,
                           out degree,
                           out knots,
                           out mults);
            cosNumerator = new NCollection_Array1<double>(1, temp_cos.Length() - 1);
            sinNumerator = new NCollection_Array1<double>(1, temp_cos.Length() - 1);
            denominator = new NCollection_Array1<double>(1, temp_cos.Length() - 1);
            for (ii = temp_cos.Lower(); ii <= temp_cos.Upper() - 1; ii++)
            {
                cosNumerator[ii] = temp_cos[ii];
                sinNumerator[ii] = temp_sin[ii];
                denominator[ii] = temp_denominator[ii];
            }
            for (ii = mults.Lower(); ii <= mults.Upper(); ii++)
            {
                mults[ii] = degree;
            }
        }
        else
        {
            // Parameterisation == Convert_RationalC1 (the C++ tests it again)
            first_param = 0.0e0;
            last_param = M_PI;
            BuildCosAndSin(RationalC1,
                           first_param,
                           last_param,
                           out temp_cos,
                           out temp_sin,
                           out temp_denominator,
                           out temp_degree,
                           out temp_knots,
                           out temp_mults);

            degree = 4;
            num_knots = 5;
            num_flat_knots = (degree - 1) * num_knots + 2 * 2;
            num_poles = num_flat_knots - degree - 1;
            num_periodic_poles = num_poles - 2;
            NCollection_Array1<double> flat_knots = new(1, num_flat_knots);
            cosNumerator = new NCollection_Array1<double>(1, num_periodic_poles);
            sinNumerator = new NCollection_Array1<double>(1, num_periodic_poles);
            denominator = new NCollection_Array1<double>(1, num_periodic_poles);

            half_pi = M_PI * 0.5e0;
            index = 1;
            for (jj = 1; jj <= 2; jj++)
            {
                flat_knots[index] = -half_pi;
                index += 1;
            }
            for (ii = 1; ii <= num_knots; ii++)
            {
                for (jj = 1; jj <= degree - 1; jj++)
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
            knots = new NCollection_Array1<double>(1, num_knots);
            mults = new NCollection_Array1<int>(1, num_knots);
            for (ii = 1; ii <= num_knots; ii++)
            {
                knots[ii] = (ii - 1) * half_pi;
                mults[ii] = degree - 1;
            }

            NCollection_Array1<double> parameters = new(1, num_poles);
            NCollection_Array1<Point3D> poles_array = new(1, num_poles);
            NCollection_Array1<int> contact_order_array = new(1, num_poles);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(degree, flat_knots, parameters);
            inverse = 1.0e0;
            for (ii = parameters.Lower(); ii <= parameters.Upper(); ii++)
            {
                param = parameters[ii];
                if (param > M_PI)
                {
                    inverse = -1.0e0;
                    param -= M_PI;
                }
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(param,
                            0,
                            temp_degree,
                            false,
                            temp_cos,
                            temp_denominator,
                            temp_knots,
                            temp_mults,
                            out value1);

                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(param,
                            0,
                            temp_degree,
                            false,
                            temp_sin,
                            temp_denominator,
                            temp_knots,
                            temp_mults,
                            out value2);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(param,
                            0,
                            temp_degree,
                            false,
                            temp_denominator,
                            global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                            temp_knots,
                            temp_mults,
                            out value3);
                contact_order_array[ii] = 0;

                poles_array[ii].SetCoord(1, value1 * value3 * inverse);
                poles_array[ii].SetCoord(2, value2 * value3 * inverse);
                poles_array[ii].SetCoord(3, value3);
            }
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree,
                                 flat_knots,
                                 parameters,
                                 contact_order_array,
                                 poles_array,
                                 out int _);
            for (ii = 1; ii <= num_periodic_poles; ii++)
            {
                inverse = 1.0e0 / poles_array[ii].Coord(3);
                cosNumerator[ii] = poles_array[ii].Coord(1) * inverse;
                sinNumerator[ii] = poles_array[ii].Coord(2) * inverse;
                denominator[ii] = poles_array[ii].Coord(3);
            }
        }
    }
}
