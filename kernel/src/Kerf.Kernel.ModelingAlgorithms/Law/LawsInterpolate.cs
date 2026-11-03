// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Interpolate.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Interpolate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Construction.Laws;

/// <summary>
/// Interpolates real values by a 1D B-spline (<see cref="BSpline"/>) of degree 3 (1 or 2 for two or three
/// values without tangents), periodic or not, through given or computed parameters, with optional derivatives.
/// </summary>
/// <remarks>
/// A working algorithm (decision 21), as GeomAPI_Interpolate. It keeps the caller's arrays as OCCT keeps the
/// handles; <see cref="Perform"/> writes the tangents and flags it computes into the arrays given to
/// <see cref="Load(NCollection_Array1{double}, NCollection_HArray1{bool})"/> (the flags) as OCCT does.
/// <see cref="Curve"/> freezes the result before returning it. OCCT's <c>ClearTangents</c> is declared but never
/// defined, and is not ported.
/// </remarks>
public sealed class LawsInterpolate
{
    private readonly double _tolerance;
    private readonly NCollection_HArray1<double> _points;
    private bool _isDone;
    private BSpline? _curve;
    private NCollection_HArray1<double> _tangents;
    private NCollection_HArray1<bool> _tangentFlags;
    private NCollection_HArray1<double> _parameters;
    private readonly bool _periodic;
    private bool _tangentRequest;

    private static bool CheckParameters(NCollection_Array1<double> parameters)
    {
        int ii;
        double distance;
        bool result = true;
        for (ii = parameters.Lower(); result && ii < parameters.Upper(); ii++)
        {
            distance = parameters.Value(ii + 1) - parameters.Value(ii);
            result = (distance >= RealSmall());
        }
        return result;
    }

    private static void BuildParameters(bool periodicFlag, NCollection_Array1<double> pointsArray,
                                        out NCollection_HArray1<double> parametersPtr)
    {
        int ii, index = 2;
        double distance;
        int num_parameters = pointsArray.Length();
        if (periodicFlag)
        {
            num_parameters += 1;
        }
        parametersPtr = new NCollection_HArray1<double>(1, num_parameters);
        parametersPtr.SetValue(1, 0.0);
        for (ii = pointsArray.Lower(); ii < pointsArray.Upper(); ii++)
        {
            distance = Math.Abs(pointsArray.Value(ii) - pointsArray.Value(ii + 1));
            // OCCT indexes the parameters by the points' index ii: right only for points indexed from 1
            parametersPtr.SetValue(index, parametersPtr.Value(ii) + distance);
            index += 1;
        }
        if (periodicFlag)
        {
            distance = Math.Abs(pointsArray.Value(pointsArray.Upper()) - pointsArray.Value(pointsArray.Lower()));
            parametersPtr.SetValue(index, parametersPtr.Value(ii) + distance);
        }
    }

    private static void BuildPeriodicTangent(NCollection_Array1<double> pointsArray,
                                             NCollection_Array1<double> tangentsArray,
                                             NCollection_Array1<bool> tangentFlags,
                                             NCollection_Array1<double> parametersArray)
    {
        Span<double> point_array = stackalloc double[3];
        Span<double> parameter_array = stackalloc double[3];
        Span<double> eval_result = stackalloc double[2];

        if (pointsArray.Length() < 2)
        {
            tangentFlags.SetValue(1, true);
            tangentsArray.SetValue(1, 0.0);
        }
        else if (!tangentFlags.Value(1))
        {
            // Pour les periodiques on evalue la tangente du point de fermeture
            // par une interpolation de degre 2 entre le dernier point, le point
            // de fermeture et le deuxieme point.
            int degree = 2;
            double period = (parametersArray.Value(parametersArray.Upper()) - parametersArray.Value(parametersArray.Lower()));
            point_array[0] = pointsArray.Value(pointsArray.Upper());
            point_array[1] = pointsArray.Value(pointsArray.Lower());
            point_array[2] = pointsArray.Value(pointsArray.Lower() + 1);
            parameter_array[0] = parametersArray.Value(parametersArray.Upper() - 1) - period;
            parameter_array[1] = parametersArray.Value(parametersArray.Lower());
            parameter_array[2] = parametersArray.Value(parametersArray.Lower() + 1);
            tangentFlags.SetValue(1, true);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(parameter_array[1], 1, degree, 1, point_array, parameter_array, eval_result);
            tangentsArray.SetValue(1, eval_result[1]);
        }
    }

    private static void BuildTangents(NCollection_Array1<double> pointsArray, NCollection_Array1<double> tangentsArray,
                                      NCollection_Array1<bool> tangentFlags, NCollection_Array1<double> parametersArray)
    {
        int degree = 3; //,ii;
        Span<double> eval_result = stackalloc double[2];

        if (pointsArray.Length() < 3)
        {
            throw new ConstructionErrorException();
        }
        if (pointsArray.Length() == 3)
        {
            degree = 2;
        }
        if (!tangentFlags.Value(1))
        {
            ReadOnlySpan<double> point_array = pointsArray.AsSpan(pointsArray.Lower());
            ReadOnlySpan<double> parameter_array = parametersArray.AsSpan(1);
            tangentFlags.SetValue(1, true);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(parametersArray.Value(1), 1, degree, 1, point_array, parameter_array, eval_result);
            tangentsArray.SetValue(1, eval_result[1]);
        }
        if (!tangentFlags.Value(tangentFlags.Upper()))
        {
            ReadOnlySpan<double> point_array = pointsArray.AsSpan(pointsArray.Upper() - degree);
            tangentFlags.SetValue(tangentFlags.Upper(), true);
            int iup = parametersArray.Upper() - degree;
            ReadOnlySpan<double> parameter_array = parametersArray.AsSpan(iup);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(parametersArray.Value(parametersArray.Upper()), 1, degree, 1, point_array,
                              parameter_array, eval_result);
            tangentsArray.SetValue(tangentsArray.Upper(), eval_result[1]);
        }
    }

    /// <summary>Interpolates the values Points at parameters computed from their differences.</summary>
    public LawsInterpolate(NCollection_HArray1<double> pointsPtr, bool periodicFlag, double tolerance)
    {
        _tolerance = tolerance;
        _points = pointsPtr;
        _isDone = false;
        _periodic = periodicFlag;
        _tangentRequest = false;

        _tangents = new NCollection_HArray1<double>(_points.Lower(), _points.Upper());
        _tangentFlags = new NCollection_HArray1<bool>(_points.Lower(), _points.Upper());

        BuildParameters(periodicFlag, pointsPtr.Array1(), out _parameters);
        _tangentFlags.Init(false);
    }

    /// <summary>
    /// Interpolates the values Points at the parameters Parameters (one more than the values for a periodic law).
    /// </summary>
    /// <exception cref="ConstructionErrorException">The parameters don't match the values or don't increase.</exception>
    public LawsInterpolate(NCollection_HArray1<double> pointsPtr, NCollection_HArray1<double> parametersPtr,
                           bool periodicFlag, double tolerance)
    {
        _tolerance = tolerance;
        _points = pointsPtr;
        _isDone = false;
        _parameters = parametersPtr;
        _periodic = periodicFlag;
        _tangentRequest = false;
        if (periodicFlag)
        {
            if ((pointsPtr.Length()) + 1 != parametersPtr.Length())
            {
                throw new ConstructionErrorException();
            }
        }
        _tangents = new NCollection_HArray1<double>(_points.Lower(), _points.Upper());
        _tangentFlags = new NCollection_HArray1<bool>(_points.Lower(), _points.Upper());
        bool result = CheckParameters(parametersPtr.Array1());
        if (!result)
        {
            throw new ConstructionErrorException();
        }
        _tangentFlags.Init(false);
    }

    /// <summary>Gives the derivatives Tangents at the points whose TangentFlags are true (the flags array is kept).</summary>
    public void Load(NCollection_Array1<double> tangents, NCollection_HArray1<bool> tangentFlagsPtr)
    {
        int ii;
        _tangentRequest = true;
        _tangentFlags = tangentFlagsPtr;
        if (tangents.Length() != _points.Length() || tangentFlagsPtr.Length() != _points.Length())
        {
            throw new ConstructionErrorException();
        }
        _tangents = new NCollection_HArray1<double>(tangents.Lower(), tangents.Upper());
        for (ii = tangents.Lower(); ii <= tangents.Upper(); ii++)
        {
            _tangents.SetValue(ii, tangents.Value(ii));
        }
    }

    /// <summary>Gives the derivatives at the first and last points.</summary>
    public void Load(double initialTangent, double finalTangent)
    {
        _tangentRequest = true;
        _tangentFlags.SetValue(1, true);
        _tangents.SetValue(1, initialTangent);
        _tangentFlags.SetValue(_points.Length(), true);
        _tangents.SetValue(_points.Length(), finalTangent);
    }

    /// <summary>Computes the interpolation.</summary>
    public void Perform()
    {
        if (_periodic)
        {
            PerformPeriodic();
        }
        else
        {
            PerformNonPeriodic();
        }
    }

    private void PerformPeriodic()
    {
        int degree, ii, index, index1, mult_index, half_order, inversion_problem, num_points, num_distinct_knots,
          num_poles;

        double period;

        num_points = _points.Length();
        period = _parameters.Value(_parameters.Upper()) - _parameters.Value(_parameters.Lower());
        num_poles = num_points + 1;
        num_distinct_knots = num_points + 1;
        half_order = 2;
        degree = 3;
        num_poles += 2;
        if (_tangentRequest)
        {
            for (ii = _tangentFlags.Lower() + 1; ii <= _tangentFlags.Upper(); ii++)
            {
                if (_tangentFlags.Value(ii))
                {
                    num_poles += 1;
                }
            }
        }
        NCollection_Array1<double> parameters = new NCollection_Array1<double>(1, num_poles);
        NCollection_Array1<double> flatknots = new NCollection_Array1<double>(1, num_poles + degree + 1);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, num_distinct_knots);
        NCollection_Array1<int> contact_order_array = new NCollection_Array1<int>(1, num_poles);
        NCollection_Array1<double> poles = new NCollection_Array1<double>(1, num_poles);

        for (ii = 1; ii <= half_order; ii++)
        {
            flatknots.SetValue(ii, _parameters.Value(_parameters.Upper() - 1) - period);
            flatknots.SetValue(ii + half_order, _parameters.Value(_parameters.Lower()));
            flatknots.SetValue(num_poles + ii, _parameters.Value(_parameters.Upper()));
            flatknots.SetValue(num_poles + half_order + ii, _parameters.Value(half_order) + period);
        }
        for (ii = 1; ii <= num_poles; ii++)
        {
            contact_order_array.SetValue(ii, 0);
        }
        for (ii = 2; ii < num_distinct_knots; ii++)
        {
            mults.SetValue(ii, 1);
        }
        mults.SetValue(1, half_order);
        mults.SetValue(num_distinct_knots, half_order);

        BuildPeriodicTangent(_points.Array1(), _tangents.ChangeArray1(), _tangentFlags.ChangeArray1(),
                             _parameters.Array1());

        contact_order_array.SetValue(2, 1);
        parameters.SetValue(1, _parameters.Value(1));
        parameters.SetValue(2, _parameters.Value(1));
        poles.SetValue(1, _points.Value(1));
        poles.SetValue(2, _tangents.Value(1));
        mult_index = 2;
        index = 3;
        index1 = degree + 2;
        if (_tangentRequest)
        {
            for (ii = _tangentFlags.Lower() + 1; ii <= _tangentFlags.Upper(); ii++)
            {
                parameters.SetValue(index, _parameters.Value(ii));
                flatknots.SetValue(index1, _parameters.Value(ii));
                poles.SetValue(index, _points.Value(ii));
                index += 1;
                index1 += 1;
                if (_tangentFlags.Value(ii))
                {
                    mults.SetValue(mult_index, mults.Value(mult_index) + 1);
                    contact_order_array[index] = 1;

                    parameters.SetValue(index, _parameters.Value(ii));
                    flatknots.SetValue(index1, _parameters.Value(ii));
                    poles.SetValue(index, _tangents.Value(ii));
                    index += 1;
                    index1 += 1;
                }
                mult_index += 1;
            }
        }
        else
        {
            index = degree + 1;
            index1 = 2;
            for (ii = _parameters.Lower(); ii <= _parameters.Upper(); ii++)
            {
                parameters.SetValue(index1, _parameters.Value(ii));
                flatknots.SetValue(index, _parameters.Value(ii));
                index += 1;
                index1 += 1;
            }
            index = 3;
            for (ii = _points.Lower() + 1; ii <= _points.Upper(); ii++)
            {
                //
                // copy all the given points since the last one will be initialized
                // below by the first point in the array myPoints
                //
                poles.SetValue(index, _points.Value(ii));
                index += 1;
            }
        }
        contact_order_array.SetValue(num_poles - 1, 1);
        parameters.SetValue(num_poles - 1, _parameters.Value(_parameters.Upper()));
        //
        // for the periodic curve ONLY  the tangent of the first point
        // will be used since the curve should close itself at the first
        // point See BuildPeriodicTangent
        //
        poles.SetValue(num_poles - 1, _tangents.Value(1));
        parameters.SetValue(num_poles, _parameters.Value(_parameters.Upper()));
        poles.SetValue(num_poles, _points.Value(1));

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, parameters, contact_order_array, 1, poles.AsSpan(1),
                             out inversion_problem);
        if (inversion_problem == 0)
        {
            NCollection_Array1<double> newpoles = new NCollection_Array1<double>(poles, 1, 1, num_poles - 2);
            _curve = new BSpline(newpoles, _parameters.Array1(), mults, degree, _periodic);
            _isDone = true;
        }
    }

    private void PerformNonPeriodic()
    {
        int degree, ii, index, index1, index2, index3, mult_index, inversion_problem, num_points, num_distinct_knots,
          num_poles;

        num_points = num_distinct_knots = num_poles = _points.Length();
        if (num_poles == 2 && !_tangentRequest)
        {
            degree = 1;
        }
        else if (num_poles == 3 && !_tangentRequest)
        {
            degree = 2;
            num_distinct_knots = 2;
        }
        else
        {
            degree = 3;
            num_poles += 2;
            if (_tangentRequest)
            {
                for (ii = _tangentFlags.Lower() + 1; ii < _tangentFlags.Upper(); ii++)
                {
                    if (_tangentFlags.Value(ii))
                    {
                        num_poles += 1;
                    }
                }
            }
        }
        NCollection_Array1<double> parameters = new NCollection_Array1<double>(1, num_poles);
        NCollection_Array1<double> flatknots = new NCollection_Array1<double>(1, num_poles + degree + 1);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, num_distinct_knots);
        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, num_distinct_knots);
        NCollection_Array1<int> contact_order_array = new NCollection_Array1<int>(1, num_poles);
        NCollection_Array1<double> poles = new NCollection_Array1<double>(1, num_poles);

        for (ii = 1; ii <= degree + 1; ii++)
        {
            flatknots.SetValue(ii, _parameters.Value(1));
            flatknots.SetValue(ii + num_poles, _parameters.Value(num_points));
        }
        for (ii = 1; ii <= num_poles; ii++)
        {
            contact_order_array.SetValue(ii, 0);
        }
        for (ii = 2; ii < num_distinct_knots; ii++)
        {
            mults.SetValue(ii, 1);
        }
        mults.SetValue(1, degree + 1);
        mults.SetValue(num_distinct_knots, degree + 1);

        switch (degree)
        {
            case 1:
                for (ii = 1; ii <= num_poles; ii++)
                {
                    poles.SetValue(ii, _points.Value(ii));
                }
                _curve = new BSpline(poles, _parameters.Array1(), mults, degree);
                _isDone = true;
                break;
            case 2:
                knots.SetValue(1, _parameters.Value(1));
                knots.SetValue(2, _parameters.Value(num_poles));
                for (ii = 1; ii <= num_poles; ii++)
                {
                    poles.SetValue(ii, _points.Value(ii));
                }
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, _parameters.Array1(), contact_order_array, 1, poles.AsSpan(1),
                                     out inversion_problem);
                if (inversion_problem == 0)
                {
                    _curve = new BSpline(poles, knots, mults, degree);
                    _isDone = true;
                }
                break;
            case 3:
                //
                // check if the boundary conditions are set
                //
                if (num_points >= 3)
                {
                    //
                    // cannot build the tangents with degree 3 with only 2 points
                    // if those where not given in advance
                    //
                    BuildTangents(_points.Array1(), _tangents.ChangeArray1(), _tangentFlags.ChangeArray1(),
                                  _parameters.Array1());
                }
                contact_order_array.SetValue(2, 1);
                parameters.SetValue(1, _parameters.Value(1));
                parameters.SetValue(2, _parameters.Value(1));
                poles.SetValue(1, _points.Value(1));
                poles.SetValue(2, _tangents.Value(1));
                mult_index = 2;
                index = 3;
                index1 = 2;
                index2 = _points.Lower() + 1;
                index3 = degree + 2;
                if (_tangentRequest)
                {
                    for (ii = _parameters.Lower() + 1; ii < _parameters.Upper(); ii++)
                    {
                        parameters.SetValue(index, _parameters.Value(ii));
                        poles.SetValue(index, _points.Value(index2));
                        flatknots.SetValue(index3, _parameters.Value(ii));
                        index += 1;
                        index3 += 1;
                        if (_tangentFlags.Value(index1))
                        {
                            //
                            // set the multiplicities, the order of the contact, the
                            // the flatknots,
                            //
                            mults.SetValue(mult_index, mults.Value(mult_index) + 1);
                            contact_order_array[index] = 1;
                            flatknots.SetValue(index3, _parameters.Value(ii));
                            parameters.SetValue(index, _parameters.Value(ii));
                            poles.SetValue(index, _tangents.Value(ii));
                            index += 1;
                            index3 += 1;
                        }
                        mult_index += 1;
                        index1 += 1;
                        index2 += 1;
                    }
                }
                else
                {
                    index1 = 2;
                    for (ii = _parameters.Lower(); ii <= _parameters.Upper(); ii++)
                    {
                        parameters.SetValue(index1, _parameters.Value(ii));
                        index1 += 1;
                    }
                    index = 3;
                    for (ii = _points.Lower() + 1; ii <= _points.Upper() - 1; ii++)
                    {
                        poles.SetValue(index, _points.Value(ii));
                        index += 1;
                    }
                    index = degree + 1;
                    for (ii = _parameters.Lower(); ii <= _parameters.Upper(); ii++)
                    {
                        flatknots.SetValue(index, _parameters.Value(ii));
                        index += 1;
                    }
                }
                poles.SetValue(num_poles - 1, _tangents.Value(num_points));

                contact_order_array.SetValue(num_poles - 1, 1);
                parameters.SetValue(num_poles, _parameters.Value(_parameters.Upper()));
                parameters.SetValue(num_poles - 1, _parameters.Value(_parameters.Upper()));

                poles.SetValue(num_poles, _points.Value(num_points));

                global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, parameters, contact_order_array, 1, poles.AsSpan(1),
                                     out inversion_problem);
                if (inversion_problem == 0)
                {
                    _curve = new BSpline(poles, _parameters.Array1(), mults, degree);
                    _isDone = true;
                }
                break;
        }
    }

    /// <summary>The interpolating curve, frozen.</summary>
    /// <exception cref="NotDoneException">The interpolation was not done.</exception>
    public BSpline Curve()
    {
        BSpline curve = InternalCurve();
        curve.Freeze();
        return curve;
    }

    /// <summary>The interpolating curve without freezing it, for ported code that consumes it at once.</summary>
    internal BSpline InternalCurve()
    {
        if (!_isDone)
        {
            throw new NotDoneException(" ");
        }
        return _curve!;
    }

    public bool IsDone => _isDone;
}
