// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_Interpolate.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_Interpolate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Interpolates a set of points by a B-spline curve (C2, degree 3, or less for two or three points), periodic or
/// not, at given or chord-length parameters, optionally with tangents at some points (BSplCLib::Interpolate).
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Load</c> the tangents, then <see cref="Perform"/>. As in OCCT, the points
/// array and the tangent flags passed to <c>Load</c> are kept, not copied, and <see cref="Perform"/> sets the
/// flags of the end tangents it computes in the caller's flags array. <see cref="Curve"/> returns the stored curve,
/// frozen (decision 243). OCCT declares <c>ClearTangents</c> but never defines it: left out. The static helpers
/// assume arrays indexed from 1 in places, as OCCT's do.
/// </remarks>
public class ApiInterpolate
{
    private readonly double _tolerance;
    private readonly NCollection_HArray1<Point3D> _points;
    private bool _isDone;
    private BSplineCurve3D? _curve;
    private NCollection_HArray1<Vector3D> _tangents;
    private NCollection_HArray1<bool> _tangentFlags;
    private NCollection_HArray1<double> _parameters;
    private readonly bool _periodic;
    private bool _tangentRequest;

    //=================================================================================================

    private static bool CheckPoints(NCollection_Array1<Point3D> pointArray, double tolerance)
    {
        int ii;
        double tolerance_squared = tolerance * tolerance, distance_squared;
        bool result = true;
        for (ii = pointArray.Lower(); result && ii < pointArray.Upper(); ii++)
        {
            distance_squared = pointArray.Value(ii).SquareDistanceTo(pointArray.Value(ii + 1));
            result = (distance_squared >= tolerance_squared);
        }
        return result;
    }

    //=================================================================================================

    private static bool CheckTangents(NCollection_Array1<Vector3D> tangents, NCollection_Array1<bool> tangentFlags,
                                      double tolerance)
    {
        int ii, index;
        double tolerance_squared = tolerance * tolerance, distance_squared;
        bool result = true;
        index = tangentFlags.Lower();
        for (ii = tangents.Lower(); result && ii <= tangents.Upper(); ii++)
        {
            if (tangentFlags.Value(index))
            {
                distance_squared = tangents.Value(ii).SquareMagnitude;
                result = (distance_squared >= tolerance_squared);
            }
            index += 1;
        }
        return result;
    }

    //=================================================================================================

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

    //=================================================================================================

    private static void BuildParameters(bool periodicFlag, NCollection_Array1<Point3D> pointsArray,
                                        out NCollection_HArray1<double> parametersPtr)
    {
        int ii, index;
        double distance;
        int num_parameters = pointsArray.Length();
        if (periodicFlag)
        {
            num_parameters += 1;
        }
        parametersPtr = new NCollection_HArray1<double>(1, num_parameters);
        parametersPtr.SetValue(1, 0.0e0);
        index = 2;
        for (ii = pointsArray.Lower(); ii < pointsArray.Upper(); ii++)
        {
            distance = pointsArray.Value(ii).DistanceTo(pointsArray.Value(ii + 1));
            // OCCT reads the parameter at the point index ii, right only for points indexed from 1
            parametersPtr.SetValue(index, parametersPtr.Value(ii) + distance);
            index += 1;
        }
        if (periodicFlag)
        {
            distance = pointsArray.Value(pointsArray.Upper()).DistanceTo(pointsArray.Value(pointsArray.Lower()));
            parametersPtr.SetValue(index, parametersPtr.Value(ii) + distance);
        }
    }

    // The start of the coordinates of PointsArray(theIndex) and the following points (OCCT's double* cast).
    private static ReadOnlySpan<double> Coordinates(NCollection_Array1<Point3D> pointsArray, int index)
        => MemoryMarshal.Cast<Point3D, double>(pointsArray.AsSpan(index));

    //=================================================================================================

    private static void BuildPeriodicTangent(NCollection_Array1<Point3D> pointsArray, NCollection_Array1<Vector3D> tangentsArray,
                                             NCollection_Array1<bool> tangentFlags,
                                             NCollection_Array1<double> parametersArray)
    {
        int ii, degree;
        ReadOnlySpan<double> point_array, parameter_array;
        Span<double> eval_result = stackalloc double[6];

        Vector3D a_vector = new();

        if (pointsArray.Length() < 3)
        {
            throw new ConstructionErrorException();
        }

        if (!tangentFlags.Value(1))
        {
            degree = 3;
            if (pointsArray.Length() == 3)
            {
                degree = 2;
            }
            point_array = Coordinates(pointsArray, pointsArray.Lower());
            parameter_array = parametersArray.AsSpan(1);
            tangentFlags.SetValue(1, true);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(parametersArray.Value(1), 1, degree, 3, point_array, parameter_array, eval_result);
            for (ii = 1; ii <= 3; ii++)
            {
                a_vector = a_vector.WithCoord(ii, eval_result[3 + ii - 1]);
            }
            tangentsArray.SetValue(1, a_vector);
        }
    }

    //=================================================================================================

    private static void BuildTangents(NCollection_Array1<Point3D> pointsArray, NCollection_Array1<Vector3D> tangentsArray,
                                      NCollection_Array1<bool> tangentFlags, NCollection_Array1<double> parametersArray)
    {
        int ii, degree;
        ReadOnlySpan<double> point_array, parameter_array;
        Span<double> eval_result = stackalloc double[6];

        Vector3D a_vector = new();

        degree = 3;

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
            point_array = Coordinates(pointsArray, pointsArray.Lower());
            parameter_array = parametersArray.AsSpan(1);
            tangentFlags.SetValue(1, true);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(parametersArray.Value(1), 1, degree, 3, point_array, parameter_array, eval_result);
            for (ii = 1; ii <= 3; ii++)
            {
                a_vector = a_vector.WithCoord(ii, eval_result[3 + ii - 1]);
            }
            tangentsArray.SetValue(1, a_vector);
        }
        if (!tangentFlags.Value(tangentFlags.Upper()))
        {
            point_array = Coordinates(pointsArray, pointsArray.Upper() - degree);
            tangentFlags.SetValue(tangentFlags.Upper(), true);
            parameter_array = parametersArray.AsSpan(parametersArray.Upper() - degree);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(parametersArray.Value(parametersArray.Upper()), 1, degree, 3, point_array, parameter_array,
                              eval_result);
            for (ii = 1; ii <= 3; ii++)
            {
                a_vector = a_vector.WithCoord(ii, eval_result[3 + ii - 1]);
            }
            tangentsArray.SetValue(tangentsArray.Upper(), a_vector);
        }
    }

    // Scales the given tangents so that they have the length of the derivative of the Lagrange interpolation.
    private static void ScaleTangents(NCollection_Array1<Point3D> pointsArray, NCollection_Array1<Vector3D> tangentsArray,
                                      NCollection_Array1<bool> tangentFlags, NCollection_Array1<double> parametersArray)
    {
        int ii, jj, degree = 0, index, num_points;
        ReadOnlySpan<double> point_array, parameter_array;
        Span<double> value = stackalloc double[2];
        double ratio;
        Span<double> eval_result = stackalloc double[6];

        Vector3D a_vector = new();

        num_points = pointsArray.Length();
        if (num_points == 2)
        {
            degree = 1;
        }
        else if (num_points >= 3)
        {
            degree = 2;
        }

        index = pointsArray.Lower();
        for (ii = tangentFlags.Lower(); ii <= tangentFlags.Upper(); ii++)
        {
            if (tangentFlags.Value(ii))
            {
                point_array = Coordinates(pointsArray, index);
                parameter_array = parametersArray.AsSpan(index);
                global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(parametersArray.Value(ii), 1, degree, 3, point_array, parameter_array, eval_result);
                value[0] = value[1] = 0.0e0;
                for (jj = 1; jj <= 3; jj++)
                {
                    value[0] += Math.Abs(tangentsArray.Value(ii).Coord(jj));
                    value[1] += Math.Abs(eval_result[3 + jj - 1]);
                }
                ratio = value[1] / value[0];
                for (jj = 1; jj <= 3; jj++)
                {
                    a_vector = a_vector.WithCoord(jj, ratio * tangentsArray.Value(ii).Coord(jj));
                }
                tangentsArray.SetValue(ii, a_vector);
                if (ii != tangentFlags.Lower())
                {
                    index += 1;
                }
                if (index > pointsArray.Upper() - degree)
                {
                    index = pointsArray.Upper() - degree;
                }
            }
        }
    }

    /// <summary>
    /// Prepares the interpolation of <paramref name="pointsPtr"/> at chord-length parameters. Throws
    /// Standard_ConstructionError if two consecutive points are closer than <paramref name="tolerance"/>.
    /// </summary>
    public ApiInterpolate(NCollection_HArray1<Point3D> pointsPtr, bool periodicFlag, double tolerance)
    {
        _tolerance = tolerance;
        _points = pointsPtr;
        _isDone = false;
        _periodic = periodicFlag;
        _tangentRequest = false;

        int ii;
        bool result = CheckPoints(pointsPtr.Array1(), tolerance);
        _tangents = new NCollection_HArray1<Vector3D>(_points.Lower(), _points.Upper());
        _tangentFlags = new NCollection_HArray1<bool>(_points.Lower(), _points.Upper());

        if (!result)
        {
            throw new ConstructionErrorException();
        }
        BuildParameters(periodicFlag, pointsPtr.Array1(), out _parameters);

        for (ii = _points.Lower(); ii <= _points.Upper(); ii++)
        {
            _tangentFlags.SetValue(ii, false);
        }
    }

    /// <summary>
    /// Prepares the interpolation of <paramref name="pointsPtr"/> at <paramref name="parametersPtr"/> (one more
    /// parameter than points when periodic). Throws Standard_ConstructionError on close points or parameters.
    /// </summary>
    public ApiInterpolate(NCollection_HArray1<Point3D> pointsPtr, NCollection_HArray1<double> parametersPtr,
                               bool periodicFlag, double tolerance)
    {
        _tolerance = tolerance;
        _points = pointsPtr;
        _isDone = false;
        _parameters = parametersPtr;
        _periodic = periodicFlag;
        _tangentRequest = false;

        int ii;

        bool result = CheckPoints(pointsPtr.Array1(), tolerance);

        if (periodicFlag)
        {
            if ((pointsPtr.Length()) + 1 != parametersPtr.Length())
            {
                throw new ConstructionErrorException();
            }
        }
        _tangents = new NCollection_HArray1<Vector3D>(_points.Lower(), _points.Upper());
        _tangentFlags = new NCollection_HArray1<bool>(_points.Lower(), _points.Upper());

        if (!result)
        {
            throw new ConstructionErrorException();
        }

        result = CheckParameters(parametersPtr.Array1());
        if (!result)
        {
            throw new ConstructionErrorException();
        }

        for (ii = _points.Lower(); ii <= _points.Upper(); ii++)
        {
            _tangentFlags.SetValue(ii, false);
        }
    }

    /// <summary>
    /// Assigns tangents to the points whose flag is set (one tangent and one flag per point), scaled to the
    /// chord-length derivative unless <paramref name="scale"/> is false. The flags array is kept.
    /// </summary>
    public void Load(NCollection_Array1<Vector3D> tangents, NCollection_HArray1<bool> tangentFlagsPtr, bool scale = true)
    {
        bool result;
        int ii;
        _tangentRequest = true;
        _tangentFlags = tangentFlagsPtr;
        if (tangents.Length() != _points.Length() || tangentFlagsPtr.Length() != _points.Length())
        {
            throw new ConstructionErrorException();
        }
        result = CheckTangents(tangents, tangentFlagsPtr.Array1(), _tolerance);
        if (result)
        {
            _tangents = new NCollection_HArray1<Vector3D>(tangents.Lower(), tangents.Upper());
            for (ii = tangents.Lower(); ii <= tangents.Upper(); ii++)
            {
                _tangents.SetValue(ii, tangents.Value(ii));
            }

            if (scale)
            {
                ScaleTangents(_points.Array1(), _tangents.ChangeArray1(), tangentFlagsPtr.Array1(), _parameters.Array1());
            }
        }
        else
        {
            throw new ConstructionErrorException();
        }
    }

    /// <summary>
    /// Assigns the tangents at the first and last points, scaled to the chord-length derivative unless
    /// <paramref name="scale"/> is false.
    /// </summary>
    public void Load(in Vector3D initialTangent, in Vector3D finalTangent, bool scale = true)
    {
        bool result;
        _tangentRequest = true;
        _tangentFlags.SetValue(1, true);
        _tangentFlags.SetValue(_points.Length(), true);
        _tangents.SetValue(1, initialTangent);
        _tangents.SetValue(_points.Length(), finalTangent);
        result = CheckTangents(_tangents.Array1(), _tangentFlags.Array1(), _tolerance);
        if (!result)
        {
            throw new ConstructionErrorException();
        }

        if (scale)
        {
            ScaleTangents(_points.Array1(), _tangents.ChangeArray1(), _tangentFlags.Array1(), _parameters.Array1());
        }
    }

    /// <summary>Computes the interpolating curve.</summary>
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
        int degree, ii, jj, index, index1, mult_index, half_order, inversion_problem, num_points, num_distinct_knots,
            num_poles;

        double period;

        Point3D a_point = new();

        num_points = _points.Length();
        period = _parameters.Value(_parameters.Upper()) - _parameters.Value(_parameters.Lower());
        num_poles = num_points + 1;
        if (num_points == 2 && !_tangentRequest)
        {
            //
            // build a periodic curve of degree 1
            //

            degree = 1;
            NCollection_Array1<int> deg1_mults = new(1, num_poles);
            for (ii = 1; ii <= num_poles; ii++)
            {
                deg1_mults.SetValue(ii, 1);
            }

            _curve = new BSplineCurve3D(_points.Array1(), _parameters.Array1(), deg1_mults, degree, _periodic);

            _isDone = true;
        }
        else
        {
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

            NCollection_Array1<double> parameters = new(1, num_poles);
            NCollection_Array1<double> flatknots = new(1, num_poles + degree + 1);
            NCollection_Array1<int> mults = new(1, num_distinct_knots);
            NCollection_Array1<int> contact_order_array = new(1, num_poles);
            NCollection_Array1<Point3D> poles = new(1, num_poles);

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
            if (num_points >= 3)
            {
                //
                //   only enter here if there are more than 3 points otherwise
                //   it means we have already the tangent
                //
                BuildPeriodicTangent(_points.Array1(), _tangents.ChangeArray1(), _tangentFlags.ChangeArray1(),
                                     _parameters.Array1());
            }
            contact_order_array.SetValue(2, 1);
            parameters.SetValue(1, _parameters.Value(1));
            parameters.SetValue(2, _parameters.Value(1));
            poles.SetValue(1, _points.Value(1));
            for (jj = 1; jj <= 3; jj++)
            {
                a_point = a_point.WithCoord(jj, _tangents.Value(1).Coord(jj));
            }
            poles.SetValue(2, a_point);

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
                        for (jj = 1; jj <= 3; jj++)
                        {
                            a_point = a_point.WithCoord(jj, _tangents.Value(ii).Coord(jj));
                        }
                        poles.SetValue(index, a_point);
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
            for (jj = 1; jj <= 3; jj++)
            {
                a_point = a_point.WithCoord(jj, _tangents.Value(1).Coord(jj));
            }
            poles.SetValue(num_poles - 1, a_point);

            parameters.SetValue(num_poles, _parameters.Value(_parameters.Upper()));

            poles.SetValue(num_poles, _points.Value(1));

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, parameters, contact_order_array, poles, out inversion_problem);
            if (inversion_problem == 0)
            {
                NCollection_Array1<Point3D> newpoles = new(poles, 1, 1, num_poles - 2);
                _curve = new BSplineCurve3D(newpoles, _parameters.Array1(), mults, degree, _periodic);
                _isDone = true;
            }
        }
    }

    private void PerformNonPeriodic()
    {
        int degree, ii, jj, index, index1, index2, index3, mult_index, inversion_problem, num_points,
            num_distinct_knots, num_poles;

        Point3D a_point = new();

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

        NCollection_Array1<double> parameters = new(1, num_poles);
        NCollection_Array1<double> flatknots = new(1, num_poles + degree + 1);
        NCollection_Array1<int> mults = new(1, num_distinct_knots);
        NCollection_Array1<double> knots = new(1, num_distinct_knots);
        NCollection_Array1<int> contact_order_array = new(1, num_poles);
        NCollection_Array1<Point3D> poles = new(1, num_poles);

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
                _curve = new BSplineCurve3D(poles, _parameters.Array1(), mults, degree);
                _isDone = true;
                break;
            case 2:
                knots.SetValue(1, _parameters.Value(1));
                knots.SetValue(2, _parameters.Value(3));
                for (ii = 1; ii <= num_poles; ii++)
                {
                    poles.SetValue(ii, _points.Value(ii));
                }
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, _parameters.Array1(), contact_order_array, poles,
                                     out inversion_problem);
                if (inversion_problem == 0)
                {
                    _curve = new BSplineCurve3D(poles, knots, mults, degree);
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
                for (jj = 1; jj <= 3; jj++)
                {
                    a_point = a_point.WithCoord(jj, _tangents.Value(1).Coord(jj));
                }
                poles.SetValue(2, a_point);
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
                            for (jj = 1; jj <= 3; jj++)
                            {
                                a_point = a_point.WithCoord(jj, _tangents.Value(ii).Coord(jj));
                            }
                            poles.SetValue(index, a_point);
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
                for (jj = 1; jj <= 3; jj++)
                {
                    a_point = a_point.WithCoord(jj, _tangents.Value(num_points).Coord(jj));
                }
                poles.SetValue(num_poles - 1, a_point);

                contact_order_array.SetValue(num_poles - 1, 1);
                parameters.SetValue(num_poles, _parameters.Value(_parameters.Upper()));
                parameters.SetValue(num_poles - 1, _parameters.Value(_parameters.Upper()));

                poles.SetValue(num_poles, _points.Value(num_points));

                global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, parameters, contact_order_array, poles, out inversion_problem);
                if (inversion_problem == 0)
                {
                    _curve = new BSplineCurve3D(poles, _parameters.Array1(), mults, degree);
                    _isDone = true;
                }
                break;
        }
    }

    /// <summary>The interpolating curve (frozen). Throws StdFail_NotDone if the interpolation failed.</summary>
    public BSplineCurve3D Curve()
    {
        if (!_isDone)
        {
            throw new NotDoneException(" ");
        }
        _curve!.Freeze();
        return _curve;
    }

    /// <summary>OCCT's <c>operator occ::handle&lt;Geom_BSplineCurve&gt;</c>: the curve, null if not computed.</summary>
    public static explicit operator BSplineCurve3D?(ApiInterpolate interp)
    {
        interp._curve?.Freeze();
        return interp._curve;
    }

    /// <summary>True if the interpolation succeeded.</summary>
    public bool IsDone => _isDone;
}
