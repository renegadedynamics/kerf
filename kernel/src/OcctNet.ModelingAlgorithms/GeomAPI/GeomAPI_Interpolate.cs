// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_Interpolate.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_Interpolate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;

namespace Occt;

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
public class GeomAPI_Interpolate
{
    private readonly double myTolerance;
    private readonly NCollection_HArray1<gp_Pnt> myPoints;
    private bool myIsDone;
    private Geom_BSplineCurve? myCurve;
    private NCollection_HArray1<gp_Vec> myTangents;
    private NCollection_HArray1<bool> myTangentFlags;
    private NCollection_HArray1<double> myParameters;
    private readonly bool myPeriodic;
    private bool myTangentRequest;

    //=================================================================================================

    private static bool CheckPoints(NCollection_Array1<gp_Pnt> PointArray, double Tolerance)
    {
        int ii;
        double tolerance_squared = Tolerance * Tolerance, distance_squared;
        bool result = true;
        for (ii = PointArray.Lower(); result && ii < PointArray.Upper(); ii++)
        {
            distance_squared = PointArray.Value(ii).SquareDistance(PointArray.Value(ii + 1));
            result = (distance_squared >= tolerance_squared);
        }
        return result;
    }

    //=================================================================================================

    private static bool CheckTangents(NCollection_Array1<gp_Vec> Tangents, NCollection_Array1<bool> TangentFlags,
                                      double Tolerance)
    {
        int ii, index;
        double tolerance_squared = Tolerance * Tolerance, distance_squared;
        bool result = true;
        index = TangentFlags.Lower();
        for (ii = Tangents.Lower(); result && ii <= Tangents.Upper(); ii++)
        {
            if (TangentFlags.Value(index))
            {
                distance_squared = Tangents.Value(ii).SquareMagnitude();
                result = (distance_squared >= tolerance_squared);
            }
            index += 1;
        }
        return result;
    }

    //=================================================================================================

    private static bool CheckParameters(NCollection_Array1<double> Parameters)
    {
        int ii;
        double distance;
        bool result = true;
        for (ii = Parameters.Lower(); result && ii < Parameters.Upper(); ii++)
        {
            distance = Parameters.Value(ii + 1) - Parameters.Value(ii);
            result = (distance >= RealSmall());
        }
        return result;
    }

    //=================================================================================================

    private static void BuildParameters(bool PeriodicFlag, NCollection_Array1<gp_Pnt> PointsArray,
                                        out NCollection_HArray1<double> ParametersPtr)
    {
        int ii, index;
        double distance;
        int num_parameters = PointsArray.Length();
        if (PeriodicFlag)
        {
            num_parameters += 1;
        }
        ParametersPtr = new NCollection_HArray1<double>(1, num_parameters);
        ParametersPtr.SetValue(1, 0.0e0);
        index = 2;
        for (ii = PointsArray.Lower(); ii < PointsArray.Upper(); ii++)
        {
            distance = PointsArray.Value(ii).Distance(PointsArray.Value(ii + 1));
            // OCCT reads the parameter at the point index ii, right only for points indexed from 1
            ParametersPtr.SetValue(index, ParametersPtr.Value(ii) + distance);
            index += 1;
        }
        if (PeriodicFlag)
        {
            distance = PointsArray.Value(PointsArray.Upper()).Distance(PointsArray.Value(PointsArray.Lower()));
            ParametersPtr.SetValue(index, ParametersPtr.Value(ii) + distance);
        }
    }

    // The start of the coordinates of PointsArray(theIndex) and the following points (OCCT's double* cast).
    private static ReadOnlySpan<double> Coordinates(NCollection_Array1<gp_Pnt> PointsArray, int theIndex)
        => MemoryMarshal.Cast<gp_Pnt, double>(PointsArray.AsSpan(theIndex));

    //=================================================================================================

    private static void BuildPeriodicTangent(NCollection_Array1<gp_Pnt> PointsArray, NCollection_Array1<gp_Vec> TangentsArray,
                                             NCollection_Array1<bool> TangentFlags,
                                             NCollection_Array1<double> ParametersArray)
    {
        int ii, degree;
        ReadOnlySpan<double> point_array, parameter_array;
        Span<double> eval_result = stackalloc double[6];

        gp_Vec a_vector = new();

        if (PointsArray.Length() < 3)
        {
            throw new Standard_ConstructionError();
        }

        if (!TangentFlags.Value(1))
        {
            degree = 3;
            if (PointsArray.Length() == 3)
            {
                degree = 2;
            }
            point_array = Coordinates(PointsArray, PointsArray.Lower());
            parameter_array = ParametersArray.AsSpan(1);
            TangentFlags.SetValue(1, true);
            PLib.EvalLagrange(ParametersArray.Value(1), 1, degree, 3, point_array, parameter_array, eval_result);
            for (ii = 1; ii <= 3; ii++)
            {
                a_vector.SetCoord(ii, eval_result[3 + ii - 1]);
            }
            TangentsArray.SetValue(1, a_vector);
        }
    }

    //=================================================================================================

    private static void BuildTangents(NCollection_Array1<gp_Pnt> PointsArray, NCollection_Array1<gp_Vec> TangentsArray,
                                      NCollection_Array1<bool> TangentFlags, NCollection_Array1<double> ParametersArray)
    {
        int ii, degree;
        ReadOnlySpan<double> point_array, parameter_array;
        Span<double> eval_result = stackalloc double[6];

        gp_Vec a_vector = new();

        degree = 3;

        if (PointsArray.Length() < 3)
        {
            throw new Standard_ConstructionError();
        }
        if (PointsArray.Length() == 3)
        {
            degree = 2;
        }
        if (!TangentFlags.Value(1))
        {
            point_array = Coordinates(PointsArray, PointsArray.Lower());
            parameter_array = ParametersArray.AsSpan(1);
            TangentFlags.SetValue(1, true);
            PLib.EvalLagrange(ParametersArray.Value(1), 1, degree, 3, point_array, parameter_array, eval_result);
            for (ii = 1; ii <= 3; ii++)
            {
                a_vector.SetCoord(ii, eval_result[3 + ii - 1]);
            }
            TangentsArray.SetValue(1, a_vector);
        }
        if (!TangentFlags.Value(TangentFlags.Upper()))
        {
            point_array = Coordinates(PointsArray, PointsArray.Upper() - degree);
            TangentFlags.SetValue(TangentFlags.Upper(), true);
            parameter_array = ParametersArray.AsSpan(ParametersArray.Upper() - degree);
            PLib.EvalLagrange(ParametersArray.Value(ParametersArray.Upper()), 1, degree, 3, point_array, parameter_array,
                              eval_result);
            for (ii = 1; ii <= 3; ii++)
            {
                a_vector.SetCoord(ii, eval_result[3 + ii - 1]);
            }
            TangentsArray.SetValue(TangentsArray.Upper(), a_vector);
        }
    }

    // Scales the given tangents so that they have the length of the derivative of the Lagrange interpolation.
    private static void ScaleTangents(NCollection_Array1<gp_Pnt> PointsArray, NCollection_Array1<gp_Vec> TangentsArray,
                                      NCollection_Array1<bool> TangentFlags, NCollection_Array1<double> ParametersArray)
    {
        int ii, jj, degree = 0, index, num_points;
        ReadOnlySpan<double> point_array, parameter_array;
        Span<double> value = stackalloc double[2];
        double ratio;
        Span<double> eval_result = stackalloc double[6];

        gp_Vec a_vector = new();

        num_points = PointsArray.Length();
        if (num_points == 2)
        {
            degree = 1;
        }
        else if (num_points >= 3)
        {
            degree = 2;
        }

        index = PointsArray.Lower();
        for (ii = TangentFlags.Lower(); ii <= TangentFlags.Upper(); ii++)
        {
            if (TangentFlags.Value(ii))
            {
                point_array = Coordinates(PointsArray, index);
                parameter_array = ParametersArray.AsSpan(index);
                PLib.EvalLagrange(ParametersArray.Value(ii), 1, degree, 3, point_array, parameter_array, eval_result);
                value[0] = value[1] = 0.0e0;
                for (jj = 1; jj <= 3; jj++)
                {
                    value[0] += Math.Abs(TangentsArray.Value(ii).Coord(jj));
                    value[1] += Math.Abs(eval_result[3 + jj - 1]);
                }
                ratio = value[1] / value[0];
                for (jj = 1; jj <= 3; jj++)
                {
                    a_vector.SetCoord(jj, ratio * TangentsArray.Value(ii).Coord(jj));
                }
                TangentsArray.SetValue(ii, a_vector);
                if (ii != TangentFlags.Lower())
                {
                    index += 1;
                }
                if (index > PointsArray.Upper() - degree)
                {
                    index = PointsArray.Upper() - degree;
                }
            }
        }
    }

    /// <summary>
    /// Prepares the interpolation of <paramref name="PointsPtr"/> at chord-length parameters. Throws
    /// Standard_ConstructionError if two consecutive points are closer than <paramref name="Tolerance"/>.
    /// </summary>
    public GeomAPI_Interpolate(NCollection_HArray1<gp_Pnt> PointsPtr, bool PeriodicFlag, double Tolerance)
    {
        myTolerance = Tolerance;
        myPoints = PointsPtr;
        myIsDone = false;
        myPeriodic = PeriodicFlag;
        myTangentRequest = false;

        int ii;
        bool result = CheckPoints(PointsPtr.Array1(), Tolerance);
        myTangents = new NCollection_HArray1<gp_Vec>(myPoints.Lower(), myPoints.Upper());
        myTangentFlags = new NCollection_HArray1<bool>(myPoints.Lower(), myPoints.Upper());

        if (!result)
        {
            throw new Standard_ConstructionError();
        }
        BuildParameters(PeriodicFlag, PointsPtr.Array1(), out myParameters);

        for (ii = myPoints.Lower(); ii <= myPoints.Upper(); ii++)
        {
            myTangentFlags.SetValue(ii, false);
        }
    }

    /// <summary>
    /// Prepares the interpolation of <paramref name="PointsPtr"/> at <paramref name="ParametersPtr"/> (one more
    /// parameter than points when periodic). Throws Standard_ConstructionError on close points or parameters.
    /// </summary>
    public GeomAPI_Interpolate(NCollection_HArray1<gp_Pnt> PointsPtr, NCollection_HArray1<double> ParametersPtr,
                               bool PeriodicFlag, double Tolerance)
    {
        myTolerance = Tolerance;
        myPoints = PointsPtr;
        myIsDone = false;
        myParameters = ParametersPtr;
        myPeriodic = PeriodicFlag;
        myTangentRequest = false;

        int ii;

        bool result = CheckPoints(PointsPtr.Array1(), Tolerance);

        if (PeriodicFlag)
        {
            if ((PointsPtr.Length()) + 1 != ParametersPtr.Length())
            {
                throw new Standard_ConstructionError();
            }
        }
        myTangents = new NCollection_HArray1<gp_Vec>(myPoints.Lower(), myPoints.Upper());
        myTangentFlags = new NCollection_HArray1<bool>(myPoints.Lower(), myPoints.Upper());

        if (!result)
        {
            throw new Standard_ConstructionError();
        }

        result = CheckParameters(ParametersPtr.Array1());
        if (!result)
        {
            throw new Standard_ConstructionError();
        }

        for (ii = myPoints.Lower(); ii <= myPoints.Upper(); ii++)
        {
            myTangentFlags.SetValue(ii, false);
        }
    }

    /// <summary>
    /// Assigns tangents to the points whose flag is set (one tangent and one flag per point), scaled to the
    /// chord-length derivative unless <paramref name="Scale"/> is false. The flags array is kept.
    /// </summary>
    public void Load(NCollection_Array1<gp_Vec> Tangents, NCollection_HArray1<bool> TangentFlagsPtr, bool Scale = true)
    {
        bool result;
        int ii;
        myTangentRequest = true;
        myTangentFlags = TangentFlagsPtr;
        if (Tangents.Length() != myPoints.Length() || TangentFlagsPtr.Length() != myPoints.Length())
        {
            throw new Standard_ConstructionError();
        }
        result = CheckTangents(Tangents, TangentFlagsPtr.Array1(), myTolerance);
        if (result)
        {
            myTangents = new NCollection_HArray1<gp_Vec>(Tangents.Lower(), Tangents.Upper());
            for (ii = Tangents.Lower(); ii <= Tangents.Upper(); ii++)
            {
                myTangents.SetValue(ii, Tangents.Value(ii));
            }

            if (Scale)
            {
                ScaleTangents(myPoints.Array1(), myTangents.ChangeArray1(), TangentFlagsPtr.Array1(), myParameters.Array1());
            }
        }
        else
        {
            throw new Standard_ConstructionError();
        }
    }

    /// <summary>
    /// Assigns the tangents at the first and last points, scaled to the chord-length derivative unless
    /// <paramref name="Scale"/> is false.
    /// </summary>
    public void Load(in gp_Vec InitialTangent, in gp_Vec FinalTangent, bool Scale = true)
    {
        bool result;
        myTangentRequest = true;
        myTangentFlags.SetValue(1, true);
        myTangentFlags.SetValue(myPoints.Length(), true);
        myTangents.SetValue(1, InitialTangent);
        myTangents.SetValue(myPoints.Length(), FinalTangent);
        result = CheckTangents(myTangents.Array1(), myTangentFlags.Array1(), myTolerance);
        if (!result)
        {
            throw new Standard_ConstructionError();
        }

        if (Scale)
        {
            ScaleTangents(myPoints.Array1(), myTangents.ChangeArray1(), myTangentFlags.Array1(), myParameters.Array1());
        }
    }

    /// <summary>Computes the interpolating curve.</summary>
    public void Perform()
    {
        if (myPeriodic)
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

        gp_Pnt a_point = new();

        num_points = myPoints.Length();
        period = myParameters.Value(myParameters.Upper()) - myParameters.Value(myParameters.Lower());
        num_poles = num_points + 1;
        if (num_points == 2 && !myTangentRequest)
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

            myCurve = new Geom_BSplineCurve(myPoints.Array1(), myParameters.Array1(), deg1_mults, degree, myPeriodic);

            myIsDone = true;
        }
        else
        {
            num_distinct_knots = num_points + 1;
            half_order = 2;
            degree = 3;
            num_poles += 2;
            if (myTangentRequest)
            {
                for (ii = myTangentFlags.Lower() + 1; ii <= myTangentFlags.Upper(); ii++)
                {
                    if (myTangentFlags.Value(ii))
                    {
                        num_poles += 1;
                    }
                }
            }

            NCollection_Array1<double> parameters = new(1, num_poles);
            NCollection_Array1<double> flatknots = new(1, num_poles + degree + 1);
            NCollection_Array1<int> mults = new(1, num_distinct_knots);
            NCollection_Array1<int> contact_order_array = new(1, num_poles);
            NCollection_Array1<gp_Pnt> poles = new(1, num_poles);

            for (ii = 1; ii <= half_order; ii++)
            {
                flatknots.SetValue(ii, myParameters.Value(myParameters.Upper() - 1) - period);
                flatknots.SetValue(ii + half_order, myParameters.Value(myParameters.Lower()));
                flatknots.SetValue(num_poles + ii, myParameters.Value(myParameters.Upper()));
                flatknots.SetValue(num_poles + half_order + ii, myParameters.Value(half_order) + period);
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
                BuildPeriodicTangent(myPoints.Array1(), myTangents.ChangeArray1(), myTangentFlags.ChangeArray1(),
                                     myParameters.Array1());
            }
            contact_order_array.SetValue(2, 1);
            parameters.SetValue(1, myParameters.Value(1));
            parameters.SetValue(2, myParameters.Value(1));
            poles.SetValue(1, myPoints.Value(1));
            for (jj = 1; jj <= 3; jj++)
            {
                a_point.SetCoord(jj, myTangents.Value(1).Coord(jj));
            }
            poles.SetValue(2, a_point);

            mult_index = 2;
            index = 3;
            index1 = degree + 2;
            if (myTangentRequest)
            {
                for (ii = myTangentFlags.Lower() + 1; ii <= myTangentFlags.Upper(); ii++)
                {
                    parameters.SetValue(index, myParameters.Value(ii));
                    flatknots.SetValue(index1, myParameters.Value(ii));
                    poles.SetValue(index, myPoints.Value(ii));
                    index += 1;
                    index1 += 1;
                    if (myTangentFlags.Value(ii))
                    {
                        mults.SetValue(mult_index, mults.Value(mult_index) + 1);
                        contact_order_array[index] = 1;

                        parameters.SetValue(index, myParameters.Value(ii));
                        flatknots.SetValue(index1, myParameters.Value(ii));
                        for (jj = 1; jj <= 3; jj++)
                        {
                            a_point.SetCoord(jj, myTangents.Value(ii).Coord(jj));
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
                for (ii = myParameters.Lower(); ii <= myParameters.Upper(); ii++)
                {
                    parameters.SetValue(index1, myParameters.Value(ii));
                    flatknots.SetValue(index, myParameters.Value(ii));
                    index += 1;
                    index1 += 1;
                }
                index = 3;
                for (ii = myPoints.Lower() + 1; ii <= myPoints.Upper(); ii++)
                {
                    //
                    // copy all the given points since the last one will be initialized
                    // below by the first point in the array myPoints
                    //
                    poles.SetValue(index, myPoints.Value(ii));
                    index += 1;
                }
            }
            contact_order_array.SetValue(num_poles - 1, 1);
            parameters.SetValue(num_poles - 1, myParameters.Value(myParameters.Upper()));
            //
            // for the periodic curve ONLY  the tangent of the first point
            // will be used since the curve should close itself at the first
            // point See BuildPeriodicTangent
            //
            for (jj = 1; jj <= 3; jj++)
            {
                a_point.SetCoord(jj, myTangents.Value(1).Coord(jj));
            }
            poles.SetValue(num_poles - 1, a_point);

            parameters.SetValue(num_poles, myParameters.Value(myParameters.Upper()));

            poles.SetValue(num_poles, myPoints.Value(1));

            BSplCLib.Interpolate(degree, flatknots, parameters, contact_order_array, poles, out inversion_problem);
            if (inversion_problem == 0)
            {
                NCollection_Array1<gp_Pnt> newpoles = new(poles, 1, 1, num_poles - 2);
                myCurve = new Geom_BSplineCurve(newpoles, myParameters.Array1(), mults, degree, myPeriodic);
                myIsDone = true;
            }
        }
    }

    private void PerformNonPeriodic()
    {
        int degree, ii, jj, index, index1, index2, index3, mult_index, inversion_problem, num_points,
            num_distinct_knots, num_poles;

        gp_Pnt a_point = new();

        num_points = num_distinct_knots = num_poles = myPoints.Length();
        if (num_poles == 2 && !myTangentRequest)
        {
            degree = 1;
        }
        else if (num_poles == 3 && !myTangentRequest)
        {
            degree = 2;
            num_distinct_knots = 2;
        }
        else
        {
            degree = 3;
            num_poles += 2;
            if (myTangentRequest)
            {
                for (ii = myTangentFlags.Lower() + 1; ii < myTangentFlags.Upper(); ii++)
                {
                    if (myTangentFlags.Value(ii))
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
        NCollection_Array1<gp_Pnt> poles = new(1, num_poles);

        for (ii = 1; ii <= degree + 1; ii++)
        {
            flatknots.SetValue(ii, myParameters.Value(1));
            flatknots.SetValue(ii + num_poles, myParameters.Value(num_points));
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
                    poles.SetValue(ii, myPoints.Value(ii));
                }
                myCurve = new Geom_BSplineCurve(poles, myParameters.Array1(), mults, degree);
                myIsDone = true;
                break;
            case 2:
                knots.SetValue(1, myParameters.Value(1));
                knots.SetValue(2, myParameters.Value(3));
                for (ii = 1; ii <= num_poles; ii++)
                {
                    poles.SetValue(ii, myPoints.Value(ii));
                }
                BSplCLib.Interpolate(degree, flatknots, myParameters.Array1(), contact_order_array, poles,
                                     out inversion_problem);
                if (inversion_problem == 0)
                {
                    myCurve = new Geom_BSplineCurve(poles, knots, mults, degree);
                    myIsDone = true;
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
                    BuildTangents(myPoints.Array1(), myTangents.ChangeArray1(), myTangentFlags.ChangeArray1(),
                                  myParameters.Array1());
                }
                contact_order_array.SetValue(2, 1);
                parameters.SetValue(1, myParameters.Value(1));
                parameters.SetValue(2, myParameters.Value(1));
                poles.SetValue(1, myPoints.Value(1));
                for (jj = 1; jj <= 3; jj++)
                {
                    a_point.SetCoord(jj, myTangents.Value(1).Coord(jj));
                }
                poles.SetValue(2, a_point);
                mult_index = 2;
                index = 3;
                index1 = 2;
                index2 = myPoints.Lower() + 1;
                index3 = degree + 2;
                if (myTangentRequest)
                {
                    for (ii = myParameters.Lower() + 1; ii < myParameters.Upper(); ii++)
                    {
                        parameters.SetValue(index, myParameters.Value(ii));
                        poles.SetValue(index, myPoints.Value(index2));
                        flatknots.SetValue(index3, myParameters.Value(ii));
                        index += 1;
                        index3 += 1;
                        if (myTangentFlags.Value(index1))
                        {
                            //
                            // set the multiplicities, the order of the contact, the
                            // the flatknots,
                            //
                            mults.SetValue(mult_index, mults.Value(mult_index) + 1);
                            contact_order_array[index] = 1;
                            flatknots.SetValue(index3, myParameters.Value(ii));
                            parameters.SetValue(index, myParameters.Value(ii));
                            for (jj = 1; jj <= 3; jj++)
                            {
                                a_point.SetCoord(jj, myTangents.Value(ii).Coord(jj));
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
                    for (ii = myParameters.Lower(); ii <= myParameters.Upper(); ii++)
                    {
                        parameters.SetValue(index1, myParameters.Value(ii));
                        index1 += 1;
                    }
                    index = 3;
                    for (ii = myPoints.Lower() + 1; ii <= myPoints.Upper() - 1; ii++)
                    {
                        poles.SetValue(index, myPoints.Value(ii));
                        index += 1;
                    }

                    index = degree + 1;
                    for (ii = myParameters.Lower(); ii <= myParameters.Upper(); ii++)
                    {
                        flatknots.SetValue(index, myParameters.Value(ii));
                        index += 1;
                    }
                }
                for (jj = 1; jj <= 3; jj++)
                {
                    a_point.SetCoord(jj, myTangents.Value(num_points).Coord(jj));
                }
                poles.SetValue(num_poles - 1, a_point);

                contact_order_array.SetValue(num_poles - 1, 1);
                parameters.SetValue(num_poles, myParameters.Value(myParameters.Upper()));
                parameters.SetValue(num_poles - 1, myParameters.Value(myParameters.Upper()));

                poles.SetValue(num_poles, myPoints.Value(num_points));

                BSplCLib.Interpolate(degree, flatknots, parameters, contact_order_array, poles, out inversion_problem);
                if (inversion_problem == 0)
                {
                    myCurve = new Geom_BSplineCurve(poles, myParameters.Array1(), mults, degree);
                    myIsDone = true;
                }
                break;
        }
    }

    /// <summary>The interpolating curve (frozen). Throws StdFail_NotDone if the interpolation failed.</summary>
    public Geom_BSplineCurve Curve()
    {
        if (!myIsDone)
        {
            throw new StdFail_NotDone(" ");
        }
        myCurve!.Freeze();
        return myCurve;
    }

    /// <summary>OCCT's <c>operator occ::handle&lt;Geom_BSplineCurve&gt;</c>: the curve, null if not computed.</summary>
    public static explicit operator Geom_BSplineCurve?(GeomAPI_Interpolate theInterp)
    {
        theInterp.myCurve?.Freeze();
        return theInterp.myCurve;
    }

    /// <summary>True if the interpolation succeeded.</summary>
    public bool IsDone() => myIsDone;
}
