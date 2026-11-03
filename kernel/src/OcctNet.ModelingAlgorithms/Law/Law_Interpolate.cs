// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Interpolate.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Interpolate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Interpolates real values by a 1D B-spline (<see cref="Law_BSpline"/>) of degree 3 (1 or 2 for two or three
/// values without tangents), periodic or not, through given or computed parameters, with optional derivatives.
/// </summary>
/// <remarks>
/// A working algorithm (decision 21), as GeomAPI_Interpolate. It keeps the caller's arrays as OCCT keeps the
/// handles; <see cref="Perform"/> writes the tangents and flags it computes into the arrays given to
/// <see cref="Load(NCollection_Array1{double}, NCollection_HArray1{bool})"/> (the flags) as OCCT does.
/// <see cref="Curve"/> freezes the result before returning it. OCCT's <c>ClearTangents</c> is declared but never
/// defined, and is not ported.
/// </remarks>
public sealed class Law_Interpolate
{
    private readonly double myTolerance;
    private readonly NCollection_HArray1<double> myPoints;
    private bool myIsDone;
    private Law_BSpline? myCurve;
    private NCollection_HArray1<double> myTangents;
    private NCollection_HArray1<bool> myTangentFlags;
    private NCollection_HArray1<double> myParameters;
    private readonly bool myPeriodic;
    private bool myTangentRequest;

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

    private static void BuildParameters(bool PeriodicFlag, NCollection_Array1<double> PointsArray,
                                        out NCollection_HArray1<double> ParametersPtr)
    {
        int ii, index = 2;
        double distance;
        int num_parameters = PointsArray.Length();
        if (PeriodicFlag)
        {
            num_parameters += 1;
        }
        ParametersPtr = new NCollection_HArray1<double>(1, num_parameters);
        ParametersPtr.SetValue(1, 0.0);
        for (ii = PointsArray.Lower(); ii < PointsArray.Upper(); ii++)
        {
            distance = Math.Abs(PointsArray.Value(ii) - PointsArray.Value(ii + 1));
            // OCCT indexes the parameters by the points' index ii: right only for points indexed from 1
            ParametersPtr.SetValue(index, ParametersPtr.Value(ii) + distance);
            index += 1;
        }
        if (PeriodicFlag)
        {
            distance = Math.Abs(PointsArray.Value(PointsArray.Upper()) - PointsArray.Value(PointsArray.Lower()));
            ParametersPtr.SetValue(index, ParametersPtr.Value(ii) + distance);
        }
    }

    private static void BuildPeriodicTangent(NCollection_Array1<double> PointsArray,
                                             NCollection_Array1<double> TangentsArray,
                                             NCollection_Array1<bool> TangentFlags,
                                             NCollection_Array1<double> ParametersArray)
    {
        Span<double> point_array = stackalloc double[3];
        Span<double> parameter_array = stackalloc double[3];
        Span<double> eval_result = stackalloc double[2];

        if (PointsArray.Length() < 2)
        {
            TangentFlags.SetValue(1, true);
            TangentsArray.SetValue(1, 0.0);
        }
        else if (!TangentFlags.Value(1))
        {
            // Pour les periodiques on evalue la tangente du point de fermeture
            // par une interpolation de degre 2 entre le dernier point, le point
            // de fermeture et le deuxieme point.
            int degree = 2;
            double period = (ParametersArray.Value(ParametersArray.Upper()) - ParametersArray.Value(ParametersArray.Lower()));
            point_array[0] = PointsArray.Value(PointsArray.Upper());
            point_array[1] = PointsArray.Value(PointsArray.Lower());
            point_array[2] = PointsArray.Value(PointsArray.Lower() + 1);
            parameter_array[0] = ParametersArray.Value(ParametersArray.Upper() - 1) - period;
            parameter_array[1] = ParametersArray.Value(ParametersArray.Lower());
            parameter_array[2] = ParametersArray.Value(ParametersArray.Lower() + 1);
            TangentFlags.SetValue(1, true);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(parameter_array[1], 1, degree, 1, point_array, parameter_array, eval_result);
            TangentsArray.SetValue(1, eval_result[1]);
        }
    }

    private static void BuildTangents(NCollection_Array1<double> PointsArray, NCollection_Array1<double> TangentsArray,
                                      NCollection_Array1<bool> TangentFlags, NCollection_Array1<double> ParametersArray)
    {
        int degree = 3; //,ii;
        Span<double> eval_result = stackalloc double[2];

        if (PointsArray.Length() < 3)
        {
            throw new ConstructionErrorException();
        }
        if (PointsArray.Length() == 3)
        {
            degree = 2;
        }
        if (!TangentFlags.Value(1))
        {
            ReadOnlySpan<double> point_array = PointsArray.AsSpan(PointsArray.Lower());
            ReadOnlySpan<double> parameter_array = ParametersArray.AsSpan(1);
            TangentFlags.SetValue(1, true);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(ParametersArray.Value(1), 1, degree, 1, point_array, parameter_array, eval_result);
            TangentsArray.SetValue(1, eval_result[1]);
        }
        if (!TangentFlags.Value(TangentFlags.Upper()))
        {
            ReadOnlySpan<double> point_array = PointsArray.AsSpan(PointsArray.Upper() - degree);
            TangentFlags.SetValue(TangentFlags.Upper(), true);
            int iup = ParametersArray.Upper() - degree;
            ReadOnlySpan<double> parameter_array = ParametersArray.AsSpan(iup);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalLagrange(ParametersArray.Value(ParametersArray.Upper()), 1, degree, 1, point_array,
                              parameter_array, eval_result);
            TangentsArray.SetValue(TangentsArray.Upper(), eval_result[1]);
        }
    }

    /// <summary>Interpolates the values Points at parameters computed from their differences.</summary>
    public Law_Interpolate(NCollection_HArray1<double> PointsPtr, bool PeriodicFlag, double Tolerance)
    {
        myTolerance = Tolerance;
        myPoints = PointsPtr;
        myIsDone = false;
        myPeriodic = PeriodicFlag;
        myTangentRequest = false;

        myTangents = new NCollection_HArray1<double>(myPoints.Lower(), myPoints.Upper());
        myTangentFlags = new NCollection_HArray1<bool>(myPoints.Lower(), myPoints.Upper());

        BuildParameters(PeriodicFlag, PointsPtr.Array1(), out myParameters);
        myTangentFlags.Init(false);
    }

    /// <summary>
    /// Interpolates the values Points at the parameters Parameters (one more than the values for a periodic law).
    /// </summary>
    /// <exception cref="ConstructionErrorException">The parameters don't match the values or don't increase.</exception>
    public Law_Interpolate(NCollection_HArray1<double> PointsPtr, NCollection_HArray1<double> ParametersPtr,
                           bool PeriodicFlag, double Tolerance)
    {
        myTolerance = Tolerance;
        myPoints = PointsPtr;
        myIsDone = false;
        myParameters = ParametersPtr;
        myPeriodic = PeriodicFlag;
        myTangentRequest = false;
        if (PeriodicFlag)
        {
            if ((PointsPtr.Length()) + 1 != ParametersPtr.Length())
            {
                throw new ConstructionErrorException();
            }
        }
        myTangents = new NCollection_HArray1<double>(myPoints.Lower(), myPoints.Upper());
        myTangentFlags = new NCollection_HArray1<bool>(myPoints.Lower(), myPoints.Upper());
        bool result = CheckParameters(ParametersPtr.Array1());
        if (!result)
        {
            throw new ConstructionErrorException();
        }
        myTangentFlags.Init(false);
    }

    /// <summary>Gives the derivatives Tangents at the points whose TangentFlags are true (the flags array is kept).</summary>
    public void Load(NCollection_Array1<double> Tangents, NCollection_HArray1<bool> TangentFlagsPtr)
    {
        int ii;
        myTangentRequest = true;
        myTangentFlags = TangentFlagsPtr;
        if (Tangents.Length() != myPoints.Length() || TangentFlagsPtr.Length() != myPoints.Length())
        {
            throw new ConstructionErrorException();
        }
        myTangents = new NCollection_HArray1<double>(Tangents.Lower(), Tangents.Upper());
        for (ii = Tangents.Lower(); ii <= Tangents.Upper(); ii++)
        {
            myTangents.SetValue(ii, Tangents.Value(ii));
        }
    }

    /// <summary>Gives the derivatives at the first and last points.</summary>
    public void Load(double InitialTangent, double FinalTangent)
    {
        myTangentRequest = true;
        myTangentFlags.SetValue(1, true);
        myTangents.SetValue(1, InitialTangent);
        myTangentFlags.SetValue(myPoints.Length(), true);
        myTangents.SetValue(myPoints.Length(), FinalTangent);
    }

    /// <summary>Computes the interpolation.</summary>
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
        int degree, ii, index, index1, mult_index, half_order, inversion_problem, num_points, num_distinct_knots,
          num_poles;

        double period;

        num_points = myPoints.Length();
        period = myParameters.Value(myParameters.Upper()) - myParameters.Value(myParameters.Lower());
        num_poles = num_points + 1;
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
        NCollection_Array1<double> parameters = new NCollection_Array1<double>(1, num_poles);
        NCollection_Array1<double> flatknots = new NCollection_Array1<double>(1, num_poles + degree + 1);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, num_distinct_knots);
        NCollection_Array1<int> contact_order_array = new NCollection_Array1<int>(1, num_poles);
        NCollection_Array1<double> poles = new NCollection_Array1<double>(1, num_poles);

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

        BuildPeriodicTangent(myPoints.Array1(), myTangents.ChangeArray1(), myTangentFlags.ChangeArray1(),
                             myParameters.Array1());

        contact_order_array.SetValue(2, 1);
        parameters.SetValue(1, myParameters.Value(1));
        parameters.SetValue(2, myParameters.Value(1));
        poles.SetValue(1, myPoints.Value(1));
        poles.SetValue(2, myTangents.Value(1));
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
                    poles.SetValue(index, myTangents.Value(ii));
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
        poles.SetValue(num_poles - 1, myTangents.Value(1));
        parameters.SetValue(num_poles, myParameters.Value(myParameters.Upper()));
        poles.SetValue(num_poles, myPoints.Value(1));

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, parameters, contact_order_array, 1, poles.AsSpan(1),
                             out inversion_problem);
        if (inversion_problem == 0)
        {
            NCollection_Array1<double> newpoles = new NCollection_Array1<double>(poles, 1, 1, num_poles - 2);
            myCurve = new Law_BSpline(newpoles, myParameters.Array1(), mults, degree, myPeriodic);
            myIsDone = true;
        }
    }

    private void PerformNonPeriodic()
    {
        int degree, ii, index, index1, index2, index3, mult_index, inversion_problem, num_points, num_distinct_knots,
          num_poles;

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
        NCollection_Array1<double> parameters = new NCollection_Array1<double>(1, num_poles);
        NCollection_Array1<double> flatknots = new NCollection_Array1<double>(1, num_poles + degree + 1);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, num_distinct_knots);
        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, num_distinct_knots);
        NCollection_Array1<int> contact_order_array = new NCollection_Array1<int>(1, num_poles);
        NCollection_Array1<double> poles = new NCollection_Array1<double>(1, num_poles);

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
                myCurve = new Law_BSpline(poles, myParameters.Array1(), mults, degree);
                myIsDone = true;
                break;
            case 2:
                knots.SetValue(1, myParameters.Value(1));
                knots.SetValue(2, myParameters.Value(num_poles));
                for (ii = 1; ii <= num_poles; ii++)
                {
                    poles.SetValue(ii, myPoints.Value(ii));
                }
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, myParameters.Array1(), contact_order_array, 1, poles.AsSpan(1),
                                     out inversion_problem);
                if (inversion_problem == 0)
                {
                    myCurve = new Law_BSpline(poles, knots, mults, degree);
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
                poles.SetValue(2, myTangents.Value(1));
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
                            poles.SetValue(index, myTangents.Value(ii));
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
                poles.SetValue(num_poles - 1, myTangents.Value(num_points));

                contact_order_array.SetValue(num_poles - 1, 1);
                parameters.SetValue(num_poles, myParameters.Value(myParameters.Upper()));
                parameters.SetValue(num_poles - 1, myParameters.Value(myParameters.Upper()));

                poles.SetValue(num_poles, myPoints.Value(num_points));

                global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, flatknots, parameters, contact_order_array, 1, poles.AsSpan(1),
                                     out inversion_problem);
                if (inversion_problem == 0)
                {
                    myCurve = new Law_BSpline(poles, myParameters.Array1(), mults, degree);
                    myIsDone = true;
                }
                break;
        }
    }

    /// <summary>The interpolating curve, frozen.</summary>
    /// <exception cref="NotDoneException">The interpolation was not done.</exception>
    public Law_BSpline Curve()
    {
        Law_BSpline aCurve = InternalCurve();
        aCurve.Freeze();
        return aCurve;
    }

    /// <summary>The interpolating curve without freezing it, for ported code that consumes it at once.</summary>
    internal Law_BSpline InternalCurve()
    {
        if (!myIsDone)
        {
            throw new NotDoneException(" ");
        }
        return myCurve!;
    }

    public bool IsDone() => myIsDone;
}
