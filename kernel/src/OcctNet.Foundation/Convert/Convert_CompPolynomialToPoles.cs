// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompPolynomialToPoles.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompPolynomialToPoles.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts a composite curve made of polynomial pieces (n-dimensional) into the poles of an equivalent
/// B-spline, by interpolation at the Schoenberg points.
/// </summary>
/// <remarks>
/// The coefficients are in the "C form" Coefficients[NumCurves][MaxDegree + 1][Dimension]; the number of
/// coefficients of a polynomial is its degree + 1. Example: f(x) = 2x + 1 on [2, 5], to a B-spline on [-1, 1]:
/// NumCurves = 1, Continuity = 1, Dimension = 1, MaxDegree = 1, NumCoeffPerCurve = {2}, Coefficients = {1, 2},
/// PolynomialIntervals = {{2, 5}}, TrueIntervals = {-1, 1}. The batch accessors return copies, where OCCT returns
/// references to its fields.
/// </remarks>
public class Convert_CompPolynomialToPoles
{
    private NCollection_Array1<double> myFlatKnots = new();
    private NCollection_Array1<double> myKnots = new();
    private NCollection_Array1<int> myMults = new();
    private NCollection_Array2<double> myPoles = new();
    private int myDegree;
    private bool myDone;

    /// <summary>
    /// Converts <paramref name="NumCurves"/> polynomials with the same continuity between them (at most the
    /// maximum degree). <paramref name="TrueIntervals"/> is the parametrisation of the result: curve n is
    /// parametrised on [TrueIntervals(n), TrueIntervals(n + 1)].
    /// </summary>
    public Convert_CompPolynomialToPoles(int NumCurves,
                                         int Continuity,
                                         int Dimension,
                                         int MaxDegree,
                                         NCollection_HArray1<int>? NumCoeffPerCurve,
                                         NCollection_HArray1<double>? Coefficients,
                                         NCollection_HArray2<double>? PolynomialIntervals,
                                         NCollection_HArray1<double>? TrueIntervals)
    {
        myDegree = 0;
        myDone = false;
        if (NumCurves <= 0 || NumCoeffPerCurve is null || Coefficients is null
            || PolynomialIntervals is null || TrueIntervals is null || Continuity < 0 || MaxDegree <= 0
            || Dimension <= 0 || PolynomialIntervals.RowLength() != 2)
        {
            throw new Standard_ConstructionError("Convert_CompPolynomialToPoles:bad arguments");
        }

        int aDelta = NumCurves - 1;
        for (int ii = NumCoeffPerCurve.Lower(); ii <= NumCoeffPerCurve.Lower() + aDelta; ii++)
        {
            myDegree = Max(NumCoeffPerCurve.Value(ii) - 1, myDegree);
        }
        if ((Continuity > myDegree) && (NumCurves > 1))
        {
            throw new Standard_ConstructionError("Convert_CompPolynomialToPoles:Continuity is too great");
        }

        myKnots = new NCollection_Array1<double>(1, NumCurves + 1);
        for (int ii = 1, Tindex = TrueIntervals.Lower(); ii <= NumCurves + 1; ii++, Tindex++)
        {
            myKnots.SetValue(ii, TrueIntervals.Value(Tindex));
        }

        int aMultiplicities = myDegree - Continuity;
        myMults = new NCollection_Array1<int>(1, NumCurves + 1);
        for (int ii = 2; ii < NumCurves + 1; ii++)
        {
            myMults.SetValue(ii, aMultiplicities);
        }
        myMults.SetValue(1, myDegree + 1);
        myMults.SetValue(NumCurves + 1, myDegree + 1);

        Perform(NumCurves,
                MaxDegree,
                Dimension,
                NumCoeffPerCurve.Array1(),
                Coefficients.Array1(),
                PolynomialIntervals.Array2(),
                TrueIntervals.Array1());
    }

    /// <summary>
    /// Converts <paramref name="NumCurves"/> polynomials with a continuity of its own at each junction
    /// (<paramref name="Continuity"/>(ii) between curves ii - 1 and ii, for ii from 2 to NumCurves).
    /// </summary>
    public Convert_CompPolynomialToPoles(int NumCurves,
                                         int Dimension,
                                         int MaxDegree,
                                         NCollection_Array1<int> Continuity,
                                         NCollection_Array1<int> NumCoeffPerCurve,
                                         NCollection_Array1<double> Coefficients,
                                         NCollection_Array2<double> PolynomialIntervals,
                                         NCollection_Array1<double> TrueIntervals)
    {
        myDegree = 0;
        myDone = false;
        if (NumCurves <= 0 || MaxDegree <= 0 || Dimension <= 0 || PolynomialIntervals.RowLength() != 2)
        {
            throw new Standard_ConstructionError("Convert_CompPolynomialToPoles:bad arguments");
        }

        int aDelta = NumCurves - 1;
        for (int ii = NumCoeffPerCurve.Lower(); ii <= NumCoeffPerCurve.Lower() + aDelta; ii++)
        {
            myDegree = Max(NumCoeffPerCurve.Value(ii) - 1, myDegree);
        }

        myKnots = new NCollection_Array1<double>(1, NumCurves + 1);
        for (int ii = 1, Tindex = TrueIntervals.Lower(); ii <= NumCurves + 1; ii++, Tindex++)
        {
            myKnots.SetValue(ii, TrueIntervals.Value(Tindex));
        }

        myMults = new NCollection_Array1<int>(1, NumCurves + 1);
        for (int ii = 2; ii < NumCurves + 1; ii++)
        {
            if ((Continuity[ii] > myDegree) && (NumCurves > 1))
            {
                throw new Standard_ConstructionError("Convert_CompPolynomialToPoles:Continuity is too great");
            }

            myMults.SetValue(ii, myDegree - Continuity[ii]);
        }
        myMults.SetValue(1, myDegree + 1);
        myMults.SetValue(NumCurves + 1, myDegree + 1);

        Perform(NumCurves,
                MaxDegree,
                Dimension,
                NumCoeffPerCurve,
                Coefficients,
                PolynomialIntervals,
                TrueIntervals);
    }

    /// <summary>Converts one polynomial of degree <paramref name="Degree"/>.</summary>
    public Convert_CompPolynomialToPoles(int Dimension,
                                         int MaxDegree,
                                         int Degree,
                                         NCollection_Array1<double> Coefficients,
                                         NCollection_Array1<double> PolynomialIntervals,
                                         NCollection_Array1<double> TrueIntervals)
    {
        myDegree = Degree;
        myDone = false;
        if (MaxDegree <= 0 || Dimension <= 0 || PolynomialIntervals.Length() != 2)
        {
            throw new Standard_ConstructionError("Convert_CompPolynomialToPoles:bad arguments");
        }

        NCollection_Array2<double> ThePolynomialIntervals = new(1, 1, 1, 2);
        ThePolynomialIntervals.SetValue(1, 1, PolynomialIntervals[PolynomialIntervals.Lower()]);
        ThePolynomialIntervals.SetValue(1, 2, PolynomialIntervals[PolynomialIntervals.Upper()]);

        NCollection_Array1<int> NumCoeffPerCurve = new(1, 1);
        NumCoeffPerCurve[1] = Degree + 1;

        myKnots = new NCollection_Array1<double>(1, 2);
        myKnots.SetValue(1, TrueIntervals.Value(TrueIntervals.Lower()));
        myKnots.SetValue(2, TrueIntervals.Value(TrueIntervals.Lower() + 1));

        myMults = new NCollection_Array1<int>(1, 2);
        myMults.Init(myDegree + 1);

        Perform(1,
                MaxDegree,
                Dimension,
                NumCoeffPerCurve,
                Coefficients,
                ThePolynomialIntervals,
                TrueIntervals);
    }

    private void Perform(int NumCurves,
                         int MaxDegree,
                         int Dimension,
                         NCollection_Array1<int> NumCoeffPerCurve,
                         NCollection_Array1<double> Coefficients,
                         NCollection_Array2<double> PolynomialIntervals,
                         NCollection_Array1<double> TrueIntervals)
    {
        int ii, num_flat_knots, index, Tindex, Pindex, coeff_index, inversion_problem, poles_index,
          num_poles;
        double normalized_value;

        num_flat_knots = 2 * myDegree + 2;
        for (ii = 2; ii < myMults.Length(); ii++)
        {
            num_flat_knots += myMults.Value(ii);
        }
        num_poles = num_flat_knots - myDegree - 1;

        myFlatKnots = new NCollection_Array1<double>(1, num_flat_knots);
        BSplCLib.KnotSequence(myKnots, myMults, myDegree, false, myFlatKnots);

        // Where OCCT uses stack buffers of 128 elements (when large enough), the port allocates
        NCollection_Array1<double> parameters = new(1, num_poles);
        BSplCLib.BuildSchoenbergPoints(myDegree, myFlatKnots, parameters);
        myPoles = new NCollection_Array2<double>(1, num_poles, 1, Dimension);
        index = 2;
        Tindex = TrueIntervals.Lower() + 1;
        Pindex = PolynomialIntervals.LowerRow();
        Span<double> poles_array = myPoles.AsSpan();

        NCollection_Array1<int> contact_array = new(1, num_poles);

        poles_index = 0;
        for (ii = 1; ii <= num_poles; ii++, poles_index += Dimension)
        {
            contact_array.SetValue(ii, 0);
            while (parameters.Value(ii) >= TrueIntervals[Tindex] && index <= NumCurves)
            {
                index++;
                Tindex++;
                Pindex++;
            }
            //
            // normalized value so that it fits the original intervals for
            // the polynomial definition of the curves
            //
            normalized_value = parameters.Value(ii) - TrueIntervals[Tindex - 1];
            normalized_value /= TrueIntervals[Tindex] - TrueIntervals[Tindex - 1];
            normalized_value =
              (1.0e0 - normalized_value) * PolynomialIntervals[Pindex, PolynomialIntervals.LowerCol()]
              + normalized_value * PolynomialIntervals[Pindex, PolynomialIntervals.UpperCol()];
            coeff_index =
              ((index - 2) * Dimension * (Max(MaxDegree, myDegree) + 1)) + Coefficients.Lower();

            ReadOnlySpan<double> coefficient_array = Coefficients.AsSpan(coeff_index);
            int Deg = NumCoeffPerCurve[NumCoeffPerCurve.Lower() + index - 2] - 1;

            PLib.NoDerivativeEvalPolynomial(normalized_value,
                                            Deg,
                                            Dimension,
                                            Deg * Dimension,
                                            coefficient_array,
                                            poles_array[poles_index..]);
        }
        //
        // interpolation at schoenberg points should yield the desired
        // result
        //
        BSplCLib.Interpolate(myDegree,
                             myFlatKnots,
                             parameters,
                             contact_array,
                             Dimension,
                             poles_array,
                             out inversion_problem);
        if (inversion_problem != 0)
        {
            throw new Standard_ConstructionError("Convert_CompPolynomialToPoles:inversion_problem");
        }
        myDone = true;
    }

    /// <summary>The number of poles of the B-spline (0 if not done).</summary>
    public int NbPoles()
    {
        if (myDone)
        {
            return myPoles.ColLength();
        }
        return 0;
    }

    /// <summary>The poles of the B-spline, [1..NumPoles][1..Dimension]: a copy.</summary>
    public NCollection_Array2<double> Poles()
    {
        StdFail_NotDone_Raise_if(!myDone, "Convert_CompPolynomialToPoles::Poles");
        return new NCollection_Array2<double>(myPoles);
    }

    /// <summary>
    /// Sets <paramref name="P"/> to a copy of the poles, if done (unchanged otherwise). Deprecated in OCCT: use
    /// <see cref="Poles()"/>.
    /// </summary>
    public void Poles(ref NCollection_HArray2<double>? P)
    {
        if (myDone)
        {
            P = new NCollection_HArray2<double>(myPoles);
        }
    }

    /// <summary>The degree of the B-spline (0 if not done).</summary>
    public int Degree()
    {
        if (myDone)
        {
            return myDegree;
        }
        return 0;
    }

    /// <summary>The number of knots of the B-spline (0 if not done).</summary>
    public int NbKnots()
    {
        if (myDone)
        {
            return myKnots.Length();
        }
        return 0;
    }

    /// <summary>The knots of the B-spline: a copy.</summary>
    public NCollection_Array1<double> Knots()
    {
        StdFail_NotDone_Raise_if(!myDone, "Convert_CompPolynomialToPoles::Knots");
        return new NCollection_Array1<double>(myKnots);
    }

    /// <summary>
    /// Sets <paramref name="K"/> to a copy of the knots, if done (unchanged otherwise). Deprecated in OCCT: use
    /// <see cref="Knots()"/>.
    /// </summary>
    public void Knots(ref NCollection_HArray1<double>? K)
    {
        if (myDone)
        {
            K = new NCollection_HArray1<double>(myKnots);
        }
    }

    /// <summary>The multiplicities of the knots: a copy.</summary>
    public NCollection_Array1<int> Multiplicities()
    {
        StdFail_NotDone_Raise_if(!myDone, "Convert_CompPolynomialToPoles::Multiplicities");
        return new NCollection_Array1<int>(myMults);
    }

    /// <summary>
    /// Sets <paramref name="M"/> to a copy of the multiplicities, if done (unchanged otherwise). Deprecated in
    /// OCCT: use <see cref="Multiplicities()"/>.
    /// </summary>
    public void Multiplicities(ref NCollection_HArray1<int>? M)
    {
        if (myDone)
        {
            M = new NCollection_HArray1<int>(myMults);
        }
    }

    /// <summary>True if the conversion succeeded.</summary>
    public bool IsDone()
    {
        return myDone;
    }
}
