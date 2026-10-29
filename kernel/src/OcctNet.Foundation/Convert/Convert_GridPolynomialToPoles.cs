// Port of: src/FoundationClasses/TKMath/Convert/Convert_GridPolynomialToPoles.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_GridPolynomialToPoles.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts a grid of polynomial surfaces (3D) into the poles of an equivalent B-spline surface, by
/// interpolation at the Schoenberg points.
/// </summary>
/// <remarks>The batch accessors return copies, where OCCT returns references to its fields.</remarks>
public class Convert_GridPolynomialToPoles
{
    private NCollection_Array1<double> myUFlatKnots = new();
    private NCollection_Array1<double> myVFlatKnots = new();
    private NCollection_Array1<double> myUKnots = new();
    private NCollection_Array1<double> myVKnots = new();
    private NCollection_Array1<int> myUMults = new();
    private NCollection_Array1<int> myVMults = new();
    private NCollection_Array2<gp_Pnt> myPoles = new();
    private int myUDegree;
    private int myVDegree;
    private bool myDone;

    /// <summary>
    /// Handle form of <see cref="Convert_GridPolynomialToPoles(int, int, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{double})"/>.
    /// </summary>
    public Convert_GridPolynomialToPoles(int theMaxUDegree,
                                         int theMaxVDegree,
                                         NCollection_HArray1<int> theNumCoeff,
                                         NCollection_HArray1<double> theCoefficients,
                                         NCollection_HArray1<double> thePolynomialUIntervals,
                                         NCollection_HArray1<double> thePolynomialVIntervals)
        : this(theMaxUDegree,
               theMaxVDegree,
               theNumCoeff.Array1(),
               theCoefficients.Array1(),
               thePolynomialUIntervals.Array1(),
               thePolynomialVIntervals.Array1())
    {
    }

    /// <summary>
    /// Handle form of <see cref="Convert_GridPolynomialToPoles(int, int, int, int, int, int, NCollection_Array2{int}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{double})"/>.
    /// </summary>
    public Convert_GridPolynomialToPoles(int theNbUSurfaces,
                                         int theNbVSurfaces,
                                         int theUContinuity,
                                         int theVContinuity,
                                         int theMaxUDegree,
                                         int theMaxVDegree,
                                         NCollection_HArray2<int> theNumCoeffPerSurface,
                                         NCollection_HArray1<double> theCoefficients,
                                         NCollection_HArray1<double> thePolynomialUIntervals,
                                         NCollection_HArray1<double> thePolynomialVIntervals,
                                         NCollection_HArray1<double> theTrueUIntervals,
                                         NCollection_HArray1<double> theTrueVIntervals)
        : this(theNbUSurfaces,
               theNbVSurfaces,
               theUContinuity,
               theVContinuity,
               theMaxUDegree,
               theMaxVDegree,
               theNumCoeffPerSurface.Array2(),
               theCoefficients.Array1(),
               thePolynomialUIntervals.Array1(),
               thePolynomialVIntervals.Array1(),
               theTrueUIntervals.Array1(),
               theTrueVIntervals.Array1())
    {
    }

    /// <summary>
    /// Converts one polynomial surface. <paramref name="thePolynomialUIntervals"/> and
    /// <paramref name="thePolynomialVIntervals"/> (2 values each) are the domain of the polynomial;
    /// <paramref name="theNumCoeff"/> (bounds 1..2) the numbers of coefficients in U and V; the coefficients are a
    /// "C array" [MaxUDegree + 1][MaxVDegree + 1][3] (bounds from 1).
    /// </summary>
    public Convert_GridPolynomialToPoles(int theMaxUDegree,
                                         int theMaxVDegree,
                                         NCollection_Array1<int> theNumCoeff,
                                         NCollection_Array1<double> theCoefficients,
                                         NCollection_Array1<double> thePolynomialUIntervals,
                                         NCollection_Array1<double> thePolynomialVIntervals)
    {
        myUDegree = 0;
        myVDegree = 0;
        myDone = false;
        if (theNumCoeff.Lower() != 1 || theNumCoeff.Upper() != 2)
        {
            throw new Standard_DomainError("Convert : Wrong Coefficients");
        }
        if (theCoefficients.Lower() != 1
            || theCoefficients.Upper() != 3 * (theMaxUDegree + 1) * (theMaxVDegree + 1))
        {
            throw new Standard_DomainError("Convert : Wrong Coefficients");
        }

        myUDegree = theNumCoeff.Value(1) - 1;
        myVDegree = theNumCoeff.Value(2) - 1;

        if (myUDegree > theMaxUDegree)
        {
            throw new Standard_DomainError("Convert : Incoherence between NumCoeffPerSurface and MaxUDegree");
        }
        if (myVDegree > theMaxVDegree)
        {
            throw new Standard_DomainError("Convert : Incoherence between NumCoeffPerSurface and MaxVDegree");
        }

        NCollection_Array2<int> aNumCoeff2D = new(1, 1, 1, 2);
        aNumCoeff2D.SetValue(1, 1, theNumCoeff.Value(1));
        aNumCoeff2D.SetValue(1, 2, theNumCoeff.Value(2));

        Perform(0,
                0,
                theMaxUDegree,
                theMaxVDegree,
                aNumCoeff2D,
                theCoefficients,
                thePolynomialUIntervals,
                thePolynomialVIntervals,
                thePolynomialUIntervals,
                thePolynomialVIntervals);
    }

    /// <summary>
    /// Converts a grid of NbUSurfaces x NbVSurfaces polynomial surfaces, with the given continuity in each
    /// direction (at most the maximum degree). <paramref name="theTrueUIntervals"/> and
    /// <paramref name="theTrueVIntervals"/> are the parametrisation of the result.
    /// <paramref name="theNumCoeffPerSurface"/> is a [1, NbVSurfaces * NbUSurfaces, 1, 2] array, the coefficients a
    /// "C array" [NbVSurfaces][NbUSurfaces][MaxUDegree + 1][MaxVDegree + 1][3] (bounds from 1);
    /// <see cref="Standard_DomainError"/> otherwise.
    /// </summary>
    public Convert_GridPolynomialToPoles(int theNbUSurfaces,
                                         int theNbVSurfaces,
                                         int theUContinuity,
                                         int theVContinuity,
                                         int theMaxUDegree,
                                         int theMaxVDegree,
                                         NCollection_Array2<int> theNumCoeffPerSurface,
                                         NCollection_Array1<double> theCoefficients,
                                         NCollection_Array1<double> thePolynomialUIntervals,
                                         NCollection_Array1<double> thePolynomialVIntervals,
                                         NCollection_Array1<double> theTrueUIntervals,
                                         NCollection_Array1<double> theTrueVIntervals)
    {
        myUDegree = 0;
        myVDegree = 0;
        myDone = false;
        int aRealUDegree = Max(theMaxUDegree, 2 * theUContinuity + 1);
        int aRealVDegree = Max(theMaxVDegree, 2 * theVContinuity + 1);

        if (theNumCoeffPerSurface.LowerRow() != 1
            || theNumCoeffPerSurface.UpperRow() != theNbUSurfaces * theNbVSurfaces
            || theNumCoeffPerSurface.LowerCol() != 1 || theNumCoeffPerSurface.UpperCol() != 2)
        {
            throw new Standard_DomainError("Convert : Wrong NumCoeffPerSurface");
        }

        if (theCoefficients.Lower() != 1
            || theCoefficients.Upper()
                 != 3 * theNbUSurfaces * theNbVSurfaces * (aRealUDegree + 1) * (aRealVDegree + 1))
        {
            throw new Standard_DomainError("Convert : Wrong Coefficients");
        }

        // Compute actual degrees from the supplied coefficient counts.
        for (int ii = 1; ii <= theNbUSurfaces * theNbVSurfaces; ++ii)
        {
            if (theNumCoeffPerSurface.Value(ii, 1) > myUDegree + 1)
            {
                myUDegree = theNumCoeffPerSurface.Value(ii, 1) - 1;
            }
            if (theNumCoeffPerSurface.Value(ii, 2) > myVDegree + 1)
            {
                myVDegree = theNumCoeffPerSurface.Value(ii, 2) - 1;
            }
        }

        if (myUDegree > aRealUDegree)
        {
            throw new Standard_DomainError("Convert : Incoherence between NumCoeffPerSurface and MaxUDegree");
        }
        if (myVDegree > aRealVDegree)
        {
            throw new Standard_DomainError("Convert : Incoherence between NumCoeffPerSurface and MaxVDegree");
        }

        Perform(theUContinuity,
                theVContinuity,
                aRealUDegree,
                aRealVDegree,
                theNumCoeffPerSurface,
                theCoefficients,
                thePolynomialUIntervals,
                thePolynomialVIntervals,
                theTrueUIntervals,
                theTrueVIntervals);
    }

    private void Perform(int theUContinuity,
                         int theVContinuity,
                         int theMaxUDegree,
                         int theMaxVDegree,
                         NCollection_Array2<int> theNumCoeffPerSurface,
                         NCollection_Array1<double> theCoefficients,
                         NCollection_Array1<double> thePolynomialUIntervals,
                         NCollection_Array1<double> thePolynomialVIntervals,
                         NCollection_Array1<double> theTrueUIntervals,
                         NCollection_Array1<double> theTrueVIntervals)
    {
        // (1) Build monodimensional knot/multiplicity/parameter tables.
        NCollection_Array1<double> aUParameters;
        NCollection_Array1<double> aVParameters;
        myUKnots = new NCollection_Array1<double>(theTrueUIntervals);
        myVKnots = new NCollection_Array1<double>(theTrueVIntervals);

        BuildArray(myUDegree, myUKnots, theUContinuity, out myUFlatKnots, out myUMults, out aUParameters);
        BuildArray(myVDegree, myVKnots, theVContinuity, out myVFlatKnots, out myVMults, out aVParameters);

        // (2) Digitalisation
        int aDimension = 3 * (myVDegree + 1);
        int aSizPatch = 3 * (theMaxUDegree + 1) * (theMaxVDegree + 1);
        myPoles = new NCollection_Array2<gp_Pnt>(1, aUParameters.Length(), 1, aVParameters.Length());

        NCollection_Array1<double> aPatch = new(1, (myUDegree + 1) * aDimension);
        Span<double> aDigit = stackalloc double[3];
        ReadOnlySpan<double> aCoeffs = aPatch.AsSpan();

        int aPatchIndice = 0;
        int aUIndex = 1;
        for (int ii = 1; ii <= aUParameters.Length(); ++ii)
        {
            while (aUParameters.Value(ii) > theTrueUIntervals.Value(aUIndex + 1)
                   && aUIndex < myUKnots.Length() - 1)
            {
                ++aUIndex;
            }
            double aNValue = (aUParameters.Value(ii) - theTrueUIntervals.Value(aUIndex))
                             / (theTrueUIntervals.Value(aUIndex + 1) - theTrueUIntervals.Value(aUIndex));
            double aUValue = (1.0 - aNValue) * thePolynomialUIntervals.Value(1)
                             + aNValue * thePolynomialUIntervals.Value(2);

            int aVIndex = 1;
            for (int jj = 1; jj <= aVParameters.Length(); ++jj)
            {
                while (aVParameters.Value(jj) > theTrueVIntervals.Value(aVIndex + 1)
                       && aVIndex < myVKnots.Length() - 1)
                {
                    ++aVIndex;
                }
                aNValue = (aVParameters.Value(jj) - theTrueVIntervals.Value(aVIndex))
                          / (theTrueVIntervals.Value(aVIndex + 1) - theTrueVIntervals.Value(aVIndex));
                double aVValue = (1.0 - aNValue) * thePolynomialVIntervals.Value(1)
                                 + aNValue * thePolynomialVIntervals.Value(2);

                // (2.1) Extract the active patch coefficients.
                if (aPatchIndice != aUIndex + (myUKnots.Length() - 1) * (aVIndex - 1))
                {
                    aPatchIndice = aUIndex + (myUKnots.Length() - 1) * (aVIndex - 1);
                    int ll = 1;
                    for (int k1 = 1; k1 <= theNumCoeffPerSurface.Value(aPatchIndice, 1); ++k1)
                    {
                        int pos = aSizPatch * (aPatchIndice - 1) + 3 * (theMaxVDegree + 1) * (k1 - 1) + 1;
                        for (int k2 = 1; k2 <= theNumCoeffPerSurface.Value(aPatchIndice, 2); ++k2, pos += 3)
                        {
                            aPatch[ll] = theCoefficients.Value(pos);
                            aPatch[ll + 1] = theCoefficients.Value(pos + 1);
                            aPatch[ll + 2] = theCoefficients.Value(pos + 2);
                            ll += 3;
                        }
                    }
                }

                // (2.2) Evaluate at (aUValue, aVValue).
                PLib.EvalPoly2Var(aUValue,
                                  aVValue,
                                  0,
                                  0,
                                  theNumCoeffPerSurface.Value(aPatchIndice, 1) - 1,
                                  theNumCoeffPerSurface.Value(aPatchIndice, 2) - 1,
                                  3,
                                  aCoeffs,
                                  aDigit);

                myPoles.SetValue(ii, jj, new gp_Pnt(aDigit[0], aDigit[1], aDigit[2]));
            }
        }

        // (3)Interpolation --------------------------------------------------------------

        BSplSLib.Interpolate(myUDegree,
                             myVDegree,
                             myUFlatKnots,
                             myVFlatKnots,
                             aUParameters,
                             aVParameters,
                             myPoles,
                             out int InversionProblem);
        myDone = (InversionProblem == 0);
    }

    /// <summary>The multiplicities, flat knots and Schoenberg parameters of one direction.</summary>
    /// <remarks>The arrays are outputs, where OCCT assigns new arrays to its reference parameters.</remarks>
    private static void BuildArray(int Degree,
                                   NCollection_Array1<double> Knots,
                                   int Continuity,
                                   out NCollection_Array1<double> FlatKnots,
                                   out NCollection_Array1<int> Mults,
                                   out NCollection_Array1<double> Parameters)
    {
        int NumCurves = Knots.Length() - 1;

        // Calcul des Multiplicites
        int multiplicities = Degree - Continuity;
        Mults = new NCollection_Array1<int>(1, Knots.Length());

        for (int ii = 2; ii < Knots.Length(); ii++)
        {
            Mults.SetValue(ii, multiplicities);
        }
        Mults.SetValue(1, Degree + 1);
        Mults.SetValue(NumCurves + 1, Degree + 1);

        // Calcul des Noeuds Plats
        int num_flat_knots = multiplicities * (NumCurves - 1) + 2 * Degree + 2;
        FlatKnots = new NCollection_Array1<double>(1, num_flat_knots);

        BSplCLib.KnotSequence(Knots, Mults, Degree, false, FlatKnots);

        // Calcul du nombre de Poles
        int num_poles = num_flat_knots - Degree - 1;

        // Cacul des parametres d'interpolation
        Parameters = new NCollection_Array1<double>(1, num_poles);
        BSplCLib.BuildSchoenbergPoints(Degree, FlatKnots, Parameters);
    }

    /// <summary>The number of poles in U.</summary>
    public int NbUPoles()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return myPoles.ColLength();
    }

    /// <summary>The number of poles in V.</summary>
    public int NbVPoles()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return myPoles.RowLength();
    }

    /// <summary>The poles of the B-spline surface: a copy.</summary>
    public NCollection_Array2<gp_Pnt> Poles()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return new NCollection_Array2<gp_Pnt>(myPoles);
    }

    /// <summary>The degree in U.</summary>
    public int UDegree()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return myUDegree;
    }

    /// <summary>The degree in V.</summary>
    public int VDegree()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return myVDegree;
    }

    /// <summary>The number of knots in U.</summary>
    public int NbUKnots()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return myUKnots.Length();
    }

    /// <summary>The number of knots in V.</summary>
    public int NbVKnots()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return myVKnots.Length();
    }

    /// <summary>The knots in U: a copy.</summary>
    public NCollection_Array1<double> UKnots()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return new NCollection_Array1<double>(myUKnots);
    }

    /// <summary>The knots in V: a copy.</summary>
    public NCollection_Array1<double> VKnots()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return new NCollection_Array1<double>(myVKnots);
    }

    /// <summary>The multiplicities of the U knots: a copy.</summary>
    public NCollection_Array1<int> UMultiplicities()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return new NCollection_Array1<int>(myUMults);
    }

    /// <summary>The multiplicities of the V knots: a copy.</summary>
    public NCollection_Array1<int> VMultiplicities()
    {
        StdFail_NotDone_Raise_if(!myDone, "GridPolynomialToPoles");
        return new NCollection_Array1<int>(myVMults);
    }

    /// <summary>True if the conversion succeeded.</summary>
    public bool IsDone()
    {
        return myDone;
    }
}
