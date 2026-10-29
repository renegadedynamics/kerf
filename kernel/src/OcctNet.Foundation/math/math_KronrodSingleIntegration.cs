// Port of: src/FoundationClasses/TKMath/math/math_KronrodSingleIntegration.hxx
// Port of: src/FoundationClasses/TKMath/math/math_KronrodSingleIntegration.lxx
// Port of: src/FoundationClasses/TKMath/math/math_KronrodSingleIntegration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The Gauss-Kronrod method of computing an integral, with adaptive subdivision.</summary>
public class math_KronrodSingleIntegration
{
    private bool myIsDone;
    private double myValue;
    private double myErrorReached;
    private double myAbsolutError;
    private int myNbPntsReached;
    private int myNbIterReached;

    public math_KronrodSingleIntegration()
    {
        myIsDone = false;
        myValue = 0.0;
        myErrorReached = 0.0;
        myAbsolutError = 0.0;
        myNbPntsReached = 0;
        myNbIterReached = 0;
    }

    /// <summary>Integrates the function between the bounds with the given initial number of Kronrod points.</summary>
    public math_KronrodSingleIntegration(math_Function theFunction, double theLower, double theUpper, int theNbPnts)
        : this()
    {
        Perform(theFunction, theLower, theUpper, theNbPnts);
    }

    /// <summary>
    /// Integrates the function between the bounds with the given initial number of Kronrod points, relative
    /// tolerance and maximal number of iterations.
    /// </summary>
    public math_KronrodSingleIntegration(math_Function theFunction, double theLower, double theUpper, int theNbPnts,
                                         double theTolerance, int theMaxNbIter)
        : this()
    {
        Perform(theFunction, theLower, theUpper, theNbPnts, theTolerance, theMaxNbIter);
    }

    /// <summary>
    /// Computes the integral with the given initial number of Kronrod points (odd, at least 3; an even number is
    /// increased by one).
    /// </summary>
    public void Perform(math_Function theFunction, double theLower, double theUpper, int theNbPnts)
    {
        // const double aMinVol = Epsilon(1.);
        const double aPtol = 1.0e-9;
        myNbIterReached = 0;

        if (theNbPnts < 3)
        {
            myIsDone = false;
            return;
        }

        if (theUpper - theLower < aPtol)
        {
            myIsDone = false;
            return;
        }

        // Get an odd value of number of initial points.
        myNbPntsReached = (theNbPnts % 2 == 0) ? theNbPnts + 1 : theNbPnts;
        myErrorReached = RealLast();

        int aNGauss = myNbPntsReached / 2;
        math_Vector aKronrodP = new(1, myNbPntsReached);
        math_Vector aKronrodW = new(1, myNbPntsReached);
        math_Vector aGaussP = new(1, aNGauss);
        math_Vector aGaussW = new(1, aNGauss);

        if (!math.KronrodPointsAndWeights(myNbPntsReached, aKronrodP, aKronrodW)
            || !math.OrderedGaussPointsAndWeights(aNGauss, aGaussP, aGaussW))
        {
            myIsDone = false;
            return;
        }

        myIsDone = GKRule(theFunction, theLower, theUpper, aGaussP, aGaussW, aKronrodP, aKronrodW,
                          out myValue, ref myErrorReached);

        if (!myIsDone)
        {
            return;
        }

        // double anAbsVal = std::abs(myValue);

        myAbsolutError = myErrorReached;

        // if (anAbsVal > aMinVol)
        // myErrorReached /= anAbsVal;

        myNbIterReached++;
    }

    /// <summary>
    /// Computes the integral with the given initial number of Kronrod points (odd, at least 3), subdividing the
    /// interval of largest error until abs(Kronrod - Gauss)/abs(Kronrod) &lt; <paramref name="theTolerance"/>
    /// (relative, positive) or <paramref name="theMaxNbIter"/> iterations are done.
    /// </summary>
    public void Perform(math_Function theFunction, double theLower, double theUpper, int theNbPnts,
                        double theTolerance, int theMaxNbIter)
    {
        double aMinVol = Epsilon(1.0);
        myNbIterReached = 0;

        // Check prerequisites.
        if (theNbPnts < 3 || theTolerance <= 0.0)
        {
            myIsDone = false;
            return;
        }

        // Get an odd value of number of initial points.
        myNbPntsReached = (theNbPnts % 2 == 0) ? theNbPnts + 1 : theNbPnts;

        int aNGauss = myNbPntsReached / 2;
        math_Vector aKronrodP = new(1, myNbPntsReached);
        math_Vector aKronrodW = new(1, myNbPntsReached);
        math_Vector aGaussP = new(1, aNGauss);
        math_Vector aGaussW = new(1, aNGauss);

        if (!math.KronrodPointsAndWeights(myNbPntsReached, aKronrodP, aKronrodW)
            || !math.OrderedGaussPointsAndWeights(aNGauss, aGaussP, aGaussW))
        {
            myIsDone = false;
            return;
        }

        // First iteration
        myIsDone = GKRule(theFunction, theLower, theUpper, aGaussP, aGaussW, aKronrodP, aKronrodW,
                          out myValue, ref myErrorReached);

        if (!myIsDone)
        {
            return;
        }

        double anAbsVal = Math.Abs(myValue);

        myAbsolutError = myErrorReached;

        if (anAbsVal > aMinVol)
        {
            myErrorReached /= anAbsVal;
        }

        myNbIterReached++;

        if (myErrorReached <= theTolerance)
        {
            return;
        }

        if (myNbIterReached >= theMaxNbIter)
        {
            return;
        }

        NCollection_Sequence<double> anIntervals = new();
        NCollection_Sequence<double> anErrors = new();
        NCollection_Sequence<double> aValues = new();
        anIntervals.Append(theLower);
        anIntervals.Append(theUpper);
        anErrors.Append(myAbsolutError);
        aValues.Append(myValue);

        int i, nint, nbints;

        double maxerr;
        int count = 0;
        while (myErrorReached > theTolerance && myNbIterReached < theMaxNbIter)
        {
            // Searching interval with max error
            nbints = anIntervals.Length() - 1;
            nint = 0;
            maxerr = 0.0;
            for (i = 1; i <= nbints; ++i)
            {
                if (anErrors[i] > maxerr)
                {
                    maxerr = anErrors[i];
                    nint = i;
                }
            }

            double a = anIntervals[nint];
            double b = anIntervals[nint + 1];
            double c = 0.5 * (a + b);

            double v1, v2, e1 = 0.0, e2 = 0.0;
            myIsDone = GKRule(theFunction, a, c, aGaussP, aGaussW, aKronrodP, aKronrodW, out v1, ref e1);
            if (!myIsDone)
            {
                return;
            }

            myIsDone = GKRule(theFunction, c, b, aGaussP, aGaussW, aKronrodP, aKronrodW, out v2, ref e2);
            if (!myIsDone)
            {
                return;
            }

            myNbIterReached++;

            double deltav = v1 + v2 - aValues[nint];
            myValue += deltav;
            if (Math.Abs(deltav) <= Epsilon(Math.Abs(myValue)))
            {
                ++count;
            }

            double deltae = e1 + e2 - anErrors[nint];
            myAbsolutError += deltae;
            if (myAbsolutError <= Epsilon(Math.Abs(myValue)))
            {
                ++count;
            }

            if (Math.Abs(myValue) > aMinVol)
            {
                myErrorReached = myAbsolutError / Math.Abs(myValue);
            }
            else
            {
                myErrorReached = myAbsolutError;
            }

            if (count > 50)
            {
                return;
            }

            // Inserting new interval

            anIntervals.InsertAfter(nint, c);
            anErrors[nint] = e1;
            anErrors.InsertAfter(nint, e2);
            aValues[nint] = v1;
            aValues.InsertAfter(nint, v2);
        }
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The value of the integral. Throws <see cref="StdFail_NotDone"/> if it was not computed.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "math_KronrodSingleIntegration");
        return myValue;
    }

    /// <summary>The relative error reached.</summary>
    public double ErrorReached()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "math_KronrodSingleIntegration");
        return myErrorReached;
    }

    /// <summary>The absolute error reached.</summary>
    public double AbsolutError()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "math_KronrodSingleIntegration");
        return myAbsolutError;
    }

    /// <summary>The number of Kronrod points of the result.</summary>
    public int OrderReached()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "math_KronrodSingleIntegration");
        return myNbPntsReached;
    }

    /// <summary>The number of iterations made.</summary>
    public int NbIterReached()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "math_KronrodSingleIntegration");
        return myNbIterReached;
    }

    /// <summary>
    /// One Gauss-Kronrod rule on [theLower, theUpper]: the Kronrod value and the error estimate. False if the function
    /// fails. <paramref name="theError"/> is <c>ref</c> because OCCT leaves it unchanged when the function fails.
    /// </summary>
    public static bool GKRule(math_Function theFunction, double theLower, double theUpper,
                              math_Vector theGaussP, math_Vector theGaussW,
                              math_Vector theKronrodP, math_Vector theKronrodW,
                              out double theValue, ref double theError)
    {
        _ = theGaussP;
        bool IsDone;
        int aNKronrod = theKronrodP.Length();

        double aGaussVal;
        int aNPnt2 = (aNKronrod + 1) / 2;
        int i;
        double aDx;
        double aVal1;
        double aVal2;

        math_Vector f1 = new(1, aNPnt2 - 1);
        math_Vector f2 = new(1, aNPnt2 - 1);

        double aXm = 0.5 * (theUpper + theLower);
        double aXr = 0.5 * (theUpper - theLower);

        // Compute Gauss quadrature
        aGaussVal = 0.0;
        theValue = 0.0;

        for (i = 2; i < aNPnt2; i += 2)
        {
            aDx = aXr * theKronrodP.Value(i);

            aVal2 = 0.0; // C++ leaves it unset when the first call fails
            if (!theFunction.Value(aXm + aDx, out aVal1) || !theFunction.Value(aXm - aDx, out aVal2))
            {
                IsDone = false;
                return IsDone;
            }

            f1[i] = aVal1;
            f2[i] = aVal2;
            aGaussVal += (aVal1 + aVal2) * theGaussW.Value(i / 2);
            theValue += (aVal1 + aVal2) * theKronrodW.Value(i);
        }

        // Compute value in the middle point.
        if (!theFunction.Value(aXm, out aVal1))
        {
            IsDone = false;
            return IsDone;
        }

        double fc = aVal1;
        theValue += aVal1 * theKronrodW.Value(aNPnt2);

        if (i == aNPnt2)
        {
            aGaussVal += aVal1 * theGaussW.Value(aNPnt2 / 2);
        }

        // Compute Kronrod quadrature
        for (i = 1; i < aNPnt2; i += 2)
        {
            aDx = aXr * theKronrodP.Value(i);

            aVal2 = 0.0; // C++ leaves it unset when the first call fails
            if (!theFunction.Value(aXm + aDx, out aVal1) || !theFunction.Value(aXm - aDx, out aVal2))
            {
                IsDone = false;
                return IsDone;
            }

            f1[i] = aVal1;
            f2[i] = aVal2;
            theValue += (aVal1 + aVal2) * theKronrodW.Value(i);
        }

        double mean = 0.5 * theValue;

        double asc = Math.Abs(fc - mean) * theKronrodW.Value(aNPnt2);
        for (i = 1; i < aNPnt2; ++i)
        {
            asc += theKronrodW.Value(i) * (Math.Abs(f1[i] - mean) + Math.Abs(f2[i] - mean));
        }

        asc *= aXr;
        theValue *= aXr;
        aGaussVal *= aXr;

        // Compute the error and the new number of Kronrod points.

        theError = Math.Abs(theValue - aGaussVal);

        double scale = 1.0;
        if (asc != 0.0 && theError != 0.0)
        {
            scale = Math.Pow((200.0 * theError / asc), 1.5);
        }

        if (scale < 1.0)
        {
            theError = Min(theError, asc * scale);
        }

        // theFunction.GetStateNumber();

        return true;
    }
}
