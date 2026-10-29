// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_NormalPolyDef.hxx
// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_NormalPolyDef.cxx


namespace Occt;

/// <summary>
/// The polynomial F(X) = Sum_{i=0}^{k0} C(k0,i) * cos^i(X) * sin^(k0-i)(X) * li(i), whose sign tells whether the
/// normal at a singular point of a surface is defined (used by <see cref="CSLib"/>).
/// </summary>
public class CSLib_NormalPolyDef : math_FunctionWithDerivative
{
    private readonly int                        myK0;    // Polynomial degree
    private readonly NCollection_Array1<double> myTABli; // Coefficients li(i) for i = 0..myK0

    /// <summary>Creates the polynomial of degree <paramref name="theK0"/> with the coefficients <paramref name="theLi"/>(0..theK0).</summary>
    public CSLib_NormalPolyDef(int theK0, NCollection_Array1<double> theLi)
    {
        myK0    = theK0;
        myTABli = new NCollection_Array1<double>(0, theK0);
        for (int i = 0; i <= theK0; ++i)
        {
            myTABli[i] = theLi[i];
        }
    }

    public override bool Value(double theX, out double theF)
    {
        theF = 0.0;

        double aCos = Math.Cos(theX);
        double aSin = Math.Sin(theX);

        // At singular points (cos or sin near zero), return zero to avoid numerical instability.
        if (Math.Abs(aCos) <= RealSmall() || Math.Abs(aSin) <= RealSmall())
        {
            return true;
        }

        for (int i = 0; i <= myK0; ++i)
        {
            theF += PLib.Bin(myK0, i) * Math.Pow(aCos, i) * Math.Pow(aSin, myK0 - i) * myTABli[i];
        }

        return true;
    }

    public override bool Derivative(double theX, out double theD)
    {
        theD = 0.0;

        double aCos = Math.Cos(theX);
        double aSin = Math.Sin(theX);

        // At singular points, return zero derivative.
        if (Math.Abs(aCos) <= RealSmall() || Math.Abs(aSin) <= RealSmall())
        {
            return true;
        }

        // dF/dX = Sum_{i=0}^{k0} C(k0,i) * cos^(i-1)(X) * sin^(k0-i-1)(X) * (k0*cos^2(X) - i) * li(i)
        for (int i = 0; i <= myK0; ++i)
        {
            theD += PLib.Bin(myK0, i) * Math.Pow(aCos, i - 1) * Math.Pow(aSin, myK0 - i - 1)
                    * (myK0 * aCos * aCos - i) * myTABli[i];
        }

        return true;
    }

    public override bool Values(double theX, out double theF, out double theD)
    {
        theF = 0.0;
        theD = 0.0;

        double aCos = Math.Cos(theX);
        double aSin = Math.Sin(theX);

        // At singular points, return zeros.
        if (Math.Abs(aCos) <= RealSmall() || Math.Abs(aSin) <= RealSmall())
        {
            return true;
        }

        for (int i = 0; i <= myK0; ++i)
        {
            double aBinCoeff = PLib.Bin(myK0, i);
            double aLiCoeff  = myTABli[i];

            theF += aBinCoeff * Math.Pow(aCos, i) * Math.Pow(aSin, myK0 - i) * aLiCoeff;
            theD += aBinCoeff * Math.Pow(aCos, i - 1) * Math.Pow(aSin, myK0 - i - 1)
                    * (myK0 * aCos * aCos - i) * aLiCoeff;
        }

        return true;
    }
}
