#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Runs the port and native OCCT on the same random inputs and compares the results.
/// </summary>
/// <remarks>
/// Each comparison records the largest difference and how many results were bit-identical, and writes both to
/// the test output, so a drift in the port shows up even while it stays within tolerance.
/// </remarks>
internal sealed class Oracle(ITestOutputHelper output, int seed = 20261028)
{
    private readonly Random myRandom = new(seed);
    private double myMaxDiff;
    private int myValues;
    private int myExact;

    /// <summary>The default tolerance: relative to the magnitude, with an absolute floor for values near zero.</summary>
    public const double Tolerance = 1e-12;

    public double Real(double theMin = -100, double theMax = 100) => theMin + (theMax - theMin) * myRandom.NextDouble();

    public double Angle() => Real(-2 * Math.PI, 2 * Math.PI);

    public int Int(int theMin, int theMax) => myRandom.Next(theMin, theMax + 1);

    public gp_Pnt Pnt() => new(Real(), Real(), Real());

    public gp_Vec Vec() => new(Real(), Real(), Real());

    /// <summary>A random direction, kept away from zero so it can be normalised.</summary>
    public gp_Dir Dir()
    {
        while (true)
        {
            var aV = new gp_Vec(Real(-1, 1), Real(-1, 1), Real(-1, 1));
            if (aV.Magnitude() > 0.1)
            {
                return new gp_Dir(aV);
            }
        }
    }

    public gp_Ax1 Ax1() => new(Pnt(), Dir());

    public gp_Ax2 Ax2() => new(Pnt(), Dir());

    /// <summary>A coordinate system, left-handed half the time.</summary>
    public gp_Ax3 Ax3()
    {
        var anAx3 = new gp_Ax3(Pnt(), Dir(), Dir());
        return myRandom.Next(2) == 0 ? anAx3 : anAx3.ZReversed();
    }

    public bool Bool() => myRandom.Next(2) == 0;

    /// <summary>
    /// A direction that the port's normalisation leaves unchanged. <see cref="ToNative(in gp_Dir)"/> goes through
    /// the native constructor, which normalises again: about one in eight random directions moves by an ulp there,
    /// and about one in a hundred settled ones still does (native OCCT's normalisation doesn't always round as .NET's).
    /// </summary>
    public static gp_Dir Settled(gp_Dir theD)
    {
        for (int i = 0; i < 8; i++)
        {
            var aNext = new gp_Dir(theD.X(), theD.Y(), theD.Z());
            if (aNext == theD)
            {
                break;
            }
            theD = aNext;
        }

        return theD;
    }

    /// <summary>The 2D twin of <see cref="Settled(gp_Dir)"/>.</summary>
    public static gp_Dir2d Settled(gp_Dir2d theD)
    {
        for (int i = 0; i < 8; i++)
        {
            var aNext = new gp_Dir2d(theD.X(), theD.Y());
            if (aNext == theD)
            {
                break;
            }
            theD = aNext;
        }

        return theD;
    }

    public gp_Pnt2d Pnt2d() => new(Real(), Real());

    /// <summary>A random 2D direction, kept away from zero so it can be normalised, and <see cref="Settled(gp_Dir2d)"/>.</summary>
    public gp_Dir2d Dir2d()
    {
        while (true)
        {
            var aV = new gp_Vec2d(Real(-1, 1), Real(-1, 1));
            if (aV.Magnitude() > 0.1)
            {
                return Settled(new gp_Dir2d(aV));
            }
        }
    }

    public gp_Ax2d Ax2d() => new(Pnt2d(), Dir2d());

    /// <summary>A 2D coordinate system, indirect half the time.</summary>
    public gp_Ax22d Ax22d() => new(Pnt2d(), Dir2d(), Bool());

    // Conversions to and from the native wrappers. Each native object is disposed by the caller's `using`.

    public static N.gp_Pnt2d ToNative(in gp_Pnt2d theP) => new(theP.X(), theP.Y());

    public static N.gp_Dir2d ToNative(in gp_Dir2d theD) => new(theD.X(), theD.Y());

    public static N.gp_Ax2d ToNative(in gp_Ax2d theA)
    {
        using var aP = ToNative(theA.Location());
        using var aD = ToNative(theA.Direction());
        return new N.gp_Ax2d(aP, aD);
    }

    /// <summary>A native gp_Ax22d with exactly the port's two directions.</summary>
    public static N.gp_Ax22d ToNative(in gp_Ax22d theA)
    {
        using var aP = ToNative(theA.Location());
        using var aX = ToNative(theA.XDirection());
        using var aY = ToNative(theA.YDirection());
        return new N.gp_Ax22d(aP, aX, aY);
    }

    public static gp_Pnt2d FromNative(N.gp_Pnt2d theP) => new(theP.X(), theP.Y());

    public static N.gp_Pnt ToNative(in gp_Pnt theP) => new(theP.X(), theP.Y(), theP.Z());

    public static N.gp_Vec ToNative(in gp_Vec theV) => new(theV.X(), theV.Y(), theV.Z());

    public static N.gp_Dir ToNative(in gp_Dir theD) => new(theD.X(), theD.Y(), theD.Z());

    public static N.gp_XYZ ToNative(in gp_XYZ theXYZ) => new(theXYZ.X(), theXYZ.Y(), theXYZ.Z());

    public static N.gp_Ax1 ToNative(in gp_Ax1 theA)
    {
        using var aP = ToNative(theA.Location());
        using var aD = ToNative(theA.Direction());
        return new N.gp_Ax1(aP, aD);
    }

    public static N.gp_Ax2 ToNative(in gp_Ax2 theA)
    {
        using var aP = ToNative(theA.Location());
        using var aN = ToNative(theA.Direction());
        using var aX = ToNative(theA.XDirection());
        return new N.gp_Ax2(aP, aN, aX);
    }

    /// <summary>A native gp_Ax3 with exactly the port's three directions (a left-handed system stays left-handed).</summary>
    public static N.gp_Ax3 ToNative(in gp_Ax3 theA)
    {
        using var aP = ToNative(theA.Location());
        using var aN = ToNative(theA.Direction());
        using var aX = ToNative(theA.XDirection());
        var aNative = new N.gp_Ax3(aP, aN, aX);
        if (!theA.Direct())
        {
            aNative.YReverse();
        }

        return aNative;
    }

    public static gp_XYZ FromNative(N.gp_XYZ theXYZ) => new(theXYZ.X(), theXYZ.Y(), theXYZ.Z());

    public static gp_Pnt FromNative(N.gp_Pnt theP) => new(theP.X(), theP.Y(), theP.Z());

    public static gp_Mat FromNative(N.gp_Mat theM)
        => new(theM.Value(1, 1), theM.Value(1, 2), theM.Value(1, 3),
               theM.Value(2, 1), theM.Value(2, 2), theM.Value(2, 3),
               theM.Value(3, 1), theM.Value(3, 2), theM.Value(3, 3));

    // Comparisons

    public void Same(double theNative, double thePort, string theWhat, double theTolerance = Tolerance)
    {
        myValues++;
        if (BitConverter.DoubleToInt64Bits(theNative) == BitConverter.DoubleToInt64Bits(thePort))
        {
            myExact++;
            return;
        }

        double aDiff = Math.Abs(theNative - thePort) / Math.Max(1.0, Math.Abs(theNative));
        myMaxDiff = Math.Max(myMaxDiff, aDiff);
        Assert.True(aDiff <= theTolerance,
                    string.Create(CultureInfo.InvariantCulture, $"{theWhat}: native {theNative:R}, port {thePort:R} (relative difference {aDiff:E2})"));
    }

    public void Same(in gp_XYZ theNative, in gp_XYZ thePort, string theWhat, double theTolerance = Tolerance)
    {
        for (int i = 1; i <= 3; i++)
        {
            Same(theNative.Coord(i), thePort.Coord(i), $"{theWhat}[{i}]", theTolerance);
        }
    }

    public void Same(in gp_Pnt2d theNative, in gp_Pnt2d thePort, string theWhat, double theTolerance = Tolerance)
    {
        Same(theNative.X(), thePort.X(), $"{theWhat}.X", theTolerance);
        Same(theNative.Y(), thePort.Y(), $"{theWhat}.Y", theTolerance);
    }

    public void Same(int theNative, int thePort, string theWhat)
    {
        myValues++;
        Assert.True(theNative == thePort, $"{theWhat}: native {theNative}, port {thePort}");
        myExact++;
    }

    public void Same(in gp_Mat theNative, in gp_Mat thePort, string theWhat, double theTolerance = Tolerance)
    {
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                Same(theNative.Value(i, j), thePort.Value(i, j), $"{theWhat}({i},{j})", theTolerance);
            }
        }
    }

    public void Same(N.gp_Trsf theNative, in gp_Trsf thePort, string theWhat, double theTolerance = Tolerance)
    {
        Assert.Equal((int)theNative.Form(), (int)thePort.Form());
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                Same(theNative.Value(i, j), thePort.Value(i, j), $"{theWhat}({i},{j})", theTolerance);
            }
        }
    }

    public void Same(bool theNative, bool thePort, string theWhat)
    {
        myValues++;
        Assert.True(theNative == thePort, $"{theWhat}: native {theNative}, port {thePort}");
        myExact++;
    }

    /// <summary>Writes how many values matched bit for bit, and the largest difference among the rest.</summary>
    public void Report(string theWhat)
        => output.WriteLine(string.Create(CultureInfo.InvariantCulture,
                                          $"{theWhat}: {myExact}/{myValues} values bit-identical, largest relative difference {myMaxDiff:E2}"));
}
#endif
