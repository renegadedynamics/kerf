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

    /// <summary>
    /// Held around every native call that reaches AppDef_Variational (GeomAPI_PointsToBSpline and
    /// GeomAPI_PointsToBSplineSurface with criterion weights, ThruSections with smoothing): OCCT's FEmTool linear
    /// criteria keep their reference integrals in unsynchronised function statics, so concurrent native runs with
    /// different constraint orders read each other's integrals (occt-issues.md, decision 945). Test classes run in
    /// parallel; the port needs no lock (decision 148).
    /// </summary>
    public static readonly Lock NativeVariational = new();

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

    /// <summary>
    /// Random knots and multiplicities for a B-spline of the given degree: 4 to 8 knots (max(6, degree + 2) if
    /// periodic) spaced 0.2 to 1.2 apart from a start in [-2, 2]; end multiplicities degree + 1 unless periodic
    /// (then the last equals the first), interior multiplicities 1 to degree.
    /// </summary>
    public void BSplineKnots(int theDegree, bool thePeriodic,
                             out NCollection_Array1<double> theKnots, out NCollection_Array1<int> theMults)
    {
        int aNbKnots = thePeriodic ? Math.Max(6, theDegree + 2) : Int(4, 8);
        theKnots = new NCollection_Array1<double>(1, aNbKnots);
        theMults = new NCollection_Array1<int>(1, aNbKnots);
        double aKnot = Real(-2, 2);
        for (int i = 1; i <= aNbKnots; i++)
        {
            theKnots[i] = aKnot;
            aKnot += Real(0.2, 1.2);
            theMults[i] = !thePeriodic && (i == 1 || i == aNbKnots) ? theDegree + 1 : Int(1, theDegree);
        }

        if (thePeriodic)
        {
            theMults[aNbKnots] = theMults[1];
        }
    }

    /// <summary>Weights in [0.5, 1.5] if rational, ones otherwise.</summary>
    public NCollection_Array1<double> Weights(int theNb, bool theRational)
    {
        var aWeights = new NCollection_Array1<double>(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            aWeights[i] = theRational ? Real(0.5, 1.5) : 1.0;
        }

        return aWeights;
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

    public gp_Vec2d Vec2d() => new(Real(), Real());

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

    public static N.gp_Vec2d ToNative(in gp_Vec2d theV) => new(theV.X(), theV.Y());

    public static N.TColStd_Array1OfReal ToNative(NCollection_Array1<double> theArray)
    {
        var aNative = new N.TColStd_Array1OfReal(theArray.Lower(), theArray.Upper());
        for (int i = theArray.Lower(); i <= theArray.Upper(); i++)
        {
            aNative.SetValue(i, theArray[i]);
        }

        return aNative;
    }

    public static N.TColStd_Array1OfInteger ToNative(NCollection_Array1<int> theArray)
    {
        var aNative = new N.TColStd_Array1OfInteger(theArray.Lower(), theArray.Upper());
        for (int i = theArray.Lower(); i <= theArray.Upper(); i++)
        {
            aNative.SetValue(i, theArray[i]);
        }

        return aNative;
    }

    public static N.TColgp_Array1OfPnt ToNative(NCollection_Array1<gp_Pnt> theArray)
    {
        var aNative = new N.TColgp_Array1OfPnt(theArray.Lower(), theArray.Upper());
        for (int i = theArray.Lower(); i <= theArray.Upper(); i++)
        {
            using var aP = ToNative(theArray[i]);
            aNative.SetValue(i, aP);
        }

        return aNative;
    }

    public static N.TColgp_Array1OfPnt2d ToNative(NCollection_Array1<gp_Pnt2d> theArray)
    {
        var aNative = new N.TColgp_Array1OfPnt2d(theArray.Lower(), theArray.Upper());
        for (int i = theArray.Lower(); i <= theArray.Upper(); i++)
        {
            using var aP = ToNative(theArray[i]);
            aNative.SetValue(i, aP);
        }

        return aNative;
    }

    public static N.TColStd_Array2OfReal ToNative(NCollection_Array2<double> theArray)
    {
        var aNative = new N.TColStd_Array2OfReal(theArray.LowerRow(), theArray.UpperRow(), theArray.LowerCol(), theArray.UpperCol());
        for (int i = theArray.LowerRow(); i <= theArray.UpperRow(); i++)
        {
            for (int j = theArray.LowerCol(); j <= theArray.UpperCol(); j++)
            {
                aNative.SetValue(i, j, theArray[i, j]);
            }
        }

        return aNative;
    }

    public static N.TColgp_Array2OfPnt ToNative(NCollection_Array2<gp_Pnt> theArray)
    {
        var aNative = new N.TColgp_Array2OfPnt(theArray.LowerRow(), theArray.UpperRow(), theArray.LowerCol(), theArray.UpperCol());
        for (int i = theArray.LowerRow(); i <= theArray.UpperRow(); i++)
        {
            for (int j = theArray.LowerCol(); j <= theArray.UpperCol(); j++)
            {
                using var aP = ToNative(theArray[i, j]);
                aNative.SetValue(i, j, aP);
            }
        }

        return aNative;
    }

    /// <summary>A native B-spline curve; non-rational (OCCT's constructor without weights) when theWeights is null.</summary>
    public static N.Geom_BSplineCurve ToNativeCurve(NCollection_Array1<gp_Pnt> thePoles, NCollection_Array1<double>? theWeights,
                                                    NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults,
                                                    int theDegree, bool thePeriodic)
    {
        using var aPoles = ToNative(thePoles);
        using var aKnots = ToNative(theKnots);
        using var aMults = ToNative(theMults);
        if (theWeights is null)
        {
            return new N.Geom_BSplineCurve(aPoles, aKnots, aMults, theDegree, thePeriodic);
        }

        using var aWeights = ToNative(theWeights);
        return new N.Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, theDegree, thePeriodic);
    }

    /// <summary>A native 2D B-spline curve; non-rational when theWeights is null.</summary>
    public static N.Geom2d_BSplineCurve ToNativeCurve(NCollection_Array1<gp_Pnt2d> thePoles, NCollection_Array1<double>? theWeights,
                                                      NCollection_Array1<double> theKnots, NCollection_Array1<int> theMults,
                                                      int theDegree, bool thePeriodic)
    {
        using var aPoles = ToNative(thePoles);
        using var aKnots = ToNative(theKnots);
        using var aMults = ToNative(theMults);
        if (theWeights is null)
        {
            return new N.Geom2d_BSplineCurve(aPoles, aKnots, aMults, theDegree, thePeriodic);
        }

        using var aWeights = ToNative(theWeights);
        return new N.Geom2d_BSplineCurve(aPoles, aWeights, aKnots, aMults, theDegree, thePeriodic);
    }

    public static gp_Vec FromNative(N.gp_Vec theV) => new(theV.X(), theV.Y(), theV.Z());

    public static gp_Vec2d FromNative(N.gp_Vec2d theV) => new(theV.X(), theV.Y());

    public static gp_XY FromNative(N.gp_Dir2d theD) => new(theD.X(), theD.Y());

    public static gp_XY FromNative(N.gp_XY theXY) => new(theXY.X(), theXY.Y());

    public static gp_XYZ FromNative(N.gp_XYZ theXYZ) => new(theXYZ.X(), theXYZ.Y(), theXYZ.Z());

    public static gp_Pnt FromNative(N.gp_Pnt theP) => new(theP.X(), theP.Y(), theP.Z());

    public static gp_Mat FromNative(N.gp_Mat theM)
        => new(theM.Value(1, 1), theM.Value(1, 2), theM.Value(1, 3),
               theM.Value(2, 1), theM.Value(2, 2), theM.Value(2, 3),
               theM.Value(3, 1), theM.Value(3, 2), theM.Value(3, 3));

    // Comparisons

    public void Same(double theNative, double thePort, string theWhat, double theTolerance = Tolerance)
        => Compare(theNative, thePort, theWhat, theTolerance, Math.Max(1.0, Math.Abs(theNative)));

    /// <summary>
    /// Like <see cref="Same(double, double, string, double)"/>, but relative to the native value however small
    /// (absolute only for a native zero): for small results such as parametric resolutions.
    /// </summary>
    public void SameRelative(double theNative, double thePort, string theWhat, double theTolerance = Tolerance)
        => Compare(theNative, thePort, theWhat, theTolerance, theNative == 0.0 ? 1.0 : Math.Abs(theNative));

    // Points and vectors against native wrappers: each coordinate is compared relative to the largest native
    // coordinate (at least 1), so a coordinate that is small next to the others isn't held to its own magnitude.

    public void Same(N.gp_Pnt theNative, in gp_Pnt thePort, string theWhat, double theTolerance = Tolerance)
        => Same3(theNative.X(), theNative.Y(), theNative.Z(), thePort.X(), thePort.Y(), thePort.Z(), theWhat, theTolerance);

    public void Same(N.gp_Vec theNative, in gp_Vec thePort, string theWhat, double theTolerance = Tolerance)
        => Same3(theNative.X(), theNative.Y(), theNative.Z(), thePort.X(), thePort.Y(), thePort.Z(), theWhat, theTolerance);

    public void Same(N.gp_Pnt2d theNative, in gp_Pnt2d thePort, string theWhat, double theTolerance = Tolerance)
        => Same2(theNative.X(), theNative.Y(), thePort.X(), thePort.Y(), theWhat, theTolerance);

    public void Same(N.gp_Vec2d theNative, in gp_Vec2d thePort, string theWhat, double theTolerance = Tolerance)
        => Same2(theNative.X(), theNative.Y(), thePort.X(), thePort.Y(), theWhat, theTolerance);

    private void Same3(double theNX, double theNY, double theNZ, double thePX, double thePY, double thePZ,
                       string theWhat, double theTolerance)
    {
        double aScale = Math.Max(1.0, Math.Max(Math.Abs(theNX), Math.Max(Math.Abs(theNY), Math.Abs(theNZ))));
        Compare(theNX, thePX, theWhat + ".X", theTolerance, aScale);
        Compare(theNY, thePY, theWhat + ".Y", theTolerance, aScale);
        Compare(theNZ, thePZ, theWhat + ".Z", theTolerance, aScale);
    }

    private void Same2(double theNX, double theNY, double thePX, double thePY, string theWhat, double theTolerance)
    {
        double aScale = Math.Max(1.0, Math.Max(Math.Abs(theNX), Math.Abs(theNY)));
        Compare(theNX, thePX, theWhat + ".X", theTolerance, aScale);
        Compare(theNY, thePY, theWhat + ".Y", theTolerance, aScale);
    }

    public void Same(int theNative, int thePort, string theWhat)
    {
        myValues++;
        Assert.True(theNative == thePort, $"{theWhat}: native {theNative}, port {thePort}");
        myExact++;
    }

    private void Compare(double theNative, double thePort, string theWhat, double theTolerance, double theScale)
    {
        myValues++;
        if (BitConverter.DoubleToInt64Bits(theNative) == BitConverter.DoubleToInt64Bits(thePort))
        {
            myExact++;
            return;
        }

        double aDiff = Math.Abs(theNative - thePort) / theScale;
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

    // 2D comparisons against the port's gp_XY coordinates (the Geom2d tests compare XY() of points, vectors and directions)

    public void Same(in gp_XY theNative, in gp_XY thePort, string theWhat, double theTolerance = Tolerance)
    {
        Same(theNative.X(), thePort.X(), $"{theWhat}.X", theTolerance);
        Same(theNative.Y(), thePort.Y(), $"{theWhat}.Y", theTolerance);
    }

    public void Same(in gp_Pnt2d theNative, in gp_XY thePort, string theWhat, double theTolerance = Tolerance)
        => Same(theNative.XY(), thePort, theWhat, theTolerance);

    public void Same(in gp_Vec2d theNative, in gp_XY thePort, string theWhat, double theTolerance = Tolerance)
        => Same(theNative.XY(), thePort, theWhat, theTolerance);

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
