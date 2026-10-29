#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using Xunit.Abstractions;
using static Occt.GeomAbs_Shape;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GCPnts_AbscissaPoint, GCPnts_UniformDeflection and GCPnts_QuasiUniformDeflection (the GCPnts classes
/// build123d.NET binds) against native OCCT, on random lines, conics, Bezier and B-spline curves built from the same
/// raw numbers (TestAdaptorOracleTests' generators). Native OCCT works on its GeomAdaptor_Curve and
/// Geom2dAdaptor_Curve, the port on the test stand-ins (TestAdaptors.cs), which match them bit for bit.
/// </summary>
public class GCPntsOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;

    private static readonly GeomAbs_Shape[] Continuities = [GeomAbs_C0, GeomAbs_C1, GeomAbs_C2];

    // Runs the native and the port computations; both must throw, or neither.
    private static bool Both(Oracle theO, Action theNative, Action thePort, string theWhat)
    {
        Exception? aNativeError = null, aPortError = null;
        try
        {
            theNative();
        }
        catch (N.OcctException anE)
        {
            aNativeError = anE;
        }
        try
        {
            thePort();
        }
        catch (Standard_Failure anE)
        {
            aPortError = anE;
        }
        theO.Same(aNativeError is not null, aPortError is not null,
            $"{theWhat} throws (native: {aNativeError?.Message}; port: {aPortError?.Message})");
        return aNativeError is null;
    }

    private static void SameAbscissaPoint(Oracle theO, N.GCPnts_AbscissaPoint theN, GCPnts_AbscissaPoint theP, string theWhat)
    {
        theO.Same(theN.IsDone(), theP.IsDone(), $"{theWhat} IsDone");
        if (theP.IsDone())
        {
            theO.Same(theN.Parameter(), theP.Parameter(), $"{theWhat} Parameter");
        }
    }

    private static void SameUniformDeflection(Oracle theO, N.GCPnts_UniformDeflection theN, GCPnts_UniformDeflection theP,
        string theWhat)
    {
        theO.Same(theN.IsDone(), theP.IsDone(), $"{theWhat} IsDone");
        if (!theP.IsDone())
        {
            return;
        }
        theO.Same(theN.NbPoints(), theP.NbPoints(), $"{theWhat} NbPoints");
        for (int i = 1; i <= theP.NbPoints(); i++)
        {
            theO.Same(theN.Parameter(i), theP.Parameter(i), $"{theWhat} Parameter({i})");
            using N.gp_Pnt aNP = theN.Value(i);
            theO.Same(aNP, theP.Value(i), $"{theWhat} Value({i})");
        }
    }

    private static void SameQuasiUniformDeflection(Oracle theO, N.GCPnts_QuasiUniformDeflection theN,
        GCPnts_QuasiUniformDeflection theP, string theWhat)
    {
        theO.Same(theN.IsDone(), theP.IsDone(), $"{theWhat} IsDone");
        if (!theP.IsDone())
        {
            return;
        }
        theO.Same(theN.NbPoints(), theP.NbPoints(), $"{theWhat} NbPoints");
        for (int i = 1; i <= theP.NbPoints(); i++)
        {
            theO.Same(theN.Parameter(i), theP.Parameter(i), $"{theWhat} Parameter({i})");
            using N.gp_Pnt aNP = theN.Value(i);
            theO.Same(aNP, theP.Value(i), $"{theWhat} Value({i})");
        }
    }

    [Fact]
    public void AbscissaPoint_3d()
    {
        var anOracle = new Oracle(output);
        int aCompared = 0;
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom_Curve aNC, Geom_Curve aPC, string aKind) = TestAdaptorOracleTests.RandomCurve(anOracle);
            using (aNC)
            {
                (double aFirst, double aLast) = TestAdaptorOracleTests.Bounds(anOracle, aPC.FirstParameter(),
                    aPC.LastParameter(), aPC.IsPeriodic());
                using var aN = new N.GeomAdaptor_Curve(aNC, aFirst, aLast);
                var aP = new TestGeomAdaptor_Curve(aPC, aFirst, aLast);
                string aW = $"{aKind} #{c}";

                double aL = 0.0;
                if (!Both(anOracle, () => aL = N.GCPnts_AbscissaPoint.Length(aN), () => GCPnts_AbscissaPoint.Length(aP), $"{aW} Length"))
                {
                    continue;
                }
                anOracle.Same(aL, GCPnts_AbscissaPoint.Length(aP), $"{aW} Length");
                anOracle.Same(N.GCPnts_AbscissaPoint.Length(aN, 1e-9), GCPnts_AbscissaPoint.Length(aP, 1e-9), $"{aW} Length(tol)");
                double aU1 = anOracle.Real(aFirst, aLast), aU2 = anOracle.Real(aFirst, aLast);
                anOracle.Same(N.GCPnts_AbscissaPoint.Length(aN, aU1, aU2), GCPnts_AbscissaPoint.Length(aP, aU1, aU2),
                    $"{aW} Length(U1, U2)");
                anOracle.Same(N.GCPnts_AbscissaPoint.Length(aN, aU1, aU2, 1e-7),
                    GCPnts_AbscissaPoint.Length(aP, aU1, aU2, 1e-7), $"{aW} Length(U1, U2, tol)");
                if (aL < 1e-3)
                {
                    continue;
                }

                for (int k = 0; k < 3; k++)
                {
                    double anAbscissa = anOracle.Real(-0.8, 0.8) * aL;
                    double aU0 = anOracle.Real(aFirst, aLast);
                    double aUi = aU0 + anAbscissa / aL * (aLast - aFirst) * anOracle.Real(0.5, 1.5);
                    double aTol = anOracle.Real(1e-10, 1e-6);
                    string aWA = $"{aW} abscissa {anAbscissa:R} from {aU0:R}";
                    N.GCPnts_AbscissaPoint? aNA = null;
                    GCPnts_AbscissaPoint? aPA = null;
                    if (Both(anOracle, () => aNA = new N.GCPnts_AbscissaPoint(aN, anAbscissa, aU0),
                            () => aPA = new GCPnts_AbscissaPoint(aP, anAbscissa, aU0), $"{aWA} (C, Abs, U0)"))
                    {
                        using (aNA)
                        {
                            SameAbscissaPoint(anOracle, aNA!, aPA!, $"{aWA} (C, Abs, U0)");
                        }
                    }
                    if (Both(anOracle, () => aNA = new N.GCPnts_AbscissaPoint(aTol, aN, anAbscissa, aU0),
                            () => aPA = new GCPnts_AbscissaPoint(aTol, aP, anAbscissa, aU0), $"{aWA} (Tol, C, Abs, U0)"))
                    {
                        using (aNA)
                        {
                            SameAbscissaPoint(anOracle, aNA!, aPA!, $"{aWA} (Tol, C, Abs, U0)");
                        }
                    }
                    if (Both(anOracle, () => aNA = new N.GCPnts_AbscissaPoint(aN, anAbscissa, aU0, aUi),
                            () => aPA = new GCPnts_AbscissaPoint(aP, anAbscissa, aU0, aUi), $"{aWA} (C, Abs, U0, Ui)"))
                    {
                        using (aNA)
                        {
                            SameAbscissaPoint(anOracle, aNA!, aPA!, $"{aWA} (C, Abs, U0, Ui)");
                        }
                    }
                    if (Both(anOracle, () => aNA = new N.GCPnts_AbscissaPoint(aN, anAbscissa, aU0, aUi, aTol),
                            () => aPA = new GCPnts_AbscissaPoint(aP, anAbscissa, aU0, aUi, aTol), $"{aWA} (C, Abs, U0, Ui, Tol)"))
                    {
                        using (aNA)
                        {
                            SameAbscissaPoint(anOracle, aNA!, aPA!, $"{aWA} (C, Abs, U0, Ui, Tol)");
                        }
                    }
                    aCompared++;
                }
            }
        }
        output.WriteLine($"{aCompared} abscissas");
        anOracle.Report("GCPnts_AbscissaPoint (3D)");
    }

    [Fact]
    public void AbscissaPoint_2d()
    {
        var anOracle = new Oracle(output);
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom2d_Curve aNC, Geom2d_Curve aPC, string aKind) = TestAdaptorOracleTests.RandomCurve2d(anOracle);
            using (aNC)
            {
                (double aFirst, double aLast) = TestAdaptorOracleTests.Bounds(anOracle, aPC.FirstParameter(),
                    aPC.LastParameter(), aPC.IsPeriodic());
                using var aN = new N.Geom2dAdaptor_Curve(aNC, aFirst, aLast);
                var aP = new TestGeom2dAdaptor_Curve(aPC, aFirst, aLast);
                string aW = $"{aKind} #{c}";

                double aL = 0.0;
                if (!Both(anOracle, () => aL = N.GCPnts_AbscissaPoint.Length(aN), () => GCPnts_AbscissaPoint.Length(aP), $"{aW} Length"))
                {
                    continue;
                }
                anOracle.Same(aL, GCPnts_AbscissaPoint.Length(aP), $"{aW} Length");
                anOracle.Same(N.GCPnts_AbscissaPoint.Length(aN, 1e-9), GCPnts_AbscissaPoint.Length(aP, 1e-9), $"{aW} Length(tol)");
                if (aL < 1e-3)
                {
                    continue;
                }

                for (int k = 0; k < 3; k++)
                {
                    double anAbscissa = anOracle.Real(-0.8, 0.8) * aL;
                    double aU0 = anOracle.Real(aFirst, aLast);
                    double aUi = aU0 + anAbscissa / aL * (aLast - aFirst) * anOracle.Real(0.5, 1.5);
                    double aTol = anOracle.Real(1e-10, 1e-6);
                    string aWA = $"{aW} abscissa {anAbscissa:R} from {aU0:R}";
                    N.GCPnts_AbscissaPoint? aNA = null;
                    GCPnts_AbscissaPoint? aPA = null;
                    if (Both(anOracle, () => aNA = new N.GCPnts_AbscissaPoint(aN, anAbscissa, aU0),
                            () => aPA = new GCPnts_AbscissaPoint(aP, anAbscissa, aU0), $"{aWA} (C, Abs, U0)"))
                    {
                        using (aNA)
                        {
                            SameAbscissaPoint(anOracle, aNA!, aPA!, $"{aWA} (C, Abs, U0)");
                        }
                    }
                    if (Both(anOracle, () => aNA = new N.GCPnts_AbscissaPoint(aTol, aN, anAbscissa, aU0),
                            () => aPA = new GCPnts_AbscissaPoint(aTol, aP, anAbscissa, aU0), $"{aWA} (Tol, C, Abs, U0)"))
                    {
                        using (aNA)
                        {
                            SameAbscissaPoint(anOracle, aNA!, aPA!, $"{aWA} (Tol, C, Abs, U0)");
                        }
                    }
                    if (Both(anOracle, () => aNA = new N.GCPnts_AbscissaPoint(aN, anAbscissa, aU0, aUi, aTol),
                            () => aPA = new GCPnts_AbscissaPoint(aP, anAbscissa, aU0, aUi, aTol), $"{aWA} (C, Abs, U0, Ui, Tol)"))
                    {
                        using (aNA)
                        {
                            SameAbscissaPoint(anOracle, aNA!, aPA!, $"{aWA} (C, Abs, U0, Ui, Tol)");
                        }
                    }
                }
            }
        }
        anOracle.Report("GCPnts_AbscissaPoint (2D)");
    }

    [Fact]
    public void Deflections_3d()
    {
        var anOracle = new Oracle(output);
        int aPoints = 0;
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom_Curve aNC, Geom_Curve aPC, string aKind) = TestAdaptorOracleTests.RandomCurve(anOracle);
            using (aNC)
            {
                (double aFirst, double aLast) = TestAdaptorOracleTests.Bounds(anOracle, aPC.FirstParameter(),
                    aPC.LastParameter(), aPC.IsPeriodic());
                using var aN = new N.GeomAdaptor_Curve(aNC, aFirst, aLast);
                var aP = new TestGeomAdaptor_Curve(aPC, aFirst, aLast);
                double aSize = aP.Value(aFirst).Distance(aP.Value(aLast)) + aP.Value(aFirst).Distance(aP.Value((aFirst + aLast) / 2));
                double aDefl = Math.Max(aSize, 1e-3) * anOracle.Real(0.002, 0.05);
                bool aWithControl = anOracle.Bool();
                GeomAbs_Shape aCont = Continuities[anOracle.Int(0, 2)];
                string aW = $"{aKind} #{c} deflection {aDefl:R}";

                N.GCPnts_UniformDeflection? aNU = null;
                GCPnts_UniformDeflection? aPU = null;
                if (Both(anOracle, () => aNU = new N.GCPnts_UniformDeflection(aN, aDefl, aWithControl),
                        () => aPU = new GCPnts_UniformDeflection(aP, aDefl, aWithControl), $"{aW} UniformDeflection"))
                {
                    using (aNU)
                    {
                        SameUniformDeflection(anOracle, aNU!, aPU!, $"{aW} UniformDeflection({aWithControl})");
                        aPoints += aPU!.IsDone() ? aPU.NbPoints() : 0;
                    }
                }

                N.GCPnts_QuasiUniformDeflection? aNQ = null;
                GCPnts_QuasiUniformDeflection? aPQ = null;
                if (Both(anOracle, () => aNQ = new N.GCPnts_QuasiUniformDeflection(aN, aDefl, (N.GeomAbs_Shape)(int)aCont),
                        () => aPQ = new GCPnts_QuasiUniformDeflection(aP, aDefl, aCont), $"{aW} QuasiUniformDeflection"))
                {
                    using (aNQ)
                    {
                        SameQuasiUniformDeflection(anOracle, aNQ!, aPQ!, $"{aW} QuasiUniformDeflection({aCont})");
                        aPoints += aPQ!.IsDone() ? aPQ.NbPoints() : 0;
                    }
                }

                // A part of the curve, bounds reversed.
                double aU1 = anOracle.Real(aFirst, aLast), aU2 = anOracle.Real(aFirst, aLast);
                if (Both(anOracle, () => aNU = new N.GCPnts_UniformDeflection(aN, aDefl, aU2, aU1, aWithControl),
                        () => aPU = new GCPnts_UniformDeflection(aP, aDefl, aU2, aU1, aWithControl), $"{aW} UniformDeflection(U1, U2)"))
                {
                    using (aNU)
                    {
                        SameUniformDeflection(anOracle, aNU!, aPU!, $"{aW} UniformDeflection(U2, U1)");
                    }
                }
                if (Both(anOracle, () => aNQ = new N.GCPnts_QuasiUniformDeflection(aN, aDefl, aU2, aU1, (N.GeomAbs_Shape)(int)aCont),
                        () => aPQ = new GCPnts_QuasiUniformDeflection(aP, aDefl, aU2, aU1, aCont), $"{aW} QuasiUniformDeflection(U1, U2)"))
                {
                    using (aNQ)
                    {
                        SameQuasiUniformDeflection(anOracle, aNQ!, aPQ!, $"{aW} QuasiUniformDeflection(U2, U1)");
                    }
                }
            }
        }
        output.WriteLine($"{aPoints} points");
        anOracle.Report("GCPnts_UniformDeflection and GCPnts_QuasiUniformDeflection (3D)");
    }

    [Fact]
    public void Deflections_2d()
    {
        var anOracle = new Oracle(output);
        for (int c = 0; c < Cases; c++)
        {
            (N.Geom2d_Curve aNC, Geom2d_Curve aPC, string aKind) = TestAdaptorOracleTests.RandomCurve2d(anOracle);
            using (aNC)
            {
                (double aFirst, double aLast) = TestAdaptorOracleTests.Bounds(anOracle, aPC.FirstParameter(),
                    aPC.LastParameter(), aPC.IsPeriodic());
                using var aN = new N.Geom2dAdaptor_Curve(aNC, aFirst, aLast);
                var aP = new TestGeom2dAdaptor_Curve(aPC, aFirst, aLast);
                double aSize = aP.Value(aFirst).Distance(aP.Value(aLast)) + aP.Value(aFirst).Distance(aP.Value((aFirst + aLast) / 2));
                double aDefl = Math.Max(aSize, 1e-3) * anOracle.Real(0.002, 0.05);
                bool aWithControl = anOracle.Bool();
                GeomAbs_Shape aCont = Continuities[anOracle.Int(0, 2)];
                string aW = $"{aKind} #{c} deflection {aDefl:R}";

                N.GCPnts_UniformDeflection? aNU = null;
                GCPnts_UniformDeflection? aPU = null;
                if (Both(anOracle, () => aNU = new N.GCPnts_UniformDeflection(aN, aDefl, aWithControl),
                        () => aPU = new GCPnts_UniformDeflection(aP, aDefl, aWithControl), $"{aW} UniformDeflection"))
                {
                    using (aNU)
                    {
                        SameUniformDeflection(anOracle, aNU!, aPU!, $"{aW} UniformDeflection({aWithControl})");
                    }
                }

                N.GCPnts_QuasiUniformDeflection? aNQ = null;
                GCPnts_QuasiUniformDeflection? aPQ = null;
                if (Both(anOracle, () => aNQ = new N.GCPnts_QuasiUniformDeflection(aN, aDefl, (N.GeomAbs_Shape)(int)aCont),
                        () => aPQ = new GCPnts_QuasiUniformDeflection(aP, aDefl, aCont), $"{aW} QuasiUniformDeflection"))
                {
                    using (aNQ)
                    {
                        SameQuasiUniformDeflection(anOracle, aNQ!, aPQ!, $"{aW} QuasiUniformDeflection({aCont})");
                    }
                }
            }
        }
        anOracle.Report("GCPnts_UniformDeflection and GCPnts_QuasiUniformDeflection (2D)");
    }
}
#endif
