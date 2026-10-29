#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;
using static OcctNet.Oracle.Tests.Geom.GeomOracleInputs;

namespace OcctNet.Oracle.Tests.Geom2d;

/// <summary>Geom2d_TrimmedCurve over lines, circles, ellipses and B-splines against native OCCT.</summary>
public class Geom2dTrimmedCurveOracleTests(ITestOutputHelper output)
{
    private const int Cases = 400;

    /// <summary>A random basis curve on both sides.</summary>
    private static (Geom2d_Curve Port, N.Geom2d_Curve Native) Basis(Oracle theOracle, int theKind)
    {
        (N.gp_Ax2d aNAx, gp_Ax2d anAx) = Ax2ds(theOracle);
        using var aNAxOwner = aNAx;
        switch (theKind)
        {
            case 0:
                return (new Geom2d_Line(anAx), new N.Geom2d_Line(aNAx));
            case 1:
            {
                double aR = theOracle.Real(0.1, 50);
                bool aSense = theOracle.Bool();
                return (new Geom2d_Circle(anAx, aR, aSense), new N.Geom2d_Circle(aNAx, aR, aSense));
            }
            case 2:
            {
                double aMaj = theOracle.Real(1, 50), aMin = theOracle.Real(0.05, 1.0) * aMaj;
                bool aSense = theOracle.Bool();
                return (new Geom2d_Ellipse(anAx, aMaj, aMin, aSense), new N.Geom2d_Ellipse(aNAx, aMaj, aMin, aSense));
            }
            default:
            {
                BSplineSpec aSpec = Geom2dOracle.RandomBSpline(theOracle);
                return (Geom2dOracle.ToPort(aSpec), Geom2dOracle.ToNative(aSpec));
            }
        }
    }

    private static void Check(Oracle theOracle, N.Geom2d_TrimmedCurve theNative, Geom2d_TrimmedCurve thePort, string theWhat)
    {
        Geom2dOracle.SameBounds(theOracle, theNative, thePort, theWhat);
        using (var aNS = theNative.StartPoint())
        using (var aNE = theNative.EndPoint())
        {
            theOracle.Same(Oracle.FromNative(aNS), thePort.StartPoint().XY(), $"{theWhat} StartPoint");
            theOracle.Same(Oracle.FromNative(aNE), thePort.EndPoint().XY(), $"{theWhat} EndPoint");
        }
        for (int k = 0; k < 2; k++)
        {
            double U = Geom2dOracle.Param(theOracle, thePort);
            Geom2dOracle.SameEval(theOracle, theNative, thePort, U, theWhat, 3);
            theOracle.Same(theNative.ReversedParameter(U), thePort.ReversedParameter(U), $"{theWhat} ReversedParameter");
        }
        for (int n = 0; n <= 4; n++)
        {
            theOracle.Same(theNative.IsCN(n), thePort.IsCN(n), $"{theWhat} IsCN({n})");
        }
        if (thePort.IsPeriodic())
        {
            theOracle.Same(theNative.Period(), thePort.Period(), $"{theWhat} Period");
        }
    }

    [Fact]
    public void TrimmedCurve()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            int aKind = i % 4;
            var (aBasis, aNBasisRaw) = Basis(anOracle, aKind);
            using var aNBasis = aNBasisRaw;

            double U1, U2;
            if (aBasis.IsPeriodic())
            {
                // Anywhere, also outside one period: the constructor adjusts them
                double aPer = aBasis.Period();
                U1 = anOracle.Real(-2 * aPer, 2 * aPer);
                U2 = anOracle.Real(-2 * aPer, 2 * aPer);
            }
            else if (Precision.IsInfinite(aBasis.LastParameter()))
            {
                U1 = anOracle.Real(-50, 50);
                U2 = anOracle.Real(-50, 50);
            }
            else
            {
                U1 = Geom2dOracle.Param(anOracle, aBasis);
                U2 = Geom2dOracle.Param(anOracle, aBasis);
            }
            if (Math.Abs(U1 - U2) < 1e-3)
            {
                continue;
            }
            bool aSense = anOracle.Bool();
            bool anAdjust = !aBasis.IsPeriodic() || anOracle.Int(0, 3) != 0;
            if (!anAdjust && Math.Abs(U2 - U1) > aBasis.Period())
            {
                // Without the adjustment the trimmed range must stay within one period
                continue;
            }
            string aWhat = $"Trimmed[{aKind}]";

            using var aNative = new N.Geom2d_TrimmedCurve(aNBasis, U1, U2, aSense, anAdjust);
            var aPort = new Geom2d_TrimmedCurve(aBasis, U1, U2, aSense, anAdjust);
            Check(anOracle, aNative, aPort, aWhat);

            // Reversed
            using (var aNRevC = aNative.Reversed())
            using (var aNRev = N.Geom2d_TrimmedCurve.DownCast(aNRevC))
            {
                Check(anOracle, aNRev!, (Geom2d_TrimmedCurve)aPort.Reversed(), aWhat + " reversed");
            }

            // Transformed
            var (aT, aNT) = Geom2dOracle.Trsf(anOracle);
            using (aNT)
            using (var aNTrG = aNative.Transformed(aNT))
            using (var aNTr = N.Geom2d_TrimmedCurve.DownCast(aNTrG))
            {
                Check(anOracle, aNTr!, (Geom2d_TrimmedCurve)aPort.Transformed(aT), aWhat + " transformed");
            }

            // SetTrim with new parameters inside the trimmed range
            double aF = aPort.FirstParameter(), aL = aPort.LastParameter();
            double V1 = anOracle.Real(aF, aL), V2 = anOracle.Real(aF, aL);
            if (Math.Abs(V1 - V2) > 1e-3)
            {
                bool aSense2 = anOracle.Bool();
                aNative.SetTrim(V1, V2, aSense2, true);
                Check(anOracle, aNative, aPort.WithTrim(V1, V2, aSense2, true), aWhat + " SetTrim");
            }
        }

        anOracle.Report("Geom2d_TrimmedCurve");
    }
}
#endif
