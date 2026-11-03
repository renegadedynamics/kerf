// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>The elementary Extrema classes against native OCCT, on random inputs.</summary>
public class ExtremaOracleTests(ITestOutputHelper output)
{
    private const int Cases = 500;

    [Fact]
    public void POnCurv()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < 100; i++)
        {
            double aU = anOracle.Real();
            gp_Pnt aP = anOracle.Pnt();
            using var aNP = Oracle.ToNative(aP);
            using var aNative = new N.Extrema_POnCurv(aU, aNP);
            var aPort = new Extrema_POnCurv(aU, aP);
            using (var aNV = aNative.Value())
            {
                anOracle.Same(Oracle.FromNative(aNV).XYZ(), aPort.Value().XYZ(), "Value");
            }
            anOracle.Same(aNative.Parameter(), aPort.Parameter(), "Parameter");

            using var aNEmpty = new N.Extrema_POnCurv();
            var anEmpty = new Extrema_POnCurv();
            using (var aNV = aNEmpty.Value())
            {
                anOracle.Same(Oracle.FromNative(aNV).XYZ(), anEmpty.Value().XYZ(), "indefinite Value");
            }
            anOracle.Same(aNEmpty.Parameter(), anEmpty.Parameter(), "indefinite Parameter");

            double aU2 = anOracle.Real();
            gp_Pnt aP2 = anOracle.Pnt();
            using var aNP2 = Oracle.ToNative(aP2);
            aNEmpty.SetValues(aU2, aNP2);
            anEmpty.SetValues(aU2, aP2);
            using (var aNV = aNEmpty.Value())
            {
                anOracle.Same(Oracle.FromNative(aNV).XYZ(), anEmpty.Value().XYZ(), "SetValues Value");
            }
            anOracle.Same(aNEmpty.Parameter(), anEmpty.Parameter(), "SetValues Parameter");
        }

        anOracle.Report("Extrema_POnCurv");
    }

    /// <summary>
    /// Extrema_ExtPElC through native Extrema_ExtPC on lines, circles and ellipses, which hands analytic curves to
    /// Extrema_ExtPElC with Precision::Confusion() and keeps its results in order (parameters of periodic curves
    /// brought into the period).
    /// </summary>
    [Fact]
    public void ExtPElC_through_Extrema_ExtPC()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            gp_Pnt aP = anOracle.Pnt();
            // The port's and the native axis system built from the same inputs (handing the port's computed X
            // direction to the native constructor would move it by an ulp now and then)
            gp_Pnt aLoc = anOracle.Pnt();
            gp_Dir aN = Oracle.Settled(anOracle.Dir()), aVx = Oracle.Settled(anOracle.Dir());
            gp_Ax2 anAx = new(aLoc, aN, aVx);
            using var aNP = Oracle.ToNative(aP);
            using var aNLoc = Oracle.ToNative(aLoc);
            using var aNN = Oracle.ToNative(aN);
            using var aNVx = Oracle.ToNative(aVx);
            using var aNAx = new N.gp_Ax2(aNLoc, aNN, aNVx);
            double aTol = Precision.Confusion();
            int aKind = i % 3;
            N.Geom_Curve aNCurve;
            Extrema_ExtPElC aPort;
            double aUinf, aUsup;
            bool isPeriodic = aKind != 0;
            if (aKind == 0)
            {
                aUinf = -150.0;
                aUsup = 150.0;
                using var aNAx1 = new N.gp_Ax1(aNLoc, aNN);
                aNCurve = new N.Geom_Line(aNAx1);
                aPort = new Extrema_ExtPElC(aP, new gp_Lin(anAx.Axis()), aTol, aUinf, aUsup);
            }
            else if (aKind == 1)
            {
                aUinf = 0.0;
                aUsup = 2.0 * Math.PI;
                double aR = anOracle.Real(1, 100);
                aNCurve = new N.Geom_Circle(aNAx, aR);
                aPort = new Extrema_ExtPElC(aP, new gp_Circ(anAx, aR), aTol, aUinf, aUsup);
            }
            else
            {
                aUinf = 0.0;
                aUsup = 2.0 * Math.PI;
                double aMinor = anOracle.Real(1, 50);
                double aMajor = aMinor + anOracle.Real(0.5, 50);
                aNCurve = new N.Geom_Ellipse(aNAx, aMajor, aMinor);
                aPort = new Extrema_ExtPElC(aP, new gp_Elips(anAx, aMajor, aMinor), aTol, aUinf, aUsup);
            }

            using (aNCurve)
            {
                using var anAdaptor = new N.GeomAdaptor_Curve(aNCurve, aUinf, aUsup);
                using var aNative = new N.Extrema_ExtPC(aNP, anAdaptor, aUinf, aUsup);
                anOracle.Same(aNative.IsDone(), aPort.IsDone(), "IsDone");
                if (!aNative.IsDone())
                {
                    continue;
                }

                // Extrema_ExtPC keeps the parameters in range; with the ranges above it drops none
                anOracle.Same(aNative.NbExt(), aPort.NbExt(), "NbExt");
                for (int j = 1; j <= aNative.NbExt(); j++)
                {
                    double aU = aPort.Point(j).Parameter();
                    if (isPeriodic)
                    {
                        aU = ElCLib.InPeriod(aU, aUinf, aUinf + 2.0 * Math.PI);
                    }
                    using var aNPoint = aNative.Point(j);
                    using var aNValue = aNPoint.Value();
                    anOracle.Same(aNPoint.Parameter(), aU, "parameter");
                    anOracle.Same(Oracle.FromNative(aNValue).XYZ(), aPort.Point(j).Value().XYZ(), "point");
                    anOracle.Same(aNative.SquareDistance(j), aPort.SquareDistance(j), "square distance");
                    anOracle.Same(aNative.IsMin(j), aPort.IsMin(j), "IsMin");
                }
            }
        }

        anOracle.Report("Extrema_ExtPElC (lines, circles, ellipses)");
    }

    /// <summary>
    /// Extrema_ExtPElC2d through native Geom2dAPI_ProjectPointOnCurve on lines, circles and ellipses, which uses
    /// Extrema_ExtPC2d (the 2D twin of Extrema_ExtPC) over the curve's whole range.
    /// </summary>
    [Fact]
    public void ExtPElC2d_through_Geom2dAPI_ProjectPointOnCurve()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            gp_Pnt2d aP = anOracle.Pnt2d();
            gp_Ax22d anAx = anOracle.Ax22d();
            using var aNP = Oracle.ToNative(aP);
            using var aNAx = Oracle.ToNative(anAx);
            double aTol = Precision.Confusion();
            int aKind = i % 3;
            N.Geom2d_Curve aNCurve;
            Extrema_ExtPElC2d aPort;
            bool isPeriodic = aKind != 0;
            if (aKind == 0)
            {
                using var aNAx1 = Oracle.ToNative(anAx.XAxis());
                aNCurve = new N.Geom2d_Line(aNAx1);
                aPort = new Extrema_ExtPElC2d(aP, new gp_Lin2d(anAx.XAxis()), aTol, -Precision.Infinite(), Precision.Infinite());
            }
            else if (aKind == 1)
            {
                double aR = anOracle.Real(1, 100);
                aNCurve = new N.Geom2d_Circle(aNAx, aR);
                aPort = new Extrema_ExtPElC2d(aP, new gp_Circ2d(anAx, aR), aTol, 0.0, 2.0 * Math.PI);
            }
            else
            {
                double aMinor = anOracle.Real(1, 50);
                double aMajor = aMinor + anOracle.Real(0.5, 50);
                aNCurve = new N.Geom2d_Ellipse(aNAx, aMajor, aMinor);
                aPort = new Extrema_ExtPElC2d(aP, new gp_Elips2d(anAx, aMajor, aMinor), aTol, 0.0, 2.0 * Math.PI);
            }

            using (aNCurve)
            {
                using var aNative = new N.Geom2dAPI_ProjectPointOnCurve(aNP, aNCurve);
                int aNb = aPort.IsDone() ? aPort.NbExt() : 0;
                anOracle.Same(aNative.NbPoints(), aNb, "NbPoints");
                for (int j = 1; j <= aNative.NbPoints(); j++)
                {
                    double aU = aPort.Point(j).Parameter();
                    if (isPeriodic)
                    {
                        aU = ElCLib.InPeriod(aU, 0.0, 2.0 * Math.PI);
                    }
                    using var aNPoint = aNative.Point(j);
                    anOracle.Same(aNative.Parameter(j), aU, "parameter");
                    anOracle.Same(Oracle.FromNative(aNPoint), aPort.Point(j).Value(), "point");
                    anOracle.Same(aNative.Distance(j), Math.Sqrt(aPort.SquareDistance(j)), "distance");
                }
            }
        }

        anOracle.Report("Extrema_ExtPElC2d (lines, circles, ellipses)");
    }
}
