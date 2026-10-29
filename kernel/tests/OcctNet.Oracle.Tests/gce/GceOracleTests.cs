#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests.gce;

/// <summary>The gce package against native OCCT, on random inputs.</summary>
public class GceOracleTests(ITestOutputHelper output)
{
    private const int Cases = 500;

    private static void SameLin(Oracle theOracle, N.gp_Lin theNative, in gp_Lin thePort, string theWhat)
    {
        using var aNP = theNative.Location();
        using var aND = theNative.Direction();
        using var aNDXYZ = aND.XYZ();
        theOracle.Same(Oracle.FromNative(aNP).XYZ(), thePort.Location().XYZ(), $"{theWhat} location");
        theOracle.Same(Oracle.FromNative(aNDXYZ), thePort.Direction().XYZ(), $"{theWhat} direction");
    }

    [Fact]
    public void MakeLin()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            gp_Pnt aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt();
            // Settled: the native gp_Dir constructor normalises again (see Oracle.Settled)
            gp_Dir aD = Oracle.Settled(anOracle.Dir());
            gp_Ax1 anAx = new(anOracle.Pnt(), Oracle.Settled(anOracle.Dir()));
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);
            using var aND = Oracle.ToNative(aD);
            using var aNAx = Oracle.ToNative(anAx);

            using (var aN = new N.gce_MakeLin(aNAx))
            using (var aNL = aN.Value())
            {
                SameLin(anOracle, aNL, new gce_MakeLin(anAx).Value(), "axis");
            }

            using (var aN = new N.gce_MakeLin(aNP1, aND))
            using (var aNL = aN.Value())
            {
                SameLin(anOracle, aNL, new gce_MakeLin(aP1, aD).Value(), "point and direction");
            }

            // Two points, coincident one time in ten
            gp_Pnt aQ = i % 10 == 0 ? aP1 : aP2;
            using var aNQ = Oracle.ToNative(aQ);
            using (var aN = new N.gce_MakeLin(aNP1, aNQ))
            {
                var aPort = new gce_MakeLin(aP1, aQ);
                anOracle.Same(aN.IsDone(), aPort.IsDone(), "IsDone");
                anOracle.Same((int)aN.Status(), (int)aPort.Status(), "Status");
                if (aN.IsDone())
                {
                    using var aNL = aN.Value();
                    SameLin(anOracle, aNL, aPort.Value(), "two points");
                }
            }

            gp_Lin aLin = new(anAx);
            using var aNLin = new N.gp_Lin(aNAx);
            using (var aN = new N.gce_MakeLin(aNLin, aNP2))
            using (var aNL = aN.Value())
            {
                SameLin(anOracle, aNL, new gce_MakeLin(aLin, aP2).Value(), "parallel through a point");
            }
        }

        anOracle.Report("gce_MakeLin");
    }

    /// <summary>
    /// gce_MakeCirc through three points (with Extrema_ExtElC between two lines), through native
    /// GC_MakeArcOfCircle, which builds its circle with it.
    /// </summary>
    [Fact]
    public void MakeCirc_three_points_through_GC_MakeArcOfCircle()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            gp_Pnt aP1 = anOracle.Pnt(), aP2 = anOracle.Pnt(), aP3 = anOracle.Pnt();
            using var aNP1 = Oracle.ToNative(aP1);
            using var aNP2 = Oracle.ToNative(aP2);
            using var aNP3 = Oracle.ToNative(aP3);

            var aPort = new gce_MakeCirc(aP1, aP2, aP3);
            using var aNArc = new N.GC_MakeArcOfCircle(aNP1, aNP2, aNP3);
            anOracle.Same(aNArc.IsDone(), aPort.IsDone(), "IsDone");
            if (!aNArc.IsDone())
            {
                continue;
            }

            using var aNTrimmed = aNArc.Value();
            using var aNBasis = aNTrimmed!.BasisCurve();
            using var aNCircle = N.Geom_Circle.DownCast(aNBasis);
            using var aNCirc = aNCircle!.Circ();
            gp_Circ aCirc = aPort.Value();

            anOracle.Same(aNCirc.Radius(), aCirc.Radius(), "radius", 1e-9);
            using var aNPos = aNCirc.Position();
            using var aNLoc = aNPos.Location();
            using var aNZ = aNPos.Direction();
            using var aNX = aNPos.XDirection();
            using var aNZXYZ = aNZ.XYZ();
            using var aNXXYZ = aNX.XYZ();
            // The center is the middle of two nearly intersecting bisectors: a few ulps of the coordinates
            anOracle.Same(Oracle.FromNative(aNLoc).XYZ(), aCirc.Location().XYZ(), "center", 1e-9);
            anOracle.Same(Oracle.FromNative(aNZXYZ), aCirc.Axis().Direction().XYZ(), "normal", 1e-9);
            anOracle.Same(Oracle.FromNative(aNXXYZ), aCirc.XAxis().Direction().XYZ(), "X direction", 1e-9);
        }

        anOracle.Report("gce_MakeCirc(P1, P2, P3)");
    }
}
#endif
