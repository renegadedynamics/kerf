// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The comparison of decision 578 (<see cref="IntersectionComparison"/>) on native GeomAPI_IntSS results, before the
/// port has its own (unit 5G): native lines of analytic pairs at random placements, read back from their BRep text by
/// native OCCT and by the port, compare bit-identical (reading normalises directions on both sides alike); the port's
/// copies moved by a known distance compare by that distance; a different number or type of lines fails.
/// </summary>
public class IntersectionComparisonOracleTests(ITestOutputHelper output)
{
    private static N.gp_Ax3 Axes(Oracle theO, double theRange)
    {
        gp_Dir aN = theO.Dir();
        gp_Dir aX = new(new gp_Vec(aN).Crossed(new gp_Vec(theO.Dir())));
        return new N.gp_Ax3(new N.gp_Pnt(theO.Real(-theRange, theRange), theO.Real(-theRange, theRange), theO.Real(-theRange, theRange)),
                            new N.gp_Dir(aN.X(), aN.Y(), aN.Z()), new N.gp_Dir(aX.X(), aX.Y(), aX.Z()));
    }

    // A random analytic surface through or near the origin.
    private static N.Geom_Surface Surface(Oracle theO, int theKind) => theKind switch
    {
        0 => new N.Geom_Plane(Axes(theO, 1)),
        1 => new N.Geom_CylindricalSurface(Axes(theO, 1), theO.Real(0.5, 3)),
        2 => new N.Geom_SphericalSurface(Axes(theO, 1), theO.Real(0.5, 3)),
        _ => new N.Geom_ConicalSurface(Axes(theO, 1), theO.Real(0.1, 1.2), theO.Real(0.5, 3)),
    };

    private static List<N.Geom_Curve> NativeLines(N.Geom_Surface theS1, N.Geom_Surface theS2)
    {
        using var anInter = new N.GeomAPI_IntSS(theS1, theS2, 1.0e-7);
        List<N.Geom_Curve> aLines = [];
        if (anInter.IsDone())
        {
            for (int i = 1; i <= anInter.NbLines(); i++)
            {
                aLines.Add(anInter.Line(i)!);
            }
        }
        return aLines;
    }

    [Fact]
    public void Native_lines_compare_bit_identical_with_themselves_and_by_their_distance_once_moved()
    {
        var anOracle = new Oracle(output, 50301);
        using var aSame = new IntersectionComparison();
        using var aMoved = new IntersectionComparison();
        int aPairs = 0;
        for (int i = 0; i < 64; i++)
        {
            using N.Geom_Surface aS1 = Surface(anOracle, i % 4);
            using N.Geom_Surface aS2 = Surface(anOracle, (i / 4) % 4);
            List<N.Geom_Curve> aNative = NativeLines(aS1, aS2);
            if (aNative.Count == 0)
            {
                continue;
            }
            aPairs++;
            List<Geom_Curve> aPort = [.. aNative.Select(aSame.PortCurve)];
            List<N.Geom_Curve> aRead = [.. aNative.Select(aSame.NativeRoundTrip)];
            aSame.CompareLines(aRead, aPort, $"pair {i}");

            gp_Vec aShift = new(1.0e-9, -2.0e-9, 2.0e-9); // 3e-9 long
            List<Geom_Curve> aShifted = [.. aPort.Select(c => (Geom_Curve)c.Translated(aShift))];
            aMoved.CompareLines(aNative, aShifted, $"pair {i}, moved");
            foreach (N.Geom_Curve aC in aNative.Concat(aRead))
            {
                aC.Dispose();
            }
        }
        output.WriteLine(aSame.Report($"{aPairs} analytic pairs"));
        output.WriteLine(aMoved.Report($"{aPairs} analytic pairs, moved"));

        Assert.True(aPairs > 20);
        Assert.Equal(aSame.Lines, aSame.IdenticalLines);
        Assert.Equal(aSame.Values, aSame.IdenticalValues);
        Assert.Equal(0.0, aSame.MaxDeviation);
        Assert.True(aMoved.IdenticalLines < aMoved.Lines);
        Assert.InRange(aMoved.MaxDeviation, 2.9e-9, 3.1e-9);
    }

    [Fact]
    public void Different_line_counts_or_types_fail()
    {
        using var aCompare = new IntersectionComparison();
        using var aNLine = new N.Geom_Line(new N.gp_Pnt(0, 0, 0), new N.gp_Dir(1, 0, 0));
        using var aNCircle = new N.Geom_Circle(new N.gp_Ax2(new N.gp_Pnt(0, 0, 0), new N.gp_Dir(0, 0, 1)), 1.0);
        Geom_Curve aLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));

        Assert.ThrowsAny<Exception>(() => aCompare.CompareLines([aNLine, aNCircle], [aLine], "count"));
        Assert.ThrowsAny<Exception>(() => aCompare.CompareLine(aNCircle, aLine, "type"));
        aCompare.CompareLine(aNLine, aLine, "same");
        Assert.Equal(1, aCompare.IdenticalLines);
    }

    [Fact]
    public void Two_dimensional_lines_compare_through_an_edge_on_a_plane()
    {
        using var aCompare = new IntersectionComparison();
        using var aNCircle = new N.Geom2d_Circle(new N.gp_Ax2d(new N.gp_Pnt2d(1, 2), new N.gp_Dir2d(1, 0)), 3.0);
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 0)), 3.0);
        aCompare.CompareLine2d(aNCircle, aCircle, "same circle");
        Geom2d_Circle aBigger = new(new gp_Ax2d(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 0)), 3.0 + 1.0e-8);
        aCompare.CompareLine2d(aNCircle, aBigger, "bigger circle");
        Assert.Equal(1, aCompare.IdenticalLines);
        Assert.InRange(aCompare.MaxDeviation, 0.99e-8, 1.01e-8);
    }
}
