#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>GeomFill_Frenet against native OCCT on random spines of every kind (<see cref="TrihedronOracle"/>).</summary>
public class GeomFill_FrenetOracleTests(ITestOutputHelper output)
{
    [Fact]
    public void RandomSpines()
    {
        TrihedronOracle.Run(output, "GeomFill_Frenet", () => new N.GeomFill_Frenet(), () => new GeomFill_Frenet(), 40,
                            20261030);
    }

    /// <summary>
    /// Planar B-splines (degree 2 to 5, C0 to C(d-1) knots, rational or not) with inflexions and straight spans:
    /// the singular points Frenet searches (C' ^ C'' = 0) and the trihedron near them.
    /// </summary>
    [Fact]
    public void PlanarSpines_SingularPoints()
    {
        var anOracle = new Oracle(output, 20261031);
        int aNbSingular = 0;
        for (int i = 0; i < 150; i++)
        {
            int aDeg = anOracle.Int(2, 5);
            int aNbKnots = anOracle.Int(2, 5);
            var aKnots = new NCollection_Array1<double>(1, aNbKnots);
            var aMults = new NCollection_Array1<int>(1, aNbKnots);
            aKnots[1] = anOracle.Real(-2, 2);
            for (int k = 2; k <= aNbKnots; k++)
            {
                aKnots[k] = aKnots[k - 1] + anOracle.Real(0.3, 2);
                aMults[k] = anOracle.Int(1, aDeg);
            }
            aMults[1] = aMults[aNbKnots] = aDeg + 1;
            int aNbPoles = Occt.BSplCLib.NbPoles(aDeg, false, aMults);
            var aPoles = new NCollection_Array1<gp_Pnt>(1, aNbPoles);
            for (int k = 1; k <= aNbPoles; k++)
            {
                // a zigzag in the XY plane (inflexions); now and then three aligned poles (a straight piece)
                double aY = (k % 2 == 0 ? 1 : -1) * anOracle.Real(0.5, 3);
                aPoles[k] = new gp_Pnt(k * 2.0 + anOracle.Real(-0.3, 0.3), aY, 0);
                if (k > 2 && anOracle.Int(0, 5) == 0)
                {
                    aPoles[k] = new gp_Pnt(2 * aPoles[k - 1].X() - aPoles[k - 2].X(), 2 * aPoles[k - 1].Y() - aPoles[k - 2].Y(), 0);
                }
            }
            NCollection_Array1<double>? aWeights = anOracle.Bool() ? null : anOracle.Weights(aNbPoles, true);
            using N.Geom_BSplineCurve aNC = Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDeg, false);
            Geom_BSplineCurve aPC = aWeights is null ? new Geom_BSplineCurve(aPoles, aKnots, aMults, aDeg)
                                                     : new Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, aDeg);
            using var aNA = new N.GeomAdaptor_Curve(aNC);
            var aPA = new GeomAdaptor_Curve(aPC);
            using var aNLaw = new N.GeomFill_Frenet();
            var aPLaw = new GeomFill_Frenet();
            TrihedronOracle.Compare(anOracle, aNLaw, aPLaw, aNA, aPA, $"planar #{i}");
            aPLaw.SetCurve(aPA);
            if (aPLaw.NbIntervals(GeomAbs_Shape.GeomAbs_C0) > aPA.NbIntervals(GeomAbs_Shape.GeomAbs_C2))
            {
                aNbSingular++;
            }
        }
        output.WriteLine($"{aNbSingular} of 150 spines have singular points");
        Assert.True(aNbSingular > 50);
        anOracle.Report("GeomFill_Frenet on planar B-splines");
    }
}
#endif
