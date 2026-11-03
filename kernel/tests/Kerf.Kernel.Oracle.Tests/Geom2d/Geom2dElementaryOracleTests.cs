// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.GeomOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>Geom2d_Line, Geom2d_Circle and Geom2d_Ellipse against native OCCT, on random inputs.</summary>
public class Geom2dElementaryOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;

    /// <summary>Evaluation, reversal and transformation, common to the three curves.</summary>
    private static void CheckCurve(Oracle theOracle, N.Geom2d_Curve theNative, Curve2D thePort, string theWhat,
                                   bool theInfinite)
    {
        Geom2dOracle.SameBounds(theOracle, theNative, thePort, theWhat);
        for (int k = 0; k < 3; k++)
        {
            double U = theInfinite ? theOracle.Real(-50, 50) : theOracle.Real(-10, 10);
            Geom2dOracle.SameEval(theOracle, theNative, thePort, U, theWhat);
            theOracle.Same(theNative.ReversedParameter(U), thePort.ReversedParameter(U), $"{theWhat} ReversedParameter");
        }

        // Reversed copy, then evaluation
        using (var aNRev = theNative.Reversed())
        {
            Curve2D aRev = thePort.Reversed();
            double U = theOracle.Real(-10, 10);
            Geom2dOracle.SameEval(theOracle, aNRev!, aRev, U, $"{theWhat} reversed");
        }

        // Transformed copy, then evaluation and parameter transformation
        var (aT, aNT) = Geom2dOracle.Trsf(theOracle);
        using (aNT)
        {
            using var aNGeom = theNative.Transformed(aNT);
            using var aNTr = N.Geom2d_Curve.DownCast(aNGeom);
            var aTr = (Curve2D)thePort.Transformed(aT);
            double U = theOracle.Real(-10, 10);
            theOracle.Same(theNative.TransformedParameter(U, aNT), thePort.TransformedParameter(U, aT),
                           $"{theWhat} TransformedParameter");
            theOracle.Same(theNative.ParametricTransformation(aNT), thePort.ParametricTransformation(aT),
                           $"{theWhat} ParametricTransformation");
            Geom2dOracle.SameEval(theOracle, aNTr!, aTr, aTr.TransformedParameter(U, aT), $"{theWhat} transformed");
            Geom2dOracle.SameBounds(theOracle, aNTr!, aTr, $"{theWhat} transformed");
        }
    }

    [Fact]
    public void Line()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Ax2d aNAx, Axis2D anAx) = Ax2ds(anOracle);
            using var aNAxOwner = aNAx;
            using var aNative = new N.Geom2d_Line(aNAx);
            var aPort = new LineCurve2D(anAx);
            CheckCurve(anOracle, aNative, aPort, "Line", true);

            Point2D aP = anOracle.Pnt2d();
            using var aNP = Oracle.ToNative(aP);
            anOracle.Same(aNative.Distance(aNP), aPort.Distance(aP), "Line Distance");
        }

        anOracle.Report("Geom2d_Line");
    }

    [Fact]
    public void Circle()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Ax2d aNAx, Axis2D anAx) = Ax2ds(anOracle);
            using var aNAxOwner = aNAx;
            double aR = anOracle.Real(0.1, 50);
            bool aSense = anOracle.Bool();
            using var aNative = new N.Geom2d_Circle(aNAx, aR, aSense);
            var aPort = new CircleCurve2D(anAx, aR, aSense);
            CheckCurve(anOracle, aNative, aPort, "Circle", false);
            anOracle.Same(aNative.Radius(), aPort.Radius(), "Circle Radius");
            anOracle.Same(aNative.Eccentricity(), aPort.Eccentricity(), "Circle Eccentricity");
            anOracle.Same(aNative.Period(), aPort.Period(), "Circle Period");
        }

        anOracle.Report("Geom2d_Circle");
    }

    [Fact]
    public void Ellipse()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Ax2d aNAx, Axis2D anAx) = Ax2ds(anOracle);
            using var aNAxOwner = aNAx;
            double aMaj = anOracle.Real(1.0, 50);
            double aMin = anOracle.Real(0.05, 0.95) * aMaj;
            bool aSense = anOracle.Bool();
            using var aNative = new N.Geom2d_Ellipse(aNAx, aMaj, aMin, aSense);
            var aPort = new EllipseCurve2D(anAx, aMaj, aMin, aSense);
            CheckCurve(anOracle, aNative, aPort, "Ellipse", false);

            anOracle.Same(aNative.Eccentricity(), aPort.Eccentricity(), "Ellipse Eccentricity");
            anOracle.Same(aNative.Focal(), aPort.Focal(), "Ellipse Focal");
            anOracle.Same(aNative.Parameter(), aPort.Parameter(), "Ellipse Parameter");
            using (var aF1 = aNative.Focus1())
            using (var aF2 = aNative.Focus2())
            {
                anOracle.Same(Oracle.FromNative(aF1), aPort.Focus1().XY(), "Ellipse Focus1");
                anOracle.Same(Oracle.FromNative(aF2), aPort.Focus2().XY(), "Ellipse Focus2");
            }
            using (var aD1 = aNative.Directrix1())
            using (var aD2 = aNative.Directrix2())
            using (var aD1P = aD1.Location())
            using (var aD1D = aD1.Direction())
            using (var aD2P = aD2.Location())
            using (var aD2D = aD2.Direction())
            {
                anOracle.Same(Oracle.FromNative(aD1P), aPort.Directrix1().Location().XY(), "Ellipse Directrix1 P");
                anOracle.Same(Oracle.FromNative(aD1D), aPort.Directrix1().Direction().XY(), "Ellipse Directrix1 D");
                anOracle.Same(Oracle.FromNative(aD2P), aPort.Directrix2().Location().XY(), "Ellipse Directrix2 P");
                anOracle.Same(Oracle.FromNative(aD2D), aPort.Directrix2().Direction().XY(), "Ellipse Directrix2 D");
            }
        }

        anOracle.Report("Geom2d_Ellipse");
    }
}
