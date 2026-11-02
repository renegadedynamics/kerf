// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// IntAna_IntConicQuad and IntAna_IntLinTorus against native OCCT through GeomAPI_IntCS. For a conic (line, circle,
/// ellipse, parabola, hyperbola) and an elementary surface, IntCurveSurface_HInter calls IntAna_IntConicQuad with
/// the conic and the plane (angular tolerance 1e-12, and 1e-8 for closed conics) or the quadric, and IntAna_IntLinTorus
/// for a line and a torus. Each point it keeps has the conic parameter IntAna computed (put in the period for closed
/// conics) and the surface parameters of ElSLib::Parameters (IntLinTorus's own for the torus), put in the period; it
/// drops points outside the surface's bounds and returns none when the conic is parallel to or in the quadric. The
/// port's results go through the same steps.
/// </summary>
public class IntAnaConicQuadOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;
    private const double TolAng = 1.0e-12;
    private const double Tol = 1.0e-8;
    private const double TolTangency = 0.00000001;

    // Against a quadric, the parameters are roots of a polynomial of degree 2 to 4 (math_DirectPolynomialRoots,
    // whose cubic and quartic solvers use cos, acos and cbrt): the last-bit differences of those functions between
    // MSVC and .NET reach 1e-11 relative, and more near double roots (a conic tangent to the quadric).
    private const double QuadricTolerance = 1.0e-9;

    private static gp_Ax3 SettledAx3(Oracle theOracle, in gp_Pnt theLoc)
    {
        gp_Ax3 anAx = theOracle.Ax3();
        var aSettled = new gp_Ax3(theLoc, Oracle.Settled(anAx.Direction()), Oracle.Settled(anAx.XDirection()));
        return anAx.Direct() ? aSettled : aSettled.YReversed();
    }

    private static gp_Ax2 SettledAx2(Oracle theOracle, in gp_Pnt theLoc)
    {
        gp_Ax2 anAx = theOracle.Ax2();
        return new gp_Ax2(theLoc, Oracle.Settled(anAx.Direction()), Oracle.Settled(anAx.XDirection()));
    }

    public enum Surf { Plane, Cylinder, Cone, Sphere }

    public enum Conic { Line, Circle, Ellipse, Parabola, Hyperbola }

    private readonly record struct PortPoint(double U, double V, double W);

    /// <summary>The points IntCurveSurface keeps (ComputeAppendPoint): parameters put in the period, then bounded.</summary>
    private static List<PortPoint> Keep(IntAna_IntConicQuad theInter, Conic theConic, Surf theSurf, in gp_Pln thePln,
                                        in gp_Cylinder theCyl, in gp_Cone theCone, in gp_Sphere theSph)
    {
        var aResult = new List<PortPoint>();
        if (!theInter.IsDone() || theInter.IsInQuadric() || theInter.IsParallel())
        {
            return aResult;
        }

        for (int i = 1; i <= theInter.NbPoints(); i++)
        {
            gp_Pnt aP = theInter.Point(i);
            double aW = theInter.ParamOnConic(i), aU = 0, aV = 0;
            switch (theSurf)
            {
                case Surf.Plane: ElSLib.Parameters(thePln, aP, out aU, out aV); break;
                case Surf.Cylinder: ElSLib.Parameters(theCyl, aP, out aU, out aV); break;
                case Surf.Cone: ElSLib.Parameters(theCone, aP, out aU, out aV); break;
                case Surf.Sphere: ElSLib.Parameters(theSph, aP, out aU, out aV); break;
            }

            if (theConic is Conic.Circle or Conic.Ellipse)
            {
                aW = ElCLib.InPeriod(aW, 0.0, 2 * Math.PI);
            }
            if (theSurf != Surf.Plane)
            {
                aU = ElCLib.InPeriod(aU, 0.0, 2 * Math.PI);
            }
            if (theSurf == Surf.Sphere
                && ((-Math.PI / 2 - aV) >= TolTangency || (aV - Math.PI / 2) >= TolTangency))
            {
                continue;
            }
            aResult.Add(new PortPoint(aU, aV, aW));
        }
        return aResult;
    }

    private static void Compare(Oracle theOracle, N.Geom_Curve theC, N.Geom_Surface theS, List<PortPoint> thePort,
                                Func<double, gp_Pnt> theValue, double theTol, string theWhat)
    {
        using var anIntCS = new N.GeomAPI_IntCS(theC, theS);
        theOracle.Same(true, anIntCS.IsDone(), $"{theWhat} IsDone");
        theOracle.Same(anIntCS.NbPoints(), thePort.Count, $"{theWhat} NbPoints");
        for (int i = 1; i <= thePort.Count; i++)
        {
            double aU = 0, aV = 0, aW = 0;
            anIntCS.Parameters(i, ref aU, ref aV, ref aW);
            PortPoint aPort = thePort[i - 1];
            theOracle.Same(aW, aPort.W, $"{theWhat} point {i} W", theTol);
            theOracle.Same(aU, aPort.U, $"{theWhat} point {i} U", theTol);
            theOracle.Same(aV, aPort.V, $"{theWhat} point {i} V", theTol);
            using var aNP = anIntCS.Point(i);
            theOracle.Same(aNP, theValue(aPort.W), $"{theWhat} point {i}", theTol);
        }
    }

    private void Run(Conic theConic, Surf theSurf)
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            string aWhat = $"{theConic}/{theSurf} case {i}";
            gp_Ax3 aSAx = SettledAx3(anOracle, anOracle.Pnt());
            double aR = anOracle.Real(1, 20);
            double aSemiAngle = anOracle.Real(0.1, 1.4);
            var aPln = new gp_Pln(aSAx);
            var aCyl = new gp_Cylinder(aSAx, aR);
            var aCone = new gp_Cone(aSAx, aSemiAngle, aR);
            var aSph = new gp_Sphere(aSAx, aR);
            using var aNSAx = Oracle.ToNative(aSAx);
            using N.Geom_Surface aNS = theSurf switch
            {
                Surf.Plane => new N.Geom_Plane(aNSAx),
                Surf.Cylinder => new N.Geom_CylindricalSurface(aNSAx, aR),
                Surf.Cone => new N.Geom_ConicalSurface(aNSAx, aSemiAngle, aR),
                _ => new N.Geom_SphericalSurface(aNSAx, aR),
            };
            IntAna_Quadric aQuad = theSurf switch
            {
                Surf.Plane => new IntAna_Quadric(aPln),
                Surf.Cylinder => new IntAna_Quadric(aCyl),
                Surf.Cone => new IntAna_Quadric(aCone),
                _ => new IntAna_Quadric(aSph),
            };

            // The conic near the surface's location
            gp_Pnt aLoc = aSAx.Location().Translated(new gp_Vec(anOracle.Real(-aR, aR), anOracle.Real(-aR, aR),
                                                                  anOracle.Real(-aR, aR)));
            gp_Ax2 aCAx = SettledAx2(anOracle, aLoc);
            double aMaj = anOracle.Real(0.5, 2 * aR), aMin = anOracle.Real(0.2, aMaj);
            using var aNCAx = Oracle.ToNative(aCAx);
            var anInter = new IntAna_IntConicQuad();
            N.Geom_Curve aNC;
            Func<double, gp_Pnt> aValue;
            switch (theConic)
            {
                case Conic.Line:
                {
                    var aLin = new gp_Lin(aCAx.Axis());
                    using var aNAx1 = Oracle.ToNative(aCAx.Axis());
                    aNC = new N.Geom_Line(aNAx1);
                    if (theSurf == Surf.Plane)
                    {
                        anInter.Perform(aLin, aPln, TolAng);
                    }
                    else
                    {
                        anInter.Perform(aLin, aQuad);
                    }
                    aValue = w => ElCLib.Value(w, aLin);
                    break;
                }
                case Conic.Circle:
                {
                    var aCirc = new gp_Circ(aCAx, aMaj);
                    aNC = new N.Geom_Circle(aNCAx, aMaj);
                    if (theSurf == Surf.Plane)
                    {
                        anInter.Perform(aCirc, aPln, TolAng, Tol);
                    }
                    else
                    {
                        anInter.Perform(aCirc, aQuad);
                    }
                    aValue = w => ElCLib.Value(w, aCirc);
                    break;
                }
                case Conic.Ellipse:
                {
                    var anElips = new gp_Elips(aCAx, aMaj, aMin);
                    aNC = new N.Geom_Ellipse(aNCAx, aMaj, aMin);
                    if (theSurf == Surf.Plane)
                    {
                        anInter.Perform(anElips, aPln, TolAng, Tol);
                    }
                    else
                    {
                        anInter.Perform(anElips, aQuad);
                    }
                    aValue = w => ElCLib.Value(w, anElips);
                    break;
                }
                case Conic.Parabola:
                {
                    var aParab = new gp_Parab(aCAx, aMin);
                    aNC = new N.Geom_Parabola(aNCAx, aMin);
                    if (theSurf == Surf.Plane)
                    {
                        anInter.Perform(aParab, aPln, TolAng);
                    }
                    else
                    {
                        anInter.Perform(aParab, aQuad);
                    }
                    aValue = w => ElCLib.Value(w, aParab);
                    break;
                }
                default:
                {
                    var aHypr = new gp_Hypr(aCAx, aMaj, aMin);
                    aNC = new N.Geom_Hyperbola(aNCAx, aMaj, aMin);
                    if (theSurf == Surf.Plane)
                    {
                        anInter.Perform(aHypr, aPln, TolAng);
                    }
                    else
                    {
                        anInter.Perform(aHypr, aQuad);
                    }
                    aValue = w => ElCLib.Value(w, aHypr);
                    break;
                }
            }

            using (aNC)
            {
                Compare(anOracle, aNC, aNS, Keep(anInter, theConic, theSurf, aPln, aCyl, aCone, aSph), aValue,
                        theSurf == Surf.Plane ? Oracle.Tolerance : QuadricTolerance, aWhat);
            }
        }
        anOracle.Report($"IntAna_IntConicQuad {theConic}/{theSurf}");
    }

    [Theory]
    [InlineData(Conic.Line, Surf.Plane)]
    [InlineData(Conic.Line, Surf.Cylinder)]
    [InlineData(Conic.Line, Surf.Cone)]
    [InlineData(Conic.Line, Surf.Sphere)]
    [InlineData(Conic.Circle, Surf.Plane)]
    [InlineData(Conic.Circle, Surf.Cylinder)]
    [InlineData(Conic.Circle, Surf.Cone)]
    [InlineData(Conic.Circle, Surf.Sphere)]
    [InlineData(Conic.Ellipse, Surf.Plane)]
    [InlineData(Conic.Ellipse, Surf.Cylinder)]
    [InlineData(Conic.Ellipse, Surf.Cone)]
    [InlineData(Conic.Ellipse, Surf.Sphere)]
    [InlineData(Conic.Parabola, Surf.Plane)]
    [InlineData(Conic.Parabola, Surf.Cylinder)]
    [InlineData(Conic.Parabola, Surf.Cone)]
    [InlineData(Conic.Parabola, Surf.Sphere)]
    [InlineData(Conic.Hyperbola, Surf.Plane)]
    [InlineData(Conic.Hyperbola, Surf.Cylinder)]
    [InlineData(Conic.Hyperbola, Surf.Cone)]
    [InlineData(Conic.Hyperbola, Surf.Sphere)]
    public void ConicQuadric(Conic theConic, Surf theSurf) => Run(theConic, theSurf);

    [Fact]
    public void Line_torus()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            string aWhat = $"Line/torus case {i}";
            gp_Ax3 aSAx = SettledAx3(anOracle, anOracle.Pnt());
            double aMajor = anOracle.Real(3, 20), aMinor = anOracle.Real(0.5, aMajor - 0.5);
            var aTorus = new gp_Torus(aSAx, aMajor, aMinor);
            using var aNSAx = Oracle.ToNative(aSAx);
            using var aNS = new N.Geom_ToroidalSurface(aNSAx, aMajor, aMinor);

            gp_Pnt aLoc = aSAx.Location().Translated(new gp_Vec(anOracle.Real(-aMajor, aMajor),
                                                                  anOracle.Real(-aMajor, aMajor),
                                                                  anOracle.Real(-aMinor, aMinor)));
            var aLin = new gp_Lin(aLoc, Oracle.Settled(anOracle.Dir()));
            using var aNAx1 = Oracle.ToNative(aLin.Position());
            using var aNC = new N.Geom_Line(aNAx1);

            var anInter = new IntAna_IntLinTorus(aLin, aTorus);
            using var anIntCS = new N.GeomAPI_IntCS(aNC, aNS);
            anOracle.Same(true, anIntCS.IsDone(), $"{aWhat} IsDone");
            // A failed IntLinTorus sends IntCurveSurface to its sampling algorithm: not compared
            if (!anInter.IsDone())
            {
                continue;
            }
            anOracle.Same(anIntCS.NbPoints(), anInter.NbPoints(), $"{aWhat} NbPoints");
            for (int j = 1; j <= anInter.NbPoints(); j++)
            {
                double aU = 0, aV = 0, aW = 0;
                anIntCS.Parameters(j, ref aU, ref aV, ref aW);
                anInter.ParamOnTorus(j, out double aFi, out double aTheta);
                anOracle.Same(aW, anInter.ParamOnLine(j), $"{aWhat} point {j} W");
                anOracle.Same(aU, ElCLib.InPeriod(aFi, 0.0, 2 * Math.PI), $"{aWhat} point {j} U");
                anOracle.Same(aV, ElCLib.InPeriod(aTheta, 0.0, 2 * Math.PI), $"{aWhat} point {j} V");
                using var aNP = anIntCS.Point(j);
                anOracle.Same(aNP, anInter.Value(j), $"{aWhat} point {j}");
            }
        }
        anOracle.Report("IntAna_IntLinTorus");
    }
}
