// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>IntAna2d_AnaIntersection and IntAna2d_IntPoint against native OCCT, on random inputs.</summary>
public class IntAna2dOracleTests(ITestOutputHelper output)
{
    private const int Cases = 1000;

    private static N.gp_Lin2d ToNative(in gp_Lin2d theL)
    {
        using var anAx = Oracle.ToNative(theL.Position());
        return new N.gp_Lin2d(anAx);
    }

    private static N.gp_Circ2d ToNative(in gp_Circ2d theC)
    {
        using var anAx = Oracle.ToNative(theC.Position());
        return new N.gp_Circ2d(anAx, theC.Radius());
    }

    private static void Compare(Oracle theOracle, N.IntAna2d_AnaIntersection theNative, IntAna2d_AnaIntersection thePort, string theWhat)
    {
        theOracle.Same(theNative.IsDone(), thePort.IsDone(), $"{theWhat} IsDone");
        if (!theNative.IsDone())
        {
            return;
        }

        theOracle.Same(theNative.IsEmpty(), thePort.IsEmpty(), $"{theWhat} IsEmpty");
        theOracle.Same(theNative.IdenticalElements(), thePort.IdenticalElements(), $"{theWhat} IdenticalElements");
        theOracle.Same(theNative.ParallelElements(), thePort.ParallelElements(), $"{theWhat} ParallelElements");
        theOracle.Same(theNative.NbPoints(), thePort.NbPoints(), $"{theWhat} NbPoints");
        for (int i = 1; i <= theNative.NbPoints(); i++)
        {
            using var aNPoint = theNative.Point(i);
            using var aNValue = aNPoint.Value();
            IntAna2d_IntPoint aPoint = thePort.Point(i);
            theOracle.Same(Oracle.FromNative(aNValue), aPoint.Value(), $"{theWhat} point {i}");
            theOracle.Same(aNPoint.SecondIsImplicit(), aPoint.SecondIsImplicit(), $"{theWhat} SecondIsImplicit {i}");
            theOracle.Same(aNPoint.ParamOnFirst(), aPoint.ParamOnFirst(), $"{theWhat} ParamOnFirst {i}");
            theOracle.Same(aNPoint.ParamOnSecond(), aPoint.ParamOnSecond(), $"{theWhat} ParamOnSecond {i}");
        }
    }

    [Fact]
    public void Two_lines()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            gp_Lin2d aL1 = new(anOracle.Ax2d());
            gp_Lin2d aL2 = (i % 10) switch
            {
                0 => new gp_Lin2d(anOracle.Pnt2d(), aL1.Direction()), // parallel
                1 => new gp_Lin2d(aL1.Location(), aL1.Direction()), // identical
                _ => new gp_Lin2d(anOracle.Ax2d()),
            };
            using var aN1 = ToNative(aL1);
            using var aN2 = ToNative(aL2);
            using var aNative = new N.IntAna2d_AnaIntersection(aN1, aN2);
            Compare(anOracle, aNative, new IntAna2d_AnaIntersection(aL1, aL2), "lines");
        }

        anOracle.Report("IntAna2d_AnaIntersection(gp_Lin2d, gp_Lin2d)");
    }

    [Fact]
    public void Two_circles()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax22d anAx1 = anOracle.Ax22d();
            double aR1 = anOracle.Real(1, 50);
            gp_Circ2d aC1 = new(anAx1, aR1);
            // Second center near the first so that most pairs intersect; concentric one time in ten
            gp_Pnt2d aCenter2 = i % 10 == 0
                                    ? anAx1.Location()
                                    : anAx1.Location().Translated(new gp_Vec2d(anOracle.Real(-60, 60), anOracle.Real(-60, 60)));
            gp_Circ2d aC2 = new(new gp_Ax22d(aCenter2, anOracle.Dir2d(), anOracle.Bool()), i % 20 == 0 ? aR1 : anOracle.Real(1, 50));
            using var aN1 = ToNative(aC1);
            using var aN2 = ToNative(aC2);
            using var aNative = new N.IntAna2d_AnaIntersection(aN1, aN2);
            // Tangency and angle parameters go through acos/asin and Angle (atan2), which MSVC and .NET may round differently
            Compare(anOracle, aNative, new IntAna2d_AnaIntersection(aC1, aC2), "circles");
        }

        anOracle.Report("IntAna2d_AnaIntersection(gp_Circ2d, gp_Circ2d)");
    }

    [Fact]
    public void Line_and_circle()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            gp_Ax22d anAx = anOracle.Ax22d();
            double aR = anOracle.Real(1, 50);
            gp_Circ2d aC = new(anAx, aR);
            // A line through a point near the circle, crossing it most of the time
            gp_Pnt2d aP = anAx.Location().Translated(new gp_Vec2d(anOracle.Real(-1.2 * aR, 1.2 * aR), anOracle.Real(-1.2 * aR, 1.2 * aR)));
            gp_Lin2d aL = new(aP, anOracle.Dir2d());
            using var aNL = ToNative(aL);
            using var aNC = ToNative(aC);
            using var aNative = new N.IntAna2d_AnaIntersection(aNL, aNC);
            Compare(anOracle, aNative, new IntAna2d_AnaIntersection(aL, aC), "line and circle");
        }

        anOracle.Report("IntAna2d_AnaIntersection(gp_Lin2d, gp_Circ2d)");
    }

    [Fact]
    public void IntPoint()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < 100; i++)
        {
            double aX = anOracle.Real(), aY = anOracle.Real(), aU1 = anOracle.Real(), aU2 = anOracle.Real();
            using var aN1 = new N.IntAna2d_IntPoint(aX, aY, aU1, aU2);
            using var aN2 = new N.IntAna2d_IntPoint(aX, aY, aU1);
            using var aN3 = new N.IntAna2d_IntPoint();
            var aP1 = new IntAna2d_IntPoint(aX, aY, aU1, aU2);
            var aP2 = new IntAna2d_IntPoint(aX, aY, aU1);
            var aP3 = new IntAna2d_IntPoint();
            foreach ((N.IntAna2d_IntPoint aN, IntAna2d_IntPoint aP) in new[] { (aN1, aP1), (aN2, aP2), (aN3, aP3) })
            {
                using var aNV = aN.Value();
                anOracle.Same(Oracle.FromNative(aNV), aP.Value(), "Value");
                anOracle.Same(aN.SecondIsImplicit(), aP.SecondIsImplicit(), "SecondIsImplicit");
                anOracle.Same(aN.ParamOnFirst(), aP.ParamOnFirst(), "ParamOnFirst");
                if (!aP.SecondIsImplicit())
                {
                    anOracle.Same(aN.ParamOnSecond(), aP.ParamOnSecond(), "ParamOnSecond");
                }
            }

            aN3.SetValue(aX, aY, aU1);
            aP3.SetValue(aX, aY, aU1);
            anOracle.Same(aN3.SecondIsImplicit(), aP3.SecondIsImplicit(), "SetValue implicit");
            aN3.SetValue(aX, aY, aU1, aU2);
            aP3.SetValue(aX, aY, aU1, aU2);
            anOracle.Same(aN3.ParamOnSecond(), aP3.ParamOnSecond(), "SetValue ParamOnSecond");
        }

        anOracle.Report("IntAna2d_IntPoint");
    }
}
