// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Geom2dConvert against native OCCT: CurveToBSplineCurve on trimmed lines, whole and trimmed circles and ellipses
/// (every parameterisation, long RationalC1 arcs included, which go through Geom2dConvert_CompCurveToBSplineCurve)
/// and B-splines (periodic or not, whole or trimmed), and both SplitBSplineCurve overloads. The bindings have no 2D
/// Bezier, offset, hyperbola or parabola, so those paths are unit-tested only.
/// </summary>
public class Geom2dConvertOracleTests(ITestOutputHelper output)
{
    private const int Cases = 600;

    private enum Kind
    {
        Line,
        Circle,
        Ellipse,
        BSpline,
    }

    private static void Ax22d(Oracle theOracle, out Frame2D thePort, out N.gp_Ax22d theNative)
    {
        var aP = new Point2D(theOracle.Real(), theOracle.Real());
        Xy aD = GeomOracleInputs.RawDir2d(theOracle);
        bool isSense = theOracle.Bool();
        thePort = new Frame2D(aP, new Direction2D(aD.X(), aD.Y()), isSense);
        using N.gp_Pnt2d aNP = Oracle.ToNative(aP);
        using var aND = new N.gp_Dir2d(aD.X(), aD.Y());
        theNative = new N.gp_Ax22d(aNP, aND, isSense);
    }

    [Fact]
    public void Curves_to_BSpline_curves()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0, aNbLongRationalC1 = 0;
        for (int i = 0; i < Cases; i++)
        {
            var aKind = (Kind)anOracle.Int(0, 3);
            var aParam = (ParameterisationType)anOracle.Int(0, 7);
            bool isWhole = aKind != Kind.Line && anOracle.Int(0, 3) == 0;
            N.Geom2d_Curve aNBasis;
            Curve2D aBasis;
            switch (aKind)
            {
                case Kind.Line:
                {
                    (N.gp_Ax2d aNAx, Axis2D anAx) = GeomOracleInputs.Ax2ds(anOracle);
                    using (aNAx)
                    {
                        aNBasis = new N.Geom2d_Line(aNAx);
                    }

                    aBasis = new LineCurve2D(anAx);
                    break;
                }
                case Kind.Circle:
                case Kind.Ellipse:
                {
                    Ax22d(anOracle, out Frame2D anAx, out N.gp_Ax22d aNAx);
                    double aR1 = anOracle.Real(0.5, 20), aR2 = anOracle.Real(0.1, aR1);
                    using (aNAx)
                    {
                        aNBasis = aKind == Kind.Circle ? new N.Geom2d_Circle(aNAx, aR1) : new N.Geom2d_Ellipse(aNAx, aR1, aR2);
                    }

                    aBasis = aKind == Kind.Circle ? new CircleCurve2D(anAx, aR1) : new EllipseCurve2D(anAx, aR1, aR2);
                    break;
                }
                default:
                {
                    BSplineSpec aSpec = Geom2dOracle.RandomBSpline(anOracle);
                    aNBasis = Geom2dOracle.ToNative(aSpec);
                    aBasis = Geom2dOracle.ToPort(aSpec);
                    break;
                }
            }

            using N.Geom2d_Curve aNBasisOwned = aNBasis;
            N.Geom2d_Curve aNCurve = aNBasis;
            Curve2D aCurve = aBasis;
            N.Geom2d_TrimmedCurve? aNTrimmed = null;
            if (!isWhole)
            {
                double aU1, aU2;
                if (aKind == Kind.BSpline)
                {
                    double aF = aBasis.FirstParameter(), aL = aBasis.LastParameter();
                    aU1 = anOracle.Real(aF, aL);
                    aU2 = aBasis.IsPeriodic && anOracle.Bool() ? aU1 + anOracle.Real(0.05, 1.0) * (aL - aF)
                                                                  : anOracle.Real(aU1, aL);
                    if (aU2 - aU1 < 1e-3)
                    {
                        aU2 = aL;
                    }
                }
                else if (aKind == Kind.Line)
                {
                    aU1 = anOracle.Real(-50, 50);
                    aU2 = aU1 + anOracle.Real(0.1, 50);
                }
                else
                {
                    aU1 = anOracle.Real(-7, 7);
                    // half the arcs long, to exercise the split of RationalC1 arcs of 6 radians or more
                    aU2 = aU1 + (anOracle.Bool() ? anOracle.Real(6.0, 2 * Math.PI) : anOracle.Real(0.05, 2 * Math.PI));
                    if (aParam == ParameterisationType.RationalC1 && aU2 - aU1 >= 6.0)
                    {
                        aNbLongRationalC1++;
                    }
                }

                bool aSense = anOracle.Int(0, 4) != 0;
                aNTrimmed = new N.Geom2d_TrimmedCurve(aNBasis, aU1, aU2, aSense);
                aNCurve = aNTrimmed;
                aCurve = new TrimmedCurve2D(aBasis, aU1, aU2, aSense);
            }

            using N.Geom2d_TrimmedCurve? aNTrimmedOwned = aNTrimmed;
            N.Geom2d_BSplineCurve? aNative = null;
            bool isNativeThrown = false;
            try
            {
                aNative = N.Geom2dConvert.CurveToBSplineCurve(aNCurve, (N.Convert_ParameterisationType)(int)aParam);
            }
            catch (Exception)
            {
                isNativeThrown = true;
            }

            BSplineCurve2D? aPort = null;
            bool isPortThrown = false;
            try
            {
                aPort = Geom2dConvert2D.CurveToBSplineCurve(aCurve, aParam);
            }
            catch (Exception)
            {
                isPortThrown = true;
            }

            string aWhat = $"{aKind} {(isWhole ? "whole" : "trimmed")} {aParam}";
            anOracle.Same(isNativeThrown, isPortThrown, aWhat + " throws");
            if (isNativeThrown)
            {
                aNbThrows++;
                continue;
            }

            using N.Geom2d_BSplineCurve aNResult = aNative!;
            Assert.False(aPort!.IsFrozen);
            Geom2dOracle.SameStructure(anOracle, aNResult, aPort, aWhat);
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} curves; {aNbLongRationalC1} long RationalC1 arcs");
        Assert.True(aNbLongRationalC1 > 10);
        anOracle.Report("Geom2dConvert::CurveToBSplineCurve");
    }

    [Fact]
    public void Long_RationalC1_arcs_to_BSpline_curves()
    {
        // Arcs of 6 radians or more with Convert_RationalC1: OCCT converts two halves and joins them with
        // Geom2dConvert_CompCurveToBSplineCurve. Whole turns (exactly 2 Pi) included.
        var anOracle = new Oracle(output);
        for (int i = 0; i < 300; i++)
        {
            bool isCircle = anOracle.Bool();
            Ax22d(anOracle, out Frame2D anAx, out N.gp_Ax22d aNAx);
            double aR1 = anOracle.Real(0.5, 20), aR2 = anOracle.Real(0.1, aR1);
            using N.gp_Ax22d aNAxOwned = aNAx;
            using N.Geom2d_Curve aNBasis = isCircle ? new N.Geom2d_Circle(aNAx, aR1) : new N.Geom2d_Ellipse(aNAx, aR1, aR2);
            Curve2D aBasis = isCircle ? new CircleCurve2D(anAx, aR1) : new EllipseCurve2D(anAx, aR1, aR2);
            double aU1 = anOracle.Real(-10, 10);
            double aU2 = aU1 + (anOracle.Int(0, 4) == 0 ? 2 * Math.PI : anOracle.Real(6.0, 2 * Math.PI));
            bool aSense = anOracle.Bool();
            using var aNTrimmed = new N.Geom2d_TrimmedCurve(aNBasis, aU1, aU2, aSense);
            var aTrimmed = new TrimmedCurve2D(aBasis, aU1, aU2, aSense);
            using N.Geom2d_BSplineCurve aNative =
                N.Geom2dConvert.CurveToBSplineCurve(aNTrimmed, N.Convert_ParameterisationType.Convert_RationalC1)!;
            BSplineCurve2D aPort = Geom2dConvert2D.CurveToBSplineCurve(aTrimmed, ParameterisationType.RationalC1);
            Geom2dOracle.SameStructure(anOracle, aNative, aPort, $"{(isCircle ? "circle" : "ellipse")} {aU2 - aU1}");
        }

        anOracle.Report("Geom2dConvert::CurveToBSplineCurve, long RationalC1 arcs");
    }

    [Fact]
    public void Parabolas_and_hyperbolas_to_BSpline_curves()
    {
        // The bindings have no 2D parabola or hyperbola: native OCCT gets them from GeomAPI::To2d, which projects a 3D
        // conic of the plane XOY onto it with ProjLib_Plane, and the port projects the same conic with its ProjLib.
        var anOracle = new Oracle(output);
        using var aNPln = new N.gp_Pln();
        var aPln = new Plane3D();
        for (int i = 0; i < Cases / 2; i++)
        {
            bool isParabola = anOracle.Bool();
            Xy aD = GeomOracleInputs.RawDir2d(anOracle);
            double aNz = anOracle.Bool() ? 1.0 : -1.0;
            double aPx = anOracle.Real(-50, 50), aPy = anOracle.Real(-50, 50);
            double aR1 = anOracle.Real(0.5, 20), aR2 = anOracle.Real(0.1, 20);
            double aU1 = isParabola ? anOracle.Real(-10, 8) : anOracle.Real(-3, 2);
            double aU2 = aU1 + anOracle.Real(0.05, isParabola ? 10 : 3);
            using var aNP = new N.gp_Pnt(aPx, aPy, 0.0);
            using var aNN = new N.gp_Dir(0.0, 0.0, aNz);
            using var aNVx = new N.gp_Dir(aD.X(), aD.Y(), 0.0);
            using var aNAx2 = new N.gp_Ax2(aNP, aNN, aNVx);
            var anAx2 = new Frame3D(new Point3D(aPx, aPy, 0.0), new Direction3D(0.0, 0.0, aNz), new Direction3D(aD.X(), aD.Y(), 0.0));
            using N.Geom_Curve aN3d = isParabola ? new N.Geom_Parabola(aNAx2, aR1) : new N.Geom_Hyperbola(aNAx2, aR1, aR2);
            using var aN3dTrimmed = new N.Geom_TrimmedCurve(aN3d, aU1, aU2);
            using N.Geom2d_Curve aN2d = N.GeomAPI.To2d(aN3dTrimmed, aNPln)!;
            Assert.True(aN2d.IsKind(isParabola ? "Geom2d_Parabola" : "Geom2d_Hyperbola") || aN2d.IsKind("Geom2d_TrimmedCurve"));

            Curve2D aBasis = isParabola ? new ParabolaCurve2D(global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, new Parabola3D(anAx2, aR1)))
                                             : new HyperbolaCurve2D(global::Kerf.Engine.Geometry.Projection.ProjLib.Project(aPln, new Hyperbola3D(anAx2, aR1, aR2)));
            var aTrimmed = new TrimmedCurve2D(aBasis, aU1, aU2);
            using N.Geom2d_BSplineCurve aNative = N.Geom2dConvert.CurveToBSplineCurve(aN2d)!;
            BSplineCurve2D aPort = Geom2dConvert2D.CurveToBSplineCurve(aTrimmed);
            Geom2dOracle.SameStructure(anOracle, aNative, aPort, isParabola ? "parabola" : "hyperbola");
        }

        anOracle.Report("Geom2dConvert::CurveToBSplineCurve, parabolas and hyperbolas");
    }

    [Fact]
    public void Split_by_knots()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0;
        for (int i = 0; i < Cases; i++)
        {
            BSplineSpec aSpec = Geom2dOracle.RandomBSpline(anOracle);
            using N.Geom2d_BSplineCurve aNC = Geom2dOracle.ToNative(aSpec);
            BSplineCurve2D aC = Geom2dOracle.ToPort(aSpec);
            int aK1 = anOracle.Int(aC.FirstUKnotIndex(), aC.LastUKnotIndex());
            int aK2 = anOracle.Int(aC.FirstUKnotIndex(), aC.LastUKnotIndex());
            bool aSame = anOracle.Bool();

            N.Geom2d_BSplineCurve? aNative = null;
            bool isNativeThrown = false;
            try
            {
                aNative = N.Geom2dConvert.SplitBSplineCurve(aNC, aK1, aK2, aSame);
            }
            catch (Exception)
            {
                isNativeThrown = true;
            }

            BSplineCurve2D? aPort = null;
            bool isPortThrown = false;
            try
            {
                aPort = Geom2dConvert2D.SplitBSplineCurve(aC, aK1, aK2, aSame);
            }
            catch (Exception)
            {
                isPortThrown = true;
            }

            anOracle.Same(isNativeThrown, isPortThrown, $"split {aK1} {aK2} throws");
            if (isNativeThrown)
            {
                aNbThrows++;
                continue;
            }

            using N.Geom2d_BSplineCurve aNResult = aNative!;
            Geom2dOracle.SameStructure(anOracle, aNResult, aPort!, $"split {aK1} {aK2} {aSame}");
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} splits");
        anOracle.Report("Geom2dConvert::SplitBSplineCurve (knots)");
    }

    [Fact]
    public void Split_by_parameters()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0;
        for (int i = 0; i < Cases; i++)
        {
            BSplineSpec aSpec = Geom2dOracle.RandomBSpline(anOracle);
            using N.Geom2d_BSplineCurve aNC = Geom2dOracle.ToNative(aSpec);
            BSplineCurve2D aC = Geom2dOracle.ToPort(aSpec);
            double aF = aC.FirstParameter(), aL = aC.LastParameter();
            double aU1 = anOracle.Real(aF, aL), aU2 = anOracle.Real(aF, aL);
            if (anOracle.Int(0, 4) == 0)
            {
                // on a knot
                aU1 = aC.Knot(anOracle.Int(aC.FirstUKnotIndex(), aC.LastUKnotIndex()));
            }

            bool aSame = anOracle.Bool();

            N.Geom2d_BSplineCurve? aNative = null;
            bool isNativeThrown = false;
            try
            {
                aNative = N.Geom2dConvert.SplitBSplineCurve(aNC, aU1, aU2, 1e-9, aSame);
            }
            catch (Exception)
            {
                isNativeThrown = true;
            }

            BSplineCurve2D? aPort = null;
            bool isPortThrown = false;
            try
            {
                aPort = Geom2dConvert2D.SplitBSplineCurve(aC, aU1, aU2, 1e-9, aSame);
            }
            catch (Exception)
            {
                isPortThrown = true;
            }

            anOracle.Same(isNativeThrown, isPortThrown, "split throws");
            if (isNativeThrown)
            {
                aNbThrows++;
                continue;
            }

            using N.Geom2d_BSplineCurve aNResult = aNative!;
            Geom2dOracle.SameStructure(anOracle, aNResult, aPort!, $"split {aU1} {aU2} {aSame}");
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} splits");
        anOracle.Report("Geom2dConvert::SplitBSplineCurve (parameters)");
    }
}
