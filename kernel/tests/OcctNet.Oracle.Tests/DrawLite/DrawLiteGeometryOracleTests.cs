// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GeomliteTest's curve and surface commands in Draw-lite (unit 3L2b) against native OCCT: each command is run with
/// random arguments (written with "R", which Draw::Atof reads back exactly), and the geometry it binds is compared
/// with the geometry built natively as the command's C++ builds it: parameters, bounds and D0 to D2 at the bounds and
/// at random parameters, bit for bit; cvalue, 2dcvalue, svalue, sderivative and bounds against the native
/// evaluations.
/// </summary>
/// <remarks>
/// Not compared, for want of a native binding: Geom2d_Hyperbola, Geom2d_Parabola, Geom2d_BezierCurve and
/// Geom2d_OffsetCurve (2D hyperbolas, parabolas, Bezier curves and offsets), and GeomLib_Tool (parameters).
/// </remarks>
public class DrawLiteGeometryOracleTests(ITestOutputHelper output)
{
    private const int Cases = 60;

    /// <summary>Bit-identical comparisons, counted.</summary>
    private sealed class Bits(Random theRandom)
    {
        public int Values { get; private set; }

        public double Real(double theMin, double theMax) => theMin + (theMax - theMin) * theRandom.NextDouble();

        public int Int(int theMin, int theMax) => theRandom.Next(theMin, theMax + 1);

        public bool Bool() => theRandom.Next(2) == 0;

        public void Same(double theNative, double thePort, string theWhat)
        {
            Values++;
            Assert.True(BitConverter.DoubleToInt64Bits(theNative) == BitConverter.DoubleToInt64Bits(thePort),
                        string.Create(CultureInfo.InvariantCulture, $"{theWhat}: native {theNative:R}, port {thePort:R}"));
        }

        public void Same(bool theNative, bool thePort, string theWhat)
        {
            Values++;
            Assert.True(theNative == thePort, $"{theWhat}: native {theNative}, port {thePort}");
        }

        public void Same(N.gp_Pnt theN, in gp_Pnt theP, string theWhat)
        {
            Same(theN.X(), theP.X(), theWhat + ".X");
            Same(theN.Y(), theP.Y(), theWhat + ".Y");
            Same(theN.Z(), theP.Z(), theWhat + ".Z");
        }

        public void Same(N.gp_Vec theN, in gp_Vec theP, string theWhat)
        {
            Same(theN.X(), theP.X(), theWhat + ".X");
            Same(theN.Y(), theP.Y(), theWhat + ".Y");
            Same(theN.Z(), theP.Z(), theWhat + ".Z");
        }

        public void Same(N.gp_Pnt2d theN, in gp_Pnt2d theP, string theWhat)
        {
            Same(theN.X(), theP.X(), theWhat + ".X");
            Same(theN.Y(), theP.Y(), theWhat + ".Y");
        }

        public void Same(N.gp_Vec2d theN, in gp_Vec2d theP, string theWhat)
        {
            Same(theN.X(), theP.X(), theWhat + ".X");
            Same(theN.Y(), theP.Y(), theWhat + ".Y");
        }

        // the parameters to evaluate at: the finite bounds and random ones between (or around 0 when infinite)
        public List<double> Parameters(double theFirst, double theLast)
        {
            bool anInfinite = Math.Abs(theFirst) > 1e100 || Math.Abs(theLast) > 1e100;
            double aF = anInfinite ? -10 : theFirst, aL = anInfinite ? 10 : theLast;
            var aParams = new List<double> { Real(aF, aL), Real(aF, aL), Real(aF, aL) };
            if (!anInfinite)
            {
                aParams.Add(theFirst);
                aParams.Add(theLast);
            }

            return aParams;
        }

        public void SameCurve(N.Geom_Curve theN, Geom_Curve theP, string theWhat)
        {
            Same(theN.FirstParameter(), theP.FirstParameter(), theWhat + " First");
            Same(theN.LastParameter(), theP.LastParameter(), theWhat + " Last");
            Same(theN.IsPeriodic(), theP.IsPeriodic(), theWhat + " IsPeriodic");
            using var aNP = new N.gp_Pnt();
            using var aNV1 = new N.gp_Vec();
            using var aNV2 = new N.gp_Vec();
            foreach (double U in Parameters(theP.FirstParameter(), theP.LastParameter()))
            {
                theN.D2(U, aNP, aNV1, aNV2);
                theP.D2(U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2);
                Same(aNP, P, $"{theWhat} D2({U:R}) P");
                Same(aNV1, V1, $"{theWhat} D2({U:R}) V1");
                Same(aNV2, V2, $"{theWhat} D2({U:R}) V2");
            }
        }

        public void SameCurve(N.Geom2d_Curve theN, Geom2d_Curve theP, string theWhat)
        {
            Same(theN.FirstParameter(), theP.FirstParameter(), theWhat + " First");
            Same(theN.LastParameter(), theP.LastParameter(), theWhat + " Last");
            using var aNP = new N.gp_Pnt2d();
            using var aNV1 = new N.gp_Vec2d();
            using var aNV2 = new N.gp_Vec2d();
            foreach (double U in Parameters(theP.FirstParameter(), theP.LastParameter()))
            {
                theN.D2(U, aNP, aNV1, aNV2);
                theP.D2(U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2);
                Same(aNP, P, $"{theWhat} D2({U:R}) P");
                Same(aNV1, V1, $"{theWhat} D2({U:R}) V1");
                Same(aNV2, V2, $"{theWhat} D2({U:R}) V2");
            }
        }

        public void SameSurface(N.Geom_Surface theN, Geom_Surface theP, string theWhat)
        {
            double aNU1 = 0, aNU2 = 0, aNV1 = 0, aNV2 = 0;
            theN.Bounds(ref aNU1, ref aNU2, ref aNV1, ref aNV2);
            theP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
            Same(aNU1, aU1, theWhat + " U1");
            Same(aNU2, aU2, theWhat + " U2");
            Same(aNV1, aV1, theWhat + " V1");
            Same(aNV2, aV2, theWhat + " V2");
            using var aNP = new N.gp_Pnt();
            using var aNU = new N.gp_Vec();
            using var aNV = new N.gp_Vec();
            using var aNUU = new N.gp_Vec();
            using var aNVV = new N.gp_Vec();
            using var aNUV = new N.gp_Vec();
            List<double> aUs = Parameters(aU1, aU2), aVs = Parameters(aV1, aV2);
            for (int k = 0; k < aUs.Count; k++)
            {
                double U = aUs[k], V = aVs[k % aVs.Count];
                theN.D2(U, V, aNP, aNU, aNV, aNUU, aNVV, aNUV);
                theP.D2(U, V, out gp_Pnt P, out gp_Vec D1U, out gp_Vec D1V, out gp_Vec D2U, out gp_Vec D2V, out gp_Vec D2UV);
                string aAt = $"{theWhat} D2({U:R}, {V:R})";
                Same(aNP, P, aAt + " P");
                Same(aNU, D1U, aAt + " D1U");
                Same(aNV, D1V, aAt + " D1V");
                Same(aNUU, D2U, aAt + " D2U");
                Same(aNVV, D2V, aAt + " D2V");
                Same(aNUV, D2UV, aAt + " D2UV");
            }
        }
    }

    private static string R(double x) => x.ToString("R", CultureInfo.InvariantCulture);

    private static string Words(IEnumerable<double> theValues) => string.Join(" ", theValues.Select(R));

    private static DrawInterpreter New() => new(new StringWriter());

    private static object Geometry(DrawInterpreter theDI, string theName) => Assert.IsType<DrawGeometry>(theDI.GetDrawable(theName)).Geometry;

    // a random direction, its components in [-1, 1], not null
    private static double[] Dir(Bits theBits, int theDim)
    {
        while (true)
        {
            double[] aD = Enumerable.Range(0, theDim).Select(_ => theBits.Real(-1, 1)).ToArray();
            if (aD.Sum(x => x * x) > 0.01)
            {
                return aD;
            }
        }
    }

    // a direction orthogonal enough to D (for the X direction of an axis): D x (a random vector)
    private static double[] XDir(Bits theBits, double[] D)
    {
        while (true)
        {
            double[] v = Dir(theBits, 3);
            double[] x = [D[1] * v[2] - D[2] * v[1], D[2] * v[0] - D[0] * v[2], D[0] * v[1] - D[1] * v[0]];
            if (x.Sum(c => c * c) > 0.01)
            {
                return x;
            }
        }
    }

    private static double[] Pnt(Bits theBits, int theDim) => Enumerable.Range(0, theDim).Select(_ => theBits.Real(-10, 10)).ToArray();

    private static N.gp_Pnt NPnt(double[] p) => new(p[0], p[1], p[2]);

    private static N.gp_Dir NDir(double[] d) => new(d[0], d[1], d[2]);

    private static N.gp_Pnt2d NPnt2d(double[] p) => new(p[0], p[1]);

    private static N.gp_Dir2d NDir2d(double[] d) => new(d[0], d[1]);

    private void Report(Bits theBits, string theWhat) => output.WriteLine($"{theWhat}: {theBits.Values} values, all bit-identical");

    [Fact]
    public void Analytic_curves_match_native()
    {
        var aBits = new Bits(new Random(3601));
        DrawInterpreter i = New();
        string[] aKinds = ["line", "circle", "ellipse", "hyperbola", "parabola"];
        for (int c = 0; c < Cases; c++)
        {
            foreach (string aKind in aKinds)
            {
                double[] P = Pnt(aBits, 3), D = Dir(aBits, 3), X = XDir(aBits, D);
                double r1 = aBits.Real(3, 10), r2 = aBits.Real(0.5, 3);
                int aForm = aKind == "line" ? 1 : aBits.Int(0, 2); // 0: Z axis, 1: N, 2: N and X
                double[] anAxis = aForm switch { 0 => P, 1 => [.. P, .. D], _ => [.. P, .. D, .. X] };
                double[] aRadii = aKind is "ellipse" or "hyperbola" ? [r1, r2] : aKind == "line" ? [] : [r1];
                i.Eval($"{aKind} g {Words(anAxis)} {Words(aRadii)}");
                var aP = Assert.IsAssignableFrom<Geom_Curve>(Geometry(i, "g"));

                using N.gp_Pnt aNP = NPnt(P);
                using N.gp_Dir aND = aForm == 0 ? new N.gp_Dir(0, 0, 1) : NDir(D);
                using N.gp_Ax2 aNAx = aForm == 2 ? new N.gp_Ax2(aNP, aND, NDir(X)) : new N.gp_Ax2(aNP, aND);
                using N.Geom_Curve aN = aKind switch
                {
                    "line" => new N.Geom_Line(aNP, aND),
                    "circle" => new N.Geom_Circle(aNAx, r1),
                    "ellipse" => new N.Geom_Ellipse(aNAx, r1, r2),
                    "hyperbola" => new N.Geom_Hyperbola(aNAx, r1, r2),
                    _ => new N.Geom_Parabola(aNAx, r1),
                };
                aBits.SameCurve(aN, aP, $"{aKind} {c}");
            }

            // 2D: lines, circles and ellipses (the other 2D conics aren't bound natively)
            {
                double[] P = Pnt(aBits, 2), D = Dir(aBits, 2);
                double r1 = aBits.Real(3, 10), r2 = aBits.Real(0.5, 3);
                bool anX = aBits.Bool();
                using N.gp_Pnt2d aNP = NPnt2d(P);
                using N.gp_Dir2d aND = anX ? new N.gp_Dir2d(1, 0) : NDir2d(D);
                using var aNAx = new N.gp_Ax22d(aNP, aND, true);
                double[] anAxis = anX ? P : [.. P, .. D];

                i.Eval($"line g {Words(P)} {Words(D)}");
                using (var aN = new N.Geom2d_Line(aNP, NDir2d(D)))
                {
                    aBits.SameCurve(aN, Assert.IsAssignableFrom<Geom2d_Curve>(Geometry(i, "g")), $"2d line {c}");
                }

                i.Eval($"circle g {Words(anAxis)} {R(r1)}");
                using (var aN = new N.Geom2d_Circle(aNAx, r1))
                {
                    aBits.SameCurve(aN, Assert.IsAssignableFrom<Geom2d_Curve>(Geometry(i, "g")), $"2d circle {c}");
                }

                i.Eval($"ellipse g {Words(anAxis)} {R(r1)} {R(r2)}");
                using (var aN = new N.Geom2d_Ellipse(aNAx, r1, r2))
                {
                    aBits.SameCurve(aN, Assert.IsAssignableFrom<Geom2d_Curve>(Geometry(i, "g")), $"2d ellipse {c}");
                }
            }
        }

        Report(aBits, "line, circle, ellipse, hyperbola, parabola");
    }

    [Fact]
    public void Analytic_surfaces_match_native()
    {
        var aBits = new Bits(new Random(3602));
        DrawInterpreter i = New();
        string[] aKinds = ["plane", "cylinder", "cone", "sphere", "torus"];
        for (int c = 0; c < Cases; c++)
        {
            foreach (string aKind in aKinds)
            {
                double[] P = Pnt(aBits, 3), D = Dir(aBits, 3), X = XDir(aBits, D);
                int aForm = aBits.Int(0, 3); // 0: the default frame, 1: a location, 2: N, 3: N and X
                double[] anAxis = aForm switch { 0 => [], 1 => P, 2 => [.. P, .. D], _ => [.. P, .. D, .. X] };
                double r1 = aBits.Real(3, 10), r2 = aBits.Real(0.5, 3), anAngle = aBits.Real(-80, 80);
                double[] aParams = aKind switch
                {
                    "plane" => [],
                    "cylinder" or "sphere" => [r1],
                    "cone" => [anAngle, r1],
                    _ => [r1, r2],
                };


                i.Eval($"{aKind} g {Words(anAxis)} {Words(aParams)}".TrimEnd());
                var aP = Assert.IsAssignableFrom<Geom_Surface>(Geometry(i, "g"));

                using N.gp_Pnt aNP = aForm == 0 ? new N.gp_Pnt(0, 0, 0) : NPnt(P);
                using N.gp_Ax3 aNAx = aForm switch
                {
                    0 or 1 => new N.gp_Ax3(aNP, new N.gp_Dir(0, 0, 1), new N.gp_Dir(1, 0, 0)),
                    2 => new N.gp_Ax3(aNP, NDir(D)),
                    _ => new N.gp_Ax3(aNP, NDir(D), NDir(X)),
                };
                using N.Geom_Surface aN = aKind switch
                {
                    "plane" => new N.Geom_Plane(aNAx),
                    "cylinder" => new N.Geom_CylindricalSurface(aNAx, r1),
                    "sphere" => new N.Geom_SphericalSurface(aNAx, r1),
                    "cone" => new N.Geom_ConicalSurface(aNAx, anAngle * (Math.PI / 180.0), r1),
                    _ => new N.Geom_ToroidalSurface(aNAx, r1, r2),
                };
                aBits.SameSurface(aN, aP, $"{aKind} {c}");
            }
        }

        Report(aBits, "plane, cylinder, cone, sphere, torus");
    }

    // knots and multiplicities of a valid B-spline: nbk distinct increasing knots
    private static (double[] Knots, int[] Mults) Knots(Bits theBits, int theDegree, bool thePeriodic, int theNbKnots)
    {
        var aKnots = new double[theNbKnots];
        double k = theBits.Real(-2, 2);
        for (int j = 0; j < theNbKnots; j++)
        {
            aKnots[j] = k;
            k += theBits.Real(0.5, 2);
        }

        var aMults = new int[theNbKnots];
        for (int j = 0; j < theNbKnots; j++)
        {
            aMults[j] = theBits.Int(1, theDegree);
        }

        if (!thePeriodic)
        {
            aMults[0] = aMults[^1] = theDegree + 1;
        }
        else
        {
            aMults[^1] = aMults[0];
        }

        return (aKnots, aMults);
    }

    private static string KnotWords(double[] theKnots, int[] theMults)
        => string.Join(" ", theKnots.Zip(theMults, (k, m) => $"{R(k)} {m}"));

    private static N.TColStd_Array1OfReal NArray(double[] theValues)
    {
        var a = new N.TColStd_Array1OfReal(1, theValues.Length);
        for (int j = 0; j < theValues.Length; j++)
        {
            a.SetValue(j + 1, theValues[j]);
        }

        return a;
    }

    private static N.TColStd_Array1OfInteger NArray(int[] theValues)
    {
        var a = new N.TColStd_Array1OfInteger(1, theValues.Length);
        for (int j = 0; j < theValues.Length; j++)
        {
            a.SetValue(j + 1, theValues[j]);
        }

        return a;
    }

    [Fact]
    public void Pole_curves_and_surfaces_match_native()
    {
        var aBits = new Bits(new Random(3603));
        DrawInterpreter i = New();
        for (int c = 0; c < Cases; c++)
        {
            // beziercurve, with or without weights
            {
                int np = aBits.Int(2, 6);
                bool hasw = aBits.Bool();
                var aPoles = Enumerable.Range(0, np).Select(_ => Pnt(aBits, 3)).ToArray();
                var aWeights = Enumerable.Range(0, np).Select(_ => aBits.Real(0.5, 2)).ToArray();
                i.Eval($"beziercurve g {np} " + string.Join(" ", aPoles.Select((p, j) => Words(p) + (hasw ? " " + R(aWeights[j]) : ""))));
                using var aNPoles = new N.TColgp_Array1OfPnt(1, np);
                for (int j = 0; j < np; j++)
                {
                    aNPoles.SetValue(j + 1, NPnt(aPoles[j]));
                }

                using N.TColStd_Array1OfReal aNW = NArray(aWeights);
                using N.Geom_BezierCurve aN = hasw ? new N.Geom_BezierCurve(aNPoles, aNW) : new N.Geom_BezierCurve(aNPoles);
                aBits.SameCurve(aN, Assert.IsType<Geom_BezierCurve>(Geometry(i, "g")), $"beziercurve {c}");
            }

            // bsplinecurve, pbsplinecurve and their 2D forms
            foreach (bool aPeriodic in new[] { false, true })
            {
                int deg = aBits.Int(1, 4);
                (double[] K, int[] M) = Knots(aBits, deg, aPeriodic, aBits.Int(aPeriodic ? 3 : 2, 5));
                int np = aPeriodic ? M.Sum() - M[^1] : M.Sum() - deg - 1;
                if (np < 2 || (aPeriodic && np <= deg))
                {
                    continue;
                }

                var aPoles = Enumerable.Range(0, np).Select(_ => Pnt(aBits, 3)).ToArray();
                var aWeights = Enumerable.Range(0, np).Select(_ => aBits.Bool() ? 1.0 : aBits.Real(0.5, 2)).ToArray();
                string aPrefix = aPeriodic ? "p" : "";
                i.Eval($"{aPrefix}bsplinecurve g {deg} {K.Length} {KnotWords(K, M)} "
                       + string.Join(" ", aPoles.Select((p, j) => Words(p) + " " + R(aWeights[j]))));
                i.Eval($"2d{aPrefix}bsplinecurve g2 {deg} {K.Length} {KnotWords(K, M)} "
                       + string.Join(" ", aPoles.Select((p, j) => R(p[0]) + " " + R(p[1]) + " " + R(aWeights[j]))));
                using var aNPoles = new N.TColgp_Array1OfPnt(1, np);
                using var aNPoles2 = new N.TColgp_Array1OfPnt2d(1, np);
                for (int j = 0; j < np; j++)
                {
                    aNPoles.SetValue(j + 1, NPnt(aPoles[j]));
                    aNPoles2.SetValue(j + 1, new N.gp_Pnt2d(aPoles[j][0], aPoles[j][1]));
                }

                using N.TColStd_Array1OfReal aNW = NArray(aWeights), aNK = NArray(K);
                using N.TColStd_Array1OfInteger aNM = NArray(M);
                using (var aN = new N.Geom_BSplineCurve(aNPoles, aNW, aNK, aNM, deg, aPeriodic))
                {
                    aBits.SameCurve(aN, Assert.IsType<Geom_BSplineCurve>(Geometry(i, "g")), $"{aPrefix}bsplinecurve {c}");
                }

                using (var aN = new N.Geom2d_BSplineCurve(aNPoles2, aNW, aNK, aNM, deg, aPeriodic))
                {
                    aBits.SameCurve(aN, Assert.IsType<Geom2d_BSplineCurve>(Geometry(i, "g2")), $"2d{aPrefix}bsplinecurve {c}");
                }
            }

            // beziersurf
            {
                int nu = aBits.Int(2, 4), nv = aBits.Int(2, 4);
                bool hasw = aBits.Bool();
                var aWords = new List<string>();
                using var aNPoles = new N.TColgp_Array2OfPnt(1, nu, 1, nv);
                using var aNW = new N.TColStd_Array2OfReal(1, nu, 1, nv);
                for (int v = 1; v <= nv; v++)
                {
                    for (int u = 1; u <= nu; u++)
                    {
                        double[] p = Pnt(aBits, 3);
                        double w = aBits.Real(0.5, 2);
                        aNPoles.SetValue(u, v, NPnt(p));
                        aNW.SetValue(u, v, w);
                        aWords.Add(Words(p) + (hasw ? " " + R(w) : ""));
                    }
                }

                i.Eval($"beziersurf g {nu} {nv} " + string.Join(" ", aWords));
                using N.Geom_BezierSurface aN = hasw ? new N.Geom_BezierSurface(aNPoles, aNW) : new N.Geom_BezierSurface(aNPoles);
                aBits.SameSurface(aN, Assert.IsType<Geom_BezierSurface>(Geometry(i, "g")), $"beziersurf {c}");
            }

            // bsplinesurf and the periodic forms
            {
                bool uper = aBits.Bool(), vper = aBits.Bool();
                int udeg = aBits.Int(1, 3), vdeg = aBits.Int(1, 3);
                (double[] UK, int[] UM) = Knots(aBits, udeg, uper, aBits.Int(uper ? 3 : 2, 4));
                (double[] VK, int[] VM) = Knots(aBits, vdeg, vper, aBits.Int(vper ? 3 : 2, 4));
                int nu = uper ? UM.Sum() - UM[^1] : UM.Sum() - udeg - 1;
                int nv = vper ? VM.Sum() - VM[^1] : VM.Sum() - vdeg - 1;
                if (nu < 2 || nv < 2 || (uper && nu <= udeg) || (vper && nv <= vdeg))
                {
                    continue;
                }

                var aWords = new List<string>();
                using var aNPoles = new N.TColgp_Array2OfPnt(1, nu, 1, nv);
                using var aNW = new N.TColStd_Array2OfReal(1, nu, 1, nv);
                for (int v = 1; v <= nv; v++)
                {
                    for (int u = 1; u <= nu; u++)
                    {
                        double[] p = Pnt(aBits, 3);
                        double w = aBits.Real(0.5, 2);
                        aNPoles.SetValue(u, v, NPnt(p));
                        aNW.SetValue(u, v, w);
                        aWords.Add(Words(p) + " " + R(w));
                    }
                }

                string aName = (uper ? "u" : "") + (vper ? "v" : "") + (uper || vper ? "p" : "") + "bsplinesurf";
                i.Eval($"{aName} g {udeg} {UK.Length} {KnotWords(UK, UM)} {vdeg} {VK.Length} {KnotWords(VK, VM)} " + string.Join(" ", aWords));
                using N.TColStd_Array1OfReal aNUK = NArray(UK), aNVK = NArray(VK);
                using N.TColStd_Array1OfInteger aNUM = NArray(UM), aNVM = NArray(VM);
                using var aN = new N.Geom_BSplineSurface(aNPoles, aNW, aNUK, aNVK, aNUM, aNVM, udeg, vdeg, uper, vper);
                aBits.SameSurface(aN, Assert.IsType<Geom_BSplineSurface>(Geometry(i, "g")), $"{aName} {c}");
            }
        }

        Report(aBits, "beziercurve, (p)bsplinecurve, 2d(p)bsplinecurve, beziersurf, (u/v/uv)(p)bsplinesurf");
    }

    [Fact]
    public void Derived_geometry_matches_native()
    {
        var aBits = new Bits(new Random(3604));
        DrawInterpreter i = New();
        for (int c = 0; c < Cases; c++)
        {
            double[] P = Pnt(aBits, 3), D = Dir(aBits, 3), D2 = Dir(aBits, 3), A = Pnt(aBits, 3);
            double r = aBits.Real(1, 5);
            i.Eval($"circle c {Words(P)} {Words(D)} {R(r)}");
            using N.gp_Pnt aNP = NPnt(P);
            using N.gp_Dir aND = NDir(D);
            using var aNAx2 = new N.gp_Ax2(aNP, aND);
            using var aNC = new N.Geom_Circle(aNAx2, r);

            // extsurf and revsurf on the circle
            i.Eval($"extsurf e c {Words(D2)}");
            using (var aN = new N.Geom_SurfaceOfLinearExtrusion(aNC, NDir(D2)))
            {
                aBits.SameSurface(aN, Assert.IsAssignableFrom<Geom_Surface>(Geometry(i, "e")), $"extsurf {c}");
            }

            i.Eval($"revsurf rv c {Words(A)} {Words(D2)}");
            using var aNRev = new N.Geom_SurfaceOfRevolution(aNC, new N.gp_Ax1(NPnt(A), NDir(D2)));
            aBits.SameSurface(aNRev, Assert.IsAssignableFrom<Geom_Surface>(Geometry(i, "rv")), $"revsurf {c}");

            // offset curve and surface
            double d = aBits.Real(-0.5, 2);
            i.Eval($"offset oc c {R(d)} {Words(D)}");
            using (var aN = new N.Geom_OffsetCurve(aNC, d, aND))
            {
                aBits.SameCurve(aN, Assert.IsAssignableFrom<Geom_Curve>(Geometry(i, "oc")), $"offset curve {c}");
            }

            i.Eval($"sphere s {Words(P)} {Words(D)} {R(r)}; offset os s {R(d)}");
            using var aNAx3 = new N.gp_Ax3(aNP, aND);
            using var aNS = new N.Geom_SphericalSurface(aNAx3, r);
            using (var aN = new N.Geom_OffsetSurface(aNS, d))
            {
                aBits.SameSurface(aN, Assert.IsAssignableFrom<Geom_Surface>(Geometry(i, "os")), $"offset surface {c}");
            }

            // trims, with either sense
            double u1 = aBits.Real(-3, 3), u2 = u1 + aBits.Real(0.5, 3), v1 = aBits.Real(-1.5, 0), v2 = aBits.Real(0.1, 1.5);
            bool us = aBits.Bool(), vs = aBits.Bool();
            i.Eval($"trim tc c {R(u1)} {R(u2)} {(us ? 1 : 0)}");
            using (var aN = new N.Geom_TrimmedCurve(aNC, u1, u2, us))
            {
                aBits.SameCurve(aN, Assert.IsType<Geom_TrimmedCurve>(Geometry(i, "tc")), $"trim curve {c}");
            }

            i.Eval($"trim ts rv {R(u1)} {R(u2)} {R(v1)} {R(v2)} {(us ? 1 : 0)} {(vs ? 1 : 0)}");
            using (var aN = new N.Geom_RectangularTrimmedSurface(aNRev, u1, u2, v1, v2, us, vs))
            {
                aBits.SameSurface(aN, Assert.IsType<Geom_RectangularTrimmedSurface>(Geometry(i, "ts")), $"trim surface {c}");
            }

            i.Eval($"trimu tu s {R(u1)} {R(u2)} {(us ? 1 : 0)}; trimv tv s {R(v1)} {R(v2)} {(vs ? 1 : 0)}");
            using (var aN = new N.Geom_RectangularTrimmedSurface(aNS, u1, u2, true, us))
            {
                aBits.SameSurface(aN, Assert.IsType<Geom_RectangularTrimmedSurface>(Geometry(i, "tu")), $"trimu {c}");
            }

            using (var aN = new N.Geom_RectangularTrimmedSurface(aNS, v1, v2, false, vs))
            {
                aBits.SameSurface(aN, Assert.IsType<Geom_RectangularTrimmedSurface>(Geometry(i, "tv")), $"trimv {c}");
            }

            // 2D trim of a 2D circle
            double[] P2 = Pnt(aBits, 2);
            i.Eval($"circle c2 {Words(P2)} {R(r)}; trim t2 c2 {R(u1)} {R(u2)} {(us ? 1 : 0)}");
            using var aNC2 = new N.Geom2d_Circle(new N.gp_Ax22d(NPnt2d(P2), new N.gp_Dir2d(1, 0), true), r);
            using (var aN = new N.Geom2d_TrimmedCurve(aNC2, u1, u2, us))
            {
                aBits.SameCurve(aN, Assert.IsType<Geom2d_TrimmedCurve>(Geometry(i, "t2")), $"2d trim {c}");
            }

            // isos of the surface of revolution
            double par = aBits.Real(-1, 1);
            i.Eval($"uiso iu rv {R(par)}; viso iv rv {R(par)}");
            using (N.Geom_Curve aN = aNRev.UIso(par)!)
            {
                aBits.SameCurve(aN, Assert.IsAssignableFrom<Geom_Curve>(Geometry(i, "iu")), $"uiso {c}");
            }

            using (N.Geom_Curve aN = aNRev.VIso(par)!)
            {
                aBits.SameCurve(aN, Assert.IsAssignableFrom<Geom_Curve>(Geometry(i, "iv")), $"viso {c}");
            }

            // reverse, ureverse, vreverse: OCCT reverses in place, natively done on the native copies
            i.Eval("reverse c; ureverse s");
            aNC.Reverse();
            aNS.UReverse();
            aBits.SameCurve(aNC, Assert.IsAssignableFrom<Geom_Curve>(Geometry(i, "c")), $"reverse {c}");
            aBits.SameSurface(aNS, Assert.IsAssignableFrom<Geom_Surface>(Geometry(i, "s")), $"ureverse {c}");
            i.Eval("vreverse s");
            aNS.VReverse();
            aBits.SameSurface(aNS, Assert.IsAssignableFrom<Geom_Surface>(Geometry(i, "s")), $"vreverse {c}");

            // segment of a B-spline
            i.Eval("bsplinecurve b 2 3 0 3 1 1 2 3 0 0 0 1 1 1 0 1 2 0 1 2 3 1 1 1");
            double f = aBits.Real(0, 0.9), l = f + aBits.Real(0.2, 1);
            i.Eval($"segment b {R(f)} {R(l)}");
            using var aNPoles = new N.TColgp_Array1OfPnt(1, 4);
            aNPoles.SetValue(1, new N.gp_Pnt(0, 0, 0));
            aNPoles.SetValue(2, new N.gp_Pnt(1, 1, 0));
            aNPoles.SetValue(3, new N.gp_Pnt(2, 0, 1));
            aNPoles.SetValue(4, new N.gp_Pnt(3, 1, 1));
            using N.TColStd_Array1OfReal aNW = NArray([1.0, 1, 2, 1]), aNK = NArray([0.0, 1, 2]);
            using N.TColStd_Array1OfInteger aNM = NArray([3, 1, 3]);
            using var aNB = new N.Geom_BSplineCurve(aNPoles, aNW, aNK, aNM, 2, false);
            aNB.Segment(f, l, 1e-9);
            aBits.SameCurve(aNB, Assert.IsType<Geom_BSplineCurve>(Geometry(i, "b")), $"segment {c}");
        }

        Report(aBits, "extsurf, revsurf, offset, trim, trimu, trimv, uiso, viso, reverse, ureverse, vreverse, segment");
    }

    [Fact]
    public void Values_and_bounds_match_native()
    {
        var aBits = new Bits(new Random(3605));
        DrawInterpreter i = New();
        string[] aD2 = ["x", "y", "z", "dx", "dy", "dz", "d2x", "d2y", "d2z"];
        string[] aS2 = ["x", "y", "z", "dux", "duy", "duz", "dvx", "dvy", "dvz", "d2ux", "d2uy", "d2uz", "d2vx", "d2vy", "d2vz", "d2uvx", "d2uvy", "d2uvz"];
        double Num(string theName) => i.TryGetNumber(theName, out double x) ? x : throw new InvalidOperationException(theName);
        for (int c = 0; c < Cases; c++)
        {
            double[] P = Pnt(aBits, 3), D = Dir(aBits, 3);
            double r1 = aBits.Real(3, 10), r2 = aBits.Real(0.5, 3), U = aBits.Real(-7, 7), V = aBits.Real(-1.5, 1.5);
            i.Eval($"ellipse e {Words(P)} {Words(D)} {R(r1)} {R(r2)}; cvalue e {R(U)} {string.Join(" ", aD2)}");
            using var aNE = new N.Geom_Ellipse(new N.gp_Ax2(NPnt(P), NDir(D)), r1, r2);
            using var aNP = new N.gp_Pnt();
            using var aNV1 = new N.gp_Vec();
            using var aNV2 = new N.gp_Vec();
            aNE.D2(U, aNP, aNV1, aNV2);
            aBits.Same(aNP, new gp_Pnt(Num("x"), Num("y"), Num("z")), $"cvalue {c} P");
            aBits.Same(aNV1, new gp_Vec(Num("dx"), Num("dy"), Num("dz")), $"cvalue {c} D1");
            aBits.Same(aNV2, new gp_Vec(Num("d2x"), Num("d2y"), Num("d2z")), $"cvalue {c} D2");
            i.Eval($"cvalue e {R(U)} pt");
            aNE.D0(U, aNP);
            aBits.Same(aNP, Assert.IsType<gp_Pnt>(Geometry(i, "pt")), $"cvalue {c} point");

            double[] P2 = Pnt(aBits, 2);
            i.Eval($"ellipse e2 {Words(P2)} {R(r1)} {R(r2)}; 2dcvalue e2 {R(U)} x y dx dy d2x d2y");
            using var aNE2 = new N.Geom2d_Ellipse(new N.gp_Ax22d(NPnt2d(P2), new N.gp_Dir2d(1, 0), true), r1, r2);
            using var aNP2 = new N.gp_Pnt2d();
            using var aNW1 = new N.gp_Vec2d();
            using var aNW2 = new N.gp_Vec2d();
            aNE2.D2(U, aNP2, aNW1, aNW2);
            aBits.Same(aNP2, new gp_Pnt2d(Num("x"), Num("y")), $"2dcvalue {c} P");
            aBits.Same(aNW1, new gp_Vec2d(Num("dx"), Num("dy")), $"2dcvalue {c} D1");
            aBits.Same(aNW2, new gp_Vec2d(Num("d2x"), Num("d2y")), $"2dcvalue {c} D2");

            i.Eval($"torus t {Words(P)} {Words(D)} {R(r1)} {R(r2)}; svalue t {R(U)} {R(V)} {string.Join(" ", aS2)}");
            using var aNT = new N.Geom_ToroidalSurface(new N.gp_Ax3(NPnt(P), NDir(D)), r1, r2);
            using var aNU = new N.gp_Vec();
            using var aNV = new N.gp_Vec();
            using var aNUU = new N.gp_Vec();
            using var aNVV = new N.gp_Vec();
            using var aNUV = new N.gp_Vec();
            aNT.D2(U, V, aNP, aNU, aNV, aNUU, aNVV, aNUV);
            aBits.Same(aNP, new gp_Pnt(Num("x"), Num("y"), Num("z")), $"svalue {c} P");
            aBits.Same(aNU, new gp_Vec(Num("dux"), Num("duy"), Num("duz")), $"svalue {c} DU");
            aBits.Same(aNV, new gp_Vec(Num("dvx"), Num("dvy"), Num("dvz")), $"svalue {c} DV");
            aBits.Same(aNUU, new gp_Vec(Num("d2ux"), Num("d2uy"), Num("d2uz")), $"svalue {c} D2U");
            aBits.Same(aNVV, new gp_Vec(Num("d2vx"), Num("d2vy"), Num("d2vz")), $"svalue {c} D2V");
            aBits.Same(aNUV, new gp_Vec(Num("d2uvx"), Num("d2uvy"), Num("d2uvz")), $"svalue {c} D2UV");

            int nu = aBits.Int(0, 2), nv = aBits.Int(0, 2);
            if (nu + nv > 0)
            {
                i.Eval($"sderivative t {R(U)} {R(V)} {nu} {nv} dnx dny dnz");
                using N.gp_Vec aNDN = aNT.DN(U, V, nu, nv);
                aBits.Same(aNDN, new gp_Vec(Num("dnx"), Num("dny"), Num("dnz")), $"sderivative {c}");
            }

            double t1 = aBits.Real(-1, 0), t2 = aBits.Real(0.1, 1);
            i.Eval($"trim tt t {R(U)} {R(U + 1)} {R(t1)} {R(t2)}; bounds tt u1 u2 v1 v2; trim te e {R(U)} {R(U + 2)}; bounds te f l");
            using var aNTT = new N.Geom_RectangularTrimmedSurface(aNT, U, U + 1, t1, t2, true, true);
            double a1 = 0, a2 = 0, b1 = 0, b2 = 0;
            aNTT.Bounds(ref a1, ref a2, ref b1, ref b2);
            aBits.Same(a1, Num("u1"), $"bounds {c} U1");
            aBits.Same(a2, Num("u2"), $"bounds {c} U2");
            aBits.Same(b1, Num("v1"), $"bounds {c} V1");
            aBits.Same(b2, Num("v2"), $"bounds {c} V2");
            using var aNTE = new N.Geom_TrimmedCurve(aNE, U, U + 2, true);
            aBits.Same(aNTE.FirstParameter(), Num("f"), $"bounds {c} First");
            aBits.Same(aNTE.LastParameter(), Num("l"), $"bounds {c} Last");
        }

        Report(aBits, "cvalue, 2dcvalue, svalue, sderivative, bounds");
    }
}
