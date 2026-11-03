// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BSplCLib against native OCCT, through the native Geom_BSplineCurve, Geom2d_BSplineCurve and Geom_BezierCurve,
/// which call BSplCLib with the same inputs: degrees 1 to 9, periodic a third of the time, rational half the time.
/// </summary>
public class BSplCLibOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(600, 81);
    private static readonly int Cases2d = Tape.Cases(300, 41);
    private static readonly int CasesBezier = Tape.Cases(200, 27);

    /// <summary>A random B-spline curve: the knots, multiplicities and weights the port and the native curve share.</summary>
    private sealed class Curve
    {
        public readonly int Degree;
        public readonly bool Periodic;
        public readonly bool Rational;
        public readonly NCollection_Array1<double> Knots;
        public readonly NCollection_Array1<int> Mults;
        public readonly NCollection_Array1<double> FlatKnots;
        public readonly int NbPoles;
        public readonly NCollection_Array1<double> AllWeights;

        public Curve(Oracle theOracle)
        {
            Degree = theOracle.Int(1, 9);
            Periodic = theOracle.Int(0, 2) == 1;
            Rational = theOracle.Int(0, 1) == 1;
            theOracle.BSplineKnots(Degree, Periodic, out Knots, out Mults);
            NbPoles = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(Degree, Periodic, Mults);
            FlatKnots = new NCollection_Array1<double>(1, Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(Mults, Degree, Periodic));
            Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(Knots, Mults, Degree, Periodic, FlatKnots);
            AllWeights = theOracle.Weights(NbPoles, Rational);
        }

        /// <summary>The weights as BSplCLib takes them: null for a non-rational curve.</summary>
        public NCollection_Array1<double>? Weights => Rational ? AllWeights : null;

        public int NbKnots => Knots.Length();

        public double First => Knots[1];

        public double Last => Knots[Knots.Upper()];

        public NCollection_Array1<Point3D> Poles(Oracle theOracle)
        {
            var aPoles = new NCollection_Array1<Point3D>(1, NbPoles);
            for (int i = 1; i <= NbPoles; i++)
            {
                aPoles[i] = new Point3D(theOracle.Real(0, 10), theOracle.Real(0, 10), theOracle.Real(0, 10));
            }

            return aPoles;
        }

        public NCollection_Array1<Point2D> Poles2d(Oracle theOracle)
        {
            var aPoles = new NCollection_Array1<Point2D>(1, NbPoles);
            for (int i = 1; i <= NbPoles; i++)
            {
                aPoles[i] = new Point2D(theOracle.Real(0, 10), theOracle.Real(0, 10));
            }

            return aPoles;
        }

        public N.Geom_BSplineCurve ToNative(NCollection_Array1<Point3D> thePoles)
            => Oracle.ToNativeCurve(thePoles, Weights, Knots, Mults, Degree, Periodic);

        public N.Geom2d_BSplineCurve ToNative(NCollection_Array1<Point2D> thePoles)
            => Oracle.ToNativeCurve(thePoles, Weights, Knots, Mults, Degree, Periodic);
    }

    [Fact]
    public void Curve_evaluation()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var aCurve = new Curve(anOracle);
            NCollection_Array1<Point3D> aPoles = aCurve.Poles(anOracle);
            using N.Geom_BSplineCurve aNative = aCurve.ToNative(aPoles);
            for (int j = 0; j <= 30; j++)
            {
                // Some parameters of periodic curves outside the period
                double aU = aCurve.First + (aCurve.Last - aCurve.First) * j / 30
                            + (aCurve.Periodic && j % 7 == 3 ? (aCurve.Last - aCurve.First) * 1.3 : 0.0);
                Kerf.Engine.Geometry.Splines.BSplCLib.D0(aU, 0, aCurve.Degree, aCurve.Periodic, aPoles, aCurve.Weights, aCurve.Knots, aCurve.Mults, out Point3D aP);
                using (N.gp_Pnt aNP = aNative.EvalD0(aU))
                {
                    anOracle.Same(aNP, aP, "D0");
                }

                for (int n = 1; n <= 4; n++)
                {
                    Kerf.Engine.Geometry.Splines.BSplCLib.DN(aU, n, 0, aCurve.Degree, aCurve.Periodic, aPoles, aCurve.Weights, aCurve.FlatKnots,
                                     Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out Vector3D aV);
                    using N.gp_Vec aNV = aNative.EvalDN(aU, n);
                    anOracle.Same(aNV, aV, $"DN{n}");
                }

                // LocalD3 on one knot span
                int aK1 = anOracle.Int(1, aCurve.NbKnots - 1), aK2 = aK1 + 1;
                using var aNP3 = new N.gp_Pnt(0, 0, 0);
                using var aNV1 = new N.gp_Vec(0, 0, 0);
                using var aNV2 = new N.gp_Vec(0, 0, 0);
                using var aNV3 = new N.gp_Vec(0, 0, 0);
                aNative.LocalD3(aU, aK1, aK2, aNP3, aNV1, aNV2, aNV3);
                Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aCurve.Degree, aCurve.FlatKnots, aU, aCurve.Periodic, aK1, aK2, out int anIndex, out double aUU);
                anIndex = Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(aCurve.Degree, anIndex, aCurve.Mults, aCurve.Periodic);
                Kerf.Engine.Geometry.Splines.BSplCLib.D3(aUU, anIndex, aCurve.Degree, aCurve.Periodic, aPoles, aCurve.Weights, aCurve.FlatKnots,
                                 Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out Point3D aP3, out Vector3D aV1, out Vector3D aV2, out Vector3D aV3);
                anOracle.Same(aNP3, aP3, "LocalD3 P");
                anOracle.Same(aNV1, aV1, "LocalD3 V1");
                anOracle.Same(aNV2, aV2, "LocalD3 V2");
                anOracle.Same(aNV3, aV3, "LocalD3 V3");
            }
        }

        anOracle.Report("BSplCLib D0, DN, LocalD3 (3D)");
    }

    [Fact]
    public void Curve_resolution()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var aCurve = new Curve(anOracle);
            NCollection_Array1<Point3D> aPoles = aCurve.Poles(anOracle);
            using N.Geom_BSplineCurve aNative = aCurve.ToNative(aPoles);
            double aNativeTol = 0.0;
            aNative.Resolution(1.0, ref aNativeTol);

            double aPortTol;
            if (aCurve.Periodic)
            {
                // As Geom_BSplineCurve::Resolution: the poles (and weights) repeated over the unperiodized knots
                Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(aCurve.Degree, aCurve.Mults, out _, out int aNbPoles);
                var aPoles2 = new NCollection_Array1<Point3D>(1, aNbPoles);
                var aWeights2 = new NCollection_Array1<double>(1, aNbPoles);
                for (int k = 1; k <= aNbPoles; k++)
                {
                    aPoles2[k] = aPoles[(k - 1) % aCurve.NbPoles + 1];
                    aWeights2[k] = aCurve.AllWeights[(k - 1) % aCurve.NbPoles + 1];
                }

                Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(aPoles2, aCurve.Rational ? aWeights2 : null, aNbPoles, aCurve.FlatKnots,
                                         aCurve.Degree, 1.0, out aPortTol);
            }
            else
            {
                Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(aPoles, aCurve.Weights, aCurve.NbPoles, aCurve.FlatKnots, aCurve.Degree, 1.0, out aPortTol);
            }

            anOracle.SameRelative(aNativeTol, aPortTol, "Resolution");
        }

        anOracle.Report("BSplCLib Resolution");
    }

    [Fact]
    public void Curve_knot_insertion_and_removal()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            var aCurve = new Curve(anOracle);
            NCollection_Array1<Point3D> aPoles = aCurve.Poles(anOracle);

            // InsertKnots: 1 to 3 knots, strictly increasing, sometimes on an existing knot. Repeated knots are left
            // out: with Add = false, OCCT's PrepareInsertKnots counts a repeated knot once, with its first
            // multiplicity, where InsertKnots takes each in turn (occt-issues.md).
            int aNbAdd = anOracle.Int(1, 3);
            var anAddKnots = new NCollection_Array1<double>(1, aNbAdd);
            var anAddMults = new NCollection_Array1<int>(1, aNbAdd);
            double anAdd = aCurve.First;
            for (int k = 1; k <= aNbAdd; k++)
            {
                anAdd += (aCurve.Last - anAdd) * anOracle.Real(0, 0.6);
                if (anOracle.Int(0, 3) == 0)
                {
                    anAdd = Math.Max(anAdd, aCurve.Knots[anOracle.Int(1, aCurve.NbKnots)]);
                }

                anAddKnots[k] = anAdd;
                anAddMults[k] = anOracle.Int(1, aCurve.Degree);
            }

            bool isIncreasing = true;
            for (int k = 2; k <= aNbAdd; k++)
            {
                isIncreasing &= anAddKnots[k] > anAddKnots[k - 1];
            }

            // Geom_BSplineCurve::InsertKnots returns early when no pole is added
            if (isIncreasing
                && Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(aCurve.Degree, aCurve.Periodic, aCurve.Knots, aCurve.Mults, anAddKnots,
                                                 anAddMults, out int aNbPoles, out int aNbKnots, 0.0, false)
                && aNbPoles != aCurve.NbPoles)
            {
                var aNewPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
                var aNewWeights = aCurve.Rational ? new NCollection_Array1<double>(1, aNbPoles) : null;
                var aNewKnots = new NCollection_Array1<double>(1, aNbKnots);
                var aNewMults = new NCollection_Array1<int>(1, aNbKnots);
                Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(aCurve.Degree, aCurve.Periodic, aPoles, aCurve.Weights, aCurve.Knots, aCurve.Mults,
                                          anAddKnots, anAddMults, aNewPoles, aNewWeights, aNewKnots, aNewMults, 0.0, false);

                using N.Geom_BSplineCurve aNative = aCurve.ToNative(aPoles);
                using N.TColStd_Array1OfReal aNKnots = Oracle.ToNative(anAddKnots);
                using N.TColStd_Array1OfInteger aNMults = Oracle.ToNative(anAddMults);
                aNative.InsertKnots(aNKnots, aNMults, 0.0);
                anOracle.Same(aNative.NbPoles(), aNbPoles, "InsertKnots NbPoles");
                for (int k = 1; k <= aNbPoles; k++)
                {
                    using N.gp_Pnt aNP = aNative.Pole(k);
                    anOracle.Same(aNP, aNewPoles[k], "InsertKnots pole");
                }
            }

            // RemoveKnot of an interior knot, with a tolerance that fails or one that always succeeds
            if (aCurve.NbKnots > 2 && !aCurve.Periodic)
            {
                int anIndex = anOracle.Int(2, aCurve.NbKnots - 1);
                int aMult = anOracle.Int(0, aCurve.Mults[anIndex] - 1);
                int aStep = aCurve.Mults[anIndex] - aMult;
                int aNbNewKnots = aCurve.NbKnots - (aMult == 0 ? 1 : 0);
                var aNewPoles = new NCollection_Array1<Point3D>(1, aCurve.NbPoles - aStep);
                var aNewWeights = aCurve.Rational ? new NCollection_Array1<double>(1, aCurve.NbPoles - aStep) : null;
                var aNewKnots = new NCollection_Array1<double>(1, aNbNewKnots);
                var aNewMults = new NCollection_Array1<int>(1, aNbNewKnots);
                double aTolerance = anOracle.Int(0, 1) == 0 ? 1e-7 : 1e3;
                bool isRemoved = Kerf.Engine.Geometry.Splines.BSplCLib.RemoveKnot(anIndex, aMult, aCurve.Degree, aCurve.Periodic, aPoles, aCurve.Weights,
                                                          aCurve.Knots, aCurve.Mults, aNewPoles, aNewWeights, aNewKnots,
                                                          aNewMults, aTolerance);

                using N.Geom_BSplineCurve aNative = aCurve.ToNative(aPoles);
                bool isNativeRemoved = aNative.RemoveKnot(anIndex, aMult, aTolerance);
                anOracle.Same(isNativeRemoved, isRemoved, "RemoveKnot result");
                if (isRemoved && isNativeRemoved)
                {
                    for (int k = 1; k <= aNewPoles.Length(); k++)
                    {
                        using N.gp_Pnt aNP = aNative.Pole(k);
                        anOracle.Same(aNP, aNewPoles[k], "RemoveKnot pole");
                    }
                }
            }
        }

        anOracle.Report("BSplCLib InsertKnots, RemoveKnot");
    }

    [Fact]
    public void Curve_unperiodize()
    {
        var anOracle = new Oracle(output);
        int aNbPeriodic = 0;
        for (int i = 0; aNbPeriodic < Cases / 3; i++)
        {
            var aCurve = new Curve(anOracle);
            NCollection_Array1<Point3D> aPoles = aCurve.Poles(anOracle);
            if (!aCurve.Periodic)
            {
                continue;
            }

            aNbPeriodic++;
            Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(aCurve.Degree, aCurve.Mults, out int aNbKnots, out int aNbPoles);
            var aNewPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
            var aNewWeights = aCurve.Rational ? new NCollection_Array1<double>(1, aNbPoles) : null;
            var aNewKnots = new NCollection_Array1<double>(1, aNbKnots);
            var aNewMults = new NCollection_Array1<int>(1, aNbKnots);
            Kerf.Engine.Geometry.Splines.BSplCLib.Unperiodize(aCurve.Degree, aCurve.Mults, aCurve.Knots, aPoles, aCurve.Weights, aNewMults, aNewKnots,
                                      aNewPoles, aNewWeights);

            using N.Geom_BSplineCurve aNative = aCurve.ToNative(aPoles);
            aNative.SetNotPeriodic();
            anOracle.Same(aNative.NbPoles(), aNbPoles, "Unperiodize NbPoles");
            for (int k = 1; k <= aNbPoles; k++)
            {
                using N.gp_Pnt aNP = aNative.Pole(k);
                anOracle.Same(aNP, aNewPoles[k], "Unperiodize pole");
            }
        }

        anOracle.Report("BSplCLib Unperiodize");
    }

    [Fact]
    public void Curve_move_point()
    {
        var anOracle = new Oracle(output);
        var aTangentOracle = new Oracle(output); // comparisons only, reported separately
        int aNbThrows = 0, aNbSingular = 0;
        for (int i = 0; i < Cases; i++)
        {
            var aCurve = new Curve(anOracle);
            NCollection_Array1<Point3D> aPoles = aCurve.Poles(anOracle);
            using N.Geom_BSplineCurve aCurveNative = aCurve.ToNative(aPoles);

            // MovePoint: OCCT throws for some periodic curves; the port must throw for the same ones
            {
                double aU = aCurve.First + (aCurve.Last - aCurve.First) * anOracle.Real(0, 1);
                using N.gp_Pnt aNP0 = aCurveNative.EvalD0(aU);
                using var aNTarget = new N.gp_Pnt(aNP0.X() + 1, aNP0.Y() - 0.5, aNP0.Z() + 0.25);
                using N.Geom_BSplineCurve aNative = aCurve.ToNative(aPoles);
                int aNFirst = 0, aNLast = 0;
                bool isNativeThrown = false, isPortThrown = false;
                try
                {
                    aNative.MovePoint(aU, aNTarget, 1, aCurve.NbPoles, ref aNFirst, ref aNLast);
                }
                catch (Exception)
                {
                    isNativeThrown = true;
                }

                var aNewPoles = new NCollection_Array1<Point3D>(1, aCurve.NbPoles);
                int aFirst = 0, aLast = 0;
                try
                {
                    var aDisplacement = new Vector3D(Oracle.FromNative(aNP0), Oracle.FromNative(aNTarget));
                    Kerf.Engine.Geometry.Splines.BSplCLib.MovePoint(aU, aDisplacement, 1, aCurve.NbPoles, aCurve.Degree, aPoles, aCurve.Weights,
                                            aCurve.FlatKnots, out aFirst, out aLast, aNewPoles);
                }
                catch (Exception)
                {
                    isPortThrown = true;
                }

                anOracle.Same(isNativeThrown, isPortThrown, "MovePoint throws");
                aNbThrows += isNativeThrown ? 1 : 0;
                if (!isNativeThrown && !isPortThrown)
                {
                    anOracle.Same(aNFirst, aFirst, "MovePoint first index");
                    anOracle.Same(aNLast, aLast, "MovePoint last index");
                    if (aFirst != 0)
                    {
                        for (int k = 1; k <= aCurve.NbPoles; k++)
                        {
                            using N.gp_Pnt aNP = aNative.Pole(k);
                            anOracle.Same(aNP, aNewPoles[k], "MovePoint pole");
                        }
                    }
                }
            }

            // MovePointAndTangent (non-periodic)
            if (!aCurve.Periodic)
            {
                double aU = aCurve.First + (aCurve.Last - aCurve.First) * anOracle.Real(0.1, 0.9);
                // The value and derivative as Geom_BSplineCurve::MovePointAndTangent computes them (Geom_Curve::D1)
                using var aNP0 = new N.gp_Pnt(0, 0, 0);
                using var aND1 = new N.gp_Vec(0, 0, 0);
                aCurveNative.D1(aU, aNP0, aND1);
                using var aNTarget = new N.gp_Pnt(aNP0.X() + 0.3, aNP0.Y(), aNP0.Z() - 0.2);
                using var aNTangent = new N.gp_Vec(aND1.X() * 1.1, aND1.Y() + 0.5, aND1.Z());
                using N.Geom_BSplineCurve aNative = aCurve.ToNative(aPoles);
                int aNError = 0;
                aNative.MovePointAndTangent(aU, aNTarget, aNTangent, 1e-9, 0, 0, ref aNError);

                var aNewPoles = new NCollection_Array1<Point3D>(1, aCurve.NbPoles);
                var aDelta = new Vector3D(aNTarget.X() - aNP0.X(), aNTarget.Y() - aNP0.Y(), aNTarget.Z() - aNP0.Z());
                var aDeltaD = new Vector3D(aNTangent.X() - aND1.X(), aNTangent.Y() - aND1.Y(), aNTangent.Z() - aND1.Z());
                int anError;
                try
                {
                    Kerf.Engine.Geometry.Splines.BSplCLib.MovePointAndTangent(aU, aDelta, aDeltaD, 1e-9, aCurve.Degree, 0, 0, aPoles, aCurve.Weights,
                                                      aCurve.FlatKnots, aNewPoles, out anError);
                }
                catch (ConstructionErrorException)
                {
                    // gp_Mat2d::Invert found the 2 x 2 system singular: the port always checks, OCCT's release build
                    // (No_Exception) doesn't and goes on with an infinite or meaningless inverse. Not comparable.
                    aNbSingular++;
                    continue;
                }

                aTangentOracle.Same(aNError, anError, "MovePointAndTangent status");
                if (aNError == 0 && anError == 0)
                {
                    for (int k = 1; k <= aCurve.NbPoles; k++)
                    {
                        using N.gp_Pnt aNP = aNative.Pole(k);
                        aTangentOracle.Same(aNP, aNewPoles[k], "MovePointAndTangent pole");
                    }
                }
            }
        }

        output.WriteLine($"MovePoint: OCCT threw for {aNbThrows} curves; MovePointAndTangent: {aNbSingular} singular systems skipped");
        anOracle.Report("BSplCLib MovePoint");
        aTangentOracle.Report("BSplCLib MovePointAndTangent");
    }

    [Fact]
    public void Curve2d_evaluation_and_degree_elevation()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases2d; i++)
        {
            var aCurve = new Curve(anOracle);
            NCollection_Array1<Point2D> aPoles = aCurve.Poles2d(anOracle);
            using N.Geom2d_BSplineCurve aNative = aCurve.ToNative(aPoles);
            for (int j = 0; j <= 20; j++)
            {
                double aU = aCurve.First + (aCurve.Last - aCurve.First) * j / 20;
                Kerf.Engine.Geometry.Splines.BSplCLib.D0(aU, 0, aCurve.Degree, aCurve.Periodic, aPoles, aCurve.Weights, aCurve.Knots, aCurve.Mults, out Point2D aP);
                using (N.gp_Pnt2d aNP = aNative.EvalD0(aU))
                {
                    anOracle.Same(aNP, aP, "2D D0");
                }

                for (int n = 1; n <= 3; n++)
                {
                    Kerf.Engine.Geometry.Splines.BSplCLib.DN(aU, n, 0, aCurve.Degree, aCurve.Periodic, aPoles, aCurve.Weights, aCurve.FlatKnots,
                                     Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out Vector2D aV);
                    using N.gp_Vec2d aNV = aNative.EvalDN(aU, n);
                    anOracle.Same(aNV, aV, $"2D DN{n}");
                }

                int aK1 = anOracle.Int(1, aCurve.NbKnots - 1), aK2 = aK1 + 1;
                using var aNP3 = new N.gp_Pnt2d(0, 0);
                using var aNV1 = new N.gp_Vec2d(0, 0);
                using var aNV2 = new N.gp_Vec2d(0, 0);
                using var aNV3 = new N.gp_Vec2d(0, 0);
                aNative.LocalD3(aU, aK1, aK2, aNP3, aNV1, aNV2, aNV3);
                Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aCurve.Degree, aCurve.FlatKnots, aU, aCurve.Periodic, aK1, aK2, out int anIndex, out double aUU);
                anIndex = Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(aCurve.Degree, anIndex, aCurve.Mults, aCurve.Periodic);
                Kerf.Engine.Geometry.Splines.BSplCLib.D3(aUU, anIndex, aCurve.Degree, aCurve.Periodic, aPoles, aCurve.Weights, aCurve.FlatKnots,
                                 Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out Point2D aP3, out Vector2D aV1, out Vector2D aV2, out Vector2D aV3);
                anOracle.Same(aNP3, aP3, "2D LocalD3 P");
                anOracle.Same(aNV1, aV1, "2D LocalD3 V1");
                anOracle.Same(aNV2, aV2, "2D LocalD3 V2");
                anOracle.Same(aNV3, aV3, "2D LocalD3 V3");
            }

            // IncreaseDegree, non-periodic only: OCCT's periodic IncreaseDegree is broken (occt-issues.md)
            if (!aCurve.Periodic)
            {
                int aNewDegree = aCurve.Degree + anOracle.Int(1, 3);
                int aNbKnots = Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegreeCountKnots(aCurve.Degree, aNewDegree, false, aCurve.Mults);
                int aFirstK = Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(aCurve.Degree, aCurve.Mults);
                int aLastK = Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(aCurve.Degree, aCurve.Mults);
                int aNbPoles = aCurve.NbPoles + (aNewDegree - aCurve.Degree) * (aLastK - aFirstK);
                var aNewPoles = new NCollection_Array1<Point2D>(1, aNbPoles);
                var aNewKnots = new NCollection_Array1<double>(1, aNbKnots);
                var aNewMults = new NCollection_Array1<int>(1, aNbKnots);
                var aNewWeights = aCurve.Rational ? new NCollection_Array1<double>(1, aNbPoles) : null;
                Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(aCurve.Degree, aNewDegree, false, aPoles, aCurve.Weights, aCurve.Knots, aCurve.Mults,
                                             aNewPoles, aNewWeights, aNewKnots, aNewMults);
                using N.Geom2d_BSplineCurve aRaised = aCurve.ToNative(aPoles);
                aRaised.IncreaseDegree(aNewDegree);
                anOracle.Same(aRaised.NbPoles(), aNbPoles, "2D IncreaseDegree NbPoles");
                for (int k = 1; k <= aNbPoles; k++)
                {
                    using N.gp_Pnt2d aNP = aRaised.Pole(k);
                    anOracle.Same(aNP, aNewPoles[k], "2D IncreaseDegree pole");
                }
            }
        }

        anOracle.Report("BSplCLib 2D D0, DN, LocalD3, IncreaseDegree");
    }

    [Fact]
    public void Bezier_evaluation()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < CasesBezier; i++)
        {
            int aDegree = anOracle.Int(1, 25);
            bool isRational = anOracle.Int(0, 1) == 1;
            var aPoles = new NCollection_Array1<Point3D>(1, aDegree + 1);
            for (int k = 1; k <= aDegree + 1; k++)
            {
                aPoles[k] = new Point3D(anOracle.Real(0, 1), anOracle.Real(0, 1), anOracle.Real(0, 1));
            }

            NCollection_Array1<double> aWeights = anOracle.Weights(aDegree + 1, isRational);
            using N.TColgp_Array1OfPnt aNPoles = Oracle.ToNative(aPoles);
            using N.TColStd_Array1OfReal aNWeights = Oracle.ToNative(aWeights);
            using N.Geom_BezierCurve aNative = isRational ? new N.Geom_BezierCurve(aNPoles, aNWeights) : new N.Geom_BezierCurve(aNPoles);
            for (int j = 0; j <= 10; j++)
            {
                double aU = j / 10.0;
                Kerf.Engine.Geometry.Splines.BSplCLib.D0(aU, aPoles, isRational ? aWeights : null, out Point3D aP);
                Kerf.Engine.Geometry.Splines.BSplCLib.D3(aU, aPoles, isRational ? aWeights : null, out _, out Vector3D aV1, out Vector3D aV2, out Vector3D aV3);
                using N.gp_Pnt aNP = aNative.EvalD0(aU);
                using N.gp_Vec aNV1 = aNative.EvalDN(aU, 1);
                using N.gp_Vec aNV2 = aNative.EvalDN(aU, 2);
                using N.gp_Vec aNV3 = aNative.EvalDN(aU, 3);
                anOracle.Same(aNP, aP, "Bezier D0");
                anOracle.Same(aNV1, aV1, "Bezier D1");
                anOracle.Same(aNV2, aV2, "Bezier D2");
                anOracle.Same(aNV3, aV3, "Bezier D3");
            }
        }

        anOracle.Report("BSplCLib Bezier D0 to D3");
    }
}
