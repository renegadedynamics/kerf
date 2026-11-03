// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Fairing;
using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Fairing.AnalysisCode;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the FairCurve package (OCCT has no FairCurve GTests): the battens' and minimal variation curves'
/// constraints, argument checks, homotopy, analysis codes and dumps; the battens' curves frozen; the energies'
/// gradients and hessians against finite differences. FairCurve_NativeTests compares everything with native OCCT.
/// </summary>
public class FairCurveTests
{
    private static double TangentAngle(BSplineCurve2D theC, double theU)
    {
        theC.D1(theU, out _, out Vector2D aV);
        return Math.Atan2(aV.Y, aV.X);
    }

    private static double Curvature(BSplineCurve2D theC, double theU)
    {
        theC.D2(theU, out _, out Vector2D aV1, out Vector2D aV2);
        return (aV1 ^ aV2) / Math.Pow(aV1.Magnitude, 3);
    }

    [Fact]
    public void Batten_starts_as_a_straight_line_of_degree_9()
    {
        Batten b = new(new Point2D(1, 2), new Point2D(10, 2), 0.5);
        BSplineCurve2D c = b.Curve();
        Assert.Equal(9, c.Degree);
        Assert.Equal(10, c.PoleCount);
        Assert.Equal(2, c.KnotCount);
        for (int i = 1; i <= 10; i++)
        {
            Assert.Equal(1 + (i - 1), c.Pole(i).X, 1e-14);
            Assert.Equal(2.0, c.Pole(i).Y);
        }
        Assert.Equal(1, b.GetConstraintOrder1());
        Assert.Equal(1, b.GetConstraintOrder2());
        Assert.False(b.GetFreeSliding());
        Assert.Equal(1.0, b.GetSlidingFactor());
        Assert.Equal(0.5, b.GetHeight());
        Assert.Equal(0.0, b.GetSlope());
        // no angle: the reference length is the distance
        Assert.Equal(9.0, b.SlidingOfReference());
    }

    [Fact]
    public void Batten_checks_its_arguments()
    {
        Point2D p = new(1, 1);
        Assert.Throws<NullValueException>(() => new Batten(p, new Point2D(1, 1 + 1e-8), 1));
        Assert.Throws<NegativeValueException>(() => new Batten(p, new Point2D(2, 1), 0));
        Assert.Throws<NegativeValueException>(() => new MinimalVariation(p, new Point2D(2, 1), -1));
        MinimalVariation b = new(p, new Point2D(2, 1), 1);
        Assert.Throws<NegativeValueException>(() => b.SetHeight(0));
        Assert.Throws<NullValueException>(() => b.SetP1(new Point2D(2, 1)));
        Assert.Throws<NullValueException>(() => b.SetP2(p));
        Assert.Throws<DomainErrorException>(() => b.SetPhysicalRatio(1.5));
        Assert.Throws<DomainErrorException>(() => b.SetPhysicalRatio(-0.1));

        // a ratio out of range given to the constructor is caught by the energy, in Compute
        MinimalVariation m = new(p, new Point2D(2, 1), 1, 0, 2);
        Assert.Throws<DomainErrorException>(() => m.Compute(out _));
    }

    [Fact]
    public void Batten_meets_its_points_and_angles()
    {
        // OCCT's battencurve and minvarcurve take angles from the chord P1P2, at P1 counterclockwise, at P2 clockwise
        foreach ((double a1, double a2) in new[] { (30.0, 30.0), (20.0, -15.0), (-45.0, 10.0), (60.0, 60.0) })
        {
            Point2D P1 = new(0, 0), P2 = new(10, 5);
            double aChord = Math.Atan2(5, 10);
            Batten b = new(P1, P2, 0.3);
            b.SetAngle1(a1 * M_PI / 180);
            b.SetAngle2(a2 * M_PI / 180);
            Assert.True(b.Compute(out AnalysisCode aCode));
            Assert.Equal(AnalysisCode.OK, aCode);
            BSplineCurve2D c = b.Curve();
            Assert.True(c.StartPoint.IsEqual(P1, 1e-12));
            Assert.True(c.EndPoint.IsEqual(P2, 1e-12));
            Assert.Equal(aChord + a1 * M_PI / 180, TangentAngle(c, c.FirstParameter), 1e-12);
            Assert.Equal(aChord - a2 * M_PI / 180, TangentAngle(c, c.LastParameter), 1e-12);
            Assert.Equal(a1 * M_PI / 180, b.GetAngle1(), 1e-12);
            Assert.Equal(a2 * M_PI / 180, b.GetAngle2(), 1e-12);
        }
    }

    [Fact]
    public void A_symmetric_batten_is_symmetric()
    {
        Batten b = new(new Point2D(-5, 0), new Point2D(5, 0), 0.2);
        b.SetAngle1(0.5);
        b.SetAngle2(0.5);
        Assert.True(b.Compute(out AnalysisCode aCode));
        Assert.Equal(AnalysisCode.OK, aCode);
        BSplineCurve2D c = b.Curve();
        int n = c.PoleCount;
        for (int i = 1; i <= n; i++)
        {
            Assert.Equal(-c.Pole(n + 1 - i).X, c.Pole(i).X, 1e-6);
            Assert.Equal(c.Pole(n + 1 - i).Y, c.Pole(i).Y, 1e-6);
        }
        Assert.True(c.Value(0.5).Y > 0.5);
    }

    [Fact]
    public void Free_angles_and_free_sliding()
    {
        Batten b = new(new Point2D(0, 0), new Point2D(10, 0), 0.5);
        b.SetAngle1(0.6);
        b.SetConstraintOrder2(0);
        b.SetFreeSliding(true);
        Assert.True(b.Compute(out AnalysisCode aCode));
        Assert.Equal(AnalysisCode.OK, aCode);
        Assert.Equal(0, b.GetConstraintOrder2());
        Assert.True(b.GetFreeSliding());
        // the free sliding is computed, and the free angle is the curve's
        Assert.NotEqual(1.0, b.GetSlidingFactor());
        BSplineCurve2D c = b.Curve();
        Assert.Equal(-b.GetAngle2(), TangentAngle(c, c.LastParameter), 1e-12);
    }

    [Fact]
    public void The_sliding_factor_sets_the_length()
    {
        // OCCT's documentation: b.SetSlidingFactor(L / b.SlidingOfReference()) gives the batten a length L
        Batten b = new(new Point2D(0, 0), new Point2D(10, 0), 0.5);
        b.SetAngle1(0.3);
        b.SetAngle2(0.3);
        double aRef = b.SlidingOfReference();
        // a circle arc of the same angles is longer than the chord
        Assert.True(aRef > 10.0);
        b.SetSlidingFactor(12.0 / aRef);
        b.Compute(out _);
        Assert.Equal(12.0 / aRef, b.GetSlidingFactor());
        // a longer batten bulges more
        Batten b2 = new(new Point2D(0, 0), new Point2D(10, 0), 0.5);
        b2.SetAngle1(0.3);
        b2.SetAngle2(0.3);
        b2.Compute(out _);
        Assert.True(b.Curve().Value(0.5).Y > b2.Curve().Value(0.5).Y);
    }

    [Fact]
    public void A_batten_without_matter_at_an_end_reports_NullHeight()
    {
        Batten b = new(new Point2D(0, 0), new Point2D(10, 0), 0.1, 0.5);
        b.SetAngle1(0.3);
        b.SetAngle2(0.3);
        Assert.True(b.Compute(out AnalysisCode aCode));
        Assert.Equal(AnalysisCode.NullHeight, aCode);
        Assert.Contains("AnalysisCode : NullHeight\n", b.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void Large_angles_need_more_knots()
    {
        Batten b = new(new Point2D(0, 0), new Point2D(10, 0), 0.5);
        // |a1| + |a2| + 2 |a2 - a1| above 5: a knot more (multiplicity Degree - 3)
        b.SetAngle1(1.5);
        b.SetAngle2(-1.5);
        Assert.True(b.Compute(out _));
        BSplineCurve2D c = b.Curve();
        Assert.True(c.KnotCount > 2);
        for (int i = 2; i < c.KnotCount; i++)
        {
            Assert.Equal(6, c.Multiplicity(i));
        }
    }

    [Fact]
    public void Minimal_variation_meets_its_curvatures()
    {
        MinimalVariation m = new(new Point2D(0, 0), new Point2D(10, 0), 0.5, 0, 0.2);
        m.SetAngle1(0.4);
        m.SetAngle2(0.2);
        m.SetConstraintOrder1(2);
        m.SetConstraintOrder2(2);
        m.SetCurvature1(0.05);
        m.SetCurvature2(-0.02);
        Assert.True(m.Compute(out AnalysisCode aCode));
        Assert.Equal(AnalysisCode.OK, aCode);
        BSplineCurve2D c = m.Curve();
        Assert.Equal(0.05, Curvature(c, c.FirstParameter), 1e-9);
        Assert.Equal(-0.02, Curvature(c, c.LastParameter), 1e-9);
        Assert.Equal(0.05, m.GetCurvature1());
        Assert.Equal(-0.02, m.GetCurvature2());
        Assert.Equal(0.2, m.GetPhysicalRatio());

        // free curvatures: the getters give the curve's (OCCT's estimate from the poles)
        m.SetConstraintOrder1(1);
        m.SetConstraintOrder2(1);
        m.Compute(out _);
        BSplineCurve2D c2 = m.Curve();
        Assert.Equal(Curvature(c2, c2.FirstParameter), m.GetCurvature1(), 1e-12);
    }

    [Fact]
    public void Curves_handed_out_are_new_and_frozen()
    {
        Batten b = new(new Point2D(0, 0), new Point2D(10, 0), 0.5);
        b.SetAngle1(0.3);
        b.Compute(out _);
        BSplineCurve2D c1 = b.Curve(), c2 = b.Curve();
        Assert.NotSame(c1, c2);
        Assert.True(c1.IsFrozen);
        Assert.Throws<ImmutableObjectException>(() => c1.SetPole(1, new Point2D(1, 1)));
        // computing again doesn't change a curve handed out
        Point2D aPole = c1.Pole(5);
        b.SetAngle2(-0.3);
        b.Compute(out _);
        Assert.Equal(aPole, c1.Pole(5));
        Assert.NotEqual(aPole, b.Curve().Pole(5));
    }

    [Fact]
    public void Dump_writes_old_and_new_values()
    {
        MinimalVariation m = new(new Point2D(0, 0), new Point2D(10, 0), 0.5);
        m.SetAngle1(0.25);
        string s = m.ToString();
        Assert.StartsWith("  MVCurve      |Old  |   New\n  P1    X      |      0 | 0\n", s, StringComparison.Ordinal);
        Assert.Contains("      Angle1   |      0 | 0.25\n", s, StringComparison.Ordinal);
        Assert.Contains(" PhysicalRatio |      0 | 0\n", s, StringComparison.Ordinal);
        Assert.EndsWith(" ConstrOrder2  |      1 | 1\nAnalysisCode : Ok\n", s, StringComparison.Ordinal);
        m.Compute(out _);
        Assert.Contains("      Angle1   |   0.25 | 0.25\n", m.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void BattenLaw_is_linear_and_copies_are_independent()
    {
        BattenLaw l = new(2.0, 0.5, 4.0);
        Assert.True(l.Value(0.5, out double h));
        Assert.Equal(2.0, h);
        Assert.True(l.Value(1.0, out h));
        Assert.Equal(3.0, h);
        Assert.False(l.Value(-0.5, out h));
        Assert.Equal(0.0, h);
        BattenLaw copy = new(l);
        l.SetSliding(8.0);
        Assert.True(copy.Value(1.0, out h));
        Assert.Equal(3.0, h);
        Assert.True(l.Value(1.0, out h));
        Assert.Equal(4.0, h);
    }

    private static NCollection_HArray1<Point2D> Poles(int n, double theBend)
    {
        NCollection_HArray1<Point2D> p = new(1, n);
        for (int i = 1; i <= n; i++)
        {
            double t = (double)(i - 1) / (n - 1);
            p.SetValue(i, new Point2D(8 * t + 0.1 * Math.Sin(7 * i), theBend * 8 * t * (1 - t) + 0.05 * Math.Cos(3 * i)));
        }
        return p;
    }

    private static NCollection_HArray1<double> FlatKnots()
    {
        NCollection_HArray1<double> k = new(1, 20);
        for (int i = 1; i <= 20; i++)
        {
            k.SetValue(i, i <= 10 ? 0.0 : 1.0);
        }
        return k;
    }

    public static IEnumerable<object[]> EnergyCases()
    {
        foreach (bool aMVC in new[] { false, true })
        {
            foreach ((int o1, int o2) in new[] { (0, 0), (1, 1), (1, 0), (2, 2), (2, 1), (0, 2) })
            {
                if (!aMVC && (o1 == 2 || o2 == 2))
                {
                    continue;
                }
                foreach (bool aFree in new[] { false, true })
                {
                    yield return [aMVC, o1, o2, aFree];
                }
            }
        }
    }

    [Theory]
    [MemberData(nameof(EnergyCases))]
    public void Energy_derivatives_match_finite_differences(bool theMVC, int theOrder1, int theOrder2, bool theFree)
    {
        NCollection_HArray1<Point2D> aPoles = Poles(10, 0.3);
        BattenLaw aLaw = new(0.4, 0.05, 9.0);
        Energy E = theMVC
            ? new EnergyOfMvc(10, FlatKnots(), aPoles, theOrder1, theOrder2, aLaw, 0.3, 9.0, theFree, 0.3, 0.2, 0.05, -0.04)
            : new EnergyOfBatten(10, FlatKnots(), aPoles, theOrder1, theOrder2, aLaw, 9.0, theFree, 0.3, 0.2);
        int n = E.VariableCount;
        Assert.Equal(2 * 8 - theOrder1 - theOrder2 + (theFree ? 1 : 0), n);
        math_Vector X = new(1, n), G = new(1, n), G1 = new(1, n), G2 = new(1, n);
        Matrix H = new(1, n, 1, n);
        Assert.True(E.Variable(X));
        Assert.True(E.Values(X, out double E0, G, H));
        Assert.True(E.Value(X, out double E00));
        Assert.Equal(E0, E00);
        double h = 1e-6;
        // the first end's constraint variables are 1..Order1, the second's the Order2 before the sliding
        int aLast = n - (theFree ? 1 : 0);
        bool IsNeglected(int r, int c) => c >= 1 && c <= theOrder1 && r > aLast - theOrder2 && r <= aLast;
        for (int i = 1; i <= n; i++)
        {
            math_Vector Xp = new(X), Xm = new(X);
            Xp[i] += h;
            Xm[i] -= h;
            Assert.True(E.Values(Xp, out double Ep, G1));
            Assert.True(E.Values(Xm, out double Em, G2));
            double aScale = 1 + Math.Abs(G[i]);
            Assert.True(Math.Abs((Ep - Em) / (2 * h) - G[i]) < 1e-5 * aScale, $"gradient {i}: {(Ep - Em) / (2 * h)} vs {G[i]}");
            for (int j = 1; j <= n; j++)
            {
                if (IsNeglected(j, i) || IsNeglected(i, j))
                {
                    // OCCT sets the terms between the two ends' constraint variables to 0 (occt-issues.md, 5Q)
                    Assert.Equal(0.0, H[j, i]);
                    continue;
                }
                double aFd = (G1[j] - G2[j]) / (2 * h);
                double aTol = 1e-4 * (1 + Math.Abs(H[j, i]));
                Assert.True(Math.Abs(aFd - H[j, i]) < aTol, $"hessian ({j}, {i}): {aFd} vs {H[j, i]}");
            }
        }
        // the evaluations write the poles: back at X, the poles are those of X
        E.Value(X, out _);
        math_Vector X2 = new(1, n);
        E.Variable(X2);
        for (int i = 1; i <= n; i++)
        {
            Assert.Equal(X[i], X2[i], 1e-12);
        }
    }
}
