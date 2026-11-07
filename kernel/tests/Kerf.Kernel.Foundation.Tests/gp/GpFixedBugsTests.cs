// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

// gp's bugs that OCCT has and Kerf fixed (occt-issues.md, unit 1's gp rows; Kerf unit 10K3, decision 1485), each
// against an independent reference: the reflection formula, repeated products, the curve's own points.
public class GpFixedBugsTests
{
    private const double Tol = 1e-12;

    private static void Near(in Xy theExpected, in Xy theActual, string theWhat, double theTol = 1e-10)
        => Assert.True(theExpected.Subtracted(theActual).Modulus() <= theTol * Math.Max(1.0, theExpected.Modulus()),
                       $"{theWhat}: expected ({theExpected.X()}, {theExpected.Y()}), got ({theActual.X()}, {theActual.Y()})");

    private static void Near(in Xyz theExpected, in Xyz theActual, string theWhat, double theTol = 1e-10)
        => Assert.True(theExpected.Subtracted(theActual).Modulus() <= theTol * Math.Max(1.0, theExpected.Modulus()),
                       $"{theWhat}: expected ({theExpected.X()}, {theExpected.Y()}, {theExpected.Z()}), "
                       + $"got ({theActual.X()}, {theActual.Y()}, {theActual.Z()})");

    // gp_Dir::Mirror(gp_Ax1): the reflection about the axis direction a is 2 (d.a) a - d
    [Fact]
    public void A_direction_mirrored_about_an_axis_is_its_reflection()
    {
        var aRandom = new Random(15);
        for (int i = 0; i < 50; i++)
        {
            var d = new Direction3D(aRandom.NextDouble() - 0.5, aRandom.NextDouble() - 0.5, aRandom.NextDouble() - 0.5);
            var a = new Direction3D(aRandom.NextDouble() - 0.5, aRandom.NextDouble() - 0.5, aRandom.NextDouble() - 0.5);
            Xyz anExpected = a.XYZ().Multiplied(2.0 * d.Dot(a)).Subtracted(d.XYZ());
            Near(anExpected, d.Mirrored(new Axis3D(new Point3D(1, 2, 3), a)).XYZ(), "Mirrored(Axis3D)");
            // the same as the vector's and the point's mirrors
            Near(new Vector3D(d).Mirrored(new Axis3D(new Point3D(), a)).XYZ(), d.Mirrored(new Axis3D(new Point3D(), a)).XYZ(), "as the vector's");
        }
    }

    private static Xy Apply(in Transform2D theT, Xy theP)
    {
        theT.Transforms(ref theP);
        return theP;
    }

    private static Xy Apply(in GeneralTransform2D theT, Xy theP)
    {
        theT.Transforms(ref theP);
        return theP;
    }

    private static Xyz Apply(in Transform3D theT, Xyz theP)
    {
        theT.Transforms(ref theP);
        return theP;
    }

    private static Xyz Apply(in GeneralTransform3D theT, Xyz theP)
    {
        theT.Transforms(ref theP);
        return theP;
    }

    private static IEnumerable<(string, Transform2D)> Transforms2D()
    {
        yield return ("rotation about the origin", new Transform2D().WithRotation(new Point2D(), 0.7));
        yield return ("rotation about a point", new Transform2D().WithRotation(new Point2D(1, -2), 0.7));
        yield return ("scale", new Transform2D().WithScale(new Point2D(1, -2), 1.3));
        yield return ("scaled rotation", new Transform2D().WithRotation(new Point2D(1, -2), 0.7)
                                                         .Multiplied(new Transform2D().WithScale(new Point2D(-0.5, 3), 1.3)));
        yield return ("scaled mirror", new Transform2D().WithMirror(new Axis2D(new Point2D(0.5, 1), new Direction2D(1, 2)))
                                                       .Multiplied(new Transform2D().WithScale(new Point2D(2, 1), 0.8)));
    }

    // gp_Mat2d's in-place squaring and gp_Trsf2d::Power's compound branch: T^n against T applied n times
    [Fact]
    public void Powers_of_2d_transformations_are_repeated_products()
    {
        var aPoint = new Xy(0.3, -1.7);
        foreach ((string aWhat, Transform2D aT) in Transforms2D())
        {
            foreach (int n in new[] { 2, 3, 4, 5, 7, 8, -2, -3, -5 })
            {
                Transform2D aStep = n > 0 ? aT : aT.Inverted();
                Xy anExpected = aPoint;
                for (int k = 0; k < Math.Abs(n); k++)
                {
                    anExpected = Apply(aStep, anExpected);
                }
                Near(anExpected, Apply(aT.Powered(n), aPoint), $"{aWhat} to the power {n}");
                Near(anExpected, Apply(new GeneralTransform2D(aT).Powered(n), aPoint), $"{aWhat} as a general transformation to the power {n}");
            }
        }
    }

    [Fact]
    public void Powers_of_2d_matrices_and_affinities_are_repeated_products()
    {
        var aM = new Matrix2x2(new Xy(1.1, 0.4), new Xy(-0.3, 0.9));
        Matrix2x2 anExpected = Matrix2x2.Identity();
        for (int n = 1; n <= 9; n++)
        {
            anExpected = anExpected.Multiplied(aM);
            for (int r = 1; r <= 2; r++)
            {
                for (int c = 1; c <= 2; c++)
                {
                    Assert.Equal(anExpected.Value(r, c), aM.Powered(n).Value(r, c), 1e-10);
                }
            }
        }

        GeneralTransform2D anAffinity = new GeneralTransform2D().WithAffinity(new Axis2D(new Point2D(1, 1), new Direction2D(1, 3)), 1.7);
        var aPoint = new Xy(0.3, -1.7);
        foreach (int n in new[] { 2, 3, 4, 5, -2, -3 })
        {
            GeneralTransform2D aStep = n > 0 ? anAffinity : anAffinity.Inverted();
            Xy anExpectedPoint = aPoint;
            for (int k = 0; k < Math.Abs(n); k++)
            {
                anExpectedPoint = Apply(aStep, anExpectedPoint);
            }
            Near(anExpectedPoint, Apply(anAffinity.Powered(n), aPoint), $"affinity to the power {n}");
        }
    }

    // gp_Trsf::Power's compound branch scaled the translation of the square by the square of the scale
    [Fact]
    public void Powers_of_scaled_3d_transformations_are_repeated_products()
    {
        Transform3D aT = new Transform3D().WithRotation(new Axis3D(new Point3D(1, 0, 2), new Direction3D(1, 2, 3)), 0.6)
                                          .Multiplied(new Transform3D().WithScale(new Point3D(-1, 2, 0.5), 1.4));
        var aPoint = new Xyz(0.3, -1.7, 2.2);
        foreach (int n in new[] { 2, 3, 4, 5, -2, -3 })
        {
            Transform3D aStep = n > 0 ? aT : aT.Inverted();
            Xyz anExpected = aPoint;
            for (int k = 0; k < Math.Abs(n); k++)
            {
                anExpected = Apply(aStep, anExpected);
            }
            Near(anExpected, Apply(aT.Powered(n), aPoint), $"scaled rotation to the power {n}");
        }
    }

    // gp_GTrsf::Power of the Other form with N below -1 didn't invert
    [Fact]
    public void Negative_powers_of_3d_affinities_invert()
    {
        GeneralTransform3D anAffinity = new GeneralTransform3D().WithAffinity(new Axis3D(new Point3D(1, 1, 0), new Direction3D(1, 3, 2)), 1.7);
        var aPoint = new Xyz(0.3, -1.7, 2.2);
        foreach (int n in new[] { 2, 3, -2, -3, -4 })
        {
            GeneralTransform3D aStep = n > 0 ? anAffinity : anAffinity.Inverted();
            Xyz anExpected = aPoint;
            for (int k = 0; k < Math.Abs(n); k++)
            {
                anExpected = Apply(aStep, anExpected);
            }
            Near(anExpected, Apply(anAffinity.Powered(n), aPoint), $"affinity to the power {n}");
        }
    }

    // gp_Trsf2d::Multiply, rotation by rotation: (T1 T2)(p) = T1(T2(p)) whether T1 has a translation or not
    [Fact]
    public void A_product_of_2d_rotations_applies_both()
    {
        Transform2D aT1 = new Transform2D().WithRotation(new Point2D(), 0.4);
        Transform2D aT2 = new Transform2D().WithRotation(new Point2D(2, -1), 1.1);
        var aPoint = new Xy(0.3, -1.7);
        Near(Apply(aT1, Apply(aT2, aPoint)), Apply(aT1.Multiplied(aT2), aPoint), "origin rotation times point rotation");
        Near(Apply(aT2, Apply(aT1, aPoint)), Apply(aT2.Multiplied(aT1), aPoint), "point rotation times origin rotation");
    }

    // gp_Hypr2d's asymptotes have directions X +- (b/a) Y, the limits of the branch's direction
    [Fact]
    public void Hyperbola_asymptotes_follow_the_branches()
    {
        foreach ((double a, double b) in new[] { (3.0, 1.2), (2.0, 2.0), (1.0, 4.0) })
        {
            var aFrame = new Frame2D(new Point2D(1, -2), new Direction2D(1, 2), false);
            var aHypr = new Hyperbola2D(aFrame, a, b);
            Xy x = aFrame.XDirection.XY(), y = aFrame.YDirection.XY();
            // the chord from the vertex to a far point of the branch, at +-t
            foreach ((Axis2D anAsymptote, double aSign) in new[] { (aHypr.Asymptote1, 1.0), (aHypr.Asymptote2, -1.0) })
            {
                double t = 20.0;
                Xy aFar = x.Multiplied(a * Math.Cosh(t)).Added(y.Multiplied(aSign * b * Math.Sinh(t)));
                Near(aFar.Normalized(), anAsymptote.Direction.XY(), $"asymptote of a = {a}, b = {b}", 1e-9);
                Near(aFrame.Location.XY(), anAsymptote.Location.XY(), "through the centre");
            }
        }
    }

    // gp_Hypr2d::Directrix2: the points of the second branch are e times as far from the second focus as from it
    [Fact]
    public void The_second_directrix_and_focus_define_the_second_branch()
    {
        var aFrame = new Frame2D(new Point2D(1, -2), new Direction2D(1, 2), true);
        var aHypr = new Hyperbola2D(aFrame, 3.0, 1.2);
        Axis2D aD2 = aHypr.Directrix2;
        Point2D aF2 = aHypr.Focus2;
        Xy x = aFrame.XDirection.XY(), y = aFrame.YDirection.XY();
        foreach (double t in new[] { -2.0, -0.5, 0.0, 0.7, 1.5 })
        {
            // a point of the branch on the side of -X
            var aP = new Point2D(aFrame.Location.XY().Subtracted(x.Multiplied(3.0 * Math.Cosh(t))).Added(y.Multiplied(1.2 * Math.Sinh(t))));
            Assert.Equal(aHypr.Eccentricity, aP.DistanceTo(aF2) / new Line2D(aD2).DistanceTo(aP), 1e-10);
        }
        Assert.Equal(0.0, aD2.Direction.XY().Dot(x), Tol);
    }

    [Fact]
    public void A_general_transform_from_a_matrix_inverts_and_converts_when_it_is_a_similarity()
    {
        // OCCT #1355: a quarter turn about Z, twice the size, moved by (1, 2, 3), given as a 4 by 4 matrix
        double[] matrix = [0, -2, 0, 1, 2, 0, 0, 2, 0, 0, 2, 3, 0, 0, 0, 1];
        GeneralTransform3D general = new GeneralTransform3D().WithMat4(matrix);
        Point3D moved = new Point3D(1, 0, 0).Transformed(general.ToTransform());
        Assert.Equal(1, moved.X, 1e-12);
        Assert.Equal(4, moved.Y, 1e-12);
        Assert.Equal(3, moved.Z, 1e-12);
        Assert.Equal(2, general.ToTransform().ScaleFactor, 1e-12);

        // The inverse takes it back
        Xyz back = new Xyz(1, 4, 3);
        general.Inverted().Transforms(ref back);
        Assert.Equal(1, back.X(), 1e-12);
        Assert.Equal(0, back.Y(), 1e-12);
        Assert.Equal(0, back.Z(), 1e-12);

        // A mirror is one, with a negative scale
        Point3D mirrored = new Point3D(1, 2, 3).Transformed(new GeneralTransform3D().WithMat4([-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1]).ToTransform());
        Assert.Equal(-1, mirrored.X, 1e-12);
        Assert.Equal(-3, mirrored.Z, 1e-12);

        // A shear and a stretch in one direction aren't, and neither is a singular matrix
        Assert.Throws<InvalidOperationException>(() => new GeneralTransform3D().WithMat4([1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]).ToTransform());
        Assert.Throws<InvalidOperationException>(() => new GeneralTransform3D().WithMat4([2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]).ToTransform());
        Assert.Throws<InvalidOperationException>(() => new GeneralTransform3D().WithMat4([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]).ToTransform());
    }
}
