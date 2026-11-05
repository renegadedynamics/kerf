// Ported from build123d tests/test_direct_api/test_matrix.py. Element-type validation is the compiler's
// job in C#. Python's Matrix.rotate changes the matrix; the port's Rotated returns a new one.
using Kerf.Geometry;

namespace Kerf.Modeling.Tests.Geometry;

using Kerf.Modeling;

public class MatrixTests
{
    static double[,] Values(GeneralTransform3D m)
    {
        var values = new double[4, 4];
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                values[r, c] = m.At(r, c);
        return values;
    }

    static void AlmostEqual(double[,] expected, GeneralTransform3D m, int places = 7)
    {
        for (int r = 0; r < expected.GetLength(0); r++)
            for (int c = 0; c < expected.GetLength(1); c++)
                Check.AlmostEqual(expected[r, c], m.At(r, c), places);
    }

    static TransformForm Form(GeneralTransform3D m)
    {
        var g = m;
        return g.Form;
    }

    [Fact]
    public void CreationAndAccess()
    {
        Assert.Equal(new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } }, Values(new GeneralTransform3D()));

        var vals4x4 = new double[,] { { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 }, { 0, 0, 0, 1 } };
        Assert.Equal(vals4x4, Values(GeneralTransform3D.FromValues(vals4x4)));

        // 12 values: the fourth row is implied
        Assert.Equal(vals4x4, Values(GeneralTransform3D.FromValues(new double[,] { { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 } })));

        Assert.Throws<ArgumentException>(() =>
            GeneralTransform3D.FromValues(new double[,] { { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 }, { 1, 2, 3, 4 } }));
        Assert.Throws<ArgumentException>(() => GeneralTransform3D.FromValues(new double[,] { { 1, 2, 3 }, { 1, 2, 3 }, { 1, 2, 3 } }));

        var m = new GeneralTransform3D();
        Assert.Throws<IndexOutOfRangeException>(() => m.At(0, 4));
        Assert.Throws<IndexOutOfRangeException>(() => m.At(4, 0));

        Assert.Equal("Matrix([[1, 0, 0, 1],\n        [0, 1, 0, 2],\n        [0, 0, 1, 3],\n        [0, 0, 0, 1]])",
                     GeneralTransform3D.FromValues(vals4x4).Describe());
    }

    [Fact]
    public void Functionality()
    {
        double r32 = Math.Sqrt(3) / 2;
        var mx = new GeneralTransform3D().Rotated(Axis.X, 30);
        AlmostEqual(new double[,] { { 1, 0, 0, 0 }, { 0, r32, -0.5, 0 }, { 0, 0.5, r32, 0 }, { 0, 0, 0, 1 } }, mx);

        var my = new GeneralTransform3D().Rotated(Axis.Y, 30);
        AlmostEqual(new double[,] { { r32, 0, 0.5, 0 }, { 0, 1, 0, 0 }, { -0.5, 0, r32, 0 }, { 0, 0, 0, 1 } }, my);

        var rotateZ30 = new double[,] { { r32, -0.5, 0, 0 }, { 0.5, r32, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        var mz = new GeneralTransform3D().Rotated(Axis.Z, 30);
        AlmostEqual(rotateZ30, mz);

        var v = new Point3D(1, 0, 0);
        Check.AlmostEqual(new Point3D(r32, 0.5, 0), (mz * v), 7);

        // The same rotation written as values multiplies a vector too
        Check.AlmostEqual(new Point3D(r32, 0.5, 0), (GeneralTransform3D.FromValues(rotateZ30) * v), 7);

        // A matrix built from values, rotated afterwards
        var listIdentity = GeneralTransform3D.FromValues(new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } }).Rotated(Axis.Z, 30);
        Check.AlmostEqual(new Point3D(r32, 0.5, 0), (listIdentity * v), 7);

        AlmostEqual(new double[,]
        {
            { r32, 0, 0.5, 0 },
            { 0.25, r32, -r32 / 2, 0 },
            { -r32 / 2, 0.5, 0.75, 0 },
            { 0, 0, 0, 1 },
        }, mx.Compose(my));

        var inverse = GeneralTransform3D.FromValues(new double[,] { { 1, 2, 3, 4 }, { 5, 1, 6, 7 }, { 8, 9, 1, 10 }, { 0, 0, 0, 1 } }).Inverted();
        AlmostEqual(new double[,]
        {
            { -53 / 144.0, 25 / 144.0, 1 / 16.0, -53 / 144.0 },
            { 43 / 144.0, -23 / 144.0, 1 / 16.0, -101 / 144.0 },
            { 37 / 144.0, 7 / 144.0, -1 / 16.0, -107 / 144.0 },
            { 0, 0, 0, 1 },
        }, inverse);

        // From a gp_Trsf
        var rotX = new Transform3D();
        var origin = new Point3D(0, 0, 0);
        var dx = new Direction3D(1, 0, 0);
        var ax = new Axis3D(origin, dx);
        double t = Math.PI;
        rotX = rotX.WithRotation(ax, t);
        var rotXMatrix = new double[,]
        {
            { 1, 0, 0, 0 }, { 0, Math.Cos(t), -Math.Sin(t), 0 }, { 0, Math.Sin(t), Math.Cos(t), 0 }, { 0, 0, 0, 1 },
        };
        var fromTrsf = new GeneralTransform3D(rotX);
        AlmostEqual(rotXMatrix, fromTrsf);
        var g = fromTrsf;
        AlmostEqual(rotXMatrix, g);
    }

    [Fact]
    public void TranslationFromValuesIsASimilarity()
    {
        var translation = GeneralTransform3D.FromValues(new double[,] { { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 }, { 0, 0, 0, 1 } });
        Assert.NotEqual(TransformForm.Other, Form(translation));
        Check.AlmostEqual(new Point3D(11, 22, 33), (translation * new Point3D(10, 20, 30)), 7);
        Assert.Equal(1.0, translation.At(0, 0));
        Assert.Equal(2.0, translation.At(1, 3));
    }

    [Fact]
    public void SimilarityFromValuesIsRecognised()
    {
        double c = Math.Cos(Math.PI / 6), s = Math.Sin(Math.PI / 6);
        var rotationScaleMirror = GeneralTransform3D.FromValues(new double[,] { { -2 * c, 2 * s, 0, 5 }, { 2 * s, 2 * c, 0, 0 }, { 0, 0, 2, 0 } });
        Assert.NotEqual(TransformForm.Other, Form(rotationScaleMirror));
        Check.AlmostEqual(new Point3D(5 - 2 * c, 2 * s, 0), (rotationScaleMirror * new Point3D(1, 0, 0)), 7);

        // A similarity converts to a gp_Trsf, so it can place a shape
        var g = rotationScaleMirror;
        var trsf = g.ToTransform();
    }

    static Vector3D Reconstruct(GeneralTransform3D matrix)
    {
        var (left, scales, right) = matrix.Svd();
        var diagonal = GeneralTransform3D.FromValues(new double[,] { { scales.X, 0, 0, 0 }, { 0, scales.Y, 0, 0 }, { 0, 0, scales.Z, 0 } });
        var product = left.Compose(diagonal).Compose(right);
        for (int r = 0; r < 3; r++)
            for (int col = 0; col < 3; col++)
                Check.AlmostEqual(matrix.At(r, col), product.At(r, col), 9);

        // Both factors are proper rotations
        foreach (var factor in new[] { left, right })
        {
            double handedness = (factor * new Point3D(1, 0, 0)).ToVector().Cross((factor * new Point3D(0, 1, 0)).ToVector())
                                      .Dot((factor * new Point3D(0, 0, 1)).ToVector());
            Check.AlmostEqual(1.0, handedness, 9);
        }
        return scales;
    }

    [Fact]
    public void SvdOfRotationScaleAndMirror()
    {
        double c = Math.Cos(Math.PI / 6), s = Math.Sin(Math.PI / 6);
        Check.AlmostEqual(new Vector3D(1, 1, 1), Reconstruct(GeneralTransform3D.FromValues(new double[,] { { c, -s, 0, 0 }, { s, c, 0, 0 }, { 0, 0, 1, 0 } })), 9);
        Check.AlmostEqual(new Vector3D(2, 2, 2), Reconstruct(GeneralTransform3D.FromValues(new double[,] { { 2, 0, 0, 5 }, { 0, 2, 0, 0 }, { 0, 0, 2, 0 } })), 9);
        Check.AlmostEqual(new Vector3D(3, 2, 1), Reconstruct(GeneralTransform3D.FromValues(new double[,] { { 3, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 2, 0 } })), 9);
        Check.AlmostEqual(new Vector3D(1, 1, -1), Reconstruct(GeneralTransform3D.FromValues(new double[,] { { -1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } })), 9);
    }

    [Fact]
    public void SvdOfAPlanarTransform()
    {
        // A map acting only in a plane sorts its zero scale last
        var (left, scales, _) = GeneralTransform3D.FromValues(new double[,] { { 0.2, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 0, 0 } }).Svd();
        Check.AlmostEqual(new Vector3D(1, 0.2, 0), scales, 9);
        Check.AlmostEqual(0, left.At(2, 0), 9);
        Check.AlmostEqual(0, left.At(2, 1), 9);
        Check.AlmostEqual(1, Math.Abs(left.At(2, 2)), 9);
    }

    [Theory]
    [InlineData("rotation")]
    [InlineData("shear")]
    [InlineData("stretch")]
    public void RotatingACopyLeavesTheOriginal(string label)
    {
        var rows = label switch
        {
            "rotation" => new double[,] { { 0, -1, 0, 1 }, { 1, 0, 0, 2 }, { 0, 0, 1, 3 } },
            "shear" => new double[,] { { 1, 0.5, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } },
            _ => new double[,] { { 2, 0, 0, 1 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } },
        };
        var original = GeneralTransform3D.FromValues(rows);
        var g = original;
        var copied = g;
        for (int r = 0; r < 3; r++)
            for (int col = 0; col < 4; col++)
                Check.AlmostEqual(original.At(r, col), copied.At(r, col), 9);
        _ = copied.Rotated(Axis.Z, 90);
        Check.AlmostEqual(rows[0, 0], original.At(0, 0), 9);
    }

    [Fact]
    public void GeneralAffineMultipliesVectors()
    {
        var nonUniform = GeneralTransform3D.FromValues(new double[,] { { 2, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        Assert.Equal(TransformForm.Other, Form(nonUniform));
        Check.AlmostEqual(new Point3D(2, 1, 1), (nonUniform * new Point3D(1, 1, 1)), 7);

        var shear = GeneralTransform3D.FromValues(new double[,] { { 1, 0.5, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        Check.AlmostEqual(new Point3D(20, 20, 30), (shear * new Point3D(10, 20, 30)), 7);

        var projection = GeneralTransform3D.FromValues(new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 0, 0 } });
        Check.AlmostEqual(new Point3D(10, 20, 0), (projection * new Point3D(10, 20, 30)), 7);
    }

    [Fact]
    public void ProductKeepsASimilaritysScaleAndMirror()
    {
        var mirror = GeneralTransform3D.FromValues(new double[,] { { -1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        var scale = GeneralTransform3D.FromValues(new double[,] { { 2, 0, 0, 0 }, { 0, 2, 0, 0 }, { 0, 0, 2, 0 } });
        var general = GeneralTransform3D.FromValues(new double[,] { { 2, 0, 0, 1 }, { 0, 3, 0, 2 }, { 0, 0, 0, 3 } });
        var swap = GeneralTransform3D.FromValues(new double[,] { { 0, 1, 0, 0 }, { 1, 0, 0, 0 }, { 0, 0, 1, 0 } });
        foreach (var (left, right) in new[] { (mirror, general), (general, mirror), (scale, general), (swap, general), (swap, scale) })
        {
            var product = left.Compose(right);
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 4; col++)
                {
                    double expected = 0;
                    for (int k = 0; k < 3; k++)
                        expected += left.At(row, k) * right.At(k, col);
                    expected += col == 3 ? left.At(row, 3) : 0.0;
                    Check.AlmostEqual(expected, product.At(row, col), 12);
                }
            }
            var point = new Point3D(1, 2, 3);
            Check.AlmostEqual(left * (right * point), product * point, 12);
        }
    }
}
