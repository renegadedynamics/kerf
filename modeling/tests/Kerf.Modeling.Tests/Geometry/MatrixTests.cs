// Ported from build123d tests/test_direct_api/test_matrix.py. Element-type validation is the compiler's
// job in C#. Python's Matrix.rotate changes the matrix; the port's Rotated returns a new one.
using Kerf.Geometry;
using Occt;

namespace Kerf.Modeling.Tests.Geometry;

public class MatrixTests
{
    static double[,] Values(Matrix m)
    {
        var values = new double[4, 4];
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
                values[r, c] = m[r, c];
        return values;
    }

    static void AlmostEqual(double[,] expected, Matrix m, int places = 7)
    {
        for (int r = 0; r < expected.GetLength(0); r++)
            for (int c = 0; c < expected.GetLength(1); c++)
                Check.AlmostEqual(expected[r, c], m[r, c], places);
    }

    static TransformForm Form(Matrix m)
    {
        var g = m.ToGpGTrsf();
        return g.Form();
    }

    [Fact]
    public void CreationAndAccess()
    {
        Assert.Equal(new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } }, Values(new Matrix()));

        var vals4x4 = new double[,] { { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 }, { 0, 0, 0, 1 } };
        Assert.Equal(vals4x4, Values(new Matrix(vals4x4)));

        // 12 values: the fourth row is implied
        Assert.Equal(vals4x4, Values(new Matrix(new double[,] { { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 } })));

        Assert.Throws<ArgumentException>(() =>
            new Matrix(new double[,] { { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 }, { 1, 2, 3, 4 } }));
        Assert.Throws<ArgumentException>(() => new Matrix(new double[,] { { 1, 2, 3 }, { 1, 2, 3 }, { 1, 2, 3 } }));

        var m = new Matrix();
        Assert.Throws<IndexOutOfRangeException>(() => m[0, 4]);
        Assert.Throws<IndexOutOfRangeException>(() => m[4, 0]);

        Assert.Equal("Matrix([[1.0, 0.0, 0.0, 1.0],\n        [0.0, 1.0, 0.0, 2.0],\n        [0.0, 0.0, 1.0, 3.0],\n        [0.0, 0.0, 0.0, 1.0]])",
                     new Matrix(vals4x4).ToString());
    }

    [Fact]
    public void Functionality()
    {
        double r32 = Math.Sqrt(3) / 2;
        var mx = new Matrix().Rotated(Axis.X, 30);
        AlmostEqual(new double[,] { { 1, 0, 0, 0 }, { 0, r32, -0.5, 0 }, { 0, 0.5, r32, 0 }, { 0, 0, 0, 1 } }, mx);

        var my = new Matrix().Rotated(Axis.Y, 30);
        AlmostEqual(new double[,] { { r32, 0, 0.5, 0 }, { 0, 1, 0, 0 }, { -0.5, 0, r32, 0 }, { 0, 0, 0, 1 } }, my);

        var rotateZ30 = new double[,] { { r32, -0.5, 0, 0 }, { 0.5, r32, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        var mz = new Matrix().Rotated(Axis.Z, 30);
        AlmostEqual(rotateZ30, mz);

        var v = new Vector(1, 0, 0);
        Check.AlmostEqual((r32, 0.5, 0), mz.Multiply(v), 7);

        // The same rotation written as values multiplies a vector too
        Check.AlmostEqual((r32, 0.5, 0), new Matrix(rotateZ30).Multiply(v), 7);

        // A matrix built from values, rotated afterwards
        var listIdentity = new Matrix(new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } }).Rotated(Axis.Z, 30);
        Check.AlmostEqual((r32, 0.5, 0), listIdentity.Multiply(v), 7);

        AlmostEqual(new double[,]
        {
            { r32, 0, 0.5, 0 },
            { 0.25, r32, -r32 / 2, 0 },
            { -r32 / 2, 0.5, 0.75, 0 },
            { 0, 0, 0, 1 },
        }, mx.Multiply(my));

        var inverse = new Matrix(new double[,] { { 1, 2, 3, 4 }, { 5, 1, 6, 7 }, { 8, 9, 1, 10 }, { 0, 0, 0, 1 } }).Inverse();
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
        var fromTrsf = new Matrix(rotX);
        AlmostEqual(rotXMatrix, fromTrsf);
        var g = fromTrsf.ToGpGTrsf();
        AlmostEqual(rotXMatrix, new Matrix(g));
    }

    [Fact]
    public void TranslationFromValuesIsASimilarity()
    {
        var translation = new Matrix(new double[,] { { 1, 0, 0, 1 }, { 0, 1, 0, 2 }, { 0, 0, 1, 3 }, { 0, 0, 0, 1 } });
        Assert.NotEqual(TransformForm.Other, Form(translation));
        Check.AlmostEqual((11, 22, 33), translation.Multiply(new Vector(10, 20, 30)), 7);
        Assert.Equal(1.0, translation[0, 0]);
        Assert.Equal(2.0, translation[1, 3]);
    }

    [Fact]
    public void SimilarityFromValuesIsRecognised()
    {
        double c = Math.Cos(Math.PI / 6), s = Math.Sin(Math.PI / 6);
        var rotationScaleMirror = new Matrix(new double[,] { { -2 * c, 2 * s, 0, 5 }, { 2 * s, 2 * c, 0, 0 }, { 0, 0, 2, 0 } });
        Assert.NotEqual(TransformForm.Other, Form(rotationScaleMirror));
        Check.AlmostEqual((5 - 2 * c, 2 * s, 0), rotationScaleMirror.Multiply(new Vector(1, 0, 0)), 7);

        // A similarity converts to a gp_Trsf, so it can place a shape
        var g = rotationScaleMirror.ToGpGTrsf();
        var trsf = g.Trsf();
    }

    static Vector Reconstruct(Matrix matrix)
    {
        var (left, scales, right) = matrix.Svd();
        var diagonal = new Matrix(new double[,] { { scales.X, 0, 0, 0 }, { 0, scales.Y, 0, 0 }, { 0, 0, scales.Z, 0 } });
        var product = left.Multiply(diagonal).Multiply(right);
        for (int r = 0; r < 3; r++)
            for (int col = 0; col < 3; col++)
                Check.AlmostEqual(matrix[r, col], product[r, col], 9);

        // Both factors are proper rotations
        foreach (var factor in new[] { left, right })
        {
            double handedness = factor.Multiply(new Vector(1, 0, 0)).Cross(factor.Multiply(new Vector(0, 1, 0)))
                                      .Dot(factor.Multiply(new Vector(0, 0, 1)));
            Check.AlmostEqual(1.0, handedness, 9);
        }
        return scales;
    }

    [Fact]
    public void SvdOfRotationScaleAndMirror()
    {
        double c = Math.Cos(Math.PI / 6), s = Math.Sin(Math.PI / 6);
        Check.AlmostEqual((1, 1, 1), Reconstruct(new Matrix(new double[,] { { c, -s, 0, 0 }, { s, c, 0, 0 }, { 0, 0, 1, 0 } })), 9);
        Check.AlmostEqual((2, 2, 2), Reconstruct(new Matrix(new double[,] { { 2, 0, 0, 5 }, { 0, 2, 0, 0 }, { 0, 0, 2, 0 } })), 9);
        Check.AlmostEqual((3, 2, 1), Reconstruct(new Matrix(new double[,] { { 3, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 2, 0 } })), 9);
        Check.AlmostEqual((1, 1, -1), Reconstruct(new Matrix(new double[,] { { -1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } })), 9);
    }

    [Fact]
    public void SvdOfAPlanarTransform()
    {
        // A map acting only in a plane sorts its zero scale last
        var (left, scales, _) = new Matrix(new double[,] { { 0.2, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 0, 0 } }).Svd();
        Check.AlmostEqual((1, 0.2, 0), scales, 9);
        Check.AlmostEqual(0, left[2, 0], 9);
        Check.AlmostEqual(0, left[2, 1], 9);
        Check.AlmostEqual(1, Math.Abs(left[2, 2]), 9);
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
        var original = new Matrix(rows);
        var g = original.ToGpGTrsf();
        var copied = new Matrix(g);
        for (int r = 0; r < 3; r++)
            for (int col = 0; col < 4; col++)
                Check.AlmostEqual(original[r, col], copied[r, col], 9);
        _ = copied.Rotated(Axis.Z, 90);
        Check.AlmostEqual(rows[0, 0], original[0, 0], 9);
    }

    [Fact]
    public void GeneralAffineMultipliesVectors()
    {
        var nonUniform = new Matrix(new double[,] { { 2, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        Assert.Equal(TransformForm.Other, Form(nonUniform));
        Check.AlmostEqual((2, 1, 1), nonUniform.Multiply(new Vector(1, 1, 1)), 7);

        var shear = new Matrix(new double[,] { { 1, 0.5, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        Check.AlmostEqual((20, 20, 30), shear.Multiply(new Vector(10, 20, 30)), 7);

        var projection = new Matrix(new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 0, 0 } });
        Check.AlmostEqual((10, 20, 0), projection.Multiply(new Vector(10, 20, 30)), 7);
    }

    [Fact]
    public void ProductKeepsASimilaritysScaleAndMirror()
    {
        var mirror = new Matrix(new double[,] { { -1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 } });
        var scale = new Matrix(new double[,] { { 2, 0, 0, 0 }, { 0, 2, 0, 0 }, { 0, 0, 2, 0 } });
        var general = new Matrix(new double[,] { { 2, 0, 0, 1 }, { 0, 3, 0, 2 }, { 0, 0, 0, 3 } });
        var swap = new Matrix(new double[,] { { 0, 1, 0, 0 }, { 1, 0, 0, 0 }, { 0, 0, 1, 0 } });
        foreach (var (left, right) in new[] { (mirror, general), (general, mirror), (scale, general), (swap, general), (swap, scale) })
        {
            var product = left * right;
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 4; col++)
                {
                    double expected = 0;
                    for (int k = 0; k < 3; k++)
                        expected += left[row, k] * right[k, col];
                    expected += col == 3 ? left[row, 3] : 0.0;
                    Check.AlmostEqual(expected, product[row, col], 12);
                }
            }
            var point = new Vector(1, 2, 3);
            Check.AlmostEqual(left * (right * point), product * point, 12);
        }
    }
}
