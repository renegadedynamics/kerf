using Kerf.Engine.Foundation;
using Kerf.Geometry;
using System.Text;
using Occt;

namespace Kerf.Modeling;

/// <summary>
/// An immutable 4×4 affine transformation matrix, used to move geometry in space. The fourth row is
/// always (0, 0, 0, 1).
/// </summary>
public sealed class Matrix
{
    /// <summary>The OCCT transformation. Never modified.</summary>
    internal GeneralTransform3D GTrsf { get; }

    Matrix(GeneralTransform3D gtrsf, bool owned) => GTrsf = gtrsf;

    /// <summary>The identity matrix.</summary>
    public Matrix() : this(new GeneralTransform3D(), owned: true) { }

    /// <summary>A matrix from an OCCT gp_GTrsf (copied).</summary>
    public Matrix(GeneralTransform3D gtrsf) : this(Copy(gtrsf), owned: true) { }

    /// <summary>A matrix from an OCCT gp_Trsf.</summary>
    public Matrix(Transform3D trsf) : this(new GeneralTransform3D(trsf), owned: true) { }

    /// <summary>
    /// A matrix from its values: 3 rows of 4 (the fourth row is implied), or 4 rows of 4 whose last row
    /// must be (0, 0, 0, 1).
    /// </summary>
    /// <exception cref="ArgumentException">The array isn't 3×4 or 4×4, or its last row isn't (0, 0, 0, 1).</exception>
    public Matrix(double[,] values) : this(FromValues(values), owned: true) { }

    static GeneralTransform3D Copy(GeneralTransform3D source)
    {
        // A similarity is copied through its gp_Trsf, which keeps OCCT's record of its form and scale; a
        // general affine transform, which Trsf() refuses, from its matrix and translation
        if (source.Form() != TransformForm.Other)
        {
            var trsf = source.Trsf();
            return new GeneralTransform3D(trsf);
        }
        var linear = source.VectorialPart();
        var translation = source.TranslationPart();
        return new GeneralTransform3D(linear, translation);
    }

    static GeneralTransform3D FromValues(double[,] values)
    {
        int rows = values.GetLength(0), cols = values.GetLength(1);
        if (cols != 4 || rows is not (3 or 4))
            throw new ArgumentException($"Matrix constructor requires 2d list of 4x3 or 4x4, but got {rows}x{cols}", nameof(values));
        if (rows == 4 && (values[3, 0], values[3, 1], values[3, 2], values[3, 3]) != (0, 0, 0, 1))
            throw new ArgumentException(
                $"Expected the last row to be [0,0,0,1], but got: [{values[3, 0]}, {values[3, 1]}, {values[3, 2]}, {values[3, 3]}]",
                nameof(values));

        if (IsSimilarity(values))
        {
            // A rotation, translation, uniform scale or mirror, or a product of them: build it as a
            // gp_Trsf so OCCT records its form and scale factor and it converts back to a gp_Trsf where
            // one is needed. Filling a gp_GTrsf value by value marks it gp_Other whatever it holds.
            var similarity = new Transform3D().WithValues(values[0, 0], values[0, 1], values[0, 2], values[0, 3],
                                                      values[1, 0], values[1, 1], values[1, 2], values[1, 3],
                                                      values[2, 0], values[2, 1], values[2, 2], values[2, 3]);
            return new GeneralTransform3D(similarity);
        }

        // A general affine transformation (non-uniform scale, shear, projection) is legitimately gp_Other
        var gtrsf = new GeneralTransform3D();
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                gtrsf = gtrsf.WithValue(i + 1, j + 1, values[i, j]);
        return gtrsf;
    }

    /// <summary>
    /// Is the 3×3 part a rotation or reflection times a uniform scale? Checked as (M/s)ᵀ(M/s) = I with s
    /// the cube root of the determinant; gp_Trsf.SetValues would orthogonalise anything without complaint.
    /// </summary>
    static bool IsSimilarity(double[,] m)
    {
        double det = m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
                   - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
                   + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
        if (Math.Abs(det) < Math.Pow(Constants.Tolerance, 3))
            return false;   // singular: a projection, which no gp_Trsf can hold
        double scale = Math.CopySign(Math.Pow(Math.Abs(det), 1.0 / 3.0), det);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                double dot = (m[0, i] * m[0, j] + m[1, i] * m[1, j] + m[2, i] * m[2, j]) / (scale * scale);
                if (Math.Abs(dot - (i == j ? 1.0 : 0.0)) > 1e-9)
                    return false;
            }
        }
        return true;
    }

    /// <summary>A value by zero-based row and column; the fourth row is (0, 0, 0, 1).</summary>
    /// <exception cref="IndexOutOfRangeException">The row or column is outside 0 to 3.</exception>
    public double this[int row, int col]
    {
        get
        {
            if (row is < 0 or > 3 || col is < 0 or > 3)
                throw new IndexOutOfRangeException($"Out of bounds access into 4x4 matrix: ({row}, {col})");
            if (row < 3)
                return GTrsf.Value(row + 1, col + 1);
            return col == 3 ? 1.0 : 0.0;
        }
    }

    /// <summary>A new OCCT gp_GTrsf for this matrix.</summary>
    public GeneralTransform3D ToGpGTrsf() => Copy(GTrsf);

    /// <summary>This matrix followed by a rotation about an axis by an angle in degrees.</summary>
    public Matrix Rotated(Axis axis, double angle)
    {
        var rotation = new Transform3D();
        var ax1 = axis.ToGpAx1();
        rotation = rotation.WithRotation(ax1, angle * Constants.DegToRad);
        var g = new GeneralTransform3D(rotation);
        return new Matrix(GTrsf.Multiplied(g), owned: true);
    }

    /// <summary>The inverse matrix.</summary>
    /// <exception cref="FailureException">The matrix is singular.</exception>
    public Matrix Inverse() => new(GTrsf.Inverted(), owned: true);

    /// <summary>A vector transformed by this matrix.</summary>
    public Vector Multiply(Vector v) => v.Transform(this);

    /// <summary>The product that applies <paramref name="other"/> first and then this matrix.</summary>
    public Matrix Multiply(Matrix other)
    {
        // OCCT's gp_GTrsf product mixes the bare rotation of a similarity with its scale factor kept
        // aside, so a mirror or a uniform scale times a general affine matrix loses that factor;
        // multiply the values instead
        var values = new double[3, 4];
        for (int row = 0; row < 3; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                double sum = 0;
                for (int k = 0; k < 3; k++)
                    sum += this[row, k] * other[k, col];
                values[row, col] = sum + (col == 3 ? this[row, 3] : 0.0);
            }
        }
        return new Matrix(values);
    }

#pragma warning disable CS1591
    public static Matrix operator *(Matrix a, Matrix b) => a.Multiply(b);
    public static Vector operator *(Matrix a, Vector v) => a.Multiply(v);
#pragma warning restore CS1591

    /// <summary>The 16 values in column-major order (each column's four values in turn), as glTF expects.</summary>
    public double[] TransposedList()
    {
        var result = new double[16];
        for (int col = 0; col < 4; col++)
            for (int row = 0; row < 4; row++)
                result[col * 4 + row] = this[row, col];
        return result;
    }

    /// <summary>
    /// The singular value decomposition of the linear (3×3) part as <c>left · diag(scales) · right</c>:
    /// read from the right, <c>right</c> turns space, the scales stretch it along the axes, and <c>left</c>
    /// turns it again. Both are proper rotations; when the transform mirrors, the last scale is negative.
    /// Scales come in descending order of magnitude.
    /// </summary>
    public (Matrix Left, Vector Scales, Matrix Right) Svd()
    {
        var a = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                a[i, j] = this[i, j];

        var (u, s, vt) = Svd3(a);
        if (Det3(u) < 0)
        {
            for (int i = 0; i < 3; i++)
                u[i, 2] = -u[i, 2];
            s[2] = -s[2];
        }
        if (Det3(vt) < 0)
        {
            for (int j = 0; j < 3; j++)
                vt[2, j] = -vt[2, j];
            s[2] = -s[2];
        }

        static Matrix AsMatrix(double[,] r) => new(new double[,]
        {
            { r[0, 0], r[0, 1], r[0, 2], 0 },
            { r[1, 0], r[1, 1], r[1, 2], 0 },
            { r[2, 0], r[2, 1], r[2, 2], 0 },
        });
        return (AsMatrix(u), new Vector(s[0], s[1], s[2]), AsMatrix(vt));
    }

    static double Det3(double[,] m) =>
        m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1])
      - m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0])
      + m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);

    /// <summary>
    /// SVD of a 3×3 matrix as numpy.linalg.svd returns it: A = U·diag(S)·Vt, S non-negative and
    /// descending. V and S come from the Jacobi eigen-decomposition of AᵀA; U from A·V, completed to an
    /// orthonormal basis where a singular value is zero.
    /// </summary>
    static (double[,] U, double[] S, double[,] Vt) Svd3(double[,] a)
    {
        // AᵀA, symmetric
        var ata = new double[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    ata[i, j] += a[k, i] * a[k, j];

        var v = new double[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        for (int sweep = 0; sweep < 50; sweep++)
        {
            double off = ata[0, 1] * ata[0, 1] + ata[0, 2] * ata[0, 2] + ata[1, 2] * ata[1, 2];
            if (off < 1e-30)
                break;
            foreach (var (p, q) in (ReadOnlySpan<(int, int)>)[(0, 1), (0, 2), (1, 2)])
            {
                if (Math.Abs(ata[p, q]) < 1e-300)
                    continue;
                double theta = (ata[q, q] - ata[p, p]) / (2 * ata[p, q]);
                double t = Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1));
                if (theta == 0)
                    t = 1;
                double c = 1 / Math.Sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; k++)
                {
                    double akp = ata[k, p], akq = ata[k, q];
                    ata[k, p] = c * akp - s * akq;
                    ata[k, q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; k++)
                {
                    double apk = ata[p, k], aqk = ata[q, k];
                    ata[p, k] = c * apk - s * aqk;
                    ata[q, k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; k++)
                {
                    double vkp = v[k, p], vkq = v[k, q];
                    v[k, p] = c * vkp - s * vkq;
                    v[k, q] = s * vkp + c * vkq;
                }
            }
        }

        // Sort by descending eigenvalue (squared singular value)
        int[] order = [0, 1, 2];
        Array.Sort(order, (i, j) => ata[j, j].CompareTo(ata[i, i]));
        var sv = new double[3];
        var vs = new double[3, 3];
        for (int c = 0; c < 3; c++)
        {
            sv[c] = Math.Sqrt(Math.Max(ata[order[c], order[c]], 0));
            for (int r = 0; r < 3; r++)
                vs[r, c] = v[r, order[c]];
        }

        // U's columns: A·v / s, or completed orthonormally where s is (near) zero
        var u = new double[3, 3];
        double tol = Math.Max(sv[0], 1) * 1e-12;
        for (int c = 0; c < 3; c++)
        {
            if (sv[c] > tol)
            {
                for (int r = 0; r < 3; r++)
                    u[r, c] = (a[r, 0] * vs[0, c] + a[r, 1] * vs[1, c] + a[r, 2] * vs[2, c]) / sv[c];
            }
            else
            {
                var col = CompleteBasis(u, c);
                for (int r = 0; r < 3; r++)
                    u[r, c] = col[r];
            }
        }

        var vt = new double[3, 3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                vt[r, c] = vs[c, r];
        return (u, sv, vt);
    }

    // A unit vector orthogonal to the first `count` columns of u
    static double[] CompleteBasis(double[,] u, int count)
    {
        Span<Vector> axes = [new(1, 0, 0), new(0, 1, 0), new(0, 0, 1)];
        foreach (var axis in axes)
        {
            var w = axis;
            for (int c = 0; c < count; c++)
            {
                var uc = new Vector(u[0, c], u[1, c], u[2, c]);
                w -= uc * w.Dot(uc);
            }
            if (w.Length > 1e-6)
            {
                var n = w.Normalized();
                return [n.X, n.Y, n.Z];
            }
        }
        return [0, 0, 1];
    }

    /// <summary>"Matrix([[1.0, 0.0, 0.0, 0.0], ...])", each row in turn.</summary>
    public override string ToString()
    {
        var sb = new StringBuilder("Matrix([");
        for (int row = 0; row < 4; row++)
        {
            if (row > 0)
                sb.Append(",\n        ");
            sb.Append('[');
            for (int col = 0; col < 4; col++)
            {
                if (col > 0)
                    sb.Append(", ");
                sb.Append(PyFormat.Repr(this[row, col]));
            }
            sb.Append(']');
        }
        return sb.Append("])").ToString();
    }
}
