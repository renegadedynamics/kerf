using System.Text;

namespace Kerf.Modeling;

/// <summary>
/// build123d's conveniences on the kernel's <see cref="GeneralTransform3D"/>, an affine transformation of which the fourth
/// row is always (0, 0, 0, 1). It took the place of the layer's <c>Matrix</c>.
/// </summary>
/// <remarks>
/// <c>At</c> counts from zero; the kernel's indexer, as OCCT's, from one. <c>Compose</c> is the product of the values,
/// where the kernel's <c>*</c> on two transformations is OCCT's (see <c>Compose</c>).
/// </remarks>
public static class TransformExtensions
{
    extension(GeneralTransform3D)
    {
        /// <summary>
        /// A transformation from its values: 3 rows of 4 (the fourth row is implied), or 4 rows of 4 whose last row
        /// must be (0, 0, 0, 1). A similarity keeps OCCT's record of its form and scale.
        /// </summary>
        /// <exception cref="ArgumentException">The array isn't 3×4 or 4×4, or its last row isn't (0, 0, 0, 1).</exception>
        public static GeneralTransform3D FromValues(double[,] values) => FromValuesCore(values);
    }

    extension(GeneralTransform3D m)
    {
        /// <summary>A value by zero-based row and column; the fourth row is (0, 0, 0, 1).</summary>
        /// <exception cref="IndexOutOfRangeException">The row or column is outside 0 to 3.</exception>
        public double At(int row, int column)
        {
            if (row is < 0 or > 3 || column is < 0 or > 3)
                throw new IndexOutOfRangeException($"Out of bounds access into 4x4 matrix: ({row}, {column})");
            if (row < 3)
                return m[row + 1, column + 1];
            return column == 3 ? 1.0 : 0.0;
        }

        /// <summary>This transformation followed by a rotation about an axis by an angle in degrees.</summary>
        public GeneralTransform3D Rotated(Axis axis, double angle)
        {
            var rotation = new Transform3D().WithRotation(axis.ToAxis3D(), angle * Constants.DegToRad);
            return m.Multiplied(new GeneralTransform3D(rotation));
        }

        /// <summary>The product that applies <paramref name="other"/> first and then this transformation.</summary>
        public GeneralTransform3D Compose(GeneralTransform3D other)
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
                        sum += m.At(row, k) * other.At(k, col);
                    values[row, col] = sum + (col == 3 ? m.At(row, 3) : 0.0);
                }
            }
            return GeneralTransform3D.FromValues(values);
        }

        /// <summary>The 16 values in column-major order (each column's four values in turn), as glTF expects.</summary>
        public double[] TransposedList()
        {
            var result = new double[16];
            for (int col = 0; col < 4; col++)
                for (int row = 0; row < 4; row++)
                    result[col * 4 + row] = m.At(row, col);
            return result;
        }

        /// <summary>
        /// The singular value decomposition of the linear (3×3) part as <c>left · diag(scales) · right</c>:
        /// read from the right, <c>right</c> turns space, the scales stretch it along the axes, and <c>left</c>
        /// turns it again. Both are proper rotations; when the transform mirrors, the last scale is negative.
        /// Scales come in descending order of magnitude.
        /// </summary>
        public (GeneralTransform3D Left, Vector3D Scales, GeneralTransform3D Right) Svd()
        {
            var a = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    a[i, j] = m.At(i, j);

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

            static GeneralTransform3D AsMatrix(double[,] r) => GeneralTransform3D.FromValues(new double[,]
            {
                { r[0, 0], r[0, 1], r[0, 2], 0 },
                { r[1, 0], r[1, 1], r[1, 2], 0 },
                { r[2, 0], r[2, 1], r[2, 2], 0 },
            });
            return (AsMatrix(u), new Vector3D(s[0], s[1], s[2]), AsMatrix(vt));
        }

        /// <summary>"Matrix([[1.0, 0.0, 0.0, 0.0], ...])", each row in turn.</summary>
        public string Describe()
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
                    sb.Append(Formatting.Number(m.At(row, col)));
                }
                sb.Append(']');
            }
            return sb.Append("])").ToString();
        }
    }

    static GeneralTransform3D FromValuesCore(double[,] values)
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
        return GeneralTransform3D.Mat4([values[0, 0], values[0, 1], values[0, 2], values[0, 3],
                                        values[1, 0], values[1, 1], values[1, 2], values[1, 3],
                                        values[2, 0], values[2, 1], values[2, 2], values[2, 3],
                                        0, 0, 0, 1]);
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
        Span<Vector3D> axes = [new(1, 0, 0), new(0, 1, 0), new(0, 0, 1)];
        foreach (var axis in axes)
        {
            var w = axis;
            for (int c = 0; c < count; c++)
            {
                var uc = new Vector3D(u[0, c], u[1, c], u[2, c]);
                w -= uc * w.Dot(uc);
            }
            if (w.Magnitude > 1e-6)
            {
                var n = w.Normalized();
                return [n.X, n.Y, n.Z];
            }
        }
        return [0, 0, 1];
    }
}
