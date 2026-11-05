// New in Kerf (unit 8T): the mesh writers' and readers' change of coordinates, written fresh (decision 1232), with
// the conventions of OCCT's RWMesh_CoordinateSystemConverter (src/DataExchange/TKDE/RWMesh): a length scale and the
// rotation from OCCT's Z-up system to glTF's Y-up one.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Mesh;

/// <summary>
/// A change of coordinates C = scale · R from the model's system to a file's: R is the identity, or the rotation taking
/// Z-up to Y-up ((x, y, z) to (x, z, −y)). Points are mapped by C, locations by C·M·C⁻¹, so placements compose.
/// </summary>
internal readonly struct CoordinateConverter
{
    private readonly double _scale;
    private readonly bool _zUpToYUp;

    /// <summary>The change: points scaled by <paramref name="scale"/>, then turned Z-up to Y-up if asked.</summary>
    public CoordinateConverter(double scale, bool isZUpToYUp)
    {
        _scale = scale;
        _zUpToYUp = isZUpToYUp;
    }

    /// <summary>True if the change does nothing.</summary>
    public bool IsIdentity => _scale == 1.0 && !_zUpToYUp;

    /// <summary>The inverse change.</summary>
    public CoordinateConverter Inverse() => new(1.0 / _scale, _zUpToYUp) { myInverse = !myInverse };

    private bool myInverse { get; init; }

    /// <summary>A point (or a vector, with <paramref name="isVector"/>: not scaled) mapped.</summary>
    public void Map(ref double x, ref double y, ref double z, bool isVector = false)
    {
        double s = isVector ? 1.0 : _scale;
        if (!_zUpToYUp)
        {
            x *= s;
            y *= s;
            z *= s;
            return;
        }
        double aX = x, aY = y, aZ = z;
        if (!myInverse)
        {
            // R: (x, y, z) -> (x, z, -y)
            x = s * aX;
            y = s * aZ;
            z = -s * aY;
        }
        else
        {
            // R^-1: (x, y, z) -> (x, -z, y)
            x = s * aX;
            y = -s * aZ;
            z = s * aY;
        }
    }

    /// <summary>
    /// A transformation mapped (C·M·C⁻¹), as a 4×4 matrix in column-major order (glTF's "matrix").
    /// </summary>
    public double[] MapMatrix(in Transform3D trsf)
    {
        double[] matrix = new double[16];
        for (int row = 1; row <= 3; row++)
        {
            for (int col = 1; col <= 4; col++)
            {
                matrix[4 * (col - 1) + row - 1] = trsf.Value(row, col);
            }
        }
        matrix[15] = 1.0;
        return MapMatrix(matrix);
    }

    /// <summary>An affine 4×4 matrix (column-major) mapped (C·M·C⁻¹).</summary>
    public double[] MapMatrix(double[] matrix)
    {
        double[] result = new double[16];
        CoordinateConverter inverse = Inverse();
        for (int j = 0; j < 3; j++)
        {
            // column j: C (M (C^-1 e_j)), as vectors (the scales cancel)
            double x = j == 0 ? 1.0 : 0.0, y = j == 1 ? 1.0 : 0.0, z = j == 2 ? 1.0 : 0.0;
            inverse.Map(ref x, ref y, ref z, true);
            double mx = matrix[0] * x + matrix[4] * y + matrix[8] * z;
            double my = matrix[1] * x + matrix[5] * y + matrix[9] * z;
            double mz = matrix[2] * x + matrix[6] * y + matrix[10] * z;
            Map(ref mx, ref my, ref mz, true);
            result[4 * j + 0] = mx;
            result[4 * j + 1] = my;
            result[4 * j + 2] = mz;
        }
        // the translation: C (M (0)) = C (t), a point
        double tx = matrix[12], ty = matrix[13], tz = matrix[14];
        Map(ref tx, ref ty, ref tz);
        result[12] = tx;
        result[13] = ty;
        result[14] = tz;
        result[15] = 1.0;
        return result;
    }
}
