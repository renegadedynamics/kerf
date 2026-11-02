// New in OCCT.NET (unit 8T): the mesh writers' and readers' change of coordinates, written fresh (decision 13104), with
// the conventions of OCCT's RWMesh_CoordinateSystemConverter (src/DataExchange/TKDE/RWMesh): a length scale and the
// rotation from OCCT's Z-up system to glTF's Y-up one.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A change of coordinates C = scale · R from the model's system to a file's: R is the identity, or the rotation taking
/// Z-up to Y-up ((x, y, z) to (x, z, −y)). Points are mapped by C, locations by C·M·C⁻¹, so placements compose.
/// </summary>
internal readonly struct RWMesh_CoordinateConverter
{
    private readonly double myScale;
    private readonly bool myZUpToYUp;

    /// <summary>The change: points scaled by <paramref name="theScale"/>, then turned Z-up to Y-up if asked.</summary>
    public RWMesh_CoordinateConverter(double theScale, bool isZUpToYUp)
    {
        myScale = theScale;
        myZUpToYUp = isZUpToYUp;
    }

    /// <summary>True if the change does nothing.</summary>
    public bool IsIdentity => myScale == 1.0 && !myZUpToYUp;

    /// <summary>The inverse change.</summary>
    public RWMesh_CoordinateConverter Inverse() => new(1.0 / myScale, myZUpToYUp) { myInverse = !myInverse };

    private bool myInverse { get; init; }

    /// <summary>A point (or a vector, with <paramref name="isVector"/>: not scaled) mapped.</summary>
    public void Map(ref double theX, ref double theY, ref double theZ, bool isVector = false)
    {
        double aS = isVector ? 1.0 : myScale;
        if (!myZUpToYUp)
        {
            theX *= aS;
            theY *= aS;
            theZ *= aS;
            return;
        }
        double aX = theX, aY = theY, aZ = theZ;
        if (!myInverse)
        {
            // R: (x, y, z) -> (x, z, -y)
            theX = aS * aX;
            theY = aS * aZ;
            theZ = -aS * aY;
        }
        else
        {
            // R^-1: (x, y, z) -> (x, -z, y)
            theX = aS * aX;
            theY = -aS * aZ;
            theZ = aS * aY;
        }
    }

    /// <summary>
    /// A transformation mapped (C·M·C⁻¹), as a 4×4 matrix in column-major order (glTF's "matrix").
    /// </summary>
    public double[] MapMatrix(in gp_Trsf theTrsf)
    {
        double[] aMatrix = new double[16];
        for (int aRow = 1; aRow <= 3; aRow++)
        {
            for (int aCol = 1; aCol <= 4; aCol++)
            {
                aMatrix[4 * (aCol - 1) + aRow - 1] = theTrsf.Value(aRow, aCol);
            }
        }
        aMatrix[15] = 1.0;
        return MapMatrix(aMatrix);
    }

    /// <summary>An affine 4×4 matrix (column-major) mapped (C·M·C⁻¹).</summary>
    public double[] MapMatrix(double[] theMatrix)
    {
        double[] aResult = new double[16];
        RWMesh_CoordinateConverter anInverse = Inverse();
        for (int j = 0; j < 3; j++)
        {
            // column j: C (M (C^-1 e_j)), as vectors (the scales cancel)
            double aX = j == 0 ? 1.0 : 0.0, aY = j == 1 ? 1.0 : 0.0, aZ = j == 2 ? 1.0 : 0.0;
            anInverse.Map(ref aX, ref aY, ref aZ, true);
            double aMx = theMatrix[0] * aX + theMatrix[4] * aY + theMatrix[8] * aZ;
            double aMy = theMatrix[1] * aX + theMatrix[5] * aY + theMatrix[9] * aZ;
            double aMz = theMatrix[2] * aX + theMatrix[6] * aY + theMatrix[10] * aZ;
            Map(ref aMx, ref aMy, ref aMz, true);
            aResult[4 * j + 0] = aMx;
            aResult[4 * j + 1] = aMy;
            aResult[4 * j + 2] = aMz;
        }
        // the translation: C (M (0)) = C (t), a point
        double aTx = theMatrix[12], aTy = theMatrix[13], aTz = theMatrix[14];
        Map(ref aTx, ref aTy, ref aTz);
        aResult[12] = aTx;
        aResult[13] = aTy;
        aResult[14] = aTz;
        aResult[15] = 1.0;
        return aResult;
    }
}
