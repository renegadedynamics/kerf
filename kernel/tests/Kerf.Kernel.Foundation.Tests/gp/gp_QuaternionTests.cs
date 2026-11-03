// Port of: src/FoundationClasses/TKMath/GTests/gp_Quaternion_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.EulerSequence;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_QuaternionTest
{
    private static readonly string[] aNames =
    [
        "Extrinsic_XYZ", "Extrinsic_XZY", "Extrinsic_YZX", "Extrinsic_YXZ",
        "Extrinsic_ZXY", "Extrinsic_ZYX", "Intrinsic_XYZ", "Intrinsic_XZY",
        "Intrinsic_YZX", "Intrinsic_YXZ", "Intrinsic_ZXY", "Intrinsic_ZYX",
        "Extrinsic_XYX", "Extrinsic_XZX", "Extrinsic_YZY", "Extrinsic_YXY",
        "Extrinsic_ZYZ", "Extrinsic_ZXZ", "Intrinsic_XYX", "Intrinsic_XZX",
        "Intrinsic_YZY", "Intrinsic_YXY", "Intrinsic_ZXZ", "Intrinsic_ZYZ",
    ];

    // Test OCC25574: gp_Quaternion Euler angle conversion is consistent and correct
    // for all supported Euler sequences, including YawPitchRoll.
    [Fact]
    public void OCC25574_EulerAnglesConsistency()
    {
        Quaternion3 aQuat = new();
        aQuat.Set(0.06766916507860499, 0.21848101129786085, 0.11994599260380681, 0.9660744746954637);

        Matrix3x3 aRinv = aQuat.GetMatrix().Inverted();
        Matrix3x3 aI = new();
        aI = aI.WithIdentity();

        // Check round-trip consistency: GetEulerAngles then SetEulerAngles should reproduce the matrix
        for (int i = (int)Extrinsic_XYZ; i <= (int)Intrinsic_ZYZ; i++)
        {
            aQuat.GetEulerAngles((EulerSequence)i, out double alpha, out double beta, out double gamma);

            Quaternion3 aQuat2 = new();
            aQuat2.SetEulerAngles((EulerSequence)i, alpha, beta, gamma);

            Matrix3x3 aR = aQuat2.GetMatrix();
            Matrix3x3 aDiff = aR * aRinv - aI;
            // Round-trip failed for Euler sequence aNames[i - gp_Extrinsic_XYZ]
            Assert.True(aDiff.Determinant() <= 1e-5, aNames[i - (int)Extrinsic_XYZ]);
        }
    }

    // Test OCC25574: Each Euler angle rotation around a named axis should not change
    // a point lying on that axis.
    [Fact]
    public void OCC25574_EulerAxisRotationPreservesAxis()
    {
        for (int i = (int)Extrinsic_XYZ; i <= (int)Intrinsic_ZYZ; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // Determine axis index from the sequence name (X=0, Y=1, Z=2)
                int anAxis = aNames[i - (int)Extrinsic_XYZ][10 + j] - 'X';
                Assert.True(anAxis >= 0);
                Assert.True(anAxis <= 2);

                // 90-degree rotation around the j-th axis of the sequence
                double[] anAngles = [0.0, 0.0, 0.0];
                anAngles[j] = 0.5 * M_PI;

                Quaternion3 q2 = new();
                q2.SetEulerAngles((EulerSequence)i, anAngles[0], anAngles[1], anAngles[2]);

                // Unit vector on the rotation axis
                Xyz v = new(0.0, 0.0, 0.0);
                v.SetCoord(anAxis + 1, 1.0);

                Transform3D aT = new();
                aT = aT.WithRotation(q2);
                Xyz v2 = v;
                aT.Transforms(ref v2);

                // Rotation around axis should not move a point on that axis
                Assert.True((v - v2).SquareModulus() <= global::Kerf.Engine.Foundation.Precision.SquareConfusion(),
                            $"Sequence: {aNames[i - (int)Extrinsic_XYZ]}, angle index: {j}");
            }
        }
    }

    // Test OCC25574: Extrinsic and intrinsic Euler sequences produce compatible matrices.
    [Fact]
    public void OCC25574_ExtrinsicIntrinsicCorrespondence()
    {
        const double alpha = 0.1517461713131;
        const double beta = 1.5162198410141;
        const double gamma = 1.9313156236541;

        EulerSequence[][] aPairs =
        [
            [Extrinsic_XYZ, Intrinsic_ZYX],
            [Extrinsic_XZY, Intrinsic_YZX],
            [Extrinsic_YZX, Intrinsic_XZY],
            [Extrinsic_YXZ, Intrinsic_ZXY],
            [Extrinsic_ZXY, Intrinsic_YXZ],
            [Extrinsic_ZYX, Intrinsic_XYZ],
        ];

        Quaternion3 aQuat = new();
        for (int i = 0; i < 6; i++)
        {
            aQuat.SetEulerAngles(aPairs[i][0], alpha, beta, gamma);

            aQuat.GetEulerAngles(aPairs[i][1], out double gamma2, out double beta2, out double alpha2);

            Assert.Equal(alpha, alpha2, 1e-5);
            Assert.Equal(beta, beta2, 1e-5);
            Assert.Equal(gamma, gamma2, 1e-5);
        }
    }

    // Test OCC25574 (YawPitchRoll): applying three sequential rotations and recovering
    // angles via gp_YawPitchRoll must reproduce the original angles.
    [Fact]
    public void OCC25574_YawPitchRollRoundTrip()
    {
        Frame3D aWorld = new();
        const double aAlpha = 0.0;
        const double aBeta = -35.0 / 180.0 * M_PI;
        const double aGamma = 90.0 / 180.0 * M_PI;

        // Build the rotated frame step by step (yaw-pitch-roll convention)
        Quaternion3 aRotZ = new(new Vector3D(aWorld.Direction()), aAlpha);
        Vector3D aRotY = aRotZ.Multiply(new Vector3D(aWorld.YDirection()));
        Vector3D aRotX = aRotZ.Multiply(new Vector3D(aWorld.XDirection()));

        Quaternion3 aRotYaw = new(aRotY, aBeta);
        Vector3D aRotZ2 = aRotYaw.Multiply(new Vector3D(aWorld.Direction()));
        Vector3D aRotX2 = aRotYaw.Multiply(aRotX);

        Quaternion3 aRotRoll = new(aRotX2, aGamma);
        Vector3D aRotZ3 = aRotRoll.Multiply(aRotZ2);

        Frame3D aResult = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(aRotZ3), new Direction3D(aRotX2));

        Transform3D aTransformation = new();
        aTransformation = aTransformation.WithDisplacement(new CoordinateSystem(new Frame3D()), new CoordinateSystem(aResult));

        aTransformation.GetRotation().GetEulerAngles(YawPitchRoll,
                                                     out double aComputedAlpha,
                                                     out double aComputedBeta,
                                                     out double aComputedGamma);

        Assert.Equal(aAlpha, aComputedAlpha, 1e-5);
        Assert.Equal(aBeta, aComputedBeta, 1e-5);
        Assert.Equal(aGamma, aComputedGamma, 1e-5);
    }

    // Test OCC25574 (issue 25946): gp_Intrinsic_ZYX and gp_Extrinsic_XYZ yield the
    // same Euler angles in reversed order.
    [Fact]
    public void OCC25574_IntrinsicZYX_vs_ExtrinsicXYZ()
    {
        Quaternion3 aQuat = new();
        aQuat.Set(0.06766916507860499, 0.21848101129786085, 0.11994599260380681, 0.9660744746954637);

        aQuat.GetEulerAngles(Intrinsic_ZYX, out double aAlpha, out double aBeta, out double aGamma);

        aQuat.GetEulerAngles(Extrinsic_XYZ, out double aAlpha2, out double aBeta2, out double aGamma2);

        // Intrinsic ZYX alpha should equal Extrinsic XYZ gamma, and the reverse
        Assert.Equal(aAlpha, aGamma2, 1e-5);
        Assert.Equal(aBeta, aBeta2, 1e-5);
        Assert.Equal(aGamma, aAlpha2, 1e-5);
    }
}
