// Port of: src/FoundationClasses/TKMath/GTests/gp_Quaternion_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_EulerSequence;

namespace OcctNet.Foundation.Tests;

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
        gp_Quaternion aQuat = new();
        aQuat.Set(0.06766916507860499, 0.21848101129786085, 0.11994599260380681, 0.9660744746954637);

        gp_Mat aRinv = aQuat.GetMatrix().Inverted();
        gp_Mat aI = new();
        aI.SetIdentity();

        // Check round-trip consistency: GetEulerAngles then SetEulerAngles should reproduce the matrix
        for (int i = (int)gp_Extrinsic_XYZ; i <= (int)gp_Intrinsic_ZYZ; i++)
        {
            aQuat.GetEulerAngles((gp_EulerSequence)i, out double alpha, out double beta, out double gamma);

            gp_Quaternion aQuat2 = new();
            aQuat2.SetEulerAngles((gp_EulerSequence)i, alpha, beta, gamma);

            gp_Mat aR = aQuat2.GetMatrix();
            gp_Mat aDiff = aR * aRinv - aI;
            // Round-trip failed for Euler sequence aNames[i - gp_Extrinsic_XYZ]
            Assert.True(aDiff.Determinant() <= 1e-5, aNames[i - (int)gp_Extrinsic_XYZ]);
        }
    }

    // Test OCC25574: Each Euler angle rotation around a named axis should not change
    // a point lying on that axis.
    [Fact]
    public void OCC25574_EulerAxisRotationPreservesAxis()
    {
        for (int i = (int)gp_Extrinsic_XYZ; i <= (int)gp_Intrinsic_ZYZ; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // Determine axis index from the sequence name (X=0, Y=1, Z=2)
                int anAxis = aNames[i - (int)gp_Extrinsic_XYZ][10 + j] - 'X';
                Assert.True(anAxis >= 0);
                Assert.True(anAxis <= 2);

                // 90-degree rotation around the j-th axis of the sequence
                double[] anAngles = [0.0, 0.0, 0.0];
                anAngles[j] = 0.5 * M_PI;

                gp_Quaternion q2 = new();
                q2.SetEulerAngles((gp_EulerSequence)i, anAngles[0], anAngles[1], anAngles[2]);

                // Unit vector on the rotation axis
                gp_XYZ v = new(0.0, 0.0, 0.0);
                v.SetCoord(anAxis + 1, 1.0);

                gp_Trsf aT = new();
                aT.SetRotation(q2);
                gp_XYZ v2 = v;
                aT.Transforms(ref v2);

                // Rotation around axis should not move a point on that axis
                Assert.True((v - v2).SquareModulus() <= Precision.SquareConfusion(),
                            $"Sequence: {aNames[i - (int)gp_Extrinsic_XYZ]}, angle index: {j}");
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

        gp_EulerSequence[][] aPairs =
        [
            [gp_Extrinsic_XYZ, gp_Intrinsic_ZYX],
            [gp_Extrinsic_XZY, gp_Intrinsic_YZX],
            [gp_Extrinsic_YZX, gp_Intrinsic_XZY],
            [gp_Extrinsic_YXZ, gp_Intrinsic_ZXY],
            [gp_Extrinsic_ZXY, gp_Intrinsic_YXZ],
            [gp_Extrinsic_ZYX, gp_Intrinsic_XYZ],
        ];

        gp_Quaternion aQuat = new();
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
        gp_Ax2 aWorld = new();
        const double aAlpha = 0.0;
        const double aBeta = -35.0 / 180.0 * M_PI;
        const double aGamma = 90.0 / 180.0 * M_PI;

        // Build the rotated frame step by step (yaw-pitch-roll convention)
        gp_Quaternion aRotZ = new(new gp_Vec(aWorld.Direction()), aAlpha);
        gp_Vec aRotY = aRotZ.Multiply(new gp_Vec(aWorld.YDirection()));
        gp_Vec aRotX = aRotZ.Multiply(new gp_Vec(aWorld.XDirection()));

        gp_Quaternion aRotYaw = new(aRotY, aBeta);
        gp_Vec aRotZ2 = aRotYaw.Multiply(new gp_Vec(aWorld.Direction()));
        gp_Vec aRotX2 = aRotYaw.Multiply(aRotX);

        gp_Quaternion aRotRoll = new(aRotX2, aGamma);
        gp_Vec aRotZ3 = aRotRoll.Multiply(aRotZ2);

        gp_Ax2 aResult = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(aRotZ3), new gp_Dir(aRotX2));

        gp_Trsf aTransformation = new();
        aTransformation.SetDisplacement(new gp_Ax3(new gp_Ax2()), new gp_Ax3(aResult));

        aTransformation.GetRotation().GetEulerAngles(gp_YawPitchRoll,
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
        gp_Quaternion aQuat = new();
        aQuat.Set(0.06766916507860499, 0.21848101129786085, 0.11994599260380681, 0.9660744746954637);

        aQuat.GetEulerAngles(gp_Intrinsic_ZYX, out double aAlpha, out double aBeta, out double aGamma);

        aQuat.GetEulerAngles(gp_Extrinsic_XYZ, out double aAlpha2, out double aBeta2, out double aGamma2);

        // Intrinsic ZYX alpha should equal Extrinsic XYZ gamma, and the reverse
        Assert.Equal(aAlpha, aGamma2, 1e-5);
        Assert.Equal(aBeta, aBeta2, 1e-5);
        Assert.Equal(aGamma, aAlpha2, 1e-5);
    }
}
