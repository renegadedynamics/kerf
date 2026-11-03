// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>The gp package against native OCCT, on random inputs.</summary>
public class GpOracleTests(ITestOutputHelper output)
{
    private const int Cases = 500;

    [Fact]
    public void Dir_angles()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            Direction3D aD1 = anOracle.Dir(), aD2 = anOracle.Dir(), aRef = anOracle.Dir();
            using var aN1 = Oracle.ToNative(aD1);
            using var aN2 = Oracle.ToNative(aD2);
            using var aNRef = Oracle.ToNative(aRef);
            anOracle.Same(aN1.Angle(aN2), aD1.Angle(aD2), "Angle");
            anOracle.Same(aN1.AngleWithRef(aN2, aNRef), aD1.AngleWithRef(aD2, aRef), "AngleWithRef");
        }

        anOracle.Report("gp_Dir angles");
    }

    [Fact]
    public void Ax2_and_Ax3_construction()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            Point3D aP = anOracle.Pnt();
            Direction3D aV = anOracle.Dir();
            using var aNP = Oracle.ToNative(aP);
            using var aNV = Oracle.ToNative(aV);

            // OCCT's own choice of X direction from a main direction alone
            using var aNAx2 = new N.gp_Ax2(aNP, aNV);
            var anAx2 = new Frame3D(aP, aV);
            using (var aNX = aNAx2.XDirection())
            using (var aNY = aNAx2.YDirection())
            {
                anOracle.Same(Oracle.FromNative(aNX.XYZ()), anAx2.XDirection().XYZ(), "Ax2 X");
                anOracle.Same(Oracle.FromNative(aNY.XYZ()), anAx2.YDirection().XYZ(), "Ax2 Y");
            }

            using var aNAx3 = new N.gp_Ax3(aNP, aNV);
            var anAx3 = new CoordinateSystem(aP, aV);
            using (var aNX = aNAx3.XDirection())
            {
                anOracle.Same(Oracle.FromNative(aNX.XYZ()), anAx3.XDirection().XYZ(), "Ax3 X");
            }

            // Changing the main direction of a possibly left-handed system
            CoordinateSystem aSystem = anOracle.Ax3();
            Direction3D aNewDir = anOracle.Dir();
            using var aNSystem = Oracle.ToNative(aSystem);
            using var aNNewDir = Oracle.ToNative(aNewDir);
            aNSystem.SetDirection(aNNewDir);
            CoordinateSystem aChanged = aSystem.WithDirection(aNewDir);
            anOracle.Same(aNSystem.Direct(), aChanged.Direct(), "Direct");
            using (var aNX = aNSystem.XDirection())
            using (var aNY = aNSystem.YDirection())
            {
                anOracle.Same(Oracle.FromNative(aNX.XYZ()), aChanged.XDirection().XYZ(), "SetDirection X");
                anOracle.Same(Oracle.FromNative(aNY.XYZ()), aChanged.YDirection().XYZ(), "SetDirection Y");
            }
        }

        anOracle.Report("gp_Ax2/gp_Ax3");
    }

    [Fact]
    public void Trsf_construction_and_application()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Trsf aNative, Transform3D aPort, string aKind) = RandomTrsf(anOracle);
            using (aNative)
            {
                anOracle.Same(aNative, aPort, aKind);

                Point3D aP = anOracle.Pnt();
                using var aNP = Oracle.ToNative(aP);
                using var aNTransformed = aNP.Transformed(aNative);
                anOracle.Same(Oracle.FromNative(aNTransformed).XYZ(), aP.Transformed(aPort).XYZ(), $"{aKind} applied to a point");

                Direction3D aD = anOracle.Dir();
                using var aND = Oracle.ToNative(aD);
                using var aNDTransformed = aND.Transformed(aNative);
                using var aNDXYZ = aNDTransformed.XYZ();
                anOracle.Same(Oracle.FromNative(aNDXYZ), aD.Transformed(aPort).XYZ(), $"{aKind} applied to a direction");
            }
        }

        anOracle.Report("gp_Trsf construction and application");
    }

    [Fact]
    public void Trsf_composition()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.gp_Trsf aN1, Transform3D aT1, string aKind1) = RandomTrsf(anOracle);
            (N.gp_Trsf aN2, Transform3D aT2, string aKind2) = RandomTrsf(anOracle);
            using (aN1)
            using (aN2)
            {
                using var aNMul = aN1.Multiplied(aN2);
                anOracle.Same(aNMul, aT1.Multiplied(aT2), $"{aKind1} * {aKind2}");

                using var aNPre = new N.gp_Trsf(aN1);
                aNPre.PreMultiply(aN2);
                anOracle.Same(aNPre, aT1.PreMultiplied(aT2), $"{aKind2} * {aKind1} (PreMultiply)");

                using var aNInv = aN1.Inverted();
                anOracle.Same(aNInv, aT1.Inverted(), $"{aKind1} inverted");

                int aPower = anOracle.Int(-4, 5);
                using var aNPow = aN1.Powered(aPower);
                if (aT1.Form() == TransformForm.CompoundTrsf && aT1.ScaleFactor() != 1.0 && Math.Abs(aPower) >= 3)
                {
                    SamePowerOfScaled(anOracle, aNPow, aT1, aPower, $"{aKind1} to the power {aPower}");
                }
                else
                {
                    anOracle.Same(aNPow, aT1.Powered(aPower), $"{aKind1} to the power {aPower}", 1e-10);
                }
            }
        }

        anOracle.Report("gp_Trsf composition");
    }

    // Plan 3.2's third level (occt-issues.md, gp_Trsf::Power's compound branch, unit 10K3's row; Kerf decision 16802):
    // OCCT squares a scaled transformation's scale before scaling the square's translation with it, so from the cube
    // on its translation is wrong. The form and the matrix are compared with native's; the translation with the
    // transformation applied |N| times (to the origin), native's read as recorded.
    private static void SamePowerOfScaled(Oracle theOracle, N.gp_Trsf theNative, in Transform3D theT, int thePower, string theWhat)
    {
        Transform3D aPort = theT.Powered(thePower);
        Transform3D aStep = thePower > 0 ? theT : theT.Inverted();
        var aMoved = new Point3D();
        for (int k = 0; k < Math.Abs(thePower); k++)
        {
            aMoved = aMoved.Transformed(aStep);
        }
        Assert.Equal((int)theNative.Form(), (int)aPort.Form());
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                if (j < 4)
                {
                    theOracle.Same(theNative.Value(i, j), aPort.Value(i, j), $"{theWhat}({i},{j})", 1e-10);
                }
                else
                {
                    _ = theNative.Value(i, j);
                    Assert.Equal(aMoved.Coord(i), aPort.Value(i, j), 1e-9 * Math.Max(1.0, Math.Abs(aMoved.Coord(i))));
                }
            }
        }
    }

    [Fact]
    public void Quaternion_matrix_and_Euler_angles()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            Vector3D anAxis = anOracle.Vec();
            double anAngle = anOracle.Angle();
            using var aNAxis = Oracle.ToNative(anAxis);
            using var aNQ = new N.gp_Quaternion(aNAxis, anAngle);
            var aQ = new Quaternion3(anAxis, anAngle);

            using var aNMat = aNQ.GetMatrix();
            anOracle.Same(Oracle.FromNative(aNMat), aQ.GetMatrix(), "GetMatrix");

            // and back from the matrix, through each branch of SetMatrix
            using var aNFromMat = new N.gp_Quaternion(aNMat);
            var aFromMat = new Quaternion3(aQ.GetMatrix());
            anOracle.Same(aNFromMat.X(), aFromMat.X(), "SetMatrix x");
            anOracle.Same(aNFromMat.W(), aFromMat.W(), "SetMatrix w");

            var aSequence = (EulerSequence)anOracle.Int(0, 25);
            double aNA = 0, aNB = 0, aNG = 0;
            aNQ.GetEulerAngles((N.gp_EulerSequence)(int)aSequence, ref aNA, ref aNB, ref aNG);
            aQ.GetEulerAngles(aSequence, out double anA, out double aB, out double aG);
            anOracle.Same(aNA, anA, $"{aSequence} alpha", 1e-9);
            anOracle.Same(aNB, aB, $"{aSequence} beta", 1e-9);
            anOracle.Same(aNG, aG, $"{aSequence} gamma", 1e-9);

            using var aNEuler = new N.gp_Quaternion();
            aNEuler.SetEulerAngles((N.gp_EulerSequence)(int)aSequence, anA, aB, aG);
            var anEuler = Quaternion3.FromEulerAngles(aSequence, anA, aB, aG);
            anOracle.Same(aNEuler.X(), anEuler.X(), "SetEulerAngles x");
            anOracle.Same(aNEuler.Y(), anEuler.Y(), "SetEulerAngles y");
            anOracle.Same(aNEuler.Z(), anEuler.Z(), "SetEulerAngles z");
            anOracle.Same(aNEuler.W(), anEuler.W(), "SetEulerAngles w");
        }

        anOracle.Report("gp_Quaternion");
    }

    /// <summary>One of gp_Trsf's kinds, made the same way natively and in the port.</summary>
    private static (N.gp_Trsf Native, Transform3D Port, string Kind) RandomTrsf(Oracle theOracle)
    {
        var aNative = new N.gp_Trsf();
        switch (theOracle.Int(0, 8))
        {
            case 0:
            {
                Axis3D anAxis = theOracle.Ax1();
                double anAngle = theOracle.Angle();
                using var aNAxis = Oracle.ToNative(anAxis);
                aNative.SetRotation(aNAxis, anAngle);
                return (aNative, Transform3D.Rotation(anAxis, anAngle), "rotation");
            }
            case 1:
            {
                Vector3D aV = theOracle.Vec();
                using var aNV = Oracle.ToNative(aV);
                aNative.SetTranslation(aNV);
                return (aNative, Transform3D.Translation(aV), "translation");
            }
            case 2:
            {
                Point3D aP = theOracle.Pnt();
                double aS = theOracle.Real(-3, 3);
                using var aNP = Oracle.ToNative(aP);
                aNative.SetScale(aNP, aS);
                return (aNative, Transform3D.Scale(aP, aS), "scale");
            }
            case 3:
            {
                Point3D aP = theOracle.Pnt();
                using var aNP = Oracle.ToNative(aP);
                aNative.SetMirror(aNP);
                return (aNative, Transform3D.Mirror(aP), "point mirror");
            }
            case 4:
            {
                Axis3D anAxis = theOracle.Ax1();
                using var aNAxis = Oracle.ToNative(anAxis);
                aNative.SetMirror(aNAxis);
                return (aNative, Transform3D.Mirror(anAxis), "axis mirror");
            }
            case 5:
            {
                Frame3D aPlane = theOracle.Ax2();
                using var aNPlane = Oracle.ToNative(aPlane);
                aNative.SetMirror(aNPlane);
                return (aNative, Transform3D.Mirror(aPlane), "plane mirror");
            }
            case 6:
            {
                CoordinateSystem aFrom = theOracle.Ax3(), aTo = theOracle.Ax3();
                using var aNFrom = Oracle.ToNative(aFrom);
                using var aNTo = Oracle.ToNative(aTo);
                aNative.SetTransformation(aNFrom, aNTo);
                return (aNative, Transform3D.Transformation(aFrom, aTo), "transformation");
            }
            case 7:
            {
                CoordinateSystem aFrom = theOracle.Ax3(), aTo = theOracle.Ax3();
                using var aNFrom = Oracle.ToNative(aFrom);
                using var aNTo = Oracle.ToNative(aTo);
                aNative.SetDisplacement(aNFrom, aNTo);
                return (aNative, Transform3D.Displacement(aFrom, aTo), "displacement");
            }
            default:
            {
                // A general matrix: SetValues extracts the scale and orthogonalises
                var aRotation = Transform3D.Rotation(theOracle.Ax1(), theOracle.Angle()).WithScaleFactor(theOracle.Real(0.5, 2));
                double[] a = new double[12];
                for (int r = 1; r <= 3; r++)
                {
                    for (int c = 1; c <= 4; c++)
                    {
                        a[(r - 1) * 4 + (c - 1)] = aRotation.Value(r, c) + theOracle.Real(-1e-3, 1e-3);
                    }
                }

                aNative.SetValues(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11]);
                return (aNative, Transform3D.Values(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11]), "values");
            }
        }
    }
}
