// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// TopLoc_Location against native OCCT: random chains of compositions, inversions, divisions and powers over a
/// few shared datums. Each step is done on both sides, so native and port locations correspond one to one, and
/// the comparisons cover the transformation, the chain of powers, and the identity-based equality between any two
/// locations.
/// </summary>
public class TopLocOracleTests(ITestOutputHelper output)
{
    private static readonly int Rounds = Tape.Cases(40, 16);
    private const int Steps = 60;

    // Powers and long chains amplify rounding, as gp_Trsf powers do.
    private const double ChainTolerance = 1e-9;

    [Fact]
    public void Location_algebra()
    {
        var anOracle = new Oracle(output);
        var aRandom = new Random(20261028);
        for (int aRound = 0; aRound < Rounds; aRound++)
        {
            var aNatives = new List<N.TopLoc_Location>();
            var aPorts = new List<Placement>();
            try
            {
                // A few elementary locations, each with its own datum. A location also appears twice through a
                // copy, which shares its datum.
                int aBase = anOracle.Int(2, 4);
                for (int i = 0; i < aBase; i++)
                {
                    (N.gp_Trsf aNT, Transform3D aT) = RandomRigidTrsf(anOracle);
                    using (aNT)
                    {
                        aNatives.Add(new N.TopLoc_Location(aNT));
                        aPorts.Add(new Placement(aT));
                    }
                }

                aNatives.Add(new N.TopLoc_Location(aNatives[0]));
                aPorts.Add(aPorts[0]);
                aNatives.Add(new N.TopLoc_Location());
                aPorts.Add(new Placement());

                for (int aStep = 0; aStep < Steps; aStep++)
                {
                    int a = aRandom.Next(aPorts.Count), b = aRandom.Next(aPorts.Count);
                    N.TopLoc_Location aNA = aNatives[a], aNB = aNatives[b];
                    Placement aA = aPorts[a], aB = aPorts[b];
                    switch (aRandom.Next(6))
                    {
                        case 0:
                            aNatives.Add(aNA.Multiplied(aNB));
                            aPorts.Add(aA * aB);
                            break;
                        case 1:
                            aNatives.Add(aNA.Inverted());
                            aPorts.Add(aA.Inverted());
                            break;
                        case 2:
                            aNatives.Add(aNA.Divided(aNB));
                            aPorts.Add(aA / aB);
                            break;
                        case 3:
                            aNatives.Add(aNA.Predivided(aNB));
                            aPorts.Add(aA.Predivided(aB));
                            break;
                        case 4:
                        {
                            int aPower = aRandom.Next(-3, 4);
                            aNatives.Add(aNA.Powered(aPower));
                            aPorts.Add(aA.Powered(aPower));
                            break;
                        }
                        default:
                            // a product that cancels, at least partly
                            aNatives.Add(aNA.Multiplied(aNB).Multiplied(aNA.Inverted()));
                            aPorts.Add(aA * aB * aA.Inverted());
                            break;
                    }

                    // Keep the chains short enough for the tolerance
                    if (Length(aPorts[^1]) > 12)
                    {
                        aNatives[^1].Dispose();
                        aNatives.RemoveAt(aNatives.Count - 1);
                        aPorts.RemoveAt(aPorts.Count - 1);
                    }
                }

                for (int i = 0; i < aPorts.Count; i++)
                {
                    CompareLocation(anOracle, aNatives[i], aPorts[i], $"round {aRound}, location {i}");
                }

                // The engine's equality is by datum identity (the public == compares the transformations, decision
                // 1349, 62): native and port must agree on every pair.
                for (int i = 0; i < aPorts.Count; i++)
                {
                    for (int j = 0; j < aPorts.Count; j++)
                    {
                        bool aNativeEqual = aNatives[i].IsEqual(aNatives[j]);
                        anOracle.Same(aNativeEqual, aPorts[i].IsEqual(aPorts[j]), $"round {aRound}: IsEqual({i}, {j})");
                        anOracle.Same(aNatives[i].IsDifferent(aNatives[j]), aPorts[i].IsDifferent(aPorts[j]), $"round {aRound}: IsDifferent({i}, {j})");
                        if (aNativeEqual)
                        {
                            Assert.Equal(aNatives[i].HashCode(), aNatives[j].HashCode());
                            Assert.Equal(aPorts[i].GetHashCode(), aPorts[j].GetHashCode());
                        }
                    }
                }
            }
            finally
            {
                foreach (N.TopLoc_Location aNative in aNatives)
                {
                    aNative.Dispose();
                }
            }
        }

        anOracle.Report("TopLoc_Location algebra");
    }

    [Fact]
    public void Location_of_a_transformation()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < 500; i++)
        {
            (N.gp_Trsf aNT, Transform3D aT) = RandomRigidTrsf(anOracle);
            using (aNT)
            {
                using var aNL = new N.TopLoc_Location(aNT);
                var aL = new Placement(aT);
                CompareLocation(anOracle, aNL, aL, "location");

                int aPower = anOracle.Int(-5, 5);
                using var aNPow = aNL.Powered(aPower);
                CompareLocation(anOracle, aNPow, aL.Powered(aPower), $"location to the power {aPower}");

                using var aNInv = aNL.Inverted();
                CompareLocation(anOracle, aNInv, aL.Inverted(), "inverted location");

                // Applying the location to a point
                Point3D aP = anOracle.Pnt();
                using var aNP = Oracle.ToNative(aP);
                using var aNTrsf = aNPow.Transformation();
                using var aNMoved = aNP.Transformed(aNTrsf);
                anOracle.Same(Oracle.FromNative(aNMoved).XYZ(), aP.Transformed(aL.Powered(aPower).Transformation).XYZ(),
                              "point moved by a location", ChainTolerance);
            }
        }

        anOracle.Report("TopLoc_Location of a transformation");
    }

    private static int Length(in Placement theLocation)
    {
        int aLength = 0;
        for (Placement aL = theLocation; !aL.IsIdentity; aL = aL.NextLocation())
        {
            aLength++;
        }

        return aLength;
    }

    /// <summary>Compares the transformations and the chains of powers of two locations.</summary>
    private static void CompareLocation(Oracle theOracle, N.TopLoc_Location theNative, in Placement thePort, string theWhat)
    {
        using (N.gp_Trsf aNTrsf = theNative.Transformation())
        {
            theOracle.Same(aNTrsf, thePort.Transformation, theWhat, ChainTolerance);
        }

        N.TopLoc_Location aNative = new N.TopLoc_Location(theNative);
        Placement aPort = thePort;
        try
        {
            while (true)
            {
                theOracle.Same(aNative.IsIdentity(), aPort.IsIdentity, $"{theWhat}: IsIdentity");
                if (aPort.IsIdentity)
                {
                    break;
                }

                Assert.Equal(aNative.FirstPower(), aPort.FirstPower());
                N.TopLoc_Location aNext = aNative.NextLocation();
                aNative.Dispose();
                aNative = aNext;
                aPort = aPort.NextLocation();
            }
        }
        finally
        {
            aNative.Dispose();
        }
    }

    /// <summary>A random rigid transformation (a datum's transformation), with its native twin.</summary>
    private static (N.gp_Trsf Native, Transform3D Port) RandomRigidTrsf(Oracle theOracle)
    {
        var aNative = new N.gp_Trsf();
        switch (theOracle.Int(0, 3))
        {
            case 0:
            {
                Axis3D anAxis = theOracle.Ax1();
                double anAngle = theOracle.Angle();
                using var aNAxis = Oracle.ToNative(anAxis);
                aNative.SetRotation(aNAxis, anAngle);
                return (aNative, Transform3D.Rotation(anAxis, anAngle));
            }
            case 1:
            {
                Vector3D aV = theOracle.Vec();
                using var aNV = Oracle.ToNative(aV);
                aNative.SetTranslation(aNV);
                return (aNative, Transform3D.Translation(aV));
            }
            case 2:
            {
                CoordinateSystem aFrom = theOracle.Ax3(), aTo = theOracle.Ax3();
                using var aNFrom = Oracle.ToNative(aFrom);
                using var aNTo = Oracle.ToNative(aTo);
                aNative.SetDisplacement(aNFrom, aNTo);
                return (aNative, Transform3D.Displacement(aFrom, aTo));
            }
            default:
            {
                Frame3D aPlane = theOracle.Ax2();
                using var aNPlane = Oracle.ToNative(aPlane);
                aNative.SetMirror(aNPlane);
                return (aNative, Transform3D.Mirror(aPlane));
            }
        }
    }
}
