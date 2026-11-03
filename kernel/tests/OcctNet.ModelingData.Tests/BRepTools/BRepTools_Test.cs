// Port of: src/ModelingData/TKBRep/GTests/BRepTools_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;

using Kerf.Engine.Topology.Tools;

using Kerf.Topology;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's BRepTools GTests: BRep text read on many threads at once gives shapes of identical volume (OCCT's bug was a
/// Strtod memory pool shared by the threads). OSD_Parallel::For is Parallel.For.
/// </summary>
public class BRepTools_Test
{
    // Write a box to a BRep string once, to be reused by all threads.
    private static string MakeBoxBrepString()
    {
        BRepPrimAPI_MakeBox aBox = new(10.0, 20.0, 30.0);
        Assert.False(aBox.Shape().IsNull);

        var anOss = new StringWriter();
        BrepTools.Write(aBox.Shape(), anOss);
        Assert.False(anOss.ToString().Length == 0);
        return anOss.ToString();
    }

    // Read a shape from a BRep buffer. Each call uses its own stream.
    private static bool ReadFromBuffer(string theData, out Shape theShape)
    {
        var anIss = new StringReader(theData);
        BrepTools.Read(out theShape, anIss);
        return !theShape.IsNull;
    }

    private static double AbsMass(in Shape theShape)
    {
        Properties aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return Math.Abs(aProps.Mass());
    }

    [Fact]
    public void ParallelRead_IdenticalMass()
    {
        string aBrepData = MakeBoxBrepString();

        // Single-threaded reference
        Assert.True(ReadFromBuffer(aBrepData, out Shape aRefShape));
        double aRefMass = AbsMass(aRefShape);
        Assert.True(aRefMass > 0.0);

        const int aNbReads = 200;

        var aShapes = new Shape[aNbReads];
        int aFailCount = 0;

        Parallel.For(0, aNbReads, theIndex =>
        {
            if (!ReadFromBuffer(aBrepData, out Shape aShape))
            {
                Interlocked.Increment(ref aFailCount);
                return;
            }
            aShapes[theIndex] = aShape;
        });

        Assert.Equal(0, aFailCount);

        const double aEps = 1.0e-6;
        for (int i = 0; i < aNbReads; ++i)
        {
            Assert.False(aShapes[i].IsNull, $"Shape {i} is null");
            Assert.Equal(aRefMass, AbsMass(aShapes[i]), aEps);
        }
    }

    [Fact]
    public void ParallelRead_RepeatedRuns()
    {
        string aBrepData = MakeBoxBrepString();

        Assert.True(ReadFromBuffer(aBrepData, out Shape aRefShape));
        double aRefMass = AbsMass(aRefShape);

        const int aNbReads = 100;
        const int aNbRuns = 5;
        const double aEps = 1.0e-6;

        for (int aRun = 0; aRun < aNbRuns; ++aRun)
        {
            var aShapes = new Shape[aNbReads];
            int aFailCount = 0;

            Parallel.For(0, aNbReads, theIndex =>
            {
                if (!ReadFromBuffer(aBrepData, out Shape aShape))
                {
                    Interlocked.Increment(ref aFailCount);
                    return;
                }
                aShapes[theIndex] = aShape;
            });

            Assert.Equal(0, aFailCount);

            for (int i = 0; i < aNbReads; ++i)
            {
                Assert.False(aShapes[i].IsNull, $"Run {aRun}, shape {i} is null");
                Assert.Equal(aRefMass, AbsMass(aShapes[i]), aEps);
            }
        }
    }
}
