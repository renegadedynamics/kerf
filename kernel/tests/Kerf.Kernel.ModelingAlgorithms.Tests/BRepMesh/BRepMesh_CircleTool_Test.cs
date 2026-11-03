// Port of: src/ModelingAlgorithms/TKMesh/GTests/BRepMesh_CircleTool_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

public class BRepMesh_CircleTool_Test
{
    // C's srand/rand as MSVC's CRT implements them (RAND_MAX 32767), so the test draws OCCT's points.
    private sealed class MsvcRand(uint theSeed)
    {
        public const int RAND_MAX = 0x7fff;
        private uint mySeed = theSeed;

        public int rand()
        {
            mySeed = unchecked(mySeed * 214013u + 2531011u);
            return (int)((mySeed >> 16) & 0x7fff);
        }
    }

    // Returns true if thePoint lies on the circle defined by theCenter and theRadius.
    private static bool isOnCircle(in Xy thePoint, in Xy theCenter, double theRadius)
    {
        double aSqPrec = global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion();
        Xy aDiff = thePoint - theCenter;
        return aDiff.SquareModulus() - theRadius * theRadius < aSqPrec;
    }

    // OCC24923: BRepMesh_CircleTool produces bad circles
    // For a large set of random non-degenerate triangles, every circumscribed circle
    // computed by BRepMesh_CircleTool::MakeCircle must pass through all three input points.
    [Fact]
    public void OCC24923_CircumCirclePassesThroughAllVertices()
    {
        MsvcRand aRand = new(42);

        double aSqPrec = global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion();
        double aMinArea = 5.0 * Math.PI / 180.0; // same threshold as the original command
        const int aNbTests = 100000;

        int aNbFailed = 0;
        int i = 0;
        Xy[] p = new Xy[3];
        while (i < aNbTests)
        {
            for (int j = 0; j < 3; ++j)
            {
                double aX = (double)aRand.rand() / MsvcRand.RAND_MAX;
                double aY = (double)aRand.rand() / MsvcRand.RAND_MAX;
                p[j] = new Xy(aX, aY);
            }

            // Skip degenerate (nearly collinear) triangles - retry like the original.
            Xy aV1 = p[1] - p[0];
            Xy aV2 = p[2] - p[0];
            if (aV1.SquareModulus() <= aSqPrec || aV2.SquareModulus() <= aSqPrec || (aV1 ^ aV2) <= aMinArea)
            {
                continue;
            }

            ++i;

            if (!CircleTool.MakeCircle(p[0], p[1], p[2], out Xy aCenter, out double aRadius))
            {
                continue;
            }

            if (!isOnCircle(p[0], aCenter, aRadius) || !isOnCircle(p[1], aCenter, aRadius)
                || !isOnCircle(p[2], aCenter, aRadius))
            {
                ++aNbFailed;
            }
        }

        // Allow at most 1% failure rate (same threshold as the original Draw test).
        double aFailRate = (double)aNbFailed / aNbTests;
        Assert.True(aFailRate <= 0.01, $"Too many bad circumscribed circles: {aNbFailed} / {aNbTests}");
    }
}
