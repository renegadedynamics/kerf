// Port of: src/FoundationClasses/TKernel/GTests/OSD_Parallel_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The OSD_ThreadPool::Launcher parts of the tests are left out: OSD_ThreadPool isn't ported (decision 487's helper
// replaces it). The functors are lambdas.

using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry.Bvh;
using Box3d = Kerf.Engine.Geometry.Bvh.Box<double, Occt.NCollection_Vec3<double>>;
using BoxSet3d = Kerf.Engine.Geometry.Bvh.BoxSet<double, Occt.NCollection_Vec3<double>>;
using Tree3d = Kerf.Engine.Geometry.Bvh.Tree<double, Occt.NCollection_Vec3<double>>;

namespace Kerf.Kernel.Foundation.Tests;

public class OSD_ParallelTest
{
    // Tests that OSD_Parallel::For produces the same result as sequential SAXPY Y[i] = scalar * X[i] + Y[i] (OCC24826).
    [Fact]
    public void OCC24826_SaxpyParallelMatchesSequential()
    {
        const int aLength = 500000;

        NCollection_Array1<double> aX = new(0, aLength - 1);
        NCollection_Array1<double> anYRef = new(0, aLength - 1);
        for (int i = 0; i < aLength; ++i)
        {
            aX[i] = anYRef[i] = i;
        }

        // Sequential reference
        for (int i = 0; i < aLength; ++i)
        {
            anYRef[i] = 1e-6 * aX[i] + anYRef[i];
        }

        // OSD_Parallel::For
        {
            NCollection_Array1<double> anY = new(0, aLength - 1);
            for (int i = 0; i < aLength; ++i)
            {
                anY[i] = i;
            }
            OsdParallel.For(0, aLength, theIndex => anY[theIndex] = 1e-6 * aX[theIndex] + anY[theIndex]);
            for (int i = 0; i < aLength; ++i)
            {
                Assert.Equal(anYRef[i], anY[i]);
            }
        }

        // OSD_Parallel::For with batched functor
        {
            NCollection_Array1<double> anY = new(0, aLength - 1);
            for (int i = 0; i < aLength; ++i)
            {
                anY[i] = i;
            }
            const int THE_BATCH_SIZE = 100000;
            int aNbBatches = (int)Math.Ceiling((double)aX.Size() / THE_BATCH_SIZE);
            OsdParallel.For(0, aNbBatches, theBatchIndex =>
            {
                int aLower = theBatchIndex * THE_BATCH_SIZE;
                int anUpper = Math.Min(aLower + THE_BATCH_SIZE - 1, aX.Upper());
                for (int i = aLower; i <= anUpper; ++i)
                {
                    anY[i] = 1e-6 * aX[i] + anY[i];
                }
            });
            for (int i = 0; i < aLength; ++i)
            {
                Assert.Equal(anYRef[i], anY[i]);
            }
        }
    }

    // Tests that OSD_Parallel::For produces the same result as sequential matrix multiplication (OCC29935).
    [Fact]
    public void OCC29935_MatrixMultiplyParallelMatchesSequential()
    {
        const int aSize = 50;

        Mt19937 aGen = new(42);
        NCollection_Array2<double> aMat1 = new(0, aSize - 1, 0, aSize - 1);
        NCollection_Array2<double> aMat2 = new(0, aSize - 1, 0, aSize - 1);
        NCollection_Array2<double> aMatRef = new(0, aSize - 1, 0, aSize - 1);
        NCollection_Array2<double> aMatRes = new(0, aSize - 1, 0, aSize - 1);

        for (int i = 0; i < aSize; ++i)
        {
            for (int j = 0; j < aSize; ++j)
            {
                aMat1[i, j] = aGen.Invoke() % 1000;
                aMat2[i, j] = aGen.Invoke() % 1000;
            }
        }

        void multiply(NCollection_Array2<double> theResult, int theIndex)
        {
            for (int j = 0; j < aSize; ++j)
            {
                double aTmp = 0.0;
                for (int k = 0; k < aSize; ++k)
                {
                    aTmp += aMat1[theIndex, k] * aMat2[k, j];
                }
                theResult[theIndex, j] = aTmp;
            }
        }

        // Sequential reference
        for (int i = 0; i < aSize; ++i)
        {
            multiply(aMatRef, i);
        }

        // OSD_Parallel::For
        aMatRes.Init(0.0);
        OsdParallel.For(0, aSize, theIndex => multiply(aMatRes, theIndex));
        for (int i = 0; i < aSize; ++i)
        {
            for (int j = 0; j < aSize; ++j)
            {
                Assert.Equal(aMatRef[i, j], aMatRes[i, j]);
            }
        }
    }
}

/// <summary>Additions: the serial fallback, exceptions, ranges, ForEach, and the BVH loops moved onto the helper.</summary>
public class OSD_ParallelAdditionsTest
{
    [Fact]
    public void ForceSerial_RunsInOrderOnTheCallingThread()
    {
        List<int> anOrder = [];
        int aThread = Environment.CurrentManagedThreadId;
        using (OsdParallel.ForceSerialScope())
        {
            Assert.True(OsdParallel.IsSerial);
            OsdParallel.For(3, 103, theIndex =>
            {
                Assert.Equal(aThread, Environment.CurrentManagedThreadId);
                anOrder.Add(theIndex);
            });
            OsdParallel.ForEach(Enumerable.Range(1000, 50), theItem => anOrder.Add(theItem));
        }
        Assert.Equal([.. Enumerable.Range(3, 100), .. Enumerable.Range(1000, 50)], anOrder);
        Assert.Equal(!OsdParallel.HasThreads, OsdParallel.IsSerial);
    }

    [Fact]
    public void ForceSerialScope_Nests()
    {
        using (OsdParallel.ForceSerialScope())
        {
            using (OsdParallel.ForceSerialScope())
            {
                Assert.True(OsdParallel.IsSerial);
            }
            Assert.True(OsdParallel.IsSerial);
        }
        Assert.Equal(!OsdParallel.HasThreads, OsdParallel.IsSerial);
    }

    [Fact]
    public void SequentialFlag_RunsInOrder()
    {
        List<int> anOrder = [];
        OsdParallel.For(0, 200, anOrder.Add, true);
        OsdParallel.ForEach(new[] { 5, 4, 3 }, anOrder.Add, true);
        Assert.Equal([.. Enumerable.Range(0, 200), 5, 4, 3], anOrder);
    }

    [Fact]
    public void EmptyAndSingleRanges()
    {
        int aCount = 0;
        OsdParallel.For(0, 0, _ => Interlocked.Increment(ref aCount));
        OsdParallel.For(5, 5, _ => Interlocked.Increment(ref aCount));
        OsdParallel.ForEach(Array.Empty<int>(), _ => Interlocked.Increment(ref aCount));
        Assert.Equal(0, aCount);

        int aThread = Environment.CurrentManagedThreadId;
        OsdParallel.For(7, 8, theIndex =>
        {
            Assert.Equal(7, theIndex);
            Assert.Equal(aThread, Environment.CurrentManagedThreadId); // one index: a plain loop, as OCCT
            Interlocked.Increment(ref aCount);
        });
        OsdParallel.ForEach([1], _ =>
        {
            Assert.Equal(aThread, Environment.CurrentManagedThreadId);
            Interlocked.Increment(ref aCount);
        }, false, 1);
        Assert.Equal(2, aCount);
    }

    [Fact]
    public void ParallelVisitsEveryIndexOnce()
    {
        int[] aVisits = new int[10000];
        OsdParallel.For(0, aVisits.Length, theIndex => Interlocked.Increment(ref aVisits[theIndex]));
        Assert.All(aVisits, theCount => Assert.Equal(1, theCount));
        int[] aVisits2 = new int[1000];
        OsdParallel.ForEach(Enumerable.Range(0, aVisits2.Length), theIndex => Interlocked.Increment(ref aVisits2[theIndex]));
        Assert.All(aVisits2, theCount => Assert.Equal(1, theCount));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Exceptions_AreUnwrapped(bool theIsSequential)
    {
        FailureException aFor = EngineAssert.Throws<DomainErrorException>(() =>
            OsdParallel.For(0, 100, theIndex =>
            {
                if (theIndex == 37)
                {
                    throw new DomainErrorException("index 37");
                }
            }, theIsSequential));
        Assert.Equal("index 37", aFor.Message);

        Assert.Throws<InvalidOperationException>(() =>
            OsdParallel.ForEach(Enumerable.Range(0, 100), theIndex =>
            {
                if (theIndex % 10 == 3)
                {
                    throw new InvalidOperationException("item");
                }
            }, theIsSequential));
    }

    [Fact]
    public void NbLogicalProcessors_IsTheProcessorCountInEveryMode()
    {
        int aCount = OsdParallel.LogicalProcessorCount;
        Assert.Equal(Environment.ProcessorCount, aCount);
        using (OsdParallel.ForceSerialScope())
        {
            Assert.Equal(aCount, OsdParallel.LogicalProcessorCount);
        }
    }

    // ---- BVH loops moved onto OSD_Parallel: the serial fallback builds the parallel build's tree

    private static BoxSet3d randomSet(Builder<double, NCollection_Vec3<double>> theBuilder, int theSeed, int theCount)
    {
        Random aRandom = new(theSeed);
        BoxSet3d aBoxSet = new(theBuilder);
        for (int i = 0; i < theCount; ++i)
        {
            double x = aRandom.NextDouble() * 100.0, y = aRandom.NextDouble() * 100.0, z = aRandom.NextDouble() * 100.0;
            aBoxSet.Add(i, new Box3d(new BVH_Vec3d(x, y, z), new BVH_Vec3d(x + aRandom.NextDouble(), y + 1.0, z + 2.0)));
        }
        return aBoxSet;
    }

    private static void assertSameTree(Tree3d theExpected, Tree3d theActual)
    {
        Assert.Equal(theExpected.Length(), theActual.Length());
        for (int i = 0; i < theExpected.Length(); ++i)
        {
            Assert.Equal(theExpected.NodeInfoBuffer()[i], theActual.NodeInfoBuffer()[i]);
            Assert.Equal(theExpected.MinPointBuffer()[i], theActual.MinPointBuffer()[i]);
            Assert.Equal(theExpected.MaxPointBuffer()[i], theActual.MaxPointBuffer()[i]);
        }
    }

    [Fact]
    public void BVH_LinearBuilder_ParallelSerialAndFallbackGiveTheSameTree()
    {
        Tree3d build(bool theIsParallel, bool theIsForcedSerial)
        {
            LinearBuilder<double, NCollection_Vec3<double>> aBuilder = new(4, 32);
            aBuilder.SetParallel(theIsParallel);
            BoxSet3d aSet = randomSet(aBuilder, 11, 5000);
            using (theIsForcedSerial ? OsdParallel.ForceSerialScope() : null)
            {
                aSet.Build();
            }
            return aSet.BVH();
        }

        Tree3d aSerial = build(false, false);
        assertSameTree(aSerial, build(true, false));
        assertSameTree(aSerial, build(true, true));
    }

    [Fact]
    public void BVH_QueueBuilder_FallbackBuildsTheSingleThreadTree()
    {
        Tree3d build(int theNbThreads, bool theIsForcedSerial)
        {
            BinnedBuilder<double, NCollection_Vec3<double>> aBuilder = new(4, 32, false, theNbThreads);
            BoxSet3d aSet = randomSet(aBuilder, 12, 3000);
            using (theIsForcedSerial ? OsdParallel.ForceSerialScope() : null)
            {
                aSet.Build();
            }
            return aSet.BVH();
        }

        Tree3d aSingle = build(1, false);
        // With 4 threads the node order depends on scheduling (as in OCCT); the fallback runs the single-thread build
        assertSameTree(aSingle, build(4, true));
        Assert.Equal(aSingle.Length(), build(4, false).Length());
    }
}
