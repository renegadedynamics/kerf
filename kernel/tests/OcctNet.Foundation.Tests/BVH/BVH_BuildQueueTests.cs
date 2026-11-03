// Port of: src/FoundationClasses/TKMath/GTests/BVH_BuildQueue_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;

namespace OcctNet.Foundation.Tests;

public class BVH_BuildQueueTest
{
    // Basic tests

    [Fact]
    public void DefaultConstructor()
    {
        BuildQueue aQueue = new();

        Assert.Equal(0, aQueue.Size());
        Assert.False(aQueue.HasBusyThreads);
    }

    [Fact]
    public void EnqueueSingle()
    {
        BuildQueue aQueue = new();

        aQueue.Enqueue(42);

        Assert.Equal(1, aQueue.Size());
    }

    [Fact]
    public void EnqueueMultiple()
    {
        BuildQueue aQueue = new();

        aQueue.Enqueue(1);
        aQueue.Enqueue(2);
        aQueue.Enqueue(3);

        Assert.Equal(3, aQueue.Size());
    }

    [Fact]
    public void FetchFromEmptyQueue()
    {
        BuildQueue aQueue = new();
        bool wasBusy = false;
        int aResult = aQueue.Fetch(ref wasBusy);

        Assert.Equal(-1, aResult);
        Assert.False(wasBusy);
        Assert.False(aQueue.HasBusyThreads);
    }

    [Fact]
    public void FetchSingleItem()
    {
        BuildQueue aQueue = new();
        aQueue.Enqueue(42);

        bool wasBusy = false;
        int aResult = aQueue.Fetch(ref wasBusy);

        Assert.Equal(42, aResult);
        Assert.True(wasBusy);
        Assert.True(aQueue.HasBusyThreads); // Thread became busy
        Assert.Equal(0, aQueue.Size());
    }

    [Fact]
    public void FetchFIFOOrder()
    {
        BuildQueue aQueue = new();

        aQueue.Enqueue(10);
        aQueue.Enqueue(20);
        aQueue.Enqueue(30);

        bool wasBusy = false;

        Assert.Equal(10, aQueue.Fetch(ref wasBusy));
        Assert.True(wasBusy);

        Assert.Equal(20, aQueue.Fetch(ref wasBusy));
        Assert.True(wasBusy);

        Assert.Equal(30, aQueue.Fetch(ref wasBusy));
        Assert.True(wasBusy);

        Assert.Equal(-1, aQueue.Fetch(ref wasBusy));
        Assert.False(wasBusy);
    }

    [Fact]
    public void ThreadCountTracking()
    {
        BuildQueue aQueue = new();

        aQueue.Enqueue(1);
        aQueue.Enqueue(2);

        Assert.False(aQueue.HasBusyThreads);

        bool wasBusy = false;

        // First fetch - thread becomes busy
        aQueue.Fetch(ref wasBusy);
        Assert.True(wasBusy);
        Assert.True(aQueue.HasBusyThreads);

        // Second fetch while already busy
        aQueue.Fetch(ref wasBusy);
        Assert.True(wasBusy);
        Assert.True(aQueue.HasBusyThreads);

        // Fetch from empty queue while busy
        aQueue.Fetch(ref wasBusy);
        Assert.False(wasBusy);
        Assert.False(aQueue.HasBusyThreads);
    }

    [Fact]
    public void AlternatingEnqueueFetch()
    {
        BuildQueue aQueue = new();

        aQueue.Enqueue(1);
        Assert.Equal(1, aQueue.Size());

        bool wasBusy = false;
        Assert.Equal(1, aQueue.Fetch(ref wasBusy));
        Assert.Equal(0, aQueue.Size());

        aQueue.Enqueue(2);
        Assert.Equal(1, aQueue.Size());

        Assert.Equal(2, aQueue.Fetch(ref wasBusy));
        Assert.Equal(0, aQueue.Size());
    }

    [Fact]
    public void LargeQueue()
    {
        BuildQueue aQueue = new();

        const int aCount = 1000;
        for (int i = 0; i < aCount; ++i)
        {
            aQueue.Enqueue(i);
        }

        Assert.Equal(aCount, aQueue.Size());

        bool wasBusy = false;
        for (int i = 0; i < aCount; ++i)
        {
            Assert.Equal(i, aQueue.Fetch(ref wasBusy));
        }

        Assert.Equal(0, aQueue.Size());
    }

    [Fact]
    public void NegativeValues()
    {
        BuildQueue aQueue = new();

        aQueue.Enqueue(-1);
        aQueue.Enqueue(-100);
        aQueue.Enqueue(0);

        bool wasBusy = false;

        Assert.Equal(-1, aQueue.Fetch(ref wasBusy));
        Assert.Equal(-100, aQueue.Fetch(ref wasBusy));
        Assert.Equal(0, aQueue.Fetch(ref wasBusy));
    }

    // Thread safety tests

    [Fact]
    public void ConcurrentEnqueue()
    {
        BuildQueue aQueue = new();

        const int aThreadCount = 4;
        const int aItemsPerThread = 100;
        List<Thread> aThreads = [];

        for (int t = 0; t < aThreadCount; ++t)
        {
            int aT = t;
            Thread aThread = new(() =>
            {
                for (int i = 0; i < aItemsPerThread; ++i)
                {
                    aQueue.Enqueue(aT * aItemsPerThread + i);
                }
            });
            aThreads.Add(aThread);
            aThread.Start();
        }

        foreach (Thread aThread in aThreads)
        {
            aThread.Join();
        }

        Assert.Equal(aThreadCount * aItemsPerThread, aQueue.Size());
    }

    [Fact]
    public void ConcurrentFetch()
    {
        BuildQueue aQueue = new();

        const int aItemCount = 400;
        for (int i = 0; i < aItemCount; ++i)
        {
            aQueue.Enqueue(i);
        }

        const int aThreadCount = 4;
        List<Thread> aThreads = [];
        int[] aFetchedCounts = new int[aThreadCount];

        for (int t = 0; t < aThreadCount; ++t)
        {
            int aT = t;
            Thread aThread = new(() =>
            {
                bool wasBusy = false;
                while (true)
                {
                    int aItem = aQueue.Fetch(ref wasBusy);
                    if (aItem == -1)
                    {
                        break;
                    }

                    aFetchedCounts[aT]++;
                }
            });
            aThreads.Add(aThread);
            aThread.Start();
        }

        foreach (Thread aThread in aThreads)
        {
            aThread.Join();
        }

        int aTotalFetched = 0;
        foreach (int aCount in aFetchedCounts)
        {
            aTotalFetched += aCount;
        }

        Assert.Equal(aItemCount, aTotalFetched);
        Assert.Equal(0, aQueue.Size());
    }

    [Fact]
    public void ConcurrentEnqueueAndFetch()
    {
        BuildQueue aQueue = new();

        const int aProducerCount = 2;
        const int aConsumerCount = 2;
        const int aItemsPerProducer = 100;
        List<Thread> aThreads = [];
        int aFetchedCount = 0;
        int aDone = 0;

        // Producer threads
        for (int t = 0; t < aProducerCount; ++t)
        {
            int aT = t;
            aThreads.Add(new Thread(() =>
            {
                for (int i = 0; i < aItemsPerProducer; ++i)
                {
                    aQueue.Enqueue(aT * aItemsPerProducer + i);
                }
            }));
        }

        // Consumer threads
        for (int t = 0; t < aConsumerCount; ++t)
        {
            aThreads.Add(new Thread(() =>
            {
                bool wasBusy = false;
                while (Volatile.Read(ref aDone) == 0 || aQueue.Size() > 0)
                {
                    int aItem = aQueue.Fetch(ref wasBusy);
                    if (aItem != -1)
                    {
                        Interlocked.Increment(ref aFetchedCount);
                    }
                }
            }));
        }

        foreach (Thread aThread in aThreads)
        {
            aThread.Start();
        }

        // Wait for producers to finish
        for (int i = 0; i < aProducerCount; ++i)
        {
            aThreads[i].Join();
        }

        // Signal consumers that production is done
        Volatile.Write(ref aDone, 1);

        // Wait for consumers to finish
        for (int i = aProducerCount; i < aProducerCount + aConsumerCount; ++i)
        {
            aThreads[i].Join();
        }

        Assert.Equal(aProducerCount * aItemsPerProducer, Volatile.Read(ref aFetchedCount));
        Assert.Equal(0, aQueue.Size());
    }

    // Edge cases

    [Fact]
    public void RepeatedFetchFromEmpty()
    {
        BuildQueue aQueue = new();
        bool wasBusy = false;

        for (int i = 0; i < 10; ++i)
        {
            int aResult = aQueue.Fetch(ref wasBusy);
            Assert.Equal(-1, aResult);
            Assert.False(wasBusy);
        }

        Assert.False(aQueue.HasBusyThreads);
    }

    [Fact]
    public void EnqueueZero()
    {
        BuildQueue aQueue = new();

        aQueue.Enqueue(0);
        Assert.Equal(1, aQueue.Size());

        bool wasBusy = false;
        Assert.Equal(0, aQueue.Fetch(ref wasBusy));
        Assert.True(wasBusy);
    }

    [Fact]
    public void DuplicateValues()
    {
        BuildQueue aQueue = new();

        aQueue.Enqueue(5);
        aQueue.Enqueue(5);
        aQueue.Enqueue(5);

        Assert.Equal(3, aQueue.Size());

        bool wasBusy = false;
        Assert.Equal(5, aQueue.Fetch(ref wasBusy));
        Assert.Equal(5, aQueue.Fetch(ref wasBusy));
        Assert.Equal(5, aQueue.Fetch(ref wasBusy));
    }

    [Fact]
    public void SingleThreadWorkflow()
    {
        BuildQueue aQueue = new();

        // Simulate single-threaded BVH building workflow
        aQueue.Enqueue(0); // Root node

        bool wasBusy = false;
        int aNode = aQueue.Fetch(ref wasBusy);

        Assert.Equal(0, aNode);
        Assert.True(wasBusy);

        // After processing root, enqueue children
        aQueue.Enqueue(1); // Left child
        aQueue.Enqueue(2); // Right child

        Assert.Equal(2, aQueue.Size());

        aNode = aQueue.Fetch(ref wasBusy);
        Assert.Equal(1, aNode);

        aNode = aQueue.Fetch(ref wasBusy);
        Assert.Equal(2, aNode);

        aNode = aQueue.Fetch(ref wasBusy);
        Assert.Equal(-1, aNode);
        Assert.False(wasBusy);
    }
}
