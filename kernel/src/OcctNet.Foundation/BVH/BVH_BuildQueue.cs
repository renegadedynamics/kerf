// Port of: src/FoundationClasses/TKMath/BVH/BVH_BuildQueue.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Command queue for parallel building of BVH nodes.</summary>
public sealed class BVH_BuildQueue
{
    /// <summary>Queue of BVH nodes to build.</summary>
    private readonly NCollection_Sequence<int> myQueue = new();

    /// <summary>Serializes queue operations (the queue builder takes it too, as OCCT's friend does).</summary>
    internal readonly Lock myMutex = new();

    /// <summary>Number of active build threads.</summary>
    private int myNbThreads;

    /// <summary>Current queue size.</summary>
    private int mySize;

    /// <summary>Creates a new BVH build queue.</summary>
    public BVH_BuildQueue()
    {
        myNbThreads = 0;
        mySize = 0;
    }

    /// <summary>The current size of the queue.</summary>
    public int Size() => Volatile.Read(ref mySize);

    /// <summary>Enqueues a new work item.</summary>
    public void Enqueue(int theWorkItem)
    {
        lock (myMutex)
        {
            myQueue.Append(theWorkItem);
            Interlocked.Increment(ref mySize);
        }
    }

    /// <summary>
    /// Fetches the first work item, or -1 when the queue is empty, and keeps the count of busy threads:
    /// <paramref name="wasBusy"/> says whether the calling thread had an item before and is set to whether it has one now.
    /// </summary>
    public int Fetch(ref bool wasBusy)
    {
        int aQuery = -1;

        // Fetch item from queue under lock
        lock (myMutex)
        {
            if (!myQueue.IsEmpty())
            {
                aQuery = myQueue.First();
                myQueue.Remove(1);
                Interlocked.Decrement(ref mySize);
            }
        }

        // Update thread counter atomically
        if (aQuery != -1)
        {
            if (!wasBusy)
            {
                Interlocked.Increment(ref myNbThreads);
            }
        }
        else if (wasBusy)
        {
            Interlocked.Decrement(ref myNbThreads);
        }

        wasBusy = (aQuery != -1);
        return aQuery;
    }

    /// <summary>True if there are active build threads (used to detect the end of the work).</summary>
    public bool HasBusyThreads() => Volatile.Read(ref myNbThreads) != 0;
}
