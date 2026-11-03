// Port of: src/FoundationClasses/TKMath/BVH/BVH_BuildQueue.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>Command queue for parallel building of BVH nodes.</summary>
internal sealed class BuildQueue
{
    /// <summary>Queue of BVH nodes to build.</summary>
    private readonly NCollection_Sequence<int> _queue = new();

    /// <summary>Serializes queue operations (the queue builder takes it too, as OCCT's friend does).</summary>
    internal readonly Lock _mutex = new();

    /// <summary>Number of active build threads.</summary>
    private int _nbThreads;

    /// <summary>Current queue size.</summary>
    private int _size;

    /// <summary>Creates a new BVH build queue.</summary>
    public BuildQueue()
    {
        _nbThreads = 0;
        _size = 0;
    }

    /// <summary>The current size of the queue.</summary>
    public int Size() => Volatile.Read(ref _size);

    /// <summary>Enqueues a new work item.</summary>
    public void Enqueue(int workItem)
    {
        lock (_mutex)
        {
            _queue.Append(workItem);
            Interlocked.Increment(ref _size);
        }
    }

    /// <summary>
    /// Fetches the first work item, or -1 when the queue is empty, and keeps the count of busy threads:
    /// <paramref name="wasBusy"/> says whether the calling thread had an item before and is set to whether it has one now.
    /// </summary>
    public int Fetch(ref bool wasBusy)
    {
        int query = -1;

        // Fetch item from queue under lock
        lock (_mutex)
        {
            if (!_queue.IsEmpty())
            {
                query = _queue.First();
                _queue.Remove(1);
                Interlocked.Decrement(ref _size);
            }
        }

        // Update thread counter atomically
        if (query != -1)
        {
            if (!wasBusy)
            {
                Interlocked.Increment(ref _nbThreads);
            }
        }
        else if (wasBusy)
        {
            Interlocked.Decrement(ref _nbThreads);
        }

        wasBusy = (query != -1);
        return query;
    }

    /// <summary>True if there are active build threads (used to detect the end of the work).</summary>
    public bool HasBusyThreads => Volatile.Read(ref _nbThreads) != 0;
}
