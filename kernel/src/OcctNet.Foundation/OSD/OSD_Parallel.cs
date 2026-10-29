// Port of: src/FoundationClasses/TKernel/OSD/OSD_Parallel.hxx
// Port of: src/FoundationClasses/TKernel/OSD/OSD_Parallel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.ExceptionServices;

namespace Occt;

/// <summary>
/// The one parallel-loop helper of the port (decision 487): OCCT's <c>OSD_Parallel::For</c> and <c>ForEach</c>, with
/// their <c>isForceSingleThreadExecution</c> flag, over <see cref="Parallel"/>.
/// </summary>
/// <remarks>
/// <para>
/// A loop runs as a plain loop, on the calling thread and in order, when the flag is set, when there is one item, or
/// when the runtime has no threads (<see cref="HasThreads"/>: browser WebAssembly and WASI, which Bento.Web is, where
/// <see cref="Parallel"/> can't block on other threads). Every loop body of the port writes only its own item, so the
/// plain loop gives the parallel loop's result (decision 490).
/// </para>
/// <para>
/// An exception thrown by a loop body is rethrown as itself, the first one of the <see cref="AggregateException"/>
/// that <see cref="Parallel"/> throws (decision 431); OCCT's thread pool also rethrows the first failure.
/// </para>
/// <para>
/// Left out: <c>ToUseOcctThreads</c>/<c>SetUseOcctThreads</c> (OCCT's choice between its own thread pool and TBB:
/// the port has one scheduler), OSD_ThreadPool and its <c>Launcher</c>, and the iterator wrappers (a functor is an
/// <see cref="Action{T}"/>).
/// </para>
/// </remarks>
internal static class OSD_Parallel
{
    /// <summary>
    /// True if the runtime can run threads: false in a browser and under WASI, where .NET's WebAssembly runtime is
    /// single-threaded (Bento.Web's build). Computed once.
    /// </summary>
    internal static readonly bool HasThreads = !OperatingSystem.IsBrowser() && !OperatingSystem.IsWasi();

    private static readonly AsyncLocal<bool> theForceSerial = new();

    private static readonly AsyncLocal<int> theMaxDegree = new();

    /// <summary>
    /// For tests: while set, every loop started from this execution flow runs as a plain loop, as without threads.
    /// The setting flows into tasks started from the flow (it is an <see cref="AsyncLocal{T}"/>), so tests running in
    /// parallel don't affect each other. Dispose the returned scope to restore the previous setting.
    /// </summary>
    internal static IDisposable ForceSerialScope()
    {
        bool aPrevious = theForceSerial.Value;
        theForceSerial.Value = true;
        return new SerialScope(aPrevious);
    }

    /// <summary>
    /// For tests (the determinism harness of decision 5005): while set, every parallel loop started from this execution
    /// flow runs on at most <paramref name="theMaxDegree"/> threads at a time (<see
    /// cref="ParallelOptions.MaxDegreeOfParallelism"/>), so a test can run the same algorithm at several degrees of
    /// parallelism. It flows as <see cref="ForceSerialScope"/> does. Dispose the returned scope to restore the previous
    /// setting.
    /// </summary>
    internal static IDisposable MaxDegreeOfParallelismScope(int theMaxDegree)
    {
        if (theMaxDegree < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(theMaxDegree), "The degree of parallelism must be at least 1.");
        }
        int aPrevious = OSD_Parallel.theMaxDegree.Value;
        OSD_Parallel.theMaxDegree.Value = theMaxDegree;
        return new DegreeScope(aPrevious);
    }

    // The options of a parallel loop: the test scope's degree of parallelism, if any.
    private static ParallelOptions LoopOptions()
    {
        int aMaxDegree = theMaxDegree.Value;
        return aMaxDegree > 0 ? new ParallelOptions { MaxDegreeOfParallelism = aMaxDegree } : DefaultOptions;
    }

    private static readonly ParallelOptions DefaultOptions = new();

    /// <summary>True if loops started from this execution flow run as plain loops (no threads, or a test scope).</summary>
    internal static bool IsSerial() => !HasThreads || theForceSerial.Value;

    /// <summary>
    /// The number of logical processors (<see cref="Environment.ProcessorCount"/>), which OSD_ThreadPool's default pool
    /// takes as its number of threads.
    /// </summary>
    /// <remarks>
    /// Several algorithms split their work into this many tasks whatever the mode (BRepExtrema_DistShapeShape,
    /// BRepCheck_Analyzer, GeomLib_CheckCurveOnSurface), and the split can decide the result. So it does not change in a
    /// serial run (<see cref="ForceSerialScope"/>, or a runtime without threads): the serial loop runs the same tasks in
    /// order, and gives the parallel run's result. A single-threaded WebAssembly runtime reports 1 itself.
    /// </remarks>
    internal static int NbLogicalProcessors() => Environment.ProcessorCount;

    /// <summary>
    /// <c>for (int i = theBegin; i != theEnd; ++i) theFunctor(i);</c>, the iterations run in parallel unless
    /// <paramref name="isForceSingleThreadExecution"/> is set, the range has one index, or the runtime has no threads.
    /// </summary>
    internal static void For(int theBegin, int theEnd, Action<int> theFunctor, bool isForceSingleThreadExecution = false)
    {
        int aRange = theEnd - theBegin;
        if (isForceSingleThreadExecution || aRange == 1 || IsSerial())
        {
            for (int it = theBegin; it < theEnd; ++it)
            {
                theFunctor(it);
            }
        }
        else if (aRange > 1)
        {
            try
            {
                Parallel.For(theBegin, theEnd, LoopOptions(), theFunctor);
            }
            catch (AggregateException anAggregate) when (anAggregate.InnerExceptions.Count > 0)
            {
                // OSD_Parallel rethrows the first exception of a task
                ExceptionDispatchInfo.Capture(anAggregate.InnerExceptions[0]).Throw();
            }
        }
    }

    /// <summary>
    /// <c>foreach (T item in theItems) theFunctor(item);</c>, the items processed in parallel unless
    /// <paramref name="isForceSingleThreadExecution"/> is set, <paramref name="theNbItems"/> is 1, or the runtime has
    /// no threads. <paramref name="theNbItems"/> is the number of items if known (-1 otherwise), as OCCT's hint.
    /// </summary>
    internal static void ForEach<T>(IEnumerable<T> theItems, Action<T> theFunctor, bool isForceSingleThreadExecution = false,
                                    int theNbItems = -1)
    {
        if (isForceSingleThreadExecution || theNbItems == 1 || IsSerial())
        {
            foreach (T anItem in theItems)
            {
                theFunctor(anItem);
            }
        }
        else
        {
            try
            {
                Parallel.ForEach(theItems, LoopOptions(), theFunctor);
            }
            catch (AggregateException anAggregate) when (anAggregate.InnerExceptions.Count > 0)
            {
                // OSD_Parallel rethrows the first exception of a task
                ExceptionDispatchInfo.Capture(anAggregate.InnerExceptions[0]).Throw();
            }
        }
    }

    private sealed class SerialScope(bool thePrevious) : IDisposable
    {
        private bool myIsDisposed;

        public void Dispose()
        {
            if (!myIsDisposed)
            {
                theForceSerial.Value = thePrevious;
                myIsDisposed = true;
            }
        }
    }

    private sealed class DegreeScope(int thePrevious) : IDisposable
    {
        private bool myIsDisposed;

        public void Dispose()
        {
            if (!myIsDisposed)
            {
                theMaxDegree.Value = thePrevious;
                myIsDisposed = true;
            }
        }
    }
}
