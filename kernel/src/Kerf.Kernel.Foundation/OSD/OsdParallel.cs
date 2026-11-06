// Port of: src/FoundationClasses/TKernel/OSD/OSD_Parallel.hxx
// Port of: src/FoundationClasses/TKernel/OSD/OSD_Parallel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.ExceptionServices;

namespace Kerf.Engine.Foundation;

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
internal static class OsdParallel
{
    /// <summary>
    /// True if the runtime can run threads: false in a browser and under WASI, where .NET's WebAssembly runtime is
    /// single-threaded (Bento.Web's build). Computed once.
    /// </summary>
    internal static readonly bool HasThreads = !OperatingSystem.IsBrowser() && !OperatingSystem.IsWasi();

    private static readonly AsyncLocal<bool> ForceSerial = new();

    private static readonly AsyncLocal<int> MaxDegree = new();

    /// <summary>
    /// For tests: while set, every loop started from this execution flow runs as a plain loop, as without threads.
    /// The setting flows into tasks started from the flow (it is an <see cref="AsyncLocal{T}"/>), so tests running in
    /// parallel don't affect each other. Dispose the returned scope to restore the previous setting.
    /// </summary>
    internal static IDisposable ForceSerialScope()
    {
        bool previous = ForceSerial.Value;
        ForceSerial.Value = true;
        return new SerialScope(previous);
    }

    /// <summary>
    /// For tests (the determinism harness of decision 580): while set, every parallel loop started from this execution
    /// flow runs on at most <paramref name="maxDegree"/> threads at a time (<see
    /// cref="ParallelOptions.MaxDegreeOfParallelism"/>), so a test can run the same algorithm at several degrees of
    /// parallelism. It flows as <see cref="ForceSerialScope"/> does. Dispose the returned scope to restore the previous
    /// setting.
    /// </summary>
    internal static IDisposable MaxDegreeOfParallelismScope(int maxDegree)
    {
        if (maxDegree < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(maxDegree), "The degree of parallelism must be at least 1.");
        }
        int previous = OsdParallel.MaxDegree.Value;
        OsdParallel.MaxDegree.Value = maxDegree;
        return new DegreeScope(previous);
    }

    // The options of a parallel loop: the test scope's degree of parallelism, if any.
    private static ParallelOptions LoopOptions()
    {
        int maxDegree = MaxDegree.Value;
        return maxDegree > 0 ? new ParallelOptions { MaxDegreeOfParallelism = maxDegree } : DefaultOptions;
    }

    private static readonly ParallelOptions DefaultOptions = new();

    /// <summary>True if loops started from this execution flow run as plain loops (no threads, or a test scope).</summary>
    internal static bool IsSerial => !HasThreads || ForceSerial.Value;

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
    internal static int LogicalProcessorCount => Environment.ProcessorCount;

    /// <summary>
    /// <c>for (int i = theBegin; i != theEnd; ++i) theFunctor(i);</c>, the iterations run in parallel unless
    /// <paramref name="isForceSingleThreadExecution"/> is set, the range has one index, or the runtime has no threads.
    /// </summary>
    internal static void For(int begin, int end, Action<int> functor, bool isForceSingleThreadExecution = false)
    {
        int range = end - begin;
        if (isForceSingleThreadExecution || range == 1 || IsSerial)
        {
            for (int it = begin; it < end; ++it)
            {
                functor(it);
            }
        }
        else if (range > 1)
        {
            try
            {
                Parallel.For(begin, end, LoopOptions(), functor);
            }
            catch (AggregateException aggregate) when (aggregate.InnerExceptions.Count > 0)
            {
                // OSD_Parallel rethrows the first exception of a task
                ExceptionDispatchInfo.Capture(aggregate.InnerExceptions[0]).Throw();
            }
        }
    }

    /// <summary>
    /// <c>foreach (T item in theItems) theFunctor(item);</c>, the items processed in parallel unless
    /// <paramref name="isForceSingleThreadExecution"/> is set, <paramref name="nbItems"/> is 1, or the runtime has
    /// no threads. <paramref name="nbItems"/> is the number of items if known (-1 otherwise), as OCCT's hint.
    /// </summary>
    internal static void ForEach<T>(IEnumerable<T> items, Action<T> functor, bool isForceSingleThreadExecution = false,
                                    int nbItems = -1)
    {
        if (isForceSingleThreadExecution || nbItems == 1 || IsSerial)
        {
            foreach (T item in items)
            {
                functor(item);
            }
        }
        else
        {
            try
            {
                Parallel.ForEach(items, LoopOptions(), functor);
            }
            catch (AggregateException aggregate) when (aggregate.InnerExceptions.Count > 0)
            {
                // OSD_Parallel rethrows the first exception of a task
                ExceptionDispatchInfo.Capture(aggregate.InnerExceptions[0]).Throw();
            }
        }
    }

    private sealed class SerialScope(bool thePrevious) : IDisposable
    {
        private bool _isDisposed;

        public void Dispose()
        {
            if (!_isDisposed)
            {
                ForceSerial.Value = thePrevious;
                _isDisposed = true;
            }
        }
    }

    private sealed class DegreeScope(int thePrevious) : IDisposable
    {
        private bool _isDisposed;

        public void Dispose()
        {
            if (!_isDisposed)
            {
                MaxDegree.Value = thePrevious;
                _isDisposed = true;
            }
        }
    }
}
