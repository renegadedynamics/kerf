// Port of: src/FoundationClasses/TKernel/OSD/OSD_Parallel.hxx
// Port of: src/FoundationClasses/TKernel/OSD/OSD_Parallel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.ExceptionServices;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The one parallel-loop helper of the port (decision 487): OCCT's <c>OSD_Parallel::For</c> and <c>ForEach</c>, with
/// their <c>isForceSingleThreadExecution</c> flag, over the thread pool.
/// </summary>
/// <remarks>
/// <para>
/// A loop runs as a plain loop, on the calling thread and in order, when the flag is set, when there is one item, or
/// when the runtime has no threads (<see cref="HasThreads"/>: browser WebAssembly and WASI, which Bento.Web is, where
/// a thread can't block on others). Every loop body of the port writes only its own item, so the plain loop gives the
/// parallel loop's result (decision 490).
/// </para>
/// <para>
/// A parallel loop never waits for the thread pool (decisions 2201, 2202). The calling thread takes items itself until none
/// is left, helped by whichever pool threads are free, and then waits only for the items a helper has already taken.
/// <see cref="Parallel"/> isn't used: called from a thread that isn't the pool's, it waits for the pool to start a
/// task it queued, even when no work is left for that task. With the pool busy, a loop of a few microseconds then
/// takes as long as the pool takes to free or add a thread, which can be half a second.
/// </para>
/// <para>
/// An exception thrown by a loop body is rethrown as itself, and items not yet taken are left (decision 431); OCCT's
/// thread pool also rethrows the first failure.
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
            throw new KernelArgumentOutOfRangeException(nameof(maxDegree), "The degree of parallelism must be at least 1.");
        }
        int previous = OsdParallel.MaxDegree.Value;
        OsdParallel.MaxDegree.Value = maxDegree;
        return new DegreeScope(previous);
    }

    // The threads a parallel loop may run on at once, the caller among them: the test scope's degree of parallelism, if
    // any, or a thread for each processor.
    private static int LoopDegree()
    {
        int maxDegree = MaxDegree.Value;
        return maxDegree > 0 ? maxDegree : Environment.ProcessorCount;
    }

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
            new Loop(begin, end, functor, LoopDegree()).Run();
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
            // read here, on the calling thread: the loop's threads then share only the list
            IReadOnlyList<T> list = items as IReadOnlyList<T> ?? [.. items];
            if (list.Count == 1)
            {
                functor(list[0]);
            }
            else if (list.Count > 1)
            {
                new Loop(0, list.Count, index => functor(list[index]), LoopDegree()).Run();
            }
        }
    }

    /// <summary>
    /// One parallel loop: the indices are taken a chunk at a time by the calling thread and by helpers on the pool's
    /// threads, each helper asking for the next when it starts (so a short loop wakes few threads).
    /// </summary>
    private sealed class Loop
    {
        // chunks for each thread, so that a thread whose items are slow leaves the rest to the others
        private const int ChunksPerThread = 16;

        private readonly int _end;
        private readonly int _chunk;
        private readonly int _maxHelpers;

        // the caller's, for the helpers: it carries the operation's context and the tests' scopes
        private readonly ExecutionContext? _context;

        // null once the loop is over
        private Action<int>? _body;

        // the next index to take; past _end once none is left, or a body has thrown
        private long _next;

        // helpers asked for so far
        private int _helpers;

        // helpers that have started and not finished: only these are waited for
        private int _running;

        private ExceptionDispatchInfo? _failure;

        internal Loop(int begin, int end, Action<int> body, int degree)
        {
            _next = begin;
            _end = end;
            _body = body;
            int range = end - begin;
            _chunk = Math.Max(1, range / (Math.Max(1, degree) * ChunksPerThread));
            _maxHelpers = Math.Min(degree, (range + _chunk - 1) / _chunk) - 1;
            _context = _maxHelpers > 0 ? ExecutionContext.Capture() : null;
        }

        /// <summary>True once <see cref="Run"/> has finished: a helper has nothing to do for it.</summary>
        internal bool IsOver => Volatile.Read(ref _body) is null;

        /// <summary>Runs the loop: returns when every index has been run, or throws what a body threw.</summary>
        internal void Run()
        {
            AskForHelper();
            Work(_body!);
            // the indices are all taken: wait for those a helper is still running, briefly without blocking
            if (Volatile.Read(ref _running) != 0)
            {
                SpinWait spin = new();
                while (!spin.NextSpinWillYield && Volatile.Read(ref _running) != 0)
                {
                    spin.SpinOnce();
                }

                lock (this)
                {
                    while (_running != 0)
                    {
                        Monitor.Wait(this);
                    }
                }
            }

            // a helper that starts from now on finds nothing to take: it mustn't keep the body's data alive until then
            Volatile.Write(ref _body, null);
            _failure?.Throw();
        }

        private void AskForHelper()
        {
            if (Interlocked.Increment(ref _helpers) <= _maxHelpers)
            {
                Ticket.Offer(this);
            }
        }

        /// <summary>A pool thread's part: it takes indices as the caller does, if any are left.</summary>
        internal void Help()
        {
            if (IsOver)
            {
                return;
            }

            if (_context is null)
            {
                HelpInContext();
            }
            else
            {
                ExecutionContext.Run(_context, static loop => ((Loop)loop!).HelpInContext(), this);
            }
        }

        private void HelpInContext()
        {
            Interlocked.Increment(ref _running);
            try
            {
                // read after the count: Run clears it only once it has seen no helper running and no index left
                if (Volatile.Read(ref _body) is { } body && Interlocked.Read(ref _next) < _end)
                {
                    AskForHelper();
                    Work(body);
                }
            }
            finally
            {
                if (Interlocked.Decrement(ref _running) == 0)
                {
                    lock (this)
                    {
                        Monitor.PulseAll(this);
                    }
                }
            }
        }

        private void Work(Action<int> body)
        {
            try
            {
                while (true)
                {
                    long first = Interlocked.Add(ref _next, _chunk) - _chunk;
                    if (first >= _end)
                    {
                        return;
                    }

                    int last = (int)Math.Min(first + _chunk, _end);
                    for (int index = (int)first; index < last; ++index)
                    {
                        if (_chunk > 1 && Volatile.Read(ref _failure) is not null)
                        {
                            return;
                        }

                        body(index);
                    }
                }
            }
            catch (Exception failure)
            {
                // the first to fail is the one rethrown; nothing more is taken
                Interlocked.CompareExchange(ref _failure, ExceptionDispatchInfo.Capture(failure), null);
                Interlocked.Exchange(ref _next, long.MaxValue / 2);
            }
        }
    }

    /// <summary>
    /// A loop's request for a helper, queued to the pool. A thread keeps those it has queued that no pool thread has
    /// taken yet, and gives its next loop to one whose own loop is over. So a busy pool is left a request for each loop
    /// the thread is in, not one for every loop it has run meanwhile.
    /// </summary>
    private sealed class Ticket : IThreadPoolWorkItem
    {
        private static readonly object Taken = new();

        [ThreadStatic]
        private static List<Ticket>? s_tWaiting;

        // the loop to help, or taken once a pool thread has it
        private object _loop;

        private Ticket(Loop loop) => _loop = loop;

        internal static void Offer(Loop loop)
        {
            List<Ticket> waiting = s_tWaiting ??= [];
            for (int i = waiting.Count - 1; i >= 0; --i)
            {
                object seen = Volatile.Read(ref waiting[i]._loop);
                if (ReferenceEquals(seen, Taken))
                {
                    waiting.RemoveAt(i);
                }
                else if (((Loop)seen).IsOver && ReferenceEquals(Interlocked.CompareExchange(ref waiting[i]._loop, loop, seen), seen))
                {
                    return;
                }
            }

            Ticket ticket = new(loop);
            waiting.Add(ticket);
            // on the calling thread's own queue if it is the pool's: free threads take it from there
            ThreadPool.UnsafeQueueUserWorkItem(ticket, preferLocal: true);
        }

        void IThreadPoolWorkItem.Execute() => (Interlocked.Exchange(ref _loop, Taken) as Loop)?.Help();
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
