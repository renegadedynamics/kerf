// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_Parallel.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Collections.Concurrent;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>
/// The parallel loops of the Boolean component: each runs <c>Perform()</c> on every solver of a vector, in parallel
/// unless the algorithm's <c>myRunParallel</c> flag is off, through <see cref="OsdParallel"/> (decision 522).
/// </summary>
/// <remarks>
/// <para>
/// Decision 580 (a parallel Boolean gives the serial Boolean's result). Each solver writes only itself; the
/// algorithm merges the solvers' results afterwards, serially, in vector order, as OCCT does.
/// </para>
/// <para>
/// The context-dependent version gives each thread its own context, as OCCT's <c>ContextFunctor</c> does: the calling
/// thread uses the algorithm's context (<c>SetContext</c>), every other thread that runs a solver makes a new one
/// (<c>new TContext()</c>, OCCT's <c>new TypeContext(CommonBaseAllocator())</c>) on its first solver and keeps it for
/// the rest of the loop. A context only caches tools (projectors, classifiers, boxes), so which context a solver gets
/// must not change its result; unit 6B audits the cached tools for state carried between calls. In a serial loop
/// every solver gets the algorithm's context, as in OCCT.
/// </para>
/// <para>
/// OCCT's two functors are one here: <c>ContextFunctor2</c> (an array of contexts indexed by OSD_ThreadPool's thread
/// index) serves OCCT's own thread pool, which the port doesn't have (decision 522); <c>ContextFunctor</c>, keyed by
/// thread id, is OCCT's path when the pool isn't used. The thread id is
/// <see cref="Environment.CurrentManagedThreadId"/>.
/// </para>
/// <para>
/// C++ calls the solvers' <c>Perform</c> and <c>SetContext</c> by template duck typing; in C# a solver implements
/// ISolver or IContextSolver. A solver vector is any indexable list of solver
/// objects (OCCT's <c>NCollection_Vector</c> or <c>std::vector</c> of solvers, which are classes in the port).
/// </para>
/// </remarks>
internal static class BooleanParallel
{
    /// <summary>A solver run by the pure version of Perform.</summary>
    internal interface ISolver
    {
        void Perform();
    }

    /// <summary>A solver that needs a context: <see cref="SetContext"/> is called before <see cref="ISolver.Perform"/>.</summary>
    internal interface IContextSolver<TContext> : ISolver
        where TContext : class
    {
        void SetContext(TContext context);
    }

    /// <summary>Pure version: runs every solver's <c>Perform()</c>, in parallel if <paramref name="isRunParallel"/>.</summary>
    internal static void Perform<TSolver>(bool isRunParallel, IReadOnlyList<TSolver> solverVector)
        where TSolver : ISolver
    {
        OsdParallel.For(0, solverVector.Count, index => solverVector[index].Perform(), !isRunParallel);
    }

    /// <summary>Pure version over an NCollection vector.</summary>
    internal static void Perform<TSolver>(bool isRunParallel, NCollection_DynamicArray<TSolver> solverVector)
        where TSolver : ISolver
    {
        OsdParallel.For(0, solverVector.Length(), index => solverVector.Value(index).Perform(), !isRunParallel);
    }

    /// <summary>
    /// Context-dependent version: as the pure version, each solver first
    /// given its thread's context (the calling thread's is <paramref name="context"/>).
    /// </summary>
    internal static void Perform<TSolver, TContext>(bool isRunParallel, IReadOnlyList<TSolver> solverVector, TContext context)
        where TSolver : IContextSolver<TContext>
        where TContext : class, new()
    {
        ContextFunctor<TContext> functor = new();
        functor.SetContext(context);
        OsdParallel.For(0, solverVector.Count, index =>
        {
            TSolver solver = solverVector[index];
            solver.SetContext(functor.GetThreadContext());
            solver.Perform();
        }, !isRunParallel);
    }

    /// <summary>Context-dependent version over an NCollection vector.</summary>
    internal static void Perform<TSolver, TContext>(bool isRunParallel, NCollection_DynamicArray<TSolver> solverVector,
                                                   TContext context)
        where TSolver : IContextSolver<TContext>
        where TContext : class, new()
    {
        ContextFunctor<TContext> functor = new();
        functor.SetContext(context);
        OsdParallel.For(0, solverVector.Length(), index =>
        {
            TSolver solver = solverVector.Value(index);
            solver.SetContext(functor.GetThreadContext());
            solver.Perform();
        }, !isRunParallel);
    }

    /// <summary>OCCT's <c>ContextFunctor</c>: the map of thread id to context.</summary>
    internal sealed class ContextFunctor<TContext>
        where TContext : class, new()
    {
        private readonly ConcurrentDictionary<int, TContext> _contextMap = new();

        /// <summary>Binds the calling thread's context.</summary>
        internal void SetContext(TContext? context)
        {
            if (context is not null)
            {
                _contextMap[Environment.CurrentManagedThreadId] = context;
            }
        }

        /// <summary>The calling thread's context, made on its first use by that thread.</summary>
        internal TContext GetThreadContext() =>
            _contextMap.GetOrAdd(Environment.CurrentManagedThreadId, static _ => new TContext());

        /// <summary>The number of contexts used so far (for tests).</summary>
        internal int ContextCount => _contextMap.Count;
    }
}
