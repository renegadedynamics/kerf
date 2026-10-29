// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_Parallel.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;

namespace Occt;

/// <summary>
/// The parallel loops of the Boolean component: each runs <c>Perform()</c> on every solver of a vector, in parallel
/// unless the algorithm's <c>myRunParallel</c> flag is off, through <see cref="OSD_Parallel"/> (decision 522).
/// </summary>
/// <remarks>
/// <para>
/// Decision 5005 (a parallel Boolean gives the serial Boolean's result). Each solver writes only itself; the
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
internal static class BOPTools_Parallel
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
        void SetContext(TContext theContext);
    }

    /// <summary>Pure version: runs every solver's <c>Perform()</c>, in parallel if <paramref name="theIsRunParallel"/>.</summary>
    internal static void Perform<TSolver>(bool theIsRunParallel, IReadOnlyList<TSolver> theSolverVector)
        where TSolver : ISolver
    {
        OSD_Parallel.For(0, theSolverVector.Count, theIndex => theSolverVector[theIndex].Perform(), !theIsRunParallel);
    }

    /// <summary>Pure version over an NCollection vector.</summary>
    internal static void Perform<TSolver>(bool theIsRunParallel, NCollection_DynamicArray<TSolver> theSolverVector)
        where TSolver : ISolver
    {
        OSD_Parallel.For(0, theSolverVector.Length(), theIndex => theSolverVector.Value(theIndex).Perform(), !theIsRunParallel);
    }

    /// <summary>
    /// Context-dependent version: as the pure version, each solver first
    /// given its thread's context (the calling thread's is <paramref name="theContext"/>).
    /// </summary>
    internal static void Perform<TSolver, TContext>(bool theIsRunParallel, IReadOnlyList<TSolver> theSolverVector, TContext theContext)
        where TSolver : IContextSolver<TContext>
        where TContext : class, new()
    {
        ContextFunctor<TContext> aFunctor = new();
        aFunctor.SetContext(theContext);
        OSD_Parallel.For(0, theSolverVector.Count, theIndex =>
        {
            TSolver aSolver = theSolverVector[theIndex];
            aSolver.SetContext(aFunctor.GetThreadContext());
            aSolver.Perform();
        }, !theIsRunParallel);
    }

    /// <summary>Context-dependent version over an NCollection vector.</summary>
    internal static void Perform<TSolver, TContext>(bool theIsRunParallel, NCollection_DynamicArray<TSolver> theSolverVector,
                                                   TContext theContext)
        where TSolver : IContextSolver<TContext>
        where TContext : class, new()
    {
        ContextFunctor<TContext> aFunctor = new();
        aFunctor.SetContext(theContext);
        OSD_Parallel.For(0, theSolverVector.Length(), theIndex =>
        {
            TSolver aSolver = theSolverVector.Value(theIndex);
            aSolver.SetContext(aFunctor.GetThreadContext());
            aSolver.Perform();
        }, !theIsRunParallel);
    }

    /// <summary>OCCT's <c>ContextFunctor</c>: the map of thread id to context.</summary>
    internal sealed class ContextFunctor<TContext>
        where TContext : class, new()
    {
        private readonly ConcurrentDictionary<int, TContext> myContextMap = new();

        /// <summary>Binds the calling thread's context.</summary>
        internal void SetContext(TContext? theContext)
        {
            if (theContext is not null)
            {
                myContextMap[Environment.CurrentManagedThreadId] = theContext;
            }
        }

        /// <summary>The calling thread's context, made on its first use by that thread.</summary>
        internal TContext GetThreadContext() =>
            myContextMap.GetOrAdd(Environment.CurrentManagedThreadId, static _ => new TContext());

        /// <summary>The number of contexts used so far (for tests).</summary>
        internal int NbContexts() => myContextMap.Count;
    }
}
