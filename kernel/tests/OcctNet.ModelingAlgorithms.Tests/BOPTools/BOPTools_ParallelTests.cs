// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;

namespace OcctNet.ModelingAlgorithms.Tests.BOPTools;

/// <summary>The per-thread context of BOPTools_Parallel (decision 5005).</summary>
public class BOPTools_ParallelTests
{
    private sealed class TestContext
    {
        public int OwnerThread = -1;
        public int NbUses;
    }

    private sealed class Solver(int theIndex, ConcurrentDictionary<TestContext, ConcurrentBag<int>> theThreadsOfContext)
        : BOPTools_Parallel.IContextSolver<TestContext>
    {
        public TestContext? Context;
        public int Thread = -1;
        public double Result;

        public void SetContext(TestContext theContext) => Context = theContext;

        public void Perform()
        {
            Thread = Environment.CurrentManagedThreadId;
            theThreadsOfContext.GetOrAdd(Context!, _ => []).Add(Thread);
            Interlocked.Increment(ref Context!.NbUses);
            double aSum = 0;
            for (int i = 1; i <= 2000; ++i)
            {
                aSum += Math.Sin(theIndex * 0.001 * i);
            }
            Result = aSum;
            if (theIndex == -1)
            {
                throw new Standard_DomainError("solver failed");
            }
        }
    }

    private sealed class PureSolver(int theIndex) : BOPTools_Parallel.ISolver
    {
        public int Result;

        public void Perform() => Result = theIndex * theIndex;
    }

    private static (List<Solver>, ConcurrentDictionary<TestContext, ConcurrentBag<int>>) Solvers(int theNb)
    {
        ConcurrentDictionary<TestContext, ConcurrentBag<int>> aMap = new(ReferenceEqualityComparer.Instance);
        return ([.. Enumerable.Range(0, theNb).Select(i => new Solver(i, aMap))], aMap);
    }

    [Fact]
    public void Serial_UsesTheGivenContextForEverySolver()
    {
        (List<Solver> aSolvers, _) = Solvers(50);
        TestContext aMain = new();
        BOPTools_Parallel.Perform(false, aSolvers, aMain);
        Assert.All(aSolvers, s => Assert.Same(aMain, s.Context));
        Assert.Equal(50, aMain.NbUses);
    }

    [Fact]
    public void Parallel_EachThreadHasItsOwnContext_AndTheResultIsTheSerialOne()
    {
        (List<Solver> aSerial, _) = Solvers(400);
        BOPTools_Parallel.Perform(false, aSerial, new TestContext());

        for (int aRun = 0; aRun < 5; ++aRun)
        {
            (List<Solver> aSolvers, ConcurrentDictionary<TestContext, ConcurrentBag<int>> aThreads) = Solvers(400);
            TestContext aMain = new();
            int aCaller = Environment.CurrentManagedThreadId;
            BOPTools_Parallel.Perform(true, aSolvers, aMain);

            // a context is used by one thread only
            foreach (KeyValuePair<TestContext, ConcurrentBag<int>> aPair in aThreads)
            {
                Assert.Single(aPair.Value.Distinct());
            }
            // the calling thread's solvers got the given context; the others a context of their own
            foreach (Solver aSolver in aSolvers)
            {
                if (aSolver.Thread == aCaller)
                {
                    Assert.Same(aMain, aSolver.Context);
                }
                else
                {
                    Assert.NotSame(aMain, aSolver.Context);
                }
            }
            // results equal the serial ones, bit for bit
            for (int i = 0; i < aSolvers.Count; ++i)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(aSerial[i].Result), BitConverter.DoubleToInt64Bits(aSolvers[i].Result));
            }
        }
    }

    [Fact]
    public void ForcedSerial_UsesTheGivenContext()
    {
        (List<Solver> aSolvers, _) = Solvers(100);
        TestContext aMain = new();
        using (OSD_Parallel.ForceSerialScope())
        {
            BOPTools_Parallel.Perform(true, aSolvers, aMain);
        }
        Assert.All(aSolvers, s => Assert.Same(aMain, s.Context));
    }

    [Fact]
    public void NCollectionVector_Overloads()
    {
        NCollection_Vector<PureSolver> aPure = new();
        for (int i = 0; i < 100; ++i)
        {
            aPure.Append(new PureSolver(i));
        }
        BOPTools_Parallel.Perform(true, aPure);
        for (int i = 0; i < 100; ++i)
        {
            Assert.Equal(i * i, aPure.Value(i).Result);
        }

        (List<Solver> aSolvers, _) = Solvers(30);
        NCollection_Vector<Solver> aVector = new();
        foreach (Solver aSolver in aSolvers)
        {
            aVector.Append(aSolver);
        }
        TestContext aMain = new();
        BOPTools_Parallel.Perform(false, aVector, aMain);
        Assert.All(aSolvers, s => Assert.Same(aMain, s.Context));
    }

    [Fact]
    public void ExceptionInASolver_IsRethrownUnwrapped()
    {
        ConcurrentDictionary<TestContext, ConcurrentBag<int>> aMap = new(ReferenceEqualityComparer.Instance);
        List<Solver> aSolvers = [.. Enumerable.Range(0, 64).Select(i => new Solver(i == 40 ? -1 : i, aMap))];
        Assert.Throws<Standard_DomainError>(() => BOPTools_Parallel.Perform(true, aSolvers, new TestContext()));
        Assert.Throws<Standard_DomainError>(() => BOPTools_Parallel.Perform(false, aSolvers, new TestContext()));
    }

    [Fact]
    public void ContextFunctor_MakesOneContextPerThread()
    {
        BOPTools_Parallel.ContextFunctor<TestContext> aFunctor = new();
        TestContext aMain = new();
        aFunctor.SetContext(aMain);
        Assert.Same(aMain, aFunctor.GetThreadContext());
        TestContext? anOther = null;
        Thread aThread = new(() => anOther = aFunctor.GetThreadContext());
        aThread.Start();
        aThread.Join();
        Assert.NotNull(anOther);
        Assert.NotSame(aMain, anOther);
        Assert.Equal(2, aFunctor.NbContexts());
    }
}
