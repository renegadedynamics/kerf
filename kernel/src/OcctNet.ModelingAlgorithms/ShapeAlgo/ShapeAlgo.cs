// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The interface to Shape Healing's algorithms for the data exchange translators: the process-wide default
/// <see cref="ShapeAlgo_AlgoContainer"/>.
/// </summary>
/// <remarks>
/// Process-wide state, with OCCT's semantics (plan 2.3, decision 1000): the container is null until
/// <see cref="Init"/> creates the default one (once per process, which also loads Shape Healing's messages through
/// <see cref="ShapeExtend.Init"/>), and <see cref="SetAlgoContainer"/> replaces it for every thread. The reads and
/// writes are atomic; tests that replace it run in the non-parallel collection.
/// </remarks>
public static class ShapeAlgo
{
    private static readonly object s_lock = new();
    private static ShapeAlgo_AlgoContainer? s_container;
    private static bool s_init;

    /// <summary>Creates and initializes the default AlgoContainer (the first call only).</summary>
    public static void Init()
    {
        lock (s_lock)
        {
            if (s_init)
            {
                return;
            }
            s_init = true;
            Volatile.Write(ref s_container, new ShapeAlgo_AlgoContainer());
        }

        // initialization of Standard Shape Healing
        ShapeExtend.Init();
    }

    /// <summary>Sets the default AlgoContainer.</summary>
    public static void SetAlgoContainer(ShapeAlgo_AlgoContainer? aContainer)
    {
        Volatile.Write(ref s_container, aContainer);
    }

    /// <summary>Returns the default AlgoContainer (null before <see cref="Init"/>).</summary>
    public static ShapeAlgo_AlgoContainer? AlgoContainer() => Volatile.Read(ref s_container);
}
