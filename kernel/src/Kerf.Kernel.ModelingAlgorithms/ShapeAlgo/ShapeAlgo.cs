// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Extend;

using Occt;

namespace Kerf.Engine.Healing;

/// <summary>
/// The interface to Shape Healing's algorithms for the data exchange translators: the process-wide default
/// <see cref="AlgoContainer"/>.
/// </summary>
/// <remarks>
/// Process-wide state, with OCCT's semantics (plan 2.3, decision 1000): the container is null until
/// <see cref="Init"/> creates the default one (once per process, which also loads Shape Healing's messages through
/// <see cref="ShapeExtend.Init"/>), and <see cref="SetAlgoContainer"/> replaces it for every thread. The reads and
/// writes are atomic; tests that replace it run in the non-parallel collection.
/// </remarks>
public static class ShapeAlgo
{
    private static readonly object Lock = new();
    private static AlgoContainer? SContainer;
    private static bool SInit;

    /// <summary>Creates and initializes the default AlgoContainer (the first call only).</summary>
    public static void Init()
    {
        lock (Lock)
        {
            if (SInit)
            {
                return;
            }
            SInit = true;
            Volatile.Write(ref SContainer, new AlgoContainer());
        }

        // initialization of Standard Shape Healing
        global::Kerf.Engine.Healing.Extend.ShapeExtend.Init();
    }

    /// <summary>Sets the default AlgoContainer.</summary>
    public static void SetAlgoContainer(AlgoContainer? aContainer)
    {
        Volatile.Write(ref SContainer, aContainer);
    }

    /// <summary>Returns the default AlgoContainer (null before <see cref="Init"/>).</summary>
    public static AlgoContainer? AlgoContainer() => Volatile.Read(ref SContainer);
}
