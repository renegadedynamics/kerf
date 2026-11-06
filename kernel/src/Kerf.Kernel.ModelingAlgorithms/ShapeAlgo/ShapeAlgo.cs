// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAlgo/ShapeAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;

namespace Kerf.Engine.Healing;

/// <summary>
/// The interface to Shape Healing's algorithms for the data exchange translators: the default
/// <see cref="AlgoContainer"/>.
/// </summary>
/// <remarks>
/// OCCT's semantics (plan 2.3, decision 1000), without process-wide state (unit 10I): the container is null until
/// <see cref="Init"/> creates the default one (once per process, which also loads Shape Healing's messages through
/// <see cref="ShapeExtend.Init"/>); <see cref="SetAlgoContainer"/> replaces it for the rest of the current flow of
/// execution (a setting of the <see cref="OperationContext"/>; in OCCT, for every thread).
/// </remarks>
internal static class ShapeAlgo
{
    private static readonly Lazy<AlgoContainer> DefaultContainer = new(() =>
    {
        var container = new AlgoContainer();
        // initialization of Standard Shape Healing
        global::Kerf.Engine.Healing.Extend.ShapeExtend.Init();
        return container;
    });

    // A container set in the flow (IsSet), standing for the default one
    private static readonly ContextKey<(bool IsSet, AlgoContainer? Container)> ContainerSetting =
        new("ShapeAlgo.AlgoContainer", (false, null));

    /// <summary>Creates and initializes the default AlgoContainer (the first call only).</summary>
    public static void Init() => _ = DefaultContainer.Value;

    /// <summary>Sets the default AlgoContainer for the rest of the current flow of execution.</summary>
    public static void SetAlgoContainer(AlgoContainer? aContainer) => OperationContext.Set(ContainerSetting, (true, aContainer));

    /// <summary>Returns the default AlgoContainer (null before <see cref="Init"/>, unless one was set).</summary>
    public static AlgoContainer? AlgoContainer() =>
        ContainerSetting.Value is (true, var container) ? container
        : DefaultContainer.IsValueCreated ? DefaultContainer.Value : null;
}
