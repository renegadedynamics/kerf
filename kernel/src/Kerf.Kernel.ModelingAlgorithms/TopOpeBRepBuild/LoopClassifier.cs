// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>Classifies loops against each other, to build areas.</summary>
internal abstract class LoopClassifier
{
    /// <summary>The state of the loop <paramref name="l1"/> compared with the loop <paramref name="l2"/>.</summary>
    public abstract Containment Compare(BuildLoop l1, BuildLoop l2);
}
