// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopClassifier.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_LoopClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>Classifies loops against each other, to build areas.</summary>
public abstract class TopOpeBRepBuild_LoopClassifier
{
    /// <summary>The state of the loop <paramref name="L1"/> compared with the loop <paramref name="L2"/>.</summary>
    public abstract Containment Compare(TopOpeBRepBuild_Loop L1, TopOpeBRepBuild_Loop L2);
}
