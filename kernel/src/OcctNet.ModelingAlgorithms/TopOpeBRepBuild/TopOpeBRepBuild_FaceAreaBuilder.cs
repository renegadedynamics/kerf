// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_FaceAreaBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_FaceAreaBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Builds the areas of faces from a <see cref="TopOpeBRepBuild_LoopSet"/> of wires and blocks of edges (an
/// <see cref="TopOpeBRepBuild_Area2dBuilder"/>).
/// </summary>
public class TopOpeBRepBuild_FaceAreaBuilder : TopOpeBRepBuild_Area2dBuilder
{
    /// <summary>An empty face area builder.</summary>
    public TopOpeBRepBuild_FaceAreaBuilder()
    {
    }

    /// <summary>The faces' areas of the loops of <paramref name="LS"/>, classified with <paramref name="LC"/>.</summary>
    public TopOpeBRepBuild_FaceAreaBuilder(TopOpeBRepBuild_LoopSet LS, TopOpeBRepBuild_LoopClassifier LC, bool ForceClass = false)
    {
        InitFaceAreaBuilder(LS, LC, ForceClass);
    }

    /// <summary>Builds the faces' areas of the loops of <paramref name="LS"/>, classified with <paramref name="LC"/>.</summary>
    public void InitFaceAreaBuilder(TopOpeBRepBuild_LoopSet LS, TopOpeBRepBuild_LoopClassifier LC, bool ForceClass = false)
    {
        InitAreaBuilder(LS, LC, ForceClass);
    }
}
