// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_FaceAreaBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_FaceAreaBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// Builds the areas of faces from a <see cref="LoopSet"/> of wires and blocks of edges (an
/// <see cref="Area2dBuilder"/>).
/// </summary>
public class FaceAreaBuilder : Area2dBuilder
{
    /// <summary>An empty face area builder.</summary>
    public FaceAreaBuilder()
    {
    }

    /// <summary>The faces' areas of the loops of <paramref name="ls"/>, classified with <paramref name="lc"/>.</summary>
    public FaceAreaBuilder(LoopSet ls, LoopClassifier lc, bool forceClass = false)
    {
        InitFaceAreaBuilder(ls, lc, forceClass);
    }

    /// <summary>Builds the faces' areas of the loops of <paramref name="ls"/>, classified with <paramref name="lc"/>.</summary>
    public void InitFaceAreaBuilder(LoopSet ls, LoopClassifier lc, bool forceClass = false)
    {
        InitAreaBuilder(ls, lc, forceClass);
    }
}
