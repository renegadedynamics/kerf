// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_GlueEnum.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The gluing option of the Boolean component's algorithms, which speeds up the intersection of arguments that
/// overlap without real intersections. <see cref="BOPAlgo_GlueOff"/> (the default) switches it off;
/// <see cref="BOPAlgo_GlueShift"/> is for partially coinciding shapes (their faces are split, the face/face
/// intersections are skipped); <see cref="BOPAlgo_GlueFull"/> for fully coinciding ones (vertex/face, edge/face and
/// face/face intersections are skipped). The caller guarantees that the arguments really coincide: the algorithms
/// don't check it, and a wrong option is likely to give a wrong result.
/// </summary>
public enum BOPAlgo_GlueEnum
{
    BOPAlgo_GlueOff,
    BOPAlgo_GlueShift,
    BOPAlgo_GlueFull,
}
