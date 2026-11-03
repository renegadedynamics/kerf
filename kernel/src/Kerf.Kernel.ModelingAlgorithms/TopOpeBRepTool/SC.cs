// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_SC.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_SC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>The legacy Booleans' shared shape classifier (<c>FSC_*</c>).</summary>
/// <remarks>
/// OCCT's free functions, as a static class named after the file. The classifier is per thread in OCCT
/// (<c>thread_local</c>), and so in the port (<c>[ThreadStatic]</c>); it keeps its state from one call to the next.
/// <c>FSC_StatePonFace</c> and <c>FSC_StateEonFace</c> (the Booleans) are left out (plan 2.11).
/// </remarks>
public static class SC
{
    [ThreadStatic]
    private static ShapeClassifier? TopOpeBRepTool_PSC;

    /// <summary>The thread's shape classifier, created on first use.</summary>
    public static ShapeClassifier FSC_GetPSC()
    {
        TopOpeBRepTool_PSC ??= new ShapeClassifier();
        return TopOpeBRepTool_PSC;
    }

    /// <summary>The thread's shape classifier, with <paramref name="s"/> as its reference shape.</summary>
    public static ShapeClassifier FSC_GetPSC(in Shape s)
    {
        TopOpeBRepTool_PSC ??= new ShapeClassifier();
        TopOpeBRepTool_PSC.SetReference(s);
        return TopOpeBRepTool_PSC;
    }
}
