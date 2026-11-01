// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Debug.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The trace flags of Blend_Debug.cxx, which Draw's debug commands set.</summary>
/// <remarks>
/// OCCT keeps them in file-scope statics. They only affect traces (plan 2.3), so they stay process-wide.
/// </remarks>
public static class Blend_Debug
{
    // recuperation des surfaces des conges approximes.
    private static bool Blend_traceDRAWSECT;

    // Contexte sans test de deflexion
    private static bool Blend_contextNOTESTDEFL;

    public static void Blend_SettraceDRAWSECT(bool b)
    {
        Blend_traceDRAWSECT = b;
    }

    public static bool Blend_GettraceDRAWSECT()
    {
        return Blend_traceDRAWSECT;
    }

    public static void Blend_SetcontextNOTESTDEFL(bool b)
    {
        Blend_contextNOTESTDEFL = b;
    }

    public static bool Blend_GetcontextNOTESTDEFL()
    {
        return Blend_contextNOTESTDEFL;
    }
}
