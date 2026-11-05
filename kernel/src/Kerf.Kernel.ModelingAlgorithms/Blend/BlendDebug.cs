// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_Debug.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>The trace flags of Blend_Debug.cxx, which Draw's debug commands set.</summary>
/// <remarks>
/// OCCT keeps them in file-scope statics. They only affect traces (plan 2.3); they are settings of the
/// <see cref="OperationContext"/> (unit 10I): a setter changes them for the rest of the current flow of execution.
/// </remarks>
internal static class BlendDebug
{
    // recuperation des surfaces des conges approximes.
    private static readonly ContextKey<bool> TraceDrawSectSetting = new("Blend.traceDRAWSECT", false);

    // Contexte sans test de deflexion
    private static readonly ContextKey<bool> ContextNoTestDeflSetting = new("Blend.contextNOTESTDEFL", false);

    public static void Blend_SettraceDRAWSECT(bool b) => OperationContext.Set(TraceDrawSectSetting, b);

    public static bool Blend_GettraceDRAWSECT() => TraceDrawSectSetting.Value;

    public static void Blend_SetcontextNOTESTDEFL(bool b) => OperationContext.Set(ContextNoTestDeflSetting, b);

    public static bool Blend_GetcontextNOTESTDEFL() => ContextNoTestDeflSetting.Value;
}
