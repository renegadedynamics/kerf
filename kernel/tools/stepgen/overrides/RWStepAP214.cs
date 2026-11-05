// Port of: src/DataExchange/TKDESTEP/RWStepAP214/RWStepAP214.pxx
// Port of: src/DataExchange/TKDESTEP/RWStepAP214/RWStepAP214.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Written by hand (tools/stepgen/overrides/RWStepAP214.cs, decision 1216): the generator copies this file.

namespace Kerf.Engine;

/// <summary>The RWStepAP214 package: the initialisation of the libraries.</summary>
internal static class RWStepAP214
{
    // OCCT's flag THE_RWStepAP214_init: the whole initialisation done once per process (decisions 1182, 1215)
    private static readonly Lazy<bool> s_init = new(DoInit);

    /// <summary>
    /// Enforces the initialisation of the libraries: the header section's, then StepAP214's protocol with its general
    /// module in the general library and its read-write module in the reader and writer libraries. Process-wide, once.
    /// </summary>
    public static void Init()
    {
        _ = s_init.Value;
    }

    private static bool DoInit()
    {
        RWHeaderSection.Init();
        StepAP214_Protocol proto = StepAP214.Protocol();
        Interface_GeneralLib.SetGlobal(new RWStepAP214_GeneralModule(), proto);
        Interface_ReaderLib.SetGlobal(new RWStepAP214_ReadWriteModule(), proto);
        StepData_WriterLib.SetGlobal(new RWStepAP214_ReadWriteModule(), proto);
        return true;
    }
}
