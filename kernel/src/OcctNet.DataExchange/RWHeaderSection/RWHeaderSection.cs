// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection.hxx
// Port of: src/DataExchange/TKDESTEP/RWHeaderSection/RWHeaderSection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The RWHeaderSection package: its initialisation.</summary>
public static class RWHeaderSection
{
    // OCCT keeps the two modules in statics created once; the whole initialisation is done once here (decision 1182),
    // so that a second call does not wrap the header protocol into a FileProtocol, as OCCT's does
    private static readonly Lazy<bool> s_init = new(DoInit);

    /// <summary>
    /// Enforces the initialisation of the libraries: registers HeaderSection's protocol as StepData's header protocol,
    /// and creates the read-write and general modules (which register themselves). Process-wide, done once.
    /// </summary>
    public static void Init()
    {
        _ = s_init.Value;
    }

    private static bool DoInit()
    {
        HeaderSection_Protocol proto = HeaderSection.Protocol();
        StepData.AddHeaderProtocol(proto);
        _ = new RWHeaderSection_ReadWriteModule();
        _ = new RWHeaderSection_GeneralModule();
        return true;
    }
}
