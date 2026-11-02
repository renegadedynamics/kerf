// Port of: src/DataExchange/TKDESTEP/StepData/StepData_WriterLib.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_WriterLib_0.cxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_GlobalNodeOfWriterLib.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_GlobalNodeOfWriterLib_0.cxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_NodeOfWriterLib.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_NodeOfWriterLib_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The library of StepData_ReadWriteModules: the writers of the entities of a protocol into a STEP file, as
/// <see cref="LibCtl_Library{TModule, TProtocol}"/> instantiated with <see cref="StepData_ReadWriteModule"/> and
/// <see cref="StepData_Protocol"/> (decision 1182: its global list is the closed generic type's own).
/// </summary>
public class StepData_WriterLib : LibCtl_Library<StepData_ReadWriteModule, StepData_Protocol>
{
    /// <summary>
    /// Creates a library which complies with a protocol: it gets the modules from the global set which are bound to
    /// the protocol and its resources.
    /// </summary>
    public StepData_WriterLib(StepData_Protocol? aprotocol) : base(aprotocol)
    {
    }

    /// <summary>A copy of another library (the same modules).</summary>
    public StepData_WriterLib(StepData_WriterLib theOther) : base(theOther)
    {
    }

    /// <summary>Creates an empty library: it will later be filled by <c>AddProtocol</c>.</summary>
    public StepData_WriterLib()
    {
    }
}
