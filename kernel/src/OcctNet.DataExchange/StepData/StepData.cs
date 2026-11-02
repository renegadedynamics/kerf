// Port of: src/DataExchange/TKDESTEP/StepData/StepData.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Gives basic data definition for Step Interface. Any class of a data model described in EXPRESS Language is
/// candidate to be managed by a Step Interface.
/// </summary>
/// <remarks>
/// The protocol and the header protocol are process-wide (decision 1182's rule): <see cref="Protocol"/> is created
/// once; the header protocol is registered once by RWHeaderSection's initialisation, under a lock, before parallel use.
/// </remarks>
public static class StepData
{
    private static readonly Lazy<StepData_Protocol> proto = new(() => new StepData_Protocol());
    private static volatile StepData_Protocol? theheader;
    private static readonly Lock theheaderLock = new();

    /// <summary>Returns the recorded HeaderProtocol, which can be: a null handle if no Header Protocol was yet defined; a simple Protocol if only one was defined; a FileProtocol if more than one Protocol was defined.</summary>
    public static StepData_Protocol? HeaderProtocol()
    {
        return theheader;
    }

    /// <summary>Records a new Header Protocol (if it is not yet recorded).</summary>
    public static void AddHeaderProtocol(StepData_Protocol header)
    {
        lock (theheaderLock)
        {
            if (theheader is null)
            {
                theheader = header;
            }
            else
            {
                if (theheader is not StepData_FileProtocol headmult)
                {
                    headmult = new StepData_FileProtocol();
                    headmult.Add(theheader);
                }
                headmult.Add(header);
                theheader = headmult;
            }
        }
    }

    /// <summary>Prepares General Data required to work with this package, which are the Protocol and Modules to be loaded into Libraries.</summary>
    public static void Init()
    {
    }

    /// <summary>Returns a Protocol from StepData (avoids to create it).</summary>
    public static StepData_Protocol Protocol()
    {
        return proto.Value; // svv #2
    }
}
