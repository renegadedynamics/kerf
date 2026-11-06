// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FileProtocol.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FileProtocol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// A FileProtocol is defined as the addition of several already existing Protocols. It corresponds to the
/// definition of a SchemaName with several Names, each one being attached to a specific Protocol. Thus, a File
/// defined with a compound Schema is processed as any other one, once built the equivalent compound Protocol, a
/// FileProtocol.
/// </summary>
internal class FileProtocol : StepProtocol
{
    private const string Thename = ""; // Empty schema name for file protocols

    private readonly NCollection_Sequence<Transient> _thecomps = new();

    //  Protocol factory created on demand with other Protocols

    /// <summary>Creates an empty FileProtocol.</summary>
    public FileProtocol()
    {
    }

    /// <summary>Adds a Protocol to the definition list of the FileProtocol, but does not add a Protocol of the same Type as one already in the list.</summary>
    public void Add(StepProtocol? protocol)
    {
        // Add a protocol to the collection, avoiding duplicates of the same type
        if (protocol is null)
        {
            return;
        }
        Type ptype = protocol.DynamicType();
        int nb = _thecomps.Length();
        // Check if a protocol of the same type is already present
        for (int i = 1; i <= nb; i++)
        {
            if (_thecomps.Value(i).DynamicType() == ptype)
            {
                return; // Protocol of this type already exists
            }
        }
        _thecomps.Append(protocol);
    }

    /// <summary>Gives the count of Protocols used as Resource (can be zero), i.e. the count of Protocols recorded by calling the method Add.</summary>
    public override int ResourceCount
    {
        get
        {
            // Return the number of component protocols in this file protocol
            return _thecomps.Length();
        }
    }

    /// <summary>Returns a Resource, given a rank. Here, rank of calling Add.</summary>
    public override Protocol? Resource(int num)
    {
        return _thecomps.Value(num) as Protocol;
    }

    /// <summary>Returns a Case Number, specific of each recognized Type. Here, NO ONE TYPE is Recognized by itself: the Protocols of the list are used as resources.</summary>
    public override int TypeNumber(Type? atype)
    {
        // FileProtocol doesn't recognize specific types directly (delegates to component protocols)
        return 0;
    }

    /// <summary>Calls GlobalCheck for each of its recorded resources.</summary>
    public override bool GlobalCheck(EntityGraph g, ref FrameworkCheck? ach)
    {
        // Perform global validation check across all component protocols
        bool res = false;
        int i, nb = ResourceCount;
        for (i = 1; i <= nb; i++)
        {
            res |= Resource(i).GlobalCheck(g, ref ach); // Aggregate results from all protocols
        }
        return res;
    }

    /// <summary>Returns the Schema Name attached to each class of Protocol. Here, returns an empty string.</summary>
    public override string SchemaName(InterfaceModel? model)
    {
        return Thename;
    }
}
