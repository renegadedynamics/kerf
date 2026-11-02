// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_Protocol.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_Protocol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Protocol for HeaderSection Entities. It requires HeaderSection as a Resource.</summary>
public class HeaderSection_Protocol : StepData_Protocol
{
    private const string schemaName = "header_section";

    public HeaderSection_Protocol()
    {
    }

    /// <summary>Returns a Case Number for each of the HeaderSection Entities.</summary>
    public override int TypeNumber(Type? atype)
    {
        if (atype == typeof(HeaderSection_FileName))
        {
            return 1;
        }
        else if (atype == typeof(HeaderSection_FileDescription))
        {
            return 2;
        }
        else if (atype == typeof(HeaderSection_FileSchema))
        {
            return 3;
        }
        else if (atype == typeof(StepData_UndefinedEntity))
        {
            return 4;
        }
        else
        {
            return 0;
        }
    }

    public override string SchemaName(Interface_InterfaceModel? theModel)
    {
        return schemaName;
    }
}
