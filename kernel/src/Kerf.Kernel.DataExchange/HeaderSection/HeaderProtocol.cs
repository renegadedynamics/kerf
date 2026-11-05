// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_Protocol.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_Protocol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;

using Kerf.Engine.Exchange.Step.Data;

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>Protocol for HeaderSection Entities. It requires HeaderSection as a Resource.</summary>
internal class HeaderProtocol : StepProtocol
{
    private const string SchemaNameText = "header_section";

    public HeaderProtocol()
    {
    }

    /// <summary>Returns a Case Number for each of the HeaderSection Entities.</summary>
    public override int TypeNumber(Type? atype)
    {
        if (atype == typeof(FileName))
        {
            return 1;
        }
        else if (atype == typeof(FileDescription))
        {
            return 2;
        }
        else if (atype == typeof(FileSchema))
        {
            return 3;
        }
        else if (atype == typeof(UndefinedEntity))
        {
            return 4;
        }
        else
        {
            return 0;
        }
    }

    public override string SchemaName(InterfaceModel? model)
    {
        return SchemaNameText;
    }
}
