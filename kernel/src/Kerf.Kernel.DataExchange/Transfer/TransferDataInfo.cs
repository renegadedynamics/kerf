// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_DataInfo.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_DataInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>Gives information on an object. Used as template to instantiate Mapper and SimpleBinder. This class is for Transient.</summary>
internal static class TransferDataInfo
{
    /// <summary>Returns the Type attached to an object. Here, the Dynamic Type of a Transient.</summary>
    public static Type? Type(Transient? ent)
    {
        return ent.DynamicType();
    }

    /// <summary>Returns Type Name (string). Allows to name type of non-handled objects.</summary>
    public static string TypeName(Transient? ent)
    {
        return ent.DynamicTypeName();
    }
}
