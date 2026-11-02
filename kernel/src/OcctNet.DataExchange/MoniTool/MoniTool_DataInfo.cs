// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_DataInfo.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_DataInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Gives information on an object Used as template to instantiate Elem, etc This class is for Transient
/// </summary>
public static class MoniTool_DataInfo
{

    /// <summary>
    /// Returns the Type attached to an object Here, the Dynamic Type of a Transient. Null Type if unknown
    /// </summary>
    public static Type? Type(Standard_Transient? ent)
    {
        return ent.DynamicType();
    }

    /// <summary>
    /// Returns Type Name (string) Allows to name type of non-handled objects
    /// </summary>
    public static string TypeName(Standard_Transient? ent)
    {
        return ent.DynamicType().Name;
    }
}
