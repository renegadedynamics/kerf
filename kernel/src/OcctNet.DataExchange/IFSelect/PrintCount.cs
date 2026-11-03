// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_PrintCount.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>The manner in which a check list or a signature list is printed: messages per entity, counts per message, short lists, lists, entities, a summary, general information, mapping, result counts.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IFSelect_PrintCount;</c>.</remarks>
public enum PrintCount
{
    ItemsByEntity,
    CountByItem,
    ShortByItem,
    ListByItem,
    EntitiesByItem,
    CountSummary,
    GeneralInfo,
    Mapping,
    ResultCount,
}
