// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_EditValue.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>Controls access on values by an editor: optional (may be removed), editable (must be present), protected (access must be validated), computed (it will be recomputed), read (read only), dynamic (not a field, only displayed).</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IFSelect_EditValue;</c>.</remarks>
public enum EditValue
{
    Optional,
    Editable,
    EditProtected,
    EditComputed,
    EditRead,
    EditDynamic,
}
