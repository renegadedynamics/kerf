// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectBase.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectBase.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// SelectBase works directly from an InterfaceModel : it is the first base for other Selections.
/// </summary>
public abstract class IFSelect_SelectBase : IFSelect_Selection
{

    /// <summary>
    /// Puts in an Iterator the Selections from which "me" depends This list is empty for all SelectBase type
    /// Selections
    /// </summary>
    public override void FillIterator(IFSelect_SelectionIterator iter)
    {

    }
}
