// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectModelRoots.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectModelRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A SelectModelRoots gets all the Root Entities of an InterfaceModel. Remember that a "Root Entity" is
/// defined as having no Sharing Entity (if there is a Loop between Entities, none of them can be a "Root").
/// </summary>
internal class SelectModelRoots : SelectBase
{

    /// <summary>
    /// Creates a SelectModelRoot
    /// </summary>
    public SelectModelRoots()
    {
    }

    /// <summary>
    /// Returns the list of selected entities : the Roots of the Model (note that this result assures naturally
    /// uniqueness)
    /// </summary>
    public override EntityIterator RootResult(EntityGraph g)
    {
        return g.RootEntities();
    }

    /// <summary>
    /// Returns a text defining the criterium : "Model Roots"
    /// </summary>
    public override string Label()
    {
        return ("Root (not shared) Entities from Model");
    }
}
