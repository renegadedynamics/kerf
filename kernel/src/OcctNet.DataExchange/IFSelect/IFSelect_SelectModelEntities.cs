// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectModelEntities.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectModelEntities.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// A SelectModelEntities gets all the Entities of an InterfaceModel.
/// </summary>
public class IFSelect_SelectModelEntities : IFSelect_SelectBase
{

    /// <summary>
    /// Creates a SelectModelRoot
    /// </summary>
    public IFSelect_SelectModelEntities()
    {
    }

    /// <summary>
    /// Returns the list of selected entities : the Entities of the Model (note that this result assures
    /// naturally uniqueness)
    /// </summary>
    public override Interface_EntityIterator RootResult(Interface_Graph G)
    {
        return G.Model().Entities();
    }

    /// <summary>
    /// The complete list of Entities (including shared ones) ... is exactly identical to RootResults in this
    /// case
    /// </summary>
    public override Interface_EntityIterator CompleteResult(Interface_Graph G)
    {
        return G.Model().Entities();
    }

    /// <summary>
    /// Returns a text defining the criterium : "Model Entities"
    /// </summary>
    public override string Label()
    {
        return ("All Entities from Model");
    }
}
