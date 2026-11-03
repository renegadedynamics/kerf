// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectShared.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectShared.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A SelectShared selects Entities which are directly Shared by the Entities of the Input list
/// </summary>
public class SelectShared : SelectDeduct
{

    /// <summary>
    /// Creates a SelectShared;
    /// </summary>
    public SelectShared()
    {
    }

    /// <summary>
    /// Returns the list of selected entities (list of entities shared by those of input list)
    /// </summary>
    public override EntityIterator RootResult(EntityGraph g)
    {
        EntityIterator iter = InputResult(g);
        EntityGraph gg = new EntityGraph(g);
        for (iter.Start(); iter.More(); iter.Next())
        {
            gg.GetFromIter(g.Shareds(iter.Value()), 0);
        }
        return new GraphContent(gg);
    }

    /// <summary>
    /// Returns a text defining the criterium : "Shared (one level)"
    /// </summary>
    public override string Label()
    {
        return ("Shared Entities (one level)");
    }
}
