// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectSharing.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectSharing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A SelectSharing selects Entities which directly Share (Level One) the Entities of the Input list Remark :
/// if an Entity of the Input List directly shares another one, it is of course present in the Result List
/// </summary>
internal class SelectSharing : SelectDeduct
{

    /// <summary>
    /// Creates a SelectSharing;
    /// </summary>
    public SelectSharing()
    {
    }

    /// <summary>
    /// Returns the list of selected entities (list of entities which share (level one) those of input list)
    /// </summary>
    public override EntityIterator RootResult(EntityGraph g)
    {
        EntityIterator iter = InputResult(g);
        EntityGraph gg = new EntityGraph(g);
        for (iter.Start(); iter.More(); iter.Next())
        {
            gg.GetFromIter(g.Sharings(iter.Value()), 0);
        }
        return new GraphContent(gg);
    }

    /// <summary>
    /// Returns a text defining the criterium : "Sharing (one level)"
    /// </summary>
    public override string Label()
    {
        return ("Sharing Entities (one level)");
    }
}
