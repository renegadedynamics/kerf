// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Selection.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Selection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A Selection allows to define a set of Interface Entities. Entities to be put on an output file should be
/// identified in a way as independent from such or such execution as possible. This permits to handle
/// comprehensive criteria, and to replay them when a new variant of an input file has to be processed. Its
/// input can be, either an Interface Model (the very source), or another-other Selection(s) or any other
/// output. All list computations start from an input Graph (from IFGraph)
/// </summary>
internal abstract class AbstractSelection : Transient
{

    /// <summary>
    /// Returns the list of selected entities, computed from Input given as a Graph. Specific to each class of
    /// Selection Note that uniqueness of each entity is not required here This method can raise an exception as
    /// necessary
    /// </summary>
    public abstract EntityIterator RootResult(EntityGraph g);

    /// <summary>
    /// Returns the list of selected entities, each of them being unique. Default definition works from
    /// RootResult. According HasUniqueResult, UniqueResult returns directly RootResult, or build a Unique Result
    /// from it with a Graph.
    /// </summary>
    public EntityIterator UniqueResult(EntityGraph g)
    {
        EntityIterator iter = RootResult(g);
        if (HasUniqueResult || !g.ModeStat())
        {
            return iter;
        }
        EntityGraph gg = new EntityGraph(g);
        gg.GetFromIter(iter, 0);
        return new GraphContent(gg); // specialized EntityIterator (same size)
    }

    /// <summary>
    /// Returns the list of entities involved by a Selection, i.e. UniqueResult plus the shared entities
    /// (directly or not)
    /// </summary>
    public virtual EntityIterator CompleteResult(EntityGraph g)
    {
        EntityIterator iter = RootResult(g);
        //  We can use the Graph now
        EntityGraph gg = new EntityGraph(g);
        for (iter.Start(); iter.More(); iter.Next())
        {
            Transient ent = iter.Value();
            gg.GetFromEntity(ent, true); // and there we go
        }
        return new GraphContent(gg); // specialized EntityIterator (same size)
    }

    /// <summary>
    /// Puts in an Iterator the Selections from which "me" depends (there can be zero, or one, or a list).
    /// Specific to each class of Selection
    /// </summary>
    public abstract void FillIterator(SelectionIterator iter);

    /// <summary>
    /// Returns a text which defines the criterium applied by a Selection (can be used to be printed, displayed
    /// ...) Specific to each class
    /// </summary>
    public abstract string Label();

    /// <summary>
    /// Returns True if RootResult guarantees uniqueness for each Entity. Called by UniqueResult. Default answer
    /// is False. Can be redefined.
    /// </summary>
    protected virtual bool HasUniqueResult
    {
        get
        {
            return false;
        }
    }
}
