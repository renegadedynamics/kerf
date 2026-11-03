// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Dispatch.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Dispatch.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// This class allows to describe how a set of Entities has to be dispatched into resulting Packets : a
/// Packet is a sub-set of the initial set of entities. Thus, it can generate zero, one, or more Packets
/// according input set and criterium of dispatching. And it can let apart some entities : it is the
/// Remainder, which can be recovered by a specific Selection (RemainderFromDispatch). Depending of
/// sub-classes, a Dispatch can potentially generate a limited or not count of packet, and a remainder or
/// none. The input set is read from a specified Selection, attached to the Dispatch : the Final Selection of
/// the Dispatch. The input is the Unique Root Entities list of the Final Selection
/// </summary>
public abstract class IFSelect_Dispatch : Transient
{

    /// <summary>
    /// Sets a Root Name as an HAsciiString To reset it, give a Null Handle (then, a ShareOut will have to define
    /// the Default Root Name)
    /// </summary>
    public void SetRootName(TCollection_HAsciiString? name)
    {
        thename = name;
    }

    /// <summary>
    /// Returns True if a specific Root Name has been set (else, the Default Root Name has to be used)
    /// </summary>
    public bool HasRootName()
    {
        return (thename is not null);
    }

    /// <summary>
    /// Returns the Root Name for files produced by this dispatch It is empty if it has not been set or if it has
    /// been reset
    /// </summary>
    public TCollection_HAsciiString? RootName()
    {
        return thename;
    }

    /// <summary>
    /// Stores (or Changes) the Final Selection for a Dispatch
    /// </summary>
    public void SetFinalSelection(IFSelect_Selection? sel)
    {
        thefinal = sel;
    }

    /// <summary>
    /// Returns the Final Selection of a Dispatch we 'd like : C++ : return const &amp;
    /// </summary>
    public IFSelect_Selection? FinalSelection()
    {
        return thefinal;
    }

    /// <summary>
    /// Returns the complete list of source Selections (starting from FinalSelection)
    /// </summary>
    public IFSelect_SelectionIterator Selections()
    {
        IFSelect_SelectionIterator iter = new IFSelect_SelectionIterator();
        iter.AddItem(thefinal);
        for (; iter.More(); iter.Next())
        {
            iter.Value().FillIterator(iter); // Iterator that runs itself after
        }
        return iter;
    }

    /// <summary>
    /// Returns True if a Dispatch can have a Remainder, i.e. if its criterium can let entities apart. It is a
    /// potential answer, remainder can be empty at run-time even if answer is True. (to attach a
    /// RemainderFromDispatch Selection is not allowed if answer is True). Default answer given here is False
    /// (can be redefined)
    /// </summary>
    public virtual bool CanHaveRemainder()
    {
        return false;
    }

    /// <summary>
    /// Returns True if a Dispatch generates a count of Packets always less than or equal to a maximum value : it
    /// can be computed from the total count of Entities to be dispatched : &lt;nbent&gt;. If answer is False, no
    /// limited maximum is expected for account If answer is True, expected maximum is given in argument
    /// &lt;max&gt; Default answer given here is False (can be redefined)
    /// </summary>
    public virtual bool LimitedMax(int nbent, out int max)
    {
        max = 0;
        return false;
    }

    /// <summary>
    /// Returns a text which defines the way a Dispatch produces packets (which will become files) from its Input
    /// </summary>
    public abstract string Label();

    /// <summary>
    /// Gets Unique Root Entities from the Final Selection, given an input Graph This the starting step for an
    /// Evaluation (Packets - Remainder)
    /// </summary>
    public Interface_EntityIterator GetEntities(Interface_Graph G)
    {
        return thefinal.UniqueResult(G);
    }

    /// <summary>
    /// Returns the list of produced Packets into argument &lt;pack&gt;. Each Packet corresponds to a Part, the
    /// Entities listed are the Roots given by the Selection. Input is given as a Graph. Thus, to create a file
    /// from a packet, it suffices to take the entities listed in a Part of Packets (that is, a Packet) without
    /// worrying about Shared entities This method can raise an Exception if data are not coherent
    /// </summary>
    public abstract void Packets(Interface_Graph G, IFGraph_SubPartsIterator packs);

    /// <summary>
    /// Returns the list of all Input Entities (see GetEntities) which are put in a Packet. That is, Entities
    /// listed in GetEntities but not in Remainder (see below). Input is given as a Graph.
    /// </summary>
    public Interface_EntityIterator Packeted(Interface_Graph G)
    {
        Interface_EntityIterator total = GetEntities(G);
        Interface_EntityIterator remain = Remainder(G);
        if (remain.NbEntities() == 0)
        {
            return total;
        }
        //  otherwise, make the difference!
        IFGraph_Compare GC = new IFGraph_Compare(G);
        GC.GetFromIter(total, true);
        GC.GetFromIter(remain, false);
        return GC.FirstOnly();
    }

    /// <summary>
    /// Returns Remainder which is a set of Entities. Can be empty. Default evaluation is empty (has to be
    /// redefined if CanHaveRemainder is redefined to return True).
    /// </summary>
    public virtual Interface_EntityIterator Remainder(Interface_Graph G)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        return iter;
    }

    private TCollection_HAsciiString? thename;
    private IFSelect_Selection? thefinal;
}
