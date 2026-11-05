// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectExtract.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectExtract.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A SelectExtract determines a list of Entities from an Input Selection, as a sub-list of the Input Result
/// It works by applying a sort criterium on each Entity of the Input. This criterium can be applied Direct
/// to Pick Items (default case) or Reverse to Remove Item Basic features (the unique Input) are inherited
/// from SelectDeduct
/// </summary>
internal abstract class SelectExtract : SelectDeduct
{

    /// <summary>
    /// Returns True if Sort criterium is Direct, False if Reverse
    /// </summary>
    public bool IsDirect
    {
        get
        {
            return _thesort;
        }
    }

    /// <summary>
    /// Sets Sort criterium sense to a new value (True : Direct , False : Reverse)
    /// </summary>
    public void SetDirect(bool direct)
    {
        _thesort = direct;
    }

    /// <summary>
    /// Returns the list of selected entities. Works by calling the method Sort on each input Entity : the Entity
    /// is kept as output if Sort returns the same value as Direct status
    /// </summary>
    public override EntityIterator RootResult(EntityGraph g)
    {
        EntityIterator iter = new EntityIterator();
        EntityIterator inputer = InputResult(g); // takes everything into account
        int rank = 0;
        for (inputer.Start(); inputer.More(); inputer.Next())
        {
            Transient ent = inputer.Value();
            rank++;
            if (SortInGraph(rank, ent, g) == _thesort)
            {
                iter.GetOneItem(ent);
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns True for an Entity if it satisfies the Sort criterium It receives : - &lt;rank&gt;, the rank of
    /// the Entity in the Iteration, - &lt;ent&gt; , the Entity itself, and - &lt;model&gt;, the Starting Model
    /// Hence, the Entity to check is "model-&gt;Value(num)" (but an InterfaceModel allows other checks) This
    /// method is specific to each class of SelectExtract
    /// </summary>
    public abstract bool Sort(int rank, Transient? ent, InterfaceModel? model);

    /// <summary>
    /// Works as Sort but works on the Graph Default directly calls Sort, but it can be redefined If SortInGraph
    /// is redefined, Sort should be defined even if not called (to avoid deferred methods in a final class)
    /// </summary>
    public virtual bool SortInGraph(int rank, Transient? ent, EntityGraph g)
    {
        return Sort(rank, ent, g.Model());
    }

    /// <summary>
    /// Returns a text saying "Picked" or "Removed", plus the specific criterium returned by ExtractLabel (see
    /// below)
    /// </summary>
    public override string Label()
    {
        string labl = "";
        if (_thesort)
        {
            labl += "Picked: ";
        }
        if (!_thesort)
        {
            labl += "Removed: ";
        }
        labl += ExtractLabel();
        return labl;
    }

    /// <summary>
    /// Returns a text defining the criterium for extraction
    /// </summary>
    public abstract string ExtractLabel();

    /// <summary>
    /// Initializes a SelectExtract : enforces the sort to be Direct
    /// </summary>
    protected SelectExtract()
    {
        _thesort = true;
    }

    private bool _thesort;
}
