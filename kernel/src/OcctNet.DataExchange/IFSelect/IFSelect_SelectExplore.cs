// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectExplore.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectExplore.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A SelectExplore determines from an input list of Entities, a list obtained by a way of exploration. This
/// implies the possibility of recursive exploration : the output list is itself reused as input, etc...
/// Examples : Shared Entities, can be considered at one level (immediate shared) or more, or max level Then,
/// for each input entity, if it is not rejected, it can be either taken itself, or explored : it then
/// produces a list. According to a level, either the produced lists or taken entities give the result (level
/// one), or lists are themselves considered and for each item, is it taken or explored. Remark that
/// rejection is just a safety : normally, an input entity is, either taken itself, or explored A maximum
/// level can be specified. Else, the process continues until all entities have been either taken or rejected
/// </summary>
public abstract class IFSelect_SelectExplore : IFSelect_SelectDeduct
{

    /// <summary>
    /// Returns the required exploring level
    /// </summary>
    public int Level()
    {
        return thelevel;
    }

    /// <summary>
    /// Returns the list of selected entities. Works by calling the method Explore on each input entity : it can
    /// be rejected, taken for output, or to explore. If the maximum level has not yet been attained, or if no
    /// max level is specified, entities to be explored are themselves used as if they were input
    /// </summary>
    public override Interface_EntityIterator RootResult(Interface_Graph G)
    {
        //  Attention, here's how we proceed
        //  We have an IndexedMapOfTransient as input (entities already processed/to process)
        //    It is initialized by InputResult
        //  And a map as output (results taken) . the result will be unique
        //  As input, a cursor of current entity
        //  For each entity, we call Explore. 3 possible cases:
        //    return False, we skip
        //    return True and empty list, we take this entity without going further
        //    return True and non-empty list, we don't take this entity but we
        //      consider its result.
        //      If last level, we take it entirely. Otherwise, it feeds the input

        int nb = G.Size();
        NCollection_IndexedMap<Transient> entrees = new NCollection_IndexedMap<Transient>();
        NCollection_IndexedMap<Transient> result = new NCollection_IndexedMap<Transient>();
        //  Initialisation
        int i, j, level = 1, ilev = 0;
        Interface_EntityIterator input = new Interface_EntityIterator();
        input = InputResult(G);
        for (input.Start(); input.More(); input.Next())
        {
            i = entrees.Add(input.Value());
        }
        ilev = entrees.Extent();

        // Exploration
        for (i = 1; i <= nb; i++)
        {
            if (i > entrees.Extent())
            {
                break;
            }
            if (i > ilev)
            {
                level++;
                if (level > thelevel && thelevel > 0)
                {
                    break;
                }
                ilev = entrees.Extent();
            }
            Transient ent = entrees.FindKey(i);
            if (ent is null)
            {
                continue;
            }
            Interface_EntityIterator exp = new Interface_EntityIterator();
            if (!Explore(level, ent, G, exp))
            {
                continue;
            }

            //  We take into account : entity to take directly ?
            //  take back input list (level not reached) or result (level reached)
            if (exp.NbEntities() == 0)
            {
                j = result.Add(ent);
                continue;
            }
            else if (level == thelevel)
            {
                for (exp.Start(); exp.More(); exp.Next())
                {
                    j = result.Add(exp.Value());
                }
            }
            else
            {
                for (exp.Start(); exp.More(); exp.Next())
                {
                    j = entrees.Add(exp.Value());
                }
            }
        }

        //  We collect the result
        Interface_EntityIterator res = new Interface_EntityIterator();
        nb = result.Extent();
        for (j = 1; j <= nb; j++)
        {
            res.AddItem(result.FindKey(j));
        }
        return res;
    }

    /// <summary>
    /// Analyses and, if required, Explores an entity, as follows : The explored list starts as empty, it has to
    /// be filled by this method. If it returns False, &lt;ent&gt; is rejected for result (this is to be used
    /// only as safety) If it returns True and &lt;explored&gt; remains empty, &lt;ent&gt; is taken itself for
    /// result, not explored If it returns True and &lt;explored&gt; is not empty, the content of this list is
    /// considered : If maximum level is attained, it is taken for result Else (or no max), each of its entity
    /// will be itself explored
    /// </summary>
    public abstract bool Explore(int level, Transient? ent, Interface_Graph G, Interface_EntityIterator explored);

    /// <summary>
    /// Returns a text saying "(Recursive)" or "(Level nn)" plus specific criterium returned by ExploreLabel (see
    /// below)
    /// </summary>
    public override string Label()
    {
        string labl = "";
        if (thelevel == 0)
        {
            labl += "(Recursive)";
        }
        else if (thelevel > 0)
        {
            labl += Sprintf("(Level %d)", thelevel);
        }
        labl += ExploreLabel();
        return labl;
    }

    /// <summary>
    /// Returns a text defining the way of exploration
    /// </summary>
    public abstract string ExploreLabel();

    /// <summary>
    /// Initializes a SelectExplore : the level must be specified on starting. 0 means all levels, 1 means level
    /// one only, etc...
    /// </summary>
    protected IFSelect_SelectExplore(int level)
    {
        thelevel = level;
    }

    private int thelevel;
}
