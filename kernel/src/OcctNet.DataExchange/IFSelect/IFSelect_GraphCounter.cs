// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_GraphCounter.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_GraphCounter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// A GraphCounter computes values to be sorted with the help of a Graph. I.E. not from a Signature The
/// default GraphCounter works with an Applied Selection (a SelectDeduct), the value is the count of selected
/// entities from each input entities)
/// </summary>
public class IFSelect_GraphCounter : IFSelect_SignCounter
{

    /// <summary>
    /// Creates a GraphCounter, without applied selection
    /// </summary>
    public IFSelect_GraphCounter(bool withmap = true, bool withlist = false) : base(withmap, withlist)
    {
    }

    /// <summary>
    /// Returns the applied selection
    /// </summary>
    public IFSelect_SelectDeduct? Applied()
    {
        return theapplied;
    }

    /// <summary>
    /// Sets a new applied selection
    /// </summary>
    public void SetApplied(IFSelect_SelectDeduct? applied)
    {
        theapplied = applied;
    }

    /// <summary>
    /// Adds a list of entities in the context given by the graph Default takes the count of entities selected by
    /// the applied selection, when it is given each entity of the list Can be redefined
    /// </summary>
    public override void AddWithGraph(NCollection_HSequence<Standard_Transient>? list, Interface_Graph graph)
    {
        if (theapplied is null)
        {
            AddList(list, graph.Model());
            return;
        }
        if (list is null)
        {
            return;
        }
        int i, nb = list.Length();
        for (i = 1; i <= nb; i++)
        {
            Standard_Transient ent = list.Value(i);
            theapplied.Alternate().SetEntity(ent);
            Interface_EntityIterator iter = theapplied.UniqueResult(graph);
            int n = iter.NbEntities();
            switch (n)
            {
                case 0:
                    Add(ent, "0");
                    break;
                case 1:
                    Add(ent, "1");
                    break;
                case 2:
                    Add(ent, "2");
                    break;
                case 3:
                    Add(ent, "3");
                    break;
                case 4:
                    Add(ent, "4");
                    break;
                case 5:
                    Add(ent, "5");
                    break;
                case 6:
                    Add(ent, "6");
                    break;
                case 7:
                    Add(ent, "7");
                    break;
                case 8:
                    Add(ent, "8");
                    break;
                case 9:
                    Add(ent, "9");
                    break;
                default:
                    Add(ent, Sprintf("%d", n));
                    break;
            }
        }
    }

    private IFSelect_SelectDeduct? theapplied;
}
