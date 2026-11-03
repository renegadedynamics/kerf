// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_ExternalSources.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_ExternalSources.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// this class gives entities which are Source of entities of a sub-part, but are not contained by this
/// sub-part
/// </summary>
public class IFGraph_ExternalSources : Interface_GraphContent
{

    /// <summary>
    /// creates empty ExternalSources, ready to work
    /// </summary>
    public IFGraph_ExternalSources(Interface_Graph agraph)
    {
        thegraph = new Interface_Graph(agraph);

    }

    /// <summary>
    /// adds an entity and its shared ones to the list
    /// </summary>
    public void GetFromEntity(Transient? ent)
    {
        thegraph.GetFromEntity(ent, true);
    }

    /// <summary>
    /// adds a list of entities (as an iterator) with shared ones
    /// </summary>
    public void GetFromIter(Interface_EntityIterator iter)
    {
        thegraph.GetFromIter(iter, 0);
    }

    /// <summary>
    /// Allows to restart on a new data set
    /// </summary>
    public void ResetData()
    {
        Reset();
        thegraph.Reset();
    }

    /// <summary>
    /// Evaluates external sources of a set of entities
    /// </summary>
    public override void Evaluate()
    {
        Reset();
        thegraph.RemoveStatus(1);
        int nb = thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (thegraph.IsPresent(i) && thegraph.Status(i) == 0)
            {
                thegraph.GetFromIter(thegraph.Sharings(thegraph.Entity(i)), 1);
            }
        }
        GetFromGraph(thegraph, 1);
    }

    /// <summary>
    /// Returns True if no External Source are found It means that we have a "root" set (performs an Evaluation
    /// as necessary)
    /// </summary>
    public bool IsEmpty()
    {
        Evaluate();
        int nb = thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (thegraph.IsPresent(i) || thegraph.Status(i) == 1)
            {
                return false;
            }
        }
        return true;
    }

    private Interface_Graph thegraph;
}
