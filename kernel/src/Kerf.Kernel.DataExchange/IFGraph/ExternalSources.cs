// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_ExternalSources.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_ExternalSources.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// this class gives entities which are Source of entities of a sub-part, but are not contained by this
/// sub-part
/// </summary>
public class ExternalSources : GraphContent
{

    /// <summary>
    /// creates empty ExternalSources, ready to work
    /// </summary>
    public ExternalSources(EntityGraph agraph)
    {
        _thegraph = new EntityGraph(agraph);

    }

    /// <summary>
    /// adds an entity and its shared ones to the list
    /// </summary>
    public void GetFromEntity(Transient? ent)
    {
        _thegraph.GetFromEntity(ent, true);
    }

    /// <summary>
    /// adds a list of entities (as an iterator) with shared ones
    /// </summary>
    public void GetFromIter(EntityIterator iter)
    {
        _thegraph.GetFromIter(iter, 0);
    }

    /// <summary>
    /// Allows to restart on a new data set
    /// </summary>
    public void ResetData()
    {
        Reset();
        _thegraph.Reset();
    }

    /// <summary>
    /// Evaluates external sources of a set of entities
    /// </summary>
    public override void Evaluate()
    {
        Reset();
        _thegraph.RemoveStatus(1);
        int nb = _thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (_thegraph.IsPresent(i) && _thegraph.Status(i) == 0)
            {
                _thegraph.GetFromIter(_thegraph.Sharings(_thegraph.Entity(i)), 1);
            }
        }
        GetFromGraph(_thegraph, 1);
    }

    /// <summary>
    /// Returns True if no External Source are found It means that we have a "root" set (performs an Evaluation
    /// as necessary)
    /// </summary>
    public bool IsEmpty()
    {
        Evaluate();
        int nb = _thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (_thegraph.IsPresent(i) || _thegraph.Status(i) == 1)
            {
                return false;
            }
        }
        return true;
    }

    private EntityGraph _thegraph;
}
