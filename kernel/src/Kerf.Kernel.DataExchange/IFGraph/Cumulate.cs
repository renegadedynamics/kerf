// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Cumulate.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Cumulate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// this class evaluates effect of cumulated sub-parts : overlapping, forgotten entities Results are kept in
/// a Graph, several question can be set Basic Iteration gives entities which are part of Cumulation
/// </summary>
public class Cumulate : GraphContent
{

    /// <summary>
    /// creates empty Cumulate, ready to work
    /// </summary>
    public Cumulate(EntityGraph agraph)
    {
        _thegraph = new EntityGraph(agraph);

    }

    /// <summary>
    /// adds an entity and its shared ones to the list
    /// </summary>
    public void GetFromEntity(Transient? ent)
    {
        AllShared iter = new AllShared(new EntityGraph(_thegraph.Model()), ent); // OCCT: not evaluated (Begin), so GetFromIter adds nothing
        GetFromIter(iter);
    }

    /// <summary>
    /// adds a list of entities (as an iterator) as such, that is, without their shared entities (use AllShared
    /// to have them)
    /// </summary>
    public void GetFromIter(EntityIterator iter)
    {
        _thegraph.GetFromIter(iter, 1, 1, true);
        _thegraph.ChangeStatus(1, 2); // once the calculation is done
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
    /// Evaluates the result of cumulation
    /// </summary>
    public override void Evaluate()
    {
        Reset();
        GetFromGraph(_thegraph); // evaluation already done in the graph
    }

    /// <summary>
    /// returns entities which are taken several times
    /// </summary>
    public EntityIterator Overlapped()
    {
        EntityIterator iter = new EntityIterator();
        int nb = _thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (_thegraph.IsPresent(i) && _thegraph.Status(i) > 2)
            {
                iter.GetOneItem(_thegraph.Entity(i));
            }
        }
        return iter;
    }

    /// <summary>
    /// returns entities which are not taken
    /// </summary>
    public EntityIterator Forgotten()
    {
        EntityIterator iter = new EntityIterator();
        int nb = _thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (!_thegraph.IsPresent(i))
            {
                iter.GetOneItem(_thegraph.Model().Value(i));
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns entities taken a given count of times (0 : same as Forgotten, 1 : same as no Overlap : default)
    /// </summary>
    public EntityIterator PerCount(int count = 1)
    {
        EntityIterator iter = new EntityIterator();
        int nb = _thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (_thegraph.IsPresent(i) && _thegraph.Status(i) == (count + 1))
            {
                iter.GetOneItem(_thegraph.Model().Value(i));
            }
        }
        return iter;
    }

    /// <summary>
    /// returns number of times an Entity has been counted (0 means forgotten, more than 1 means overlap, 1 is
    /// normal)
    /// </summary>
    public int NbTimes(Transient? ent)
    {
        int num = _thegraph.EntityNumber(ent);
        if (num == 0)
        {
            return 0;
        }
        int stat = _thegraph.Status(num);
        return stat - 1;
    }

    /// <summary>
    /// Returns the highest number of times recorded for every Entity (0 means empty, 1 means no overlap)
    /// </summary>
    public int HighestNbTimes()
    {
        int max = 0;
        int nb = _thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (!_thegraph.IsPresent(i))
            {
                continue;
            }
            int count = _thegraph.Status(i) - 1;
            if (count > max)
            {
                max = count;
            }
        }
        return max;
    }

    private EntityGraph _thegraph;
}
