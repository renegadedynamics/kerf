// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Cumulate.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Cumulate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// this class evaluates effect of cumulated sub-parts : overlapping, forgotten entities Results are kept in
/// a Graph, several question can be set Basic Iteration gives entities which are part of Cumulation
/// </summary>
public class IFGraph_Cumulate : Interface_GraphContent
{

    /// <summary>
    /// creates empty Cumulate, ready to work
    /// </summary>
    public IFGraph_Cumulate(Interface_Graph agraph)
    {
        thegraph = new Interface_Graph(agraph);

    }

    /// <summary>
    /// adds an entity and its shared ones to the list
    /// </summary>
    public void GetFromEntity(Standard_Transient? ent)
    {
        IFGraph_AllShared iter = new IFGraph_AllShared(new Interface_Graph(thegraph.Model()), ent); // OCCT: not evaluated (Begin), so GetFromIter adds nothing
        GetFromIter(iter);
    }

    /// <summary>
    /// adds a list of entities (as an iterator) as such, that is, without their shared entities (use AllShared
    /// to have them)
    /// </summary>
    public void GetFromIter(Interface_EntityIterator iter)
    {
        thegraph.GetFromIter(iter, 1, 1, true);
        thegraph.ChangeStatus(1, 2); // once the calculation is done
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
    /// Evaluates the result of cumulation
    /// </summary>
    public override void Evaluate()
    {
        Reset();
        GetFromGraph(thegraph); // evaluation already done in the graph
    }

    /// <summary>
    /// returns entities which are taken several times
    /// </summary>
    public Interface_EntityIterator Overlapped()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int nb = thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (thegraph.IsPresent(i) && thegraph.Status(i) > 2)
            {
                iter.GetOneItem(thegraph.Entity(i));
            }
        }
        return iter;
    }

    /// <summary>
    /// returns entities which are not taken
    /// </summary>
    public Interface_EntityIterator Forgotten()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int nb = thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (!thegraph.IsPresent(i))
            {
                iter.GetOneItem(thegraph.Model().Value(i));
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns entities taken a given count of times (0 : same as Forgotten, 1 : same as no Overlap : default)
    /// </summary>
    public Interface_EntityIterator PerCount(int count = 1)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int nb = thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (thegraph.IsPresent(i) && thegraph.Status(i) == (count + 1))
            {
                iter.GetOneItem(thegraph.Model().Value(i));
            }
        }
        return iter;
    }

    /// <summary>
    /// returns number of times an Entity has been counted (0 means forgotten, more than 1 means overlap, 1 is
    /// normal)
    /// </summary>
    public int NbTimes(Standard_Transient? ent)
    {
        int num = thegraph.EntityNumber(ent);
        if (num == 0)
        {
            return 0;
        }
        int stat = thegraph.Status(num);
        return stat - 1;
    }

    /// <summary>
    /// Returns the highest number of times recorded for every Entity (0 means empty, 1 means no overlap)
    /// </summary>
    public int HighestNbTimes()
    {
        int max = 0;
        int nb = thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (!thegraph.IsPresent(i))
            {
                continue;
            }
            int count = thegraph.Status(i) - 1;
            if (count > max)
            {
                max = count;
            }
        }
        return max;
    }

    private Interface_Graph thegraph;
}
