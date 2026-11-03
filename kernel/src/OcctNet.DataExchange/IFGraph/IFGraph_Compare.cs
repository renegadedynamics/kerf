// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Compare.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Compare.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// this class evaluates effect of two compared sub-parts : cumulation (union), common part
/// (intersection-overlapping) part specific to first sub-part or to the second one Results are kept in a
/// Graph, several question can be set Basic Iteration gives Cumulation (union)
/// </summary>
public class IFGraph_Compare : Interface_GraphContent
{

    /// <summary>
    /// creates empty Compare, ready to work
    /// </summary>
    public IFGraph_Compare(Interface_Graph agraph)
    {
        thegraph = new Interface_Graph(agraph);

    }

    /// <summary>
    /// adds an entity and its shared ones to the list : first True means adds to the first sub-list, else to the
    /// 2nd
    /// </summary>
    public void GetFromEntity(Transient? ent, bool first)
    {
        IFGraph_AllShared iter = new IFGraph_AllShared(new Interface_Graph(thegraph.Model()), ent); // OCCT: not evaluated (Begin), so GetFromIter adds nothing
        GetFromIter(iter, first);
    }

    /// <summary>
    /// adds a list of entities (as an iterator) as such, that is, their shared entities are not considered (use
    /// AllShared to have them) first True means adds to the first sub-list, else to the 2nd
    /// </summary>
    public void GetFromIter(Interface_EntityIterator iter, bool first)
    {
        int stat = 2;
        if (first)
        {
            stat = 1;
        }
        thegraph.GetFromIter(iter, stat, 3, false);
    }

    /// <summary>
    /// merges the second list into the first one, hence the second list is empty
    /// </summary>
    public void Merge()
    {
        thegraph.ChangeStatus(2, 1);
        thegraph.ChangeStatus(3, 1);
    }

    /// <summary>
    /// Removes the contents of second list
    /// </summary>
    public void RemoveSecond()
    {
        thegraph.ChangeStatus(3, 1);
        thegraph.RemoveStatus(2);
    }

    /// <summary>
    /// Keeps only Common part, sets it as First list and clears second list
    /// </summary>
    public void KeepCommon()
    {
        thegraph.RemoveStatus(1);
        thegraph.RemoveStatus(2);
        thegraph.ChangeStatus(3, 1);
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
    /// Recomputes result of comparing to sub-parts
    /// </summary>
    public override void Evaluate()
    {
        Reset();
        GetFromGraph(thegraph); // Evaluation already done by the graph
    }

    /// <summary>
    /// returns entities common to the both parts
    /// </summary>
    public Interface_EntityIterator Common()
    {
        return new Interface_GraphContent(thegraph, 3);
    }

    /// <summary>
    /// returns entities which are exclusively in the first list
    /// </summary>
    public Interface_EntityIterator FirstOnly()
    {
        return new Interface_GraphContent(thegraph, 1);
    }

    /// <summary>
    /// returns entities which are exclusively in the second part
    /// </summary>
    public Interface_EntityIterator SecondOnly()
    {
        return new Interface_GraphContent(thegraph, 2);
    }

    private Interface_Graph thegraph;
}
