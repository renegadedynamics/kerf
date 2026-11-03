// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Articulations.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Articulations.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// this class gives entities which are Articulation points in a whole Model or in a sub-part An Articulation
/// Point divides the graph in two (or more) disconnected sub-graphs Identifying Articulation Points allows
/// improving efficiency of splitting a set of Entities into sub-sets
/// </summary>
public class IFGraph_Articulations : Interface_GraphContent
{

    /// <summary>
    /// creates Articulations to evaluate a Graph whole True : works on the whole Model whole False : remains
    /// empty, ready to work on a sub-part
    /// </summary>
    public IFGraph_Articulations(Interface_Graph agraph, bool whole)
    {
        thegraph = new Interface_Graph(agraph);
        if (whole)
        {
            thegraph.GetFromModel();
        }
    }

    /// <summary>
    /// adds an entity and its shared ones to the list
    /// </summary>
    public void GetFromEntity(Transient? ent)
    {
        thegraph.GetFromEntity(ent, true);
    }

    /// <summary>
    /// adds a list of entities (as an iterator)
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
        thelist = new NCollection_HSequence<int>();
    }

    /// <summary>
    /// Evaluates the list of Articulation points
    /// </summary>
    public override void Evaluate()
    {
        //  Algorithm, see Sedgewick "Algorithms", p 392
        thelist = new NCollection_HSequence<int>();
        //  Use of Visit
        int nb = thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            thenow = 0;
            if (thegraph.IsPresent(i))
            {
                Visit(i);
            }
        }
        //  Result in thelist
        Reset();
        int nbres = thelist.Length();
        for (int ires = 1; ires <= nbres; ires++)
        {
            int num = thelist.Value(ires);
            GetOneItem(thegraph.Model().Value(num));
        }
    }

    /// <summary>
    /// basic routine of computation (see book Sedgewick "Algorithms", p 392)
    /// </summary>
    private int Visit(int num)
    {
        thenow++;
        thegraph.SetStatus(num, thenow);
        int min = thenow;

        for (Interface_EntityIterator iter = thegraph.Shareds(thegraph.Entity(num)); iter.More();
                  iter.Next())
        {
            Transient ent = iter.Value();
            int nument = thegraph.EntityNumber(ent);
            if (!thegraph.IsPresent(num))
            {
                thegraph.GetFromEntity(ent, false);
                nument = thegraph.EntityNumber(ent);
            }
            int statent = thegraph.Status(nument); // not reevaluated
            if (statent == 0)
            {
                int mm = Visit(nument);
                if (mm < min)
                {
                    min = mm;
                }
                if (mm > thegraph.Status(num))
                {
                    thelist.Append(num); // WE HAVE ONE: num
                }
            }
            else if (statent < min)
            {
                min = statent;
            }
        }
        return min;
    }

    private Interface_Graph thegraph;
    private int thenow;
    private NCollection_HSequence<int>? thelist;
}
