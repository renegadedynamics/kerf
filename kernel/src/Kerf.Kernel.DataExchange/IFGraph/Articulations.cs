// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Articulations.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_Articulations.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// this class gives entities which are Articulation points in a whole Model or in a sub-part An Articulation
/// Point divides the graph in two (or more) disconnected sub-graphs Identifying Articulation Points allows
/// improving efficiency of splitting a set of Entities into sub-sets
/// </summary>
public class Articulations : GraphContent
{

    /// <summary>
    /// creates Articulations to evaluate a Graph whole True : works on the whole Model whole False : remains
    /// empty, ready to work on a sub-part
    /// </summary>
    public Articulations(EntityGraph agraph, bool whole)
    {
        _thegraph = new EntityGraph(agraph);
        if (whole)
        {
            _thegraph.GetFromModel();
        }
    }

    /// <summary>
    /// adds an entity and its shared ones to the list
    /// </summary>
    public void GetFromEntity(Transient? ent)
    {
        _thegraph.GetFromEntity(ent, true);
    }

    /// <summary>
    /// adds a list of entities (as an iterator)
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
        _thelist = new NCollection_HSequence<int>();
    }

    /// <summary>
    /// Evaluates the list of Articulation points
    /// </summary>
    public override void Evaluate()
    {
        //  Algorithm, see Sedgewick "Algorithms", p 392
        _thelist = new NCollection_HSequence<int>();
        //  Use of Visit
        int nb = _thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            _thenow = 0;
            if (_thegraph.IsPresent(i))
            {
                Visit(i);
            }
        }
        //  Result in thelist
        Reset();
        int nbres = _thelist.Length();
        for (int ires = 1; ires <= nbres; ires++)
        {
            int num = _thelist.Value(ires);
            GetOneItem(_thegraph.Model().Value(num));
        }
    }

    /// <summary>
    /// basic routine of computation (see book Sedgewick "Algorithms", p 392)
    /// </summary>
    private int Visit(int num)
    {
        _thenow++;
        _thegraph.SetStatus(num, _thenow);
        int min = _thenow;

        for (EntityIterator iter = _thegraph.Shareds(_thegraph.Entity(num)); iter.More();
                  iter.Next())
        {
            Transient ent = iter.Value();
            int nument = _thegraph.EntityNumber(ent);
            if (!_thegraph.IsPresent(num))
            {
                _thegraph.GetFromEntity(ent, false);
                nument = _thegraph.EntityNumber(ent);
            }
            int statent = _thegraph.Status(nument); // not reevaluated
            if (statent == 0)
            {
                int mm = Visit(nument);
                if (mm < min)
                {
                    min = mm;
                }
                if (mm > _thegraph.Status(num))
                {
                    _thelist.Append(num); // WE HAVE ONE: num
                }
            }
            else if (statent < min)
            {
                min = statent;
            }
        }
        return min;
    }

    private EntityGraph _thegraph;
    private int _thenow;
    private NCollection_HSequence<int>? _thelist;
}
