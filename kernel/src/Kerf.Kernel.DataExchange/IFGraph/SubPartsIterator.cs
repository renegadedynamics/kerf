// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_SubPartsIterator.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_SubPartsIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Graph;

/// <summary>
/// defines general form for graph classes of which result is not a single iteration on Entities, but a
/// nested one : External iteration works on sub-parts, identified by each class (according to its algorithm)
/// Internal Iteration concerns Entities of a sub-part Sub-Parts are assumed to be disjoined; if they are
/// not, the first one has priority A SubPartsIterator can work in two steps : first, load entities which
/// have to be processed then, analyse to set those entities into sub-parts
/// </summary>
internal class SubPartsIterator
{

    /// <summary>
    /// Creates with a Graph, whole or parts of it whole True : works on the entire Model whole False : empty,
    /// ready to be filled SubPartIterator is set to load entities
    /// </summary>
    public SubPartsIterator(EntityGraph agraph, bool whole)
    {
        _thegraph = new EntityGraph(agraph);
        if (whole)
        {
            _thegraph.GetFromModel();
        }
        _theparts = new NCollection_HSequence<int>();
        _thefirsts = new NCollection_HSequence<int>();
        _thepart = 0;
        _thecurr = 0;
    }

    /// <summary>
    /// Creates a SubPartIterator from another one and gets its Data Note that only non-empty sub-parts are taken
    /// into account PartNum is set to the last one
    /// </summary>
    public SubPartsIterator(SubPartsIterator other)
    {
        _thegraph = new EntityGraph(other.Graph());
        int nb = _thegraph.Size();
        _theparts = new NCollection_HSequence<int>();
        _thepart = 0;
        for (other.Start(); other.More(); other.Next())
        {
            _thepart++;
            int nbent = 0;
            GetFromIter(other.Entities());
            for (int i = 1; i <= nb; i++)
            {
                if (_thegraph.Status(i) == _thepart)
                {
                    nbent++;
                }
            }
            _theparts.Append(nbent); // empty count
        }
        _thepart = 0;
        _thecurr = 1;
    }

    /// <summary>
    /// Gets Parts from another SubPartsIterator (in addition to the ones already recorded) Error if both
    /// SubPartsIterators are not based on the same Model
    /// </summary>
    public void GetParts(SubPartsIterator other)
    {
        if (Model() != other.Model())
        {
            throw new InterfaceErrorException("SubPartsIterator : GetParts");
        }
        //  We ADD the Parts from other, without losing our own
        //  (same principle as the constructor above)
        int nb = _thegraph.Size();
        _thepart = _theparts.Length();
        for (other.Start(); other.More(); other.Next())
        {
            _thepart++;
            int nbent = 0;
            GetFromIter(other.Entities());
            for (int i = 1; i <= nb; i++)
            {
                if (_thegraph.Status(i) == _thepart)
                {
                    nbent++;
                }
            }
            _theparts.Append(nbent); // empty count
        }
    }

    /// <summary>
    /// Returns the Model with which this Iterator was created
    /// </summary>
    public InterfaceModel? Model()
    {
        return _thegraph.Model();
    }

    /// <summary>
    /// Adds an empty part and sets it to receive entities
    /// </summary>
    public void AddPart()
    {
        _theparts.Append((int)(0));
        _thepart = _theparts.Length();
    }

    /// <summary>
    /// Returns count of registered parts
    /// </summary>
    public int PartCount
    {
        get
        {
            return _theparts.Length();
        }
    }

    /// <summary>
    /// Returns numero of part which currently receives entities (0 at load time)
    /// </summary>
    public int PartNum()
    {
        return _thepart;
    }

    /// <summary>
    /// Sets SubPartIterator to get Entities (by GetFromEntity &amp; GetFromIter) into load status, to be
    /// analysed later
    /// </summary>
    public void SetLoad()
    {
        _thepart = 0;
    }

    /// <summary>
    /// Sets numero of receiving part to a new value Error if not in range (1-NbParts)
    /// </summary>
    public void SetPartNum(int num)
    {
        if (num <= 0 || num > _theparts.Length())
        {
            throw new OutOfRangeException("IFGraph_SubPartsIterator : SetPartNum");
        }
        _thepart = num;
    }

    /// <summary>
    /// Adds an Entity : into load status if in Load mode, to the current part if there is one. If shared is
    /// True, adds also its shared ones (shared at all levels)
    /// </summary>
    public void GetFromEntity(Transient? ent, bool shared)
    {
        _thegraph.GetFromEntity(ent, shared, _thepart, _thepart, false);
    }

    /// <summary>
    /// Adds a list of Entities (into Load mode or to a Part), given as an Iterator
    /// </summary>
    public void GetFromIter(EntityIterator iter)
    {
        _thegraph.GetFromIter(iter, _thepart, _thepart, false);
    }

    /// <summary>
    /// Erases data (parts, entities) : "me" becomes empty and in load status
    /// </summary>
    public void Reset()
    {
        _thegraph.Reset();
        _theparts.Clear();
        _thepart = 0;
        _thecurr = 0;
    }

    /// <summary>
    /// Called by Clear, this method allows evaluation just before iteration; its default is doing nothing, it is
    /// designed to be redefined
    /// </summary>
    public virtual void Evaluate()
    {

    }

    /// <summary>
    /// Returns entities which where loaded (not set into a sub-part)
    /// </summary>
    public GraphContent Loaded()
    {
        EntityIterator iter = new EntityIterator();
        //  int nb = thegraph.Size();
        return new GraphContent(_thegraph, 0);
    }

    /// <summary>
    /// Same as above, but under the form of a Graph
    /// </summary>
    public EntityGraph LoadedGraph()
    {
        EntityGraph g = new EntityGraph(Model());
        int nb = _thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (_thegraph.IsPresent(i) && _thegraph.Status(i) == 0)
            {
                g.GetFromEntity(_thegraph.Entity(i), false);
            }
        }
        return g;
    }

    /// <summary>
    /// Returns True if an Entity is loaded (either set into a sub-part or not)
    /// </summary>
    public bool IsLoaded(Transient? ent)
    {
        return _thegraph.IsPresent(_thegraph.EntityNumber(ent));
    }

    /// <summary>
    /// Returns True if an Entity is Present in a sub-part
    /// </summary>
    public bool IsInPart(Transient? ent)
    {
        int num = _thegraph.EntityNumber(ent);
        if (!_thegraph.IsPresent(num))
        {
            return false;
        }
        return (_thegraph.Status(num) != 0);
    }

    /// <summary>
    /// Returns number of the sub-part in which an Entity has been set if it is not in a sub-part (or not loaded
    /// at all), Returns 0
    /// </summary>
    public int EntityPartNum(Transient? ent)
    {
        int num = _thegraph.EntityNumber(ent);
        if (!_thegraph.IsPresent(num))
        {
            return 0;
        }
        return _thegraph.Status(num);
    }

    /// <summary>
    /// Sets iteration to its beginning; calls Evaluate
    /// </summary>
    public void Start()
    {
        Evaluate();
        //  Evaluate the sizes of the Parts contents
        int nb = _thegraph.Size();
        int nbp = _theparts.Length();
        if (_thepart > nbp)
        {
            _thepart = nbp;
        }
        if (nbp == 0)
        {
            _thecurr = 1;
            return;
        } // Iteration stops immediately

        //  - Perform counts (via arrays for performance)
        NCollection_Array1<int> partcounts = new NCollection_Array1<int>(1, nbp);
        partcounts.Init(0);
        NCollection_Array1<int> partfirsts = new NCollection_Array1<int>(1, nbp);
        partfirsts.Init(0);
        for (int i = 1; i <= nb; i++)
        {
            if (!_thegraph.IsPresent(i))
            {
                continue;
            }
            int nump = _thegraph.Status(i);
            if (nump < 1 || nump > nbp)
            {
                continue;
            }
            int nbent = partcounts.Value(nump);
            partcounts.SetValue(nump, nbent + 1);
            if (nbent == 0)
            {
                partfirsts.SetValue(nump, i);
            }
        }
        //  - Format them (i.e. in sequences)
        _theparts.Clear();
        _thefirsts.Clear();
        int lastp = 0;
        for (int np = 1; np <= nbp; np++)
        {
            int nbent = partcounts.Value(np);
            if (np != 0)
            {
                lastp = np;
            }
            _theparts.Append(nbent);
            _thefirsts.Append(partfirsts.Value(np));
        }
        if (lastp < nbp)
        {
            _theparts.Remove(lastp + 1, nbp);
        }
        //  Finally, prepare to iterate
        _thecurr = 1;
    }

    /// <summary>
    /// Returns True if there are more sub-parts to iterate on Note : an empty sub-part is not taken in account
    /// by Iteration
    /// </summary>
    public bool More()
    {
        if (_thecurr == 0)
        {
            Start();
        }
        return (_thecurr <= _theparts.Length());
    }

    /// <summary>
    /// Sets iteration to the next sub-part if there is not, IsSingle-Entities will raises an exception
    /// </summary>
    public void Next()
    {
        _thecurr++;
        if (_thecurr > _theparts.Length())
        {
            return;
        }
        if (_theparts.Value(_thecurr) == 0)
        {
            Next(); // skip empty parts
        }
    }

    /// <summary>
    /// Returns True if current sub-part is single (has only one Entity) Error if there is no sub-part to iterate
    /// now
    /// </summary>
    public bool IsSingle
    {
        get
        {
            if (_thecurr < 1 || _thecurr > _theparts.Length())
            {
                throw new NoSuchObjectException("IFGraph_SubPartsIterator : IsSingle");
            }
            return (_theparts.Value(_thecurr) == 1);
        }
    }

    /// <summary>
    /// Returns the first entity of current sub-part, that is for a Single one, the only one it contains Error :
    /// same as above (end of iteration)
    /// </summary>
    public Transient? FirstEntity()
    {
        if (_thecurr < 1 || _thecurr > _theparts.Length())
        {
            throw new NoSuchObjectException("IFGraph_SubPartsIterator : FirstEntity");
        }
        int nument = _thefirsts.Value(_thecurr);
        if (nument == 0)
        {
            throw new NoSuchObjectException("IFGraph_SubPartsIterator : FirstEntity (current part is empty)");
        }
        return _thegraph.Entity(nument);
    }

    /// <summary>
    /// Returns current sub-part, not as a "Value", but as an Iterator on Entities it contains Error : same as
    /// above (end of iteration)
    /// </summary>
    public EntityIterator Entities()
    {
        if (_thecurr < 1 || _thecurr > _theparts.Length())
        {
            throw new NoSuchObjectException("IFGraph_SubPartsIterator : Entities");
        }
        EntityIterator iter = new EntityIterator();
        int nb = _thegraph.Size();
        int nument = _thefirsts.Value(_thecurr);
        if (nument == 0)
        {
            return iter;
        }
        if (_theparts.Value(_thecurr) == 1)
        {
            nb = nument; // obvious: 1 single Entity
        }
        for (int i = nument; i <= nb; i++)
        {
            if (_thegraph.Status(i) == _thecurr && _thegraph.IsPresent(i))
            {
                iter.GetOneItem(_thegraph.Entity(i));
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns the Graph used by &lt;me&gt;. Used to create another SubPartsIterator from &lt;me&gt;
    /// </summary>
    private EntityGraph Graph()
    {
        return _thegraph;
    }

    protected internal EntityGraph _thegraph;
    private NCollection_HSequence<int>? _theparts;
    private NCollection_HSequence<int>? _thefirsts;
    private int _thepart;
    private int _thecurr;
}
