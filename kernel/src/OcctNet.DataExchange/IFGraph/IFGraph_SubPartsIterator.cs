// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_SubPartsIterator.hxx
// Port of: src/DataExchange/TKXSBase/IFGraph/IFGraph_SubPartsIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// defines general form for graph classes of which result is not a single iteration on Entities, but a
/// nested one : External iteration works on sub-parts, identified by each class (according to its algorithm)
/// Internal Iteration concerns Entities of a sub-part Sub-Parts are assumed to be disjoined; if they are
/// not, the first one has priority A SubPartsIterator can work in two steps : first, load entities which
/// have to be processed then, analyse to set those entities into sub-parts
/// </summary>
public class IFGraph_SubPartsIterator
{

    /// <summary>
    /// Creates with a Graph, whole or parts of it whole True : works on the entire Model whole False : empty,
    /// ready to be filled SubPartIterator is set to load entities
    /// </summary>
    public IFGraph_SubPartsIterator(Interface_Graph agraph, bool whole)
    {
        thegraph = new Interface_Graph(agraph);
        if (whole)
        {
            thegraph.GetFromModel();
        }
        theparts = new NCollection_HSequence<int>();
        thefirsts = new NCollection_HSequence<int>();
        thepart = 0;
        thecurr = 0;
    }

    /// <summary>
    /// Creates a SubPartIterator from another one and gets its Data Note that only non-empty sub-parts are taken
    /// into account PartNum is set to the last one
    /// </summary>
    public IFGraph_SubPartsIterator(IFGraph_SubPartsIterator other)
    {
        thegraph = new Interface_Graph(other.Graph());
        int nb = thegraph.Size();
        theparts = new NCollection_HSequence<int>();
        thepart = 0;
        for (other.Start(); other.More(); other.Next())
        {
            thepart++;
            int nbent = 0;
            GetFromIter(other.Entities());
            for (int i = 1; i <= nb; i++)
            {
                if (thegraph.Status(i) == thepart)
                {
                    nbent++;
                }
            }
            theparts.Append(nbent); // empty count
        }
        thepart = 0;
        thecurr = 1;
    }

    /// <summary>
    /// Gets Parts from another SubPartsIterator (in addition to the ones already recorded) Error if both
    /// SubPartsIterators are not based on the same Model
    /// </summary>
    public void GetParts(IFGraph_SubPartsIterator other)
    {
        if (Model() != other.Model())
        {
            throw new Interface_InterfaceError("SubPartsIterator : GetParts");
        }
        //  We ADD the Parts from other, without losing our own
        //  (same principle as the constructor above)
        int nb = thegraph.Size();
        thepart = theparts.Length();
        for (other.Start(); other.More(); other.Next())
        {
            thepart++;
            int nbent = 0;
            GetFromIter(other.Entities());
            for (int i = 1; i <= nb; i++)
            {
                if (thegraph.Status(i) == thepart)
                {
                    nbent++;
                }
            }
            theparts.Append(nbent); // empty count
        }
    }

    /// <summary>
    /// Returns the Model with which this Iterator was created
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return thegraph.Model();
    }

    /// <summary>
    /// Adds an empty part and sets it to receive entities
    /// </summary>
    public void AddPart()
    {
        theparts.Append((int)(0));
        thepart = theparts.Length();
    }

    /// <summary>
    /// Returns count of registered parts
    /// </summary>
    public int NbParts()
    {
        return theparts.Length();
    }

    /// <summary>
    /// Returns numero of part which currently receives entities (0 at load time)
    /// </summary>
    public int PartNum()
    {
        return thepart;
    }

    /// <summary>
    /// Sets SubPartIterator to get Entities (by GetFromEntity &amp; GetFromIter) into load status, to be
    /// analysed later
    /// </summary>
    public void SetLoad()
    {
        thepart = 0;
    }

    /// <summary>
    /// Sets numero of receiving part to a new value Error if not in range (1-NbParts)
    /// </summary>
    public void SetPartNum(int num)
    {
        if (num <= 0 || num > theparts.Length())
        {
            throw new OutOfRangeException("IFGraph_SubPartsIterator : SetPartNum");
        }
        thepart = num;
    }

    /// <summary>
    /// Adds an Entity : into load status if in Load mode, to the current part if there is one. If shared is
    /// True, adds also its shared ones (shared at all levels)
    /// </summary>
    public void GetFromEntity(Transient? ent, bool shared)
    {
        thegraph.GetFromEntity(ent, shared, thepart, thepart, false);
    }

    /// <summary>
    /// Adds a list of Entities (into Load mode or to a Part), given as an Iterator
    /// </summary>
    public void GetFromIter(Interface_EntityIterator iter)
    {
        thegraph.GetFromIter(iter, thepart, thepart, false);
    }

    /// <summary>
    /// Erases data (parts, entities) : "me" becomes empty and in load status
    /// </summary>
    public void Reset()
    {
        thegraph.Reset();
        theparts.Clear();
        thepart = 0;
        thecurr = 0;
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
    public Interface_GraphContent Loaded()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        //  int nb = thegraph.Size();
        return new Interface_GraphContent(thegraph, 0);
    }

    /// <summary>
    /// Same as above, but under the form of a Graph
    /// </summary>
    public Interface_Graph LoadedGraph()
    {
        Interface_Graph G = new Interface_Graph(Model());
        int nb = thegraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (thegraph.IsPresent(i) && thegraph.Status(i) == 0)
            {
                G.GetFromEntity(thegraph.Entity(i), false);
            }
        }
        return G;
    }

    /// <summary>
    /// Returns True if an Entity is loaded (either set into a sub-part or not)
    /// </summary>
    public bool IsLoaded(Transient? ent)
    {
        return thegraph.IsPresent(thegraph.EntityNumber(ent));
    }

    /// <summary>
    /// Returns True if an Entity is Present in a sub-part
    /// </summary>
    public bool IsInPart(Transient? ent)
    {
        int num = thegraph.EntityNumber(ent);
        if (!thegraph.IsPresent(num))
        {
            return false;
        }
        return (thegraph.Status(num) != 0);
    }

    /// <summary>
    /// Returns number of the sub-part in which an Entity has been set if it is not in a sub-part (or not loaded
    /// at all), Returns 0
    /// </summary>
    public int EntityPartNum(Transient? ent)
    {
        int num = thegraph.EntityNumber(ent);
        if (!thegraph.IsPresent(num))
        {
            return 0;
        }
        return thegraph.Status(num);
    }

    /// <summary>
    /// Sets iteration to its beginning; calls Evaluate
    /// </summary>
    public void Start()
    {
        Evaluate();
        //  Evaluate the sizes of the Parts contents
        int nb = thegraph.Size();
        int nbp = theparts.Length();
        if (thepart > nbp)
        {
            thepart = nbp;
        }
        if (nbp == 0)
        {
            thecurr = 1;
            return;
        } // Iteration stops immediately

        //  - Perform counts (via arrays for performance)
        NCollection_Array1<int> partcounts = new NCollection_Array1<int>(1, nbp);
        partcounts.Init(0);
        NCollection_Array1<int> partfirsts = new NCollection_Array1<int>(1, nbp);
        partfirsts.Init(0);
        for (int i = 1; i <= nb; i++)
        {
            if (!thegraph.IsPresent(i))
            {
                continue;
            }
            int nump = thegraph.Status(i);
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
        theparts.Clear();
        thefirsts.Clear();
        int lastp = 0;
        for (int np = 1; np <= nbp; np++)
        {
            int nbent = partcounts.Value(np);
            if (np != 0)
            {
                lastp = np;
            }
            theparts.Append(nbent);
            thefirsts.Append(partfirsts.Value(np));
        }
        if (lastp < nbp)
        {
            theparts.Remove(lastp + 1, nbp);
        }
        //  Finally, prepare to iterate
        thecurr = 1;
    }

    /// <summary>
    /// Returns True if there are more sub-parts to iterate on Note : an empty sub-part is not taken in account
    /// by Iteration
    /// </summary>
    public bool More()
    {
        if (thecurr == 0)
        {
            Start();
        }
        return (thecurr <= theparts.Length());
    }

    /// <summary>
    /// Sets iteration to the next sub-part if there is not, IsSingle-Entities will raises an exception
    /// </summary>
    public void Next()
    {
        thecurr++;
        if (thecurr > theparts.Length())
        {
            return;
        }
        if (theparts.Value(thecurr) == 0)
        {
            Next(); // skip empty parts
        }
    }

    /// <summary>
    /// Returns True if current sub-part is single (has only one Entity) Error if there is no sub-part to iterate
    /// now
    /// </summary>
    public bool IsSingle()
    {
        if (thecurr < 1 || thecurr > theparts.Length())
        {
            throw new NoSuchObjectException("IFGraph_SubPartsIterator : IsSingle");
        }
        return (theparts.Value(thecurr) == 1);
    }

    /// <summary>
    /// Returns the first entity of current sub-part, that is for a Single one, the only one it contains Error :
    /// same as above (end of iteration)
    /// </summary>
    public Transient? FirstEntity()
    {
        if (thecurr < 1 || thecurr > theparts.Length())
        {
            throw new NoSuchObjectException("IFGraph_SubPartsIterator : FirstEntity");
        }
        int nument = thefirsts.Value(thecurr);
        if (nument == 0)
        {
            throw new NoSuchObjectException("IFGraph_SubPartsIterator : FirstEntity (current part is empty)");
        }
        return thegraph.Entity(nument);
    }

    /// <summary>
    /// Returns current sub-part, not as a "Value", but as an Iterator on Entities it contains Error : same as
    /// above (end of iteration)
    /// </summary>
    public Interface_EntityIterator Entities()
    {
        if (thecurr < 1 || thecurr > theparts.Length())
        {
            throw new NoSuchObjectException("IFGraph_SubPartsIterator : Entities");
        }
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int nb = thegraph.Size();
        int nument = thefirsts.Value(thecurr);
        if (nument == 0)
        {
            return iter;
        }
        if (theparts.Value(thecurr) == 1)
        {
            nb = nument; // obvious: 1 single Entity
        }
        for (int i = nument; i <= nb; i++)
        {
            if (thegraph.Status(i) == thecurr && thegraph.IsPresent(i))
            {
                iter.GetOneItem(thegraph.Entity(i));
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns the Graph used by &lt;me&gt;. Used to create another SubPartsIterator from &lt;me&gt;
    /// </summary>
    private Interface_Graph Graph()
    {
        return thegraph;
    }

    protected internal Interface_Graph thegraph;
    private NCollection_HSequence<int>? theparts;
    private NCollection_HSequence<int>? thefirsts;
    private int thepart;
    private int thecurr;
}
