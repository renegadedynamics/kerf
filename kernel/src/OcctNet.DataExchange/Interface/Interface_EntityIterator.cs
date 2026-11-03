// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityIterator.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_EntityIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Defines an Iterator on Entities. Allows considering of various criteria
/// </summary>
public class Interface_EntityIterator
{

    /// <summary>
    /// Defines an empty iterator (see AddList &amp; AddItem)
    /// </summary>
    public Interface_EntityIterator()
    {
        //  thecurr = new Interface_IntVal();
        //  thecurr.CValue() = 0;
        //  thelist = new NCollection_HSequence<Standard_Transient>();  // empty constructor
        //  thelist sera construit au premier Add (quelquefois, il nyena pas)
    }

    /// <summary>
    /// Defines an iterator on a list, directly i.e. without copying it
    /// </summary>
    public Interface_EntityIterator(NCollection_HSequence<Transient>? list)
    {
        thecurr = new Interface_IntVal();
        thecurr.CValue() = 0;
        thelist = list;
    }

    /// <summary>
    /// Gets a list of entities and adds its to the iteration list
    /// </summary>
    public void AddList(NCollection_HSequence<Transient>? list)
    {
        if (thelist is null)
        {
            thelist = new NCollection_HSequence<Transient>();
        }
        if (thecurr is null)
        {
            thecurr = new Interface_IntVal();
        }
        thecurr.CValue() = 0;
        thelist.Append(list);
    }

    /// <summary>
    /// Adds to the iteration list a defined entity
    /// </summary>
    public void AddItem(Transient? anentity)
    {
        if (anentity is null)
        {
            return;
        }
        if (thecurr is null)
        {
            thecurr = new Interface_IntVal();
        }
        if (thelist is null)
        {
            thelist = new NCollection_HSequence<Transient>();
        }
        thecurr.CValue() = 0;
        thelist.Append(anentity);
    }

    /// <summary>
    /// same as AddItem (kept for compatibility)
    /// </summary>
    public void GetOneItem(Transient? anentity)
    {
        AddItem(anentity);
    }

    /// <summary>
    /// Selects entities with are Kind of a given type, keep only them (is keep is True) or reject only them (if
    /// keep is False)
    /// </summary>
    public void SelectType(Type? atype, bool keep)
    {
        if (thelist is null)
        {
            return;
        }
        int i, n = thelist.Length();
        NCollection_HSequence<Transient> nlist =
            new NCollection_HSequence<Transient>();
        for (i = 1; i <= n; i++)
        {
            if (thelist.Value(i).IsKind(atype) == keep)
            {
                nlist.Append(thelist.Value(i));
            }
        }
        thelist = nlist;
    }

    /// <summary>
    /// Returns count of entities which will be iterated on Calls Start if not yet done
    /// </summary>
    public int NbEntities()
    {
        if (thelist is null)
        {
            return 0;
        }
        if (thecurr is not null && thecurr.Value() == 0)
        {
            Start();
        }
        return thelist.Length();
    }

    /// <summary>
    /// Returns count of entities of a given type (kind of)
    /// </summary>
    public int NbTyped(Type? atype)
    {
        int res = 0;
        if (thelist is null)
        {
            return res;
        }
        int i, n = thelist.Length();
        for (i = 1; i <= n; i++)
        {
            if (thelist.Value(i).IsKind(atype))
            {
                res++;
            }
        }
        return res;
    }

    /// <summary>
    /// Returns the list of entities of a given type (kind of)
    /// </summary>
    public Interface_EntityIterator Typed(Type? atype)
    {
        Interface_EntityIterator res = new Interface_EntityIterator();
        if (thelist is null)
        {
            return res;
        }
        int i, n = thelist.Length();
        for (i = 1; i <= n; i++)
        {
            if (thelist.Value(i).IsKind(atype))
            {
                res.AddItem(thelist.Value(i));
            }
        }
        return res;
    }

    /// <summary>
    /// Allows re-iteration (useless for the first iteration)
    /// </summary>
    public virtual void Start()
    {
        if (thecurr is not null)
        {
            thecurr.CValue() = 1;
        }
    }

    /// <summary>
    /// Says if there are other entities (vertices) to iterate the first time, calls Start
    /// </summary>
    public bool More()
    {
        if (thecurr is null)
        {
            return false;
        }
        if (thecurr.Value() == 0)
        {
            Start(); // iteration preparation
        }
        if (thelist is null)
        {
            return false;
        }
        return (thecurr.Value() <= thelist.Length());
    }

    /// <summary>
    /// Sets iteration to the next entity (vertex) to give
    /// </summary>
    public void Next()
    {
        thecurr.CValue()++;
    }

    /// <summary>
    /// Returns the current Entity iterated, to be used by Interface tools
    /// </summary>
    public Transient? Value()
    {
        //  NbEntity not (we don't know how it is implemented after all)
        if (thelist is null)
        {
            throw new NoSuchObjectException("Interface_EntityIterator");
        }
        if (thecurr.Value() < 1 || thecurr.Value() > thelist.Length())
        {
            throw new NoSuchObjectException("Interface_EntityIterator");
        }
        return thelist.Value(thecurr.Value());
    }

    /// <summary>
    /// Returns the content of the Iterator, accessed through a Handle to be used by a frontal-engine logic
    /// Returns an empty Sequence if the Iterator is empty Calls Start if not yet done
    /// </summary>
    public NCollection_HSequence<Transient>? Content()
    {
        if (thecurr is not null && thecurr.Value() == 0)
        {
            Start();
        }
        if (thelist is null)
        {
            return new NCollection_HSequence<Transient>(); // empty
        }
        return thelist;
    }

    /// <summary>
    /// Clears data of iteration
    /// </summary>
    public void Destroy()
    {
        thecurr = null;
    }


    /// <summary>
    /// Allows subclasses of EntityIterator to reevaluate an iteration
    /// </summary>
    protected internal void Reset()
    {
        if (thecurr is null)
        {
            thecurr = new Interface_IntVal();
        }
        thecurr.CValue() = 0;
        thelist = new NCollection_HSequence<Transient>();
    }


    private Interface_IntVal? thecurr;
    private NCollection_HSequence<Transient>? thelist;
}
