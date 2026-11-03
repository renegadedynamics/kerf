// Port of: src/DataExchange/TKXSBase/Interface/Interface_Graph.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Graph.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Gives basic data structure for operating and storing graph results (usage is normally internal) Entities
/// are Mapped according their Number in the Model Each Entity from the Model can be known as "Present" or
/// not; if it is, it is Mapped with a Status : an Integer which can be used according to needs of each
/// algorithm In addition, the Graph brings a BitMap which can be used by any caller Also, it is bound with
/// two lists : a list of Shared Entities (in fact, their Numbers in the Model) which is filled by a
/// ShareTool, and a list of Sharing Entities, computed by deduction from the Shared Lists Moreover, it is
/// possible to redefine the list of Entities Shared by an Entity (instead of standard answer by general
/// service Shareds) : this new list can be empty; it can be changed or reset (i.e. to come back to standard
/// answer)
/// </summary>
public class EntityGraph
{
    private static void AddTypedSharings(Transient ent,
                                         Type type,
                                         EntityIterator iter,
                                         int n,
                                         EntityGraph g)
    {
        if (ent is null)
        {
            return;
        }
        if (ent.IsKind(type))
        {
            iter.AddItem(ent);
            return;
        }
        if (iter.NbEntities() > n)
        {
            return;
        }

        NCollection_HSequence<Transient> list = g.GetSharings(ent);
        if (list is null)
        {
            return;
        }

        int nb = list.Length();
        for (int i = 1; i <= nb; i++)
        {
            AddTypedSharings(list.Value(i), type, iter, nb, g);
        }
    }

    /// <summary>
    /// Creates an empty graph, ready to receive Entities from amodel Note that this way of Creation allows
    /// &lt;me&gt; to verify that Entities to work with are contained in &lt;amodel&gt; Basic Shared and Sharing
    /// lists are obtained from a General Services Library, given directly as an argument
    /// </summary>
    public EntityGraph(InterfaceModel? amodel, GeneralLib lib, bool modeStat = true)
    {
        _themodel = amodel;
        _thepresents = "";
        if (modeStat)
        {
            InitStats();
        }
        Evaluate();
    }

    /// <summary>
    /// Same as above, but the Library is defined through a Protocol
    /// </summary>
    public EntityGraph(InterfaceModel? amodel, Protocol? protocol, bool modeStat = true)
    {
        _themodel = amodel;
        _thepresents = "";
        if (modeStat)
        {
            InitStats();
        }
        Evaluate();
    }

    /// <summary>
    /// Same as above, but the Library is defined through a Protocol
    /// </summary>
    public EntityGraph(InterfaceModel? amodel, GTool? gtool, bool modeStat = true)
    {
        _themodel = amodel;
        _thepresents = "";
        if (modeStat)
        {
            InitStats();
        }
        Evaluate();
    }

    /// <summary>
    /// Same a above but works with the Protocol recorded in the Model
    /// </summary>
    public EntityGraph(InterfaceModel? amodel, bool modeStat = true)
    {
        _themodel = amodel;
        _thepresents = "";
        if (modeStat)
        {
            InitStats();
        }
        Evaluate();
    }

    /// <summary>
    /// Creates a Graph from another one, getting all its data Remark that status are copied from &lt;agraph&gt;,
    /// but the other lists (sharing/shared) are copied only if &lt;copied&gt; = True
    /// </summary>
    public EntityGraph(EntityGraph agraph, bool copied = false)
    {
        _themodel = agraph.Model();
        _thepresents = "";
        _thesharings = agraph.SharingTable();
        int nb = agraph.StatuseCount;
        if (nb == 0)
        {
            return;
        }
        if (_thestats is null)
        {
            _thestats = new NCollection_HArray1<int>(1, nb);
        }
        for (int i = 1; i <= nb; i++)
        {
            _thestats.SetValue(i, agraph.Status(i));
        }
        _theflags.Initialize(agraph.BitMap(), true);
    }

    /// <summary>
    /// Erases data, making graph ready to rebegin from void (also resets Shared lists redefinitions)
    /// </summary>
    public void Reset()
    {
        if (_thestats is not null)
        {
            _thestats = null;
            _theflags.Init(false, Graph_Present);
        }
    }

    /// <summary>
    /// Erases Status (Values and Flags of Presence), making graph ready to rebegin from void. Does not concerns
    /// Shared lists
    /// </summary>
    public void ResetStatus()
    {
        if (_thestats is not null)
        {
            _thestats.Init(0);
            _theflags.Init(false, Graph_Present);
        }
    }

    /// <summary>
    /// Returns size (max nb of entities, i.e. Model's nb of entities)
    /// </summary>
    public int Size()
    {
        return _themodel.EntityCount;
    }

    /// <summary>
    /// Returns size of array of statuses
    /// </summary>
    public int StatuseCount
    {
        get
        {
            return (_thestats is null ? 0 : _thestats.Length());
        }
    }

    /// <summary>
    /// Returns the Number of the entity in the Map, computed at creation time (Entities loaded from the Model)
    /// Returns 0 if &lt;ent&gt; not contained by Model used to create &lt;me&gt; (that is, &lt;ent&gt; is
    /// unknown from &lt;me&gt;)
    /// </summary>
    public int EntityNumber(Transient? ent)
    {
        return _themodel.Number(ent);
    }

    /// <summary>
    /// Returns True if an Entity is noted as present in the graph (See methods Get... which determine this
    /// status) Returns False if &lt;num&gt; is out of range too
    /// </summary>
    public bool IsPresent(int num)
    {
        if (num <= 0 || num > Size())
        {
            return false;
        }
        return (_thestats is not null ? _theflags.Value(num, Graph_Present) : false);
    }

    /// <summary>
    /// Same as above but directly on an Entity &lt;ent&gt; : if it is not contained in the Model, returns False.
    /// Else calls IsPresent(num) with &lt;num&gt; given by EntityNumber
    /// </summary>
    public bool IsPresent(Transient? ent)
    {
        return IsPresent(EntityNumber(ent));
    }

    /// <summary>
    /// Returns mapped Entity given its no (if it is present)
    /// </summary>
    public Transient? Entity(int num)
    {
        return _themodel.Value(num);
    }

    /// <summary>
    /// Returns Status associated to a numero (only to read it)
    /// </summary>
    public int Status(int num)
    {
        return (_thestats is not null ? _thestats.Value(num) : 0);
    }

    /// <summary>
    /// Modifies Status associated to a numero
    /// </summary>
    public void SetStatus(int num, int stat)
    {
        if (_thestats is not null)
        {
            _thestats.SetValue(num, stat);
        }
    }

    /// <summary>
    /// Clears Entity and sets Status to 0, for a numero
    /// </summary>
    public void RemoveItem(int num)
    {
        if (_thestats is not null)
        {
            _thestats.SetValue(num, 0);
            _theflags.SetFalse(num, Graph_Present);
        }
    }

    /// <summary>
    /// Changes all status which value is oldstat to new value newstat
    /// </summary>
    public void ChangeStatus(int oldstat, int newstat)
    {
        if (_thestats is null)
        {
            return;
        }
        int nb = _thestats.Upper();
        for (int i = 1; i <= nb; i++)
        {
            if (_thestats.Value(i) == oldstat)
            {
                _thestats.SetValue(i, newstat);
            }
        }
    }

    /// <summary>
    /// Removes all items of which status has a given value stat
    /// </summary>
    public void RemoveStatus(int stat)
    {
        if (_thestats is null)
        {
            return;
        }
        int nb = _thestats.Upper();
        for (int i = 1; i <= nb; i++)
        {
            if (_thestats.Value(i) == stat)
            {
                RemoveItem(i);
            }
        }
    }

    /// <summary>
    /// Returns the Bit Map in order to read or edit flag values
    /// </summary>
    public BitMap BitMap()
    {
        return _theflags;
    }

    /// <summary>
    /// Returns the Bit Map in order to edit it (add new flags)
    /// </summary>
    public BitMap CBitMap()
    {
        return _theflags;
    }

    /// <summary>
    /// Returns the Model with which this Graph was created
    /// </summary>
    public InterfaceModel? Model()
    {
        return _themodel;
    }

    /// <summary>
    /// Loads Graph with all Entities contained in the Model
    /// </summary>
    public void GetFromModel()
    {
        if (_themodel is null || _thestats is null)
        {
            return; // no model ... (. we won't go far)
        }
        _theflags.Init(true, Graph_Present);
        _thestats.Init(0);
    }

    /// <summary>
    /// Gets an Entity, plus its shared ones (at every level) if "shared" is True. New items are set to status
    /// "newstat" Items already present in graph remain unchanged Of course, redefinitions of Shared lists are
    /// taken into account if there are some
    /// </summary>
    public void GetFromEntity(Transient? ent, bool shared, int newstat = 0)
    {
        if (_thestats is null)
        {
            return;
        }
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return;
        }
        if (_theflags.CTrue(num, Graph_Present))
        {
            return; // already taken : we skip
        }
        _thestats.SetValue(num, newstat);
        if (!shared)
        {
            return;
        }
        //  Watch out for redefinition !
        EntityIterator iter = new EntityIterator(GetShareds(ent));

        for (; iter.More(); iter.Next())
        {
            GetFromEntity(iter.Value(), true, newstat);
        }
    }

    /// <summary>
    /// Gets an Entity, plus its shared ones (at every level) if "shared" is True. New items are set to status
    /// "newstat". Items already present in graph are processed as follows : - if they already have status
    /// "newstat", they remain unchanged - if they have another status, this one is modified : if cumul is True,
    /// to former status + overlapstat (cumul) if cumul is False, to overlapstat (enforce)
    /// </summary>
    public void GetFromEntity(Transient? ent, bool shared, int newstat, int overlapstat, bool cumul)
    {
        if (_thestats is null)
        {
            return;
        }
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return;
        }
        bool pasla = !_theflags.CTrue(num, Graph_Present);
        int stat = _thestats.Value(num);

        if (pasla)
        {
            //    theflags.SetTrue (num, Graph_Present);   // new : note with newstat
            _thestats.SetValue(num, newstat);
        }
        else
        {
            int overstat = stat;
            if (stat != newstat)
            { // already taken, same status : skip
                if (cumul)
                {
                    overstat += overlapstat; // new status : with cumulation ...
                }
                else
                {
                    overstat = overlapstat; // ... or without (forced status)
                }
                if (stat != overstat)
                { // if repass already done, skip
                    _thestats.SetValue(num, overstat);
                }
            }
        }
        if (!shared)
        {
            return;
        }
        //  Watch out for redefinition !
        EntityIterator iter = new EntityIterator(GetShareds(ent));

        for (; iter.More(); iter.Next())
        {
            GetFromEntity(iter.Value(), true, newstat);
        }
    }

    /// <summary>
    /// Gets Entities given by an EntityIterator. Entities which were not yet present in the graph are mapped
    /// with status "newstat" Entities already present remain unchanged
    /// </summary>
    public void GetFromIter(EntityIterator iter, int newstat)
    {
        if (_thestats is null)
        {
            return;
        }
        for (iter.Start(); iter.More(); iter.Next())
        {
            Transient ent = iter.Value();
            int num = EntityNumber(ent);
            if (num == 0)
            {
                continue;
            }
            if (_theflags.CTrue(num, Graph_Present))
            {
                continue;
            }
            _thestats.SetValue(num, newstat);
        }
    }

    /// <summary>
    /// Gets Entities given by an EntityIterator and distinguishes those already present in the Graph : - new
    /// entities added to the Graph with status "newstst" - entities already present with status = "newstat"
    /// remain unchanged - entities already present with status different form "newstat" have their status
    /// modified : if cumul is True, to former status + overlapstat (cumul) if cumul is False, to overlapstat
    /// (enforce) (Note : works as GetEntity, shared = False, for each entity)
    /// </summary>
    public void GetFromIter(EntityIterator iter, int newstat, int overlapstat, bool cumul)
    {
        if (_thestats is null)
        {
            return;
        }
        for (iter.Start(); iter.More(); iter.Next())
        {
            Transient ent = iter.Value();
            int num = EntityNumber(ent);
            if (num == 0)
            {
                continue;
            }
            /*bool pasla = !*/ _theflags.Value(num, Graph_Present);
            /*int stat = */ _thestats.Value(num);
            GetFromEntity(ent, false, newstat, overlapstat, cumul);
        }
    }

    /// <summary>
    /// Gets all present items from another graph
    /// </summary>
    public void GetFromGraph(EntityGraph agraph)
    {
        if (Model() != agraph.Model())
        {
            throw new DomainErrorException("Graph from Interface : GetFromGraph");
        }
        int nb = Size();
        for (int i = 1; i <= nb; i++)
        {
            if (agraph.IsPresent(i))
            {
                GetFromEntity(agraph.Entity(i), false, agraph.Status(i));
            }
        }
    }

    /// <summary>
    /// Gets items from another graph which have a specific Status
    /// </summary>
    public void GetFromGraph(EntityGraph agraph, int stat)
    {
        if (Model() != agraph.Model())
        {
            throw new DomainErrorException("Graph from Interface : GetFromGraph");
        }
        int nb = Size();
        for (int i = 1; i <= nb; i++)
        {
            if (agraph.IsPresent(i) && agraph.Status(i) == stat)
            {
                GetFromEntity(agraph.Entity(i), false, stat);
            }
        }
    }

    /// <summary>
    /// Returns True if &lt;ent&gt; or the list of entities shared by &lt;ent&gt; (not redefined) contains items
    /// unknown from this Graph Remark : apart from the status HasShareError, these items are ignored
    /// </summary>
    public bool HasShareErrors(Transient? ent)
    {
        if (_thestats is null)
        {
            return false;
        }
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return true;
        }
        return _theflags.Value(num, Graph_ShareError);
    }

    /// <summary>
    /// Returns the sequence of Entities Shared by an Entity
    /// </summary>
    public NCollection_HSequence<Transient>? GetShareds(Transient? ent)
    {
        return Shareds(ent).Content();
    }

    /// <summary>
    /// Returns the list of Entities Shared by an Entity, as recorded by the Graph. That is, by default Basic
    /// Shared List, else it can be redefined by methods SetShare, SetNoShare ... see below
    /// </summary>
    public EntityIterator Shareds(Transient? ent)
    {
        EntityIterator iter = new EntityIterator();
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return iter;
        }

        Transient curEnt = ent;
        if (_themodel.IsRedefinedContent(num))
        {
            curEnt = _themodel.ReportEntity(num).Content();
        }

        // if (num == 0)  throw new Standard_DomainError("Interface : Shareds");
        GeneralModule module = null;
        int cn;
        if (_themodel.GTool().Select(curEnt, out module, out cn))
        {
            module.FillShared(_themodel, cn, curEnt, iter);
        }
        return iter;
    }

    /// <summary>
    /// Returns the list of Entities which Share an Entity, computed from the Basic or Redefined Shared Lists
    /// </summary>
    public EntityIterator Sharings(Transient? ent)
    {
        return new EntityIterator(GetSharings(ent));
    }

    /// <summary>
    /// Returns the sequence of Entities Sharings by an Entity
    /// </summary>
    public NCollection_HSequence<Transient>? GetSharings(Transient? ent)
    {
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return null;
        }
        // return
        // occ.down_cast<NCollection_HSequence<Standard_Transient>>(thesharings.Value(num));
        NCollection_List<int> alist = _thesharings.Value(num);
        NCollection_HSequence<Transient> sharings =
            new NCollection_HSequence<Transient>();
        NCollection_List<int>.Iterator it = new NCollection_List<int>.Iterator(alist);
        for (; it.More(); it.Next())
        {
            sharings.Append(Entity(it.Value()));
        }
        return sharings;
    }

    /// <summary>
    /// Returns the list of sharings entities, AT ANY LEVEL, which are kind of a given type. A sharing entity
    /// kind of this type ends the exploration of its branch
    /// </summary>
    public EntityIterator TypedSharings(Transient? ent, Type? type)
    {
        EntityIterator iter = new EntityIterator();
        int n = Size();
        AddTypedSharings(ent, type, iter, n, this);
        return iter;
    }

    /// <summary>
    /// Returns the Entities which are not Shared (their Sharing List is empty) in the Model
    /// </summary>
    public EntityIterator RootEntities()
    {
        EntityIterator iter = new EntityIterator();
        int nb = _thesharings.Length();
        for (int i = 1; i <= nb; i++)
        {
            if (!_thesharings.Value(i).IsEmpty())
            {
                continue;
            }
            iter.AddItem(Entity(i));
        }
        return iter;
    }

    /// <summary>
    /// Determines the name attached to an entity, by using the general service Name in GeneralModule Returns a
    /// null handle if no name could be computed or if the entity is not in the model
    /// </summary>
    public TCollection_HAsciiString? Name(Transient? ent)
    {
        TCollection_HAsciiString str = null;
        if (_themodel is null)
        {
            return str;
        }
        if (_themodel.Number(ent) != 0) // OCCT: returns no name for an entity of the model (the test is inverted)
        {
            return str;
        }

        GTool gtool = _themodel.GTool();
        if (gtool is null)
        {
            return str;
        }

        GeneralModule module = null;
        int cn;
        if (!gtool.Select(ent, out module, out cn))
        {
            return str;
        }

        ShareTool sht = new ShareTool(this);
        return module.Name(cn, ent, sht);
    }

    /// <summary>
    /// Returns the Table of Sharing lists. Used to Create another Graph from &lt;me&gt;
    /// </summary>
    public NCollection_HArray1<NCollection_List<int>>? SharingTable()
    {
        return _thesharings;
    }

    /// <summary>
    /// Returns mode responsible for computation of statuses;
    /// </summary>
    public bool ModeStat()
    {
        return (_thestats is not null);
    }

    /// <summary>
    /// Initialize statuses and flags
    /// </summary>
    protected internal void InitStats()
    {
        _thestats = new NCollection_HArray1<int>(1, _themodel.EntityCount);
        _theflags.Initialize(_themodel.EntityCount, 2);
        _theflags.AddFlag("ShareError");
    }

    /// <summary>
    /// Performs the Evaluation of the Graph, from an initial Library, either defined through a Protocol, or
    /// given dierctly Called by the non-empty Constructors Normally, gtool suffices. But if a Graph is created
    /// from a GeneralLib directly, it cannot be used If &lt;gtool&gt; is defined, it has priority
    /// </summary>
    private void Evaluate()
    {
        // Evaluation is performed on all entities of the model
        int entityNumber = Size();

        _thesharings = new NCollection_HArray1<NCollection_List<int>>(1, entityNumber);
        NCollection_Array1<NCollection_List<int>> sharingArrayOfLists = _thesharings.ChangeArray1();
        for (int i = 1; i <= entityNumber; i++)
        {
            sharingArrayOfLists[i] = new NCollection_List<int>(); // OCCT's allocator dropped
        }

        if (_themodel.GTool() is null)
        {
            return;
        }

        // Fill the sharing table with the entities shared by each entity
        // and the entities which share each entity.
        // The entities are iterated in the order of their numbers in the model.
        // The entities which are not present in the model are ignored.
        // The entities which are not shared by any other entity are ignored.
        // Allocator is used to reuse memory for the lists of shared entities.
        NCollection_HSequence<Transient> listOfEntities =
            new NCollection_HSequence<Transient>();
        for (int i = 1; i <= entityNumber; i++)
        {
            listOfEntities.Clear();
            Transient entity = _themodel.Value(i);
            EntityIterator iter = new EntityIterator(listOfEntities);
            int number = EntityNumber(entity);
            if (number == 0)
            {
                continue;
            }

            Transient curEnt = entity;
            if (_themodel.IsRedefinedContent(number))
            {
                curEnt = _themodel.ReportEntity(number).Content();
            }

            GeneralModule module = null;
            int cn;
            if (_themodel.GTool().Select(curEnt, out module, out cn))
            {
                module.FillShared(_themodel, cn, curEnt, iter);
            }

            for (iter.Start(); iter.More(); iter.Next())
            {
                Transient entShare = iter.Value();
                if (entShare == entity)
                {
                    continue;
                }

                int shareNum = EntityNumber(entShare);

                if (shareNum == 0)
                {
                    if (_thestats is not null)
                    {
                        _theflags.SetTrue(i, Graph_ShareError);
                    }
                    continue;
                }
                sharingArrayOfLists[shareNum].Append(i);
            }
        }
    }

    /// <summary>Assigns another graph to this one (OCCT's <c>operator=</c>): same model, sharings and statuses.</summary>
    public EntityGraph Assign(EntityGraph other)
    {
        _themodel = other.Model();
        _thepresents = other._thepresents;
        _thesharings = other.SharingTable();
        _thestats = null;

        int nb = other.StatuseCount;
        if (nb != 0)
        {
            _thestats = new NCollection_HArray1<int>(1, nb);
            for (int i = 1; i <= nb; ++i)
            {
                _thestats.SetValue(i, other.Status(i));
            }
            _theflags.Initialize(other.BitMap(), true);
        }
        return this;
    }

    // OCCT's flags: 0 = Presence, 1 = Sharing Error
    private const int Graph_Present = 0;
    private const int Graph_ShareError = 1;

    protected internal InterfaceModel? _themodel;
    protected internal string _thepresents = "";
    protected internal NCollection_HArray1<int>? _thestats;
    protected internal NCollection_HArray1<NCollection_List<int>>? _thesharings;
    private BitMap _theflags = new BitMap();
}
