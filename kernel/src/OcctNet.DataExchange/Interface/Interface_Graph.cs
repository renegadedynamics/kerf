// Port of: src/DataExchange/TKXSBase/Interface/Interface_Graph.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_Graph.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

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
public class Interface_Graph
{
    private static void AddTypedSharings(Standard_Transient ent,
                                         Type type,
                                         Interface_EntityIterator iter,
                                         int n,
                                         Interface_Graph G)
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

        NCollection_HSequence<Standard_Transient> list = G.GetSharings(ent);
        if (list is null)
        {
            return;
        }

        int nb = list.Length();
        for (int i = 1; i <= nb; i++)
        {
            AddTypedSharings(list.Value(i), type, iter, nb, G);
        }
    }

    /// <summary>
    /// Creates an empty graph, ready to receive Entities from amodel Note that this way of Creation allows
    /// &lt;me&gt; to verify that Entities to work with are contained in &lt;amodel&gt; Basic Shared and Sharing
    /// lists are obtained from a General Services Library, given directly as an argument
    /// </summary>
    public Interface_Graph(Interface_InterfaceModel? amodel, Interface_GeneralLib lib, bool theModeStat = true)
    {
        themodel = amodel;
        thepresents = "";
        if (theModeStat)
        {
            InitStats();
        }
        Evaluate();
    }

    /// <summary>
    /// Same as above, but the Library is defined through a Protocol
    /// </summary>
    public Interface_Graph(Interface_InterfaceModel? amodel, Interface_Protocol? protocol, bool theModeStat = true)
    {
        themodel = amodel;
        thepresents = "";
        if (theModeStat)
        {
            InitStats();
        }
        Evaluate();
    }

    /// <summary>
    /// Same as above, but the Library is defined through a Protocol
    /// </summary>
    public Interface_Graph(Interface_InterfaceModel? amodel, Interface_GTool? gtool, bool theModeStat = true)
    {
        themodel = amodel;
        thepresents = "";
        if (theModeStat)
        {
            InitStats();
        }
        Evaluate();
    }

    /// <summary>
    /// Same a above but works with the Protocol recorded in the Model
    /// </summary>
    public Interface_Graph(Interface_InterfaceModel? amodel, bool theModeStat = true)
    {
        themodel = amodel;
        thepresents = "";
        if (theModeStat)
        {
            InitStats();
        }
        Evaluate();
    }

    /// <summary>
    /// Creates a Graph from another one, getting all its data Remark that status are copied from &lt;agraph&gt;,
    /// but the other lists (sharing/shared) are copied only if &lt;copied&gt; = True
    /// </summary>
    public Interface_Graph(Interface_Graph agraph, bool copied = false)
    {
        themodel = agraph.Model();
        thepresents = "";
        thesharings = agraph.SharingTable();
        int nb = agraph.NbStatuses();
        if (nb == 0)
        {
            return;
        }
        if (thestats is null)
        {
            thestats = new NCollection_HArray1<int>(1, nb);
        }
        for (int i = 1; i <= nb; i++)
        {
            thestats.SetValue(i, agraph.Status(i));
        }
        theflags.Initialize(agraph.BitMap(), true);
    }

    /// <summary>
    /// Erases data, making graph ready to rebegin from void (also resets Shared lists redefinitions)
    /// </summary>
    public void Reset()
    {
        if (thestats is not null)
        {
            thestats = null;
            theflags.Init(false, Graph_Present);
        }
    }

    /// <summary>
    /// Erases Status (Values and Flags of Presence), making graph ready to rebegin from void. Does not concerns
    /// Shared lists
    /// </summary>
    public void ResetStatus()
    {
        if (thestats is not null)
        {
            thestats.Init(0);
            theflags.Init(false, Graph_Present);
        }
    }

    /// <summary>
    /// Returns size (max nb of entities, i.e. Model's nb of entities)
    /// </summary>
    public int Size()
    {
        return themodel.NbEntities();
    }

    /// <summary>
    /// Returns size of array of statuses
    /// </summary>
    public int NbStatuses()
    {
        return (thestats is null ? 0 : thestats.Length());
    }

    /// <summary>
    /// Returns the Number of the entity in the Map, computed at creation time (Entities loaded from the Model)
    /// Returns 0 if &lt;ent&gt; not contained by Model used to create &lt;me&gt; (that is, &lt;ent&gt; is
    /// unknown from &lt;me&gt;)
    /// </summary>
    public int EntityNumber(Standard_Transient? ent)
    {
        return themodel.Number(ent);
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
        return (thestats is not null ? theflags.Value(num, Graph_Present) : false);
    }

    /// <summary>
    /// Same as above but directly on an Entity &lt;ent&gt; : if it is not contained in the Model, returns False.
    /// Else calls IsPresent(num) with &lt;num&gt; given by EntityNumber
    /// </summary>
    public bool IsPresent(Standard_Transient? ent)
    {
        return IsPresent(EntityNumber(ent));
    }

    /// <summary>
    /// Returns mapped Entity given its no (if it is present)
    /// </summary>
    public Standard_Transient? Entity(int num)
    {
        return themodel.Value(num);
    }

    /// <summary>
    /// Returns Status associated to a numero (only to read it)
    /// </summary>
    public int Status(int num)
    {
        return (thestats is not null ? thestats.Value(num) : 0);
    }

    /// <summary>
    /// Modifies Status associated to a numero
    /// </summary>
    public void SetStatus(int num, int stat)
    {
        if (thestats is not null)
        {
            thestats.SetValue(num, stat);
        }
    }

    /// <summary>
    /// Clears Entity and sets Status to 0, for a numero
    /// </summary>
    public void RemoveItem(int num)
    {
        if (thestats is not null)
        {
            thestats.SetValue(num, 0);
            theflags.SetFalse(num, Graph_Present);
        }
    }

    /// <summary>
    /// Changes all status which value is oldstat to new value newstat
    /// </summary>
    public void ChangeStatus(int oldstat, int newstat)
    {
        if (thestats is null)
        {
            return;
        }
        int nb = thestats.Upper();
        for (int i = 1; i <= nb; i++)
        {
            if (thestats.Value(i) == oldstat)
            {
                thestats.SetValue(i, newstat);
            }
        }
    }

    /// <summary>
    /// Removes all items of which status has a given value stat
    /// </summary>
    public void RemoveStatus(int stat)
    {
        if (thestats is null)
        {
            return;
        }
        int nb = thestats.Upper();
        for (int i = 1; i <= nb; i++)
        {
            if (thestats.Value(i) == stat)
            {
                RemoveItem(i);
            }
        }
    }

    /// <summary>
    /// Returns the Bit Map in order to read or edit flag values
    /// </summary>
    public Interface_BitMap BitMap()
    {
        return theflags;
    }

    /// <summary>
    /// Returns the Bit Map in order to edit it (add new flags)
    /// </summary>
    public Interface_BitMap CBitMap()
    {
        return theflags;
    }

    /// <summary>
    /// Returns the Model with which this Graph was created
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return themodel;
    }

    /// <summary>
    /// Loads Graph with all Entities contained in the Model
    /// </summary>
    public void GetFromModel()
    {
        if (themodel is null || thestats is null)
        {
            return; // no model ... (. we won't go far)
        }
        theflags.Init(true, Graph_Present);
        thestats.Init(0);
    }

    /// <summary>
    /// Gets an Entity, plus its shared ones (at every level) if "shared" is True. New items are set to status
    /// "newstat" Items already present in graph remain unchanged Of course, redefinitions of Shared lists are
    /// taken into account if there are some
    /// </summary>
    public void GetFromEntity(Standard_Transient? ent, bool shared, int newstat = 0)
    {
        if (thestats is null)
        {
            return;
        }
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return;
        }
        if (theflags.CTrue(num, Graph_Present))
        {
            return; // already taken : we skip
        }
        thestats.SetValue(num, newstat);
        if (!shared)
        {
            return;
        }
        //  Watch out for redefinition !
        Interface_EntityIterator aIter = new Interface_EntityIterator(GetShareds(ent));

        for (; aIter.More(); aIter.Next())
        {
            GetFromEntity(aIter.Value(), true, newstat);
        }
    }

    /// <summary>
    /// Gets an Entity, plus its shared ones (at every level) if "shared" is True. New items are set to status
    /// "newstat". Items already present in graph are processed as follows : - if they already have status
    /// "newstat", they remain unchanged - if they have another status, this one is modified : if cumul is True,
    /// to former status + overlapstat (cumul) if cumul is False, to overlapstat (enforce)
    /// </summary>
    public void GetFromEntity(Standard_Transient? ent, bool shared, int newstat, int overlapstat, bool cumul)
    {
        if (thestats is null)
        {
            return;
        }
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return;
        }
        bool pasla = !theflags.CTrue(num, Graph_Present);
        int stat = thestats.Value(num);

        if (pasla)
        {
            //    theflags.SetTrue (num, Graph_Present);   // new : note with newstat
            thestats.SetValue(num, newstat);
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
                    thestats.SetValue(num, overstat);
                }
            }
        }
        if (!shared)
        {
            return;
        }
        //  Watch out for redefinition !
        Interface_EntityIterator aIter = new Interface_EntityIterator(GetShareds(ent));

        for (; aIter.More(); aIter.Next())
        {
            GetFromEntity(aIter.Value(), true, newstat);
        }
    }

    /// <summary>
    /// Gets Entities given by an EntityIterator. Entities which were not yet present in the graph are mapped
    /// with status "newstat" Entities already present remain unchanged
    /// </summary>
    public void GetFromIter(Interface_EntityIterator iter, int newstat)
    {
        if (thestats is null)
        {
            return;
        }
        for (iter.Start(); iter.More(); iter.Next())
        {
            Standard_Transient ent = iter.Value();
            int num = EntityNumber(ent);
            if (num == 0)
            {
                continue;
            }
            if (theflags.CTrue(num, Graph_Present))
            {
                continue;
            }
            thestats.SetValue(num, newstat);
        }
    }

    /// <summary>
    /// Gets Entities given by an EntityIterator and distinguishes those already present in the Graph : - new
    /// entities added to the Graph with status "newstst" - entities already present with status = "newstat"
    /// remain unchanged - entities already present with status different form "newstat" have their status
    /// modified : if cumul is True, to former status + overlapstat (cumul) if cumul is False, to overlapstat
    /// (enforce) (Note : works as GetEntity, shared = False, for each entity)
    /// </summary>
    public void GetFromIter(Interface_EntityIterator iter, int newstat, int overlapstat, bool cumul)
    {
        if (thestats is null)
        {
            return;
        }
        for (iter.Start(); iter.More(); iter.Next())
        {
            Standard_Transient ent = iter.Value();
            int num = EntityNumber(ent);
            if (num == 0)
            {
                continue;
            }
            /*bool pasla = !*/ theflags.Value(num, Graph_Present);
            /*int stat = */ thestats.Value(num);
            GetFromEntity(ent, false, newstat, overlapstat, cumul);
        }
    }

    /// <summary>
    /// Gets all present items from another graph
    /// </summary>
    public void GetFromGraph(Interface_Graph agraph)
    {
        if (Model() != agraph.Model())
        {
            throw new Standard_DomainError("Graph from Interface : GetFromGraph");
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
    public void GetFromGraph(Interface_Graph agraph, int stat)
    {
        if (Model() != agraph.Model())
        {
            throw new Standard_DomainError("Graph from Interface : GetFromGraph");
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
    public bool HasShareErrors(Standard_Transient? ent)
    {
        if (thestats is null)
        {
            return false;
        }
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return true;
        }
        return theflags.Value(num, Graph_ShareError);
    }

    /// <summary>
    /// Returns the sequence of Entities Shared by an Entity
    /// </summary>
    public NCollection_HSequence<Standard_Transient>? GetShareds(Standard_Transient? ent)
    {
        return Shareds(ent).Content();
    }

    /// <summary>
    /// Returns the list of Entities Shared by an Entity, as recorded by the Graph. That is, by default Basic
    /// Shared List, else it can be redefined by methods SetShare, SetNoShare ... see below
    /// </summary>
    public Interface_EntityIterator Shareds(Standard_Transient? ent)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return iter;
        }

        Standard_Transient aCurEnt = ent;
        if (themodel.IsRedefinedContent(num))
        {
            aCurEnt = themodel.ReportEntity(num).Content();
        }

        // if (num == 0)  throw new Standard_DomainError("Interface : Shareds");
        Interface_GeneralModule module = null;
        int CN;
        if (themodel.GTool().Select(aCurEnt, out module, out CN))
        {
            module.FillShared(themodel, CN, aCurEnt, iter);
        }
        return iter;
    }

    /// <summary>
    /// Returns the list of Entities which Share an Entity, computed from the Basic or Redefined Shared Lists
    /// </summary>
    public Interface_EntityIterator Sharings(Standard_Transient? ent)
    {
        return new Interface_EntityIterator(GetSharings(ent));
    }

    /// <summary>
    /// Returns the sequence of Entities Sharings by an Entity
    /// </summary>
    public NCollection_HSequence<Standard_Transient>? GetSharings(Standard_Transient? ent)
    {
        int num = EntityNumber(ent);
        if (num == 0)
        {
            return null;
        }
        // return
        // occ.down_cast<NCollection_HSequence<Standard_Transient>>(thesharings.Value(num));
        NCollection_List<int> alist = thesharings.Value(num);
        NCollection_HSequence<Standard_Transient> aSharings =
            new NCollection_HSequence<Standard_Transient>();
        NCollection_List<int>.Iterator aIt = new NCollection_List<int>.Iterator(alist);
        for (; aIt.More(); aIt.Next())
        {
            aSharings.Append(Entity(aIt.Value()));
        }
        return aSharings;
    }

    /// <summary>
    /// Returns the list of sharings entities, AT ANY LEVEL, which are kind of a given type. A sharing entity
    /// kind of this type ends the exploration of its branch
    /// </summary>
    public Interface_EntityIterator TypedSharings(Standard_Transient? ent, Type? type)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int n = Size();
        AddTypedSharings(ent, type, iter, n, this);
        return iter;
    }

    /// <summary>
    /// Returns the Entities which are not Shared (their Sharing List is empty) in the Model
    /// </summary>
    public Interface_EntityIterator RootEntities()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int nb = thesharings.Length();
        for (int i = 1; i <= nb; i++)
        {
            if (!thesharings.Value(i).IsEmpty())
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
    public TCollection_HAsciiString? Name(Standard_Transient? ent)
    {
        TCollection_HAsciiString str = null;
        if (themodel is null)
        {
            return str;
        }
        if (themodel.Number(ent) != 0) // OCCT: returns no name for an entity of the model (the test is inverted)
        {
            return str;
        }

        Interface_GTool gtool = themodel.GTool();
        if (gtool is null)
        {
            return str;
        }

        Interface_GeneralModule module = null;
        int CN;
        if (!gtool.Select(ent, out module, out CN))
        {
            return str;
        }

        Interface_ShareTool sht = new Interface_ShareTool(this);
        return module.Name(CN, ent, sht);
    }

    /// <summary>
    /// Returns the Table of Sharing lists. Used to Create another Graph from &lt;me&gt;
    /// </summary>
    public NCollection_HArray1<NCollection_List<int>>? SharingTable()
    {
        return thesharings;
    }

    /// <summary>
    /// Returns mode responsible for computation of statuses;
    /// </summary>
    public bool ModeStat()
    {
        return (thestats is not null);
    }

    /// <summary>
    /// Initialize statuses and flags
    /// </summary>
    protected internal void InitStats()
    {
        thestats = new NCollection_HArray1<int>(1, themodel.NbEntities());
        theflags.Initialize(themodel.NbEntities(), 2);
        theflags.AddFlag("ShareError");
    }

    /// <summary>
    /// Performs the Evaluation of the Graph, from an initial Library, either defined through a Protocol, or
    /// given dierctly Called by the non-empty Constructors Normally, gtool suffices. But if a Graph is created
    /// from a GeneralLib directly, it cannot be used If &lt;gtool&gt; is defined, it has priority
    /// </summary>
    private void Evaluate()
    {
        // Evaluation is performed on all entities of the model
        int anEntityNumber = Size();

        thesharings = new NCollection_HArray1<NCollection_List<int>>(1, anEntityNumber);
        NCollection_Array1<NCollection_List<int>> aSharingArrayOfLists = thesharings.ChangeArray1();
        for (int i = 1; i <= anEntityNumber; i++)
        {
            aSharingArrayOfLists[i] = new NCollection_List<int>(); // OCCT's allocator dropped
        }

        if (themodel.GTool() is null)
        {
            return;
        }

        // Fill the sharing table with the entities shared by each entity
        // and the entities which share each entity.
        // The entities are iterated in the order of their numbers in the model.
        // The entities which are not present in the model are ignored.
        // The entities which are not shared by any other entity are ignored.
        // Allocator is used to reuse memory for the lists of shared entities.
        NCollection_HSequence<Standard_Transient> aListOfEntities =
            new NCollection_HSequence<Standard_Transient>();
        for (int i = 1; i <= anEntityNumber; i++)
        {
            aListOfEntities.Clear();
            Standard_Transient anEntity = themodel.Value(i);
            Interface_EntityIterator anIter = new Interface_EntityIterator(aListOfEntities);
            int aNumber = EntityNumber(anEntity);
            if (aNumber == 0)
            {
                continue;
            }

            Standard_Transient aCurEnt = anEntity;
            if (themodel.IsRedefinedContent(aNumber))
            {
                aCurEnt = themodel.ReportEntity(aNumber).Content();
            }

            Interface_GeneralModule aModule = null;
            int aCN;
            if (themodel.GTool().Select(aCurEnt, out aModule, out aCN))
            {
                aModule.FillShared(themodel, aCN, aCurEnt, anIter);
            }

            for (anIter.Start(); anIter.More(); anIter.Next())
            {
                Standard_Transient anEntShare = anIter.Value();
                if (anEntShare == anEntity)
                {
                    continue;
                }

                int aShareNum = EntityNumber(anEntShare);

                if (aShareNum == 0)
                {
                    if (thestats is not null)
                    {
                        theflags.SetTrue(i, Graph_ShareError);
                    }
                    continue;
                }
                aSharingArrayOfLists[aShareNum].Append(i);
            }
        }
    }

    /// <summary>Assigns another graph to this one (OCCT's <c>operator=</c>): same model, sharings and statuses.</summary>
    public Interface_Graph Assign(Interface_Graph theOther)
    {
        themodel = theOther.Model();
        thepresents = theOther.thepresents;
        thesharings = theOther.SharingTable();
        thestats = null;

        int nb = theOther.NbStatuses();
        if (nb != 0)
        {
            thestats = new NCollection_HArray1<int>(1, nb);
            for (int i = 1; i <= nb; ++i)
            {
                thestats.SetValue(i, theOther.Status(i));
            }
            theflags.Initialize(theOther.BitMap(), true);
        }
        return this;
    }

    // OCCT's flags: 0 = Presence, 1 = Sharing Error
    private const int Graph_Present = 0;
    private const int Graph_ShareError = 1;

    protected internal Interface_InterfaceModel? themodel;
    protected internal string thepresents = "";
    protected internal NCollection_HArray1<int>? thestats;
    protected internal NCollection_HArray1<NCollection_List<int>>? thesharings;
    private Interface_BitMap theflags = new Interface_BitMap();
}
