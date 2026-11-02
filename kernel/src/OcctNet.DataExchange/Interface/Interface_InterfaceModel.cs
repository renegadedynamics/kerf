// Port of: src/DataExchange/TKXSBase/Interface/Interface_InterfaceModel.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_InterfaceModel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.Interface_DataState;

namespace Occt;

/// <summary>
/// Defines an (Indexed) Set of data corresponding to a complete Transfer by a File Interface, i.e. File
/// Header and Transient Entities (Objects) contained in a File. Contained Entities are identified in the
/// Model by unique and consecutive Numbers. In addition, a Model can attach to each entity, a specific Label
/// according to the norm (e.g. Name for VDA, #ident for Step ...), intended to be output on a string or a
/// stream (remark : labels are not obliged to be unique) InterfaceModel itself is not Transient, it is
/// intended to work on a set of Transient Data. The services offered are basic Listing and Identification
/// operations on Transient Entities, storage of Error Reports, Copying. Moreovere, it is possible to define
/// and use templates. These are empty Models, from which copies can be obtained in order to be filled with
/// effective data. This allows to record standard definitions for headers, avoiding to recreate them for
/// each sendings, and assuring customisation of produced files for a given site. A template is attached to a
/// name. It is possible to define a template from another one (get it, edit it then record it under another
/// name). See also Graph, ShareTool, CheckTool for more
/// </summary>
public abstract class Interface_InterfaceModel : Standard_Transient
{
    // An Interface Model is a closed set of interface Entities: each one
    // is in a single model at a time; it has a number (Number) which allows to
    // verify that an entity is indeed in a single model, to define very
    // efficient Maps, to provide a numerical identifier
    // It is able to be used in Graph processing

    // OCCT.NET: OCCT's templates (HasTemplate, Template, SetTemplate, ListTemplates: a process-wide dictionary of
    // models by name, which only IGES uses) are left out (decision 1183).

    // OCCT's nulch(): a static empty check; a new one here, so that a caller can't change a shared one (decision 1183)
    private static Interface_Check nulch() => new Interface_Check();

    //  ..                Special Access (Report, etc...)                ..

    //=================================================================================================

    /// <summary>
    /// Clears the list of entities (service WhenDelete)
    /// </summary>
    public void Destroy()
    {
        //   Less than Clear which, itself, is adapted to each standard
        ClearEntities();
        thecheckstx.Clear();
        thechecksem.Clear();
        thecategory = null;
    }

    // OCCT's destructor calls Destroy(); the garbage collector replaces it.

    /// <summary>
    /// Sets a Protocol for this Model It is also set by a call to AddWithRefs with Protocol It is used for :
    /// DumpHeader (as required), ClearEntities ...
    /// </summary>
    public void SetProtocol(Interface_Protocol? proto)
    {
        thegtool = new Interface_GTool(proto);
    }

    /// <summary>
    /// Returns the Protocol which has been set by SetProtocol, or AddWithRefs with Protocol
    /// </summary>
    public virtual Interface_Protocol? Protocol()
    {
        Interface_Protocol proto = null;
        if (thegtool is not null)
        {
            return thegtool.Protocol();
        }
        return proto;
    }

    /// <summary>
    /// Sets a GTool for this model, which already defines a Protocol
    /// </summary>
    public void SetGTool(Interface_GTool? gtool)
    {
        thegtool = gtool;
    }

    /// <summary>
    /// Returns the GTool, set by SetProtocol or by SetGTool
    /// </summary>
    public Interface_GTool? GTool()
    {
        return thegtool;
    }

    /// <summary>
    /// Returns the Dispatch Status, either for get or set A Model which is produced from Dispatch may share
    /// entities with the original (according to the Protocol), hence these non-copied entities should not be
    /// deleted
    /// </summary>
    public ref bool DispatchStatus()
    {
        return ref isdispatch;
    }

    /// <summary>
    /// Erases contained data; used when a Model is copied to others : the new copied ones begin from clear Clear
    /// calls specific method ClearHeader (see below)
    /// </summary>
    public virtual void Clear()
    {
        ClearEntities();
        thecheckstx.Clear();
        thechecksem.Clear();
        ClearHeader();
        ClearLabels();
        thecategory = null;
    }

    /// <summary>
    /// Clears the entities; uses the general service WhenDelete, in addition to the standard Memory Manager; can
    /// be redefined
    /// </summary>
    public virtual void ClearEntities()
    {
        thereports.Clear();
        therepch.Clear();
        haschecksem = false;

        if (thegtool is not null)
        {
            // WhenDeleteCase is not applicable
            /* Interface_GeneralModule module; int CN;
                    int nb = NbEntities();
                    for (int i = 1; i <= nb ; i ++) {
                        Standard_Transient anent = Value(i);
                        if (thegtool.Select (anent,module,CN))
                    module.WhenDeleteCase (CN,anent,isdispatch);
                    }*/
            thegtool.ClearEntities(); // smh#14 FRA62479
        }
        isdispatch = false;
        theentities.Clear();
    }

    /// <summary>
    /// Erases information about labels, if any : specific to each norm
    /// </summary>
    public abstract void ClearLabels();

    /// <summary>
    /// Clears Model's header : specific to each norm
    /// </summary>
    public abstract void ClearHeader();

    /// <summary>
    /// Returns count of contained Entities
    /// </summary>
    public int NbEntities()
    {
        return theentities.Extent();
    }

    /// <summary>
    /// Returns True if a Model contains an Entity (for a ReportEntity, looks for the ReportEntity itself AND its
    /// Concerned Entity)
    /// </summary>
    public bool Contains(Standard_Transient? anentity)
    {
        if (theentities.Contains(anentity))
        {
            return true;
        }
        Interface_ReportEntity rep = (anentity as Interface_ReportEntity);
        if (rep is not null)
        {
            return Contains(rep.Concerned());
        }
        return false;
    }

    /// <summary>
    /// Returns the Number of an Entity in the Model if it contains it. Else returns 0. For a ReportEntity, looks
    /// at Concerned Entity. Returns the Directory entry Number of an Entity in the Model if it contains it. Else
    /// returns 0. For a ReportEntity, looks at Concerned Entity.
    /// </summary>
    public int Number(Standard_Transient? anentity)
    {
        if (anentity is null)
        {
            return 0;
        }
        int num = theentities.FindIndex(anentity);
        if (num > 0)
        {
            return num;
        }
        if (anentity.IsKind(typeof(Interface_ReportEntity)))
        {
            Interface_ReportEntity rep = (anentity as Interface_ReportEntity);
            if (rep is not null)
            {
                return Number(rep.Concerned());
            }
        }
        return 0;
    }

    /// <summary>
    /// Returns an Entity identified by its number in the Model Each sub-class of InterfaceModel can define its
    /// own method Entity to return its specific class of Entity (e.g. for VDA, VDAModel returns a VDAEntity),
    /// working by calling Value Remark : For a Reported Entity, (Erroneous, Corrected, Unknown), this method
    /// returns this Reported Entity. See ReportEntity for other questions.
    /// </summary>
    public Standard_Transient? Value(int num)
    {
        return theentities.FindKey(num);
    }

    /// <summary>
    /// Returns the count of DISTINCT types under which an entity may be processed. Defined by the Protocol,
    /// which gives default as 1 (dynamic Type).
    /// </summary>
    public int NbTypes(Standard_Transient? ent)
    {
        if (Protocol() is null)
        {
            return 1;
        }
        return Protocol().NbTypes(ent);
    }

    /// <summary>
    /// Returns a type, given its rank : defined by the Protocol (by default, the first one)
    /// </summary>
    public Type Type(Standard_Transient? ent, int nt = 1)
    {
        if (Protocol() is null)
        {
            return ent.DynamicType();
        }
        return Protocol().Type(ent, nt);
    }

    /// <summary>
    /// Returns the type name of an entity, from the list of types (one or more ...) &lt;complete&gt; True (D)
    /// gives the complete type, else packages are removed WARNING : buffered, to be immediately copied or
    /// printed
    /// </summary>
    public string TypeName(Standard_Transient? ent, bool complet = true)
    {
        if (thegtool is not null)
        {
            return thegtool.SignValue(ent, this);
        }
        string tn = ent.DynamicType().Name;
        if (complet)
        {
            return tn;
        }
        return Interface_InterfaceModel.ClassName(tn);
    }

    /// <summary>
    /// From a CDL Type Name, returns the Class part (package dropped) WARNING : buffered, to be immediately
    /// copied or printed
    /// </summary>
    public static string ClassName(string typnam)
    {
        return Interface_SignType.ClassName(typnam);
    }

    /// <summary>
    /// Returns the State of an entity, given its number
    /// </summary>
    public Interface_DataState EntityState(int num)
    {
        Interface_ReportEntity rep = null;
        if (!thereports.IsBound(num))
        {
            if (!therepch.IsBound(num))
            {
                return Interface_StateOK;
            }
            rep = (therepch.Find(num) as Interface_ReportEntity);
            if (rep.IsError())
            {
                return Interface_DataFail;
            }
            return Interface_DataWarning;
        }
        rep = (thereports.Find(num) as Interface_ReportEntity);
        if (rep is null)
        {
            return Interface_StateUnknown;
        }
        if (rep.IsUnknown())
        {
            return Interface_StateUnknown;
        }
        if (rep.HasNewContent())
        {
            return Interface_StateUnloaded;
        }
        if (rep.IsError())
        {
            return Interface_LoadFail;
        }

        if (!therepch.IsBound(num))
        {
            return Interface_LoadWarning;
        }
        rep = (therepch.Find(num) as Interface_ReportEntity);
        if (rep.IsError())
        {
            return Interface_DataFail;
        }
        return Interface_DataWarning;
    }

    /// <summary>
    /// Returns True if &lt;num&gt; identifies a ReportEntity in the Model Hence, ReportEntity can be called. By
    /// default, queries main report, if &lt;semantic&gt; is True, it queries report for semantic check Remember
    /// that a Report Entity can be defined for an Unknown Entity, or a Corrected or Erroneous (at read time)
    /// Entity. The ReportEntity is defined before call to method AddEntity.
    /// </summary>
    public bool IsReportEntity(int num, bool semantic = false)
    {
        return (semantic ? therepch.IsBound(num) : thereports.IsBound(num));
    }

    /// <summary>
    /// Returns a ReportEntity identified by its number in the Model, or a Null Handle If &lt;num&gt; does not
    /// identify a ReportEntity. By default, queries main report, if &lt;semantic&gt; is True, it queries report
    /// for semantic check
    /// </summary>
    public Interface_ReportEntity? ReportEntity(int num, bool semantic = false)
    {
        Interface_ReportEntity rep = null;
        if (!IsReportEntity(num, semantic))
        {
            return rep;
        }
        if (semantic)
        {
            rep = (therepch.Find(num) as Interface_ReportEntity);
        }
        else
        {
            rep = (thereports.Find(num) as Interface_ReportEntity);
        }
        return rep;
    }

    /// <summary>
    /// Returns True if &lt;num&gt; identifies an Error Entity : in this case, a ReportEntity brings Fail
    /// Messages and possibly an "undefined" Content, see IsRedefinedEntity
    /// </summary>
    public bool IsErrorEntity(int num)
    {
        Interface_ReportEntity rep = ReportEntity(num);
        if (rep is null)
        {
            return false;
        }
        return rep.IsError();
    }

    /// <summary>
    /// Returns True if &lt;num&gt; identifies an Entity which content is redefined through a ReportEntity (i.e.
    /// with literal data only) This happens when an entity is syntactically erroneous in the way that its basic
    /// content remains empty. For more details (such as content itself), see ReportEntity
    /// </summary>
    public bool IsRedefinedContent(int num)
    {
        Interface_ReportEntity rep = ReportEntity(num);
        if (rep is null)
        {
            return false;
        }
        return rep.HasNewContent();
    }

    /// <summary>
    /// Removes the ReportEntity attached to Entity &lt;num&gt;. Returns True if done, False if no ReportEntity
    /// was attached to &lt;num&gt;. Warning : the caller must assume that this clearing is meaningful
    /// </summary>
    public bool ClearReportEntity(int num)
    {
        if (!thereports.IsBound(num))
        {
            return false;
        }
        thereports.UnBind(num);
        return true;
    }

    /// <summary>
    /// Sets or Replaces a ReportEntity for the Entity &lt;num&gt;. Returns True if Report is replaced, False if
    /// it has been replaced Warning : the caller must assume that this setting is meaningful
    /// </summary>
    public bool SetReportEntity(int num, Interface_ReportEntity? rep)
    {
        int nm = num;
        Standard_Transient ent = null;
        if (num > 0)
        {
            ent = Value(nm);
            if (!(ent == rep.Concerned()))
            {
                throw new Interface_InterfaceMismatch("InterfaceModel : SetReportEntity");
            }
        }
        else if (num < 0)
        {
            nm = -num;
            ent = Value(nm);
            if (!(ent == rep.Concerned()))
            {
                throw new Interface_InterfaceMismatch("InterfaceModel : SetReportEntity");
            }
        }
        else
        {
            ent = rep.Concerned();
            nm = Number(ent);
            if (nm == 0)
            {
                throw new Interface_InterfaceMismatch("InterfaceModel : SetReportEntity");
            }
        }
        if (!thereports.IsBound(nm))
        {
            long aMaxRep = thereports.NbBuckets();
            if (aMaxRep <= (long)(thereports.Extent() + 10))
            {
                thereports.ReSize(aMaxRep * 3 / 2);
            }
        }
        if (nm <= 0)
        {
            return false;
        }
        return thereports.Bind(nm, rep);
    }

    /// <summary>
    /// Adds a ReportEntity as such. Returns False if the concerned entity is not recorded in the Model Else,
    /// adds it into, either the main report list or the list for semantic checks, then returns True
    /// </summary>
    public bool AddReportEntity(Interface_ReportEntity? rep, bool semantic = false)
    {
        if (rep is null)
        {
            return false;
        }
        Standard_Transient ent = rep.Concerned();
        if (ent is null)
        {
            return false;
        }
        int num = Number(ent);
        if (num == 0)
        {
            return false;
        }
        // OCCT: <semantic> true adds to the main reports, false to the semantic ones (the reverse of the documentation)
        if (semantic)
        {
            return thereports.Bind(num, rep);
        }
        else
        {
            return therepch.Bind(num, rep);
        }
    }

    /// <summary>
    /// Returns True if &lt;num&gt; identifies an Unknown Entity : in this case, a ReportEntity with no Check
    /// Messages designates it.
    /// </summary>
    public bool IsUnknownEntity(int num)
    {
        Interface_ReportEntity rep = ReportEntity(num);
        if (rep is null)
        {
            return false;
        }
        return rep.IsUnknown();
    }

    /// <summary>
    /// Fills the list of semantic checks. This list is computed (by CheckTool). Hence, it can be stored in the
    /// model for later queries &lt;clear&gt; True (D) : new list replaces &lt;clear&gt; False : new list is
    /// cumulated
    /// </summary>
    public void FillSemanticChecks(Interface_CheckIterator checks, bool clear = true)
    {
        if (checks.Model() is not null)
        {
            Standard_Transient t1 = checks.Model();
            Standard_Transient t2 = this;
            if (t2 != t1)
            {
                return;
            }
        }
        if (clear)
        {
            therepch.Clear();
            thechecksem.Clear();
        }
        int nb = 0;
        for (checks.Start(); checks.More(); checks.Next())
        {
            nb++;
        }
        therepch.ReSize(therepch.Extent() + nb + 2);
        for (checks.Start(); checks.More(); checks.Next())
        {
            Interface_Check ach = checks.Value();
            int num = checks.Number();
            //    global check : ok if SAME MODEL
            if (num == 0)
            {
                thechecksem.GetMessages(ach);
            }
            else
            {
                Standard_Transient ent = Value(num);
                Interface_ReportEntity rep = new Interface_ReportEntity(ach, ent);
                therepch.Bind(num, rep);
            }
        }
        haschecksem = true;
    }

    /// <summary>
    /// Returns True if semantic checks have been filled
    /// </summary>
    public bool HasSemanticChecks()
    {
        return haschecksem;
    }

    /// <summary>
    /// Returns the check attached to an entity, designated by its Number. 0 for global check &lt;semantic&gt;
    /// True : recorded semantic check &lt;semantic&gt; False : recorded syntactic check (see ReportEntity) If no
    /// check is recorded for &lt;num&gt;, returns an empty Check
    /// </summary>
    public Interface_Check? Check(int num, bool syntactic)
    {
        if (num == 0)
        {
            if (syntactic)
            {
                return thecheckstx;
            }
            else
            {
                return thechecksem;
            }
        }
        if (!(syntactic ? thereports.IsBound(num) : therepch.IsBound(num)))
        {
            return nulch();
        }
        Standard_Transient trep = null;
        if (syntactic)
        {
            trep = thereports.Find(num);
        }
        else
        {
            trep = therepch.Find(num);
        }
        Interface_ReportEntity rep = (trep as Interface_ReportEntity);
        if (rep is null)
        {
            return nulch();
        }
        return rep.Check();
    }

    /// <summary>
    /// Does a reservation for the List of Entities (for optimized storage management). If it is not called,
    /// storage management can be less efficient. &lt;nbent&gt; is the expected count of Entities to store
    /// </summary>
    public virtual void Reservate(int nbent)
    {
        // OCCT.NET: the entities' indexed map needs no reservation
        if (nbent < 0 && (long)(-nbent) > thereports.NbBuckets())
        {
            thereports.ReSize((long)(-nbent));
        }
    }

    /// <summary>
    /// Internal method for adding an Entity. Used by file reading (defined by each Interface) and Transfer
    /// tools. It adds the entity required to be added, not its refs : see AddWithRefs. If &lt;anentity&gt; is a
    /// ReportEntity, it is added to the list of Reports, its Concerned Entity (Erroneous or Corrected, else
    /// Unknown) is added to the list of Entities. That is, the ReportEntity must be created before Adding
    /// </summary>
    public virtual void AddEntity(Standard_Transient? anentity)
    {
        // int newnum; svv #2
        if (!anentity.IsKind(typeof(Interface_ReportEntity)))
        {
            theentities.Add(anentity);
            //  Report : Add Concerned, but note presence Report and its value
        }
        else
        {
            Interface_ReportEntity rep = (anentity as Interface_ReportEntity);
            AddEntity(rep.Concerned());
            long aMaxRep = thereports.NbBuckets();
            if (aMaxRep <= (long)(thereports.Extent() + 10))
            {
                thereports.ReSize(aMaxRep * 3 / 2);
            }
            thereports.Bind(Number(rep.Concerned()), rep);
        }
    }

    /// <summary>
    /// Adds to the Model, an Entity with all its References, as they are defined by General Services FillShared
    /// and ListImplied. Process is recursive (any sub-levels) if &lt;level&gt; = 0 (Default) Else, adds
    /// sub-entities until the required sub-level. Especially, if &lt;level&gt; = 1, adds immediate subs and
    /// that's all If &lt;listall&gt; is False (Default), an entity (&lt;anentity&gt; itself or one of its subs
    /// at any level) which is already recorded in the Model is not analysed, only the newly added ones are. If
    /// &lt;listall&gt; is True, all items are analysed (this allows to ensure the consistency of an adding made
    /// by steps)
    /// </summary>
    public void AddWithRefs(Standard_Transient? anent, Interface_Protocol? proto, int level = 0, bool listall = false)
    {
        if (anent is null)
        {
            return;
        }
        if (theentities.FindIndex(anent) != 0)
        {
            if (!listall)
            {
                return;
            }
        }
        Interface_GeneralLib lib = new Interface_GeneralLib(proto);
        AddWithRefs(anent, lib, level, listall);
        if (Protocol() is null && proto is not null)
        {
            SetProtocol(proto);
        }
    }

    /// <summary>
    /// Same as above, but works with the Protocol of the Model
    /// </summary>
    public void AddWithRefs(Standard_Transient? anent, int level = 0, bool listall = false)
    {
        Interface_Protocol proto = Protocol();
        if (proto is null)
        {
            throw new Interface_InterfaceMismatch("InterfaceModel : AddWithRefs");
        }
        AddWithRefs(anent, proto, level, listall);
    }

    /// <summary>
    /// Same as above, but works with an already created GeneralLib
    /// </summary>
    public void AddWithRefs(Standard_Transient? anent, Interface_GeneralLib lib, int level = 0, bool listall = false)
    {
        if (anent is null)
        {
            return;
        }
        if (theentities.FindIndex(anent) != 0)
        {
            if (!listall)
            {
                return;
            }
        }
        else
        {
            AddEntity(anent);
        }

        Interface_EntityIterator iter = new Interface_EntityIterator();
        Interface_GeneralModule module = null;
        int CN;
        if (lib.Select(anent, out module, out CN))
        {
            module.FillSharedCase(CN, anent, iter);
            //    FillShared simply : would suppose that the model is already ready
            //    or precisely, we are in the process of building it ...
            module.ListImpliedCase(CN, anent, iter);
        }
        int lev1 = level - 1;
        if (lev1 == 0)
        {
            return; // level = 0 . all levels; otherwise still n-1
        }
        for (iter.Start(); iter.More(); iter.Next())
        {
            AddWithRefs(iter.Value(), lib, lev1, listall);
        }
    }

    /// <summary>
    /// Replace Entity with Number=nument on other entity - "anent"
    /// </summary>
    public void ReplaceEntity(int nument, Standard_Transient? anent)
    {
        theentities.Substitute(nument, anent);
    }

    /// <summary>
    /// Reverses the Numbers of the Entities, between &lt;after&gt; and the total count of Entities. Thus, the
    /// entities : 1,2 ... after, after+1 ... nb-1, nb become numbered as : 1,2 ... after, nb, nb-1 ... after+1
    /// By default (after = 0) the whole list of Entities is reversed
    /// </summary>
    public void ReverseOrders(int after = 0)
    {
        int nb = NbEntities(); // int num; svv #2
        if (nb < 2 || after >= nb)
        {
            return;
        }
        NCollection_Array1<Standard_Transient> ents = new NCollection_Array1<Standard_Transient>(1, nb);
        int i; // svv #1
        for (i = 1; i <= nb; i++)
        {
            ents.SetValue(i, theentities.FindKey(i));
        }
        //    We will empty the Map, then reload it : in order until after
        //        in reverse order after
        theentities.Clear();
        Reservate(nb);
        for (i = 1; i <= after; i++)
        {
            theentities.Add(ents[i]); // svv #2
        }
        for (i = nb; i > after; i--)
        {
            theentities.Add(ents[i]);
        }
        //    Will also have to take care of the Reports
        for (i = nb; i > after; i--)
        {
            int i2 = nb + after - i;
            Standard_Transient rep1 = null, rep2 = null;
            if (thereports.IsBound(i))
            {
                rep1 = thereports.Find(i);
            }
            if (thereports.IsBound(i2))
            {
                rep2 = thereports.Find(i2);
            }
            if (rep1 is not null)
            {
                thereports.Bind(i2, rep1);
            }
            else
            {
                thereports.UnBind(i2);
            }
            if (rep2 is not null)
            {
                thereports.Bind(i, rep2);
            }
            else
            {
                thereports.UnBind(i);
            }
        }
    }

    /// <summary>
    /// Changes the Numbers of some Entities : &lt;oldnum&gt; is moved to &lt;newnum&gt;, same for &lt;count&gt;
    /// entities. Thus : 1,2 ... newnum-1 newnum ... oldnum .. oldnum+count oldnum+count+1 .. gives 1,2 ...
    /// newnum-1 oldnum .. oldnum+count newnum ... oldnum+count+1 (can be seen as a circular permutation)
    /// </summary>
    public void ChangeOrder(int oldnum, int newnum, int cnt = 1)
    {
        int nb = NbEntities();
        int i; //, num; svv #2
        if (nb < 2 || newnum >= nb || cnt <= 0)
        {
            return;
        }
        NCollection_Array1<Standard_Transient> ents = new NCollection_Array1<Standard_Transient>(1, nb);
        //  We will prepare the change
        int minum = (oldnum > newnum ? newnum : oldnum);
        int mxnum = (oldnum < newnum ? newnum : oldnum);
        int kount = (oldnum > newnum ? cnt : -cnt);
        if (cnt <= 0 || cnt > mxnum - minum)
        {
            throw new Interface_InterfaceMismatch("InterfaceModel : ChangeOrder, Overlap");
        }
        for (i = 1; i < minum; i++)
        {
            ents.SetValue(i, theentities.FindKey(i));
        }
        for (i = mxnum + cnt; i <= nb; i++)
        {
            ents.SetValue(i, theentities.FindKey(i));
        }
        for (i = minum; i < mxnum; i++)
        {
            ents.SetValue(i + kount, theentities.FindKey(i));
        }
        for (i = oldnum; i < oldnum + cnt; i++)
        {
            ents.SetValue(i + (newnum - oldnum), theentities.FindKey(i));
        }

        theentities.Clear();
        Reservate(nb);
        for (i = 1; i <= nb; i++)
        {
            theentities.Add(ents[i]); // svv #2
        }

        int difnum = mxnum - minum;
        for (i = minum; i < minum + cnt; i++)
        {
            Standard_Transient rep1 = null, rep2 = null;
            if (thereports.IsBound(i))
            {
                rep1 = thereports.Find(i);
            }
            if (thereports.IsBound(i + difnum))
            {
                rep1 = thereports.Find(i + difnum); // OCCT: rep1 again, so rep2 stays null
            }
            if (rep1 is not null)
            {
                thereports.Bind(i + difnum, rep1);
            }
            else
            {
                thereports.UnBind(i + difnum);
            }
            if (rep2 is not null)
            {
                thereports.Bind(i, rep2);
            }
            else
            {
                thereports.UnBind(i);
            }
        }
    }

    /// <summary>
    /// Gets contents from an EntityIterator, prepared by a Transfer tool (e.g TransferCopy). Starts from clear
    /// </summary>
    public void GetFromTransfer(Interface_EntityIterator aniter)
    {
        theentities.Clear();
        for (aniter.Start(); aniter.More(); aniter.Next())
        {
            Standard_Transient ent = aniter.Value();
            AddEntity(ent);
        }
    }

    /// <summary>
    /// Gets header (data specific of a defined Interface) from another InterfaceModel; called from TransferCopy
    /// </summary>
    public abstract void GetFromAnother(Interface_InterfaceModel? other);

    /// <summary>
    /// Returns a New Empty Model, same type as &lt;me&gt; (whatever its Type); called to Copy parts a Model into
    /// other ones, then followed by a call to GetFromAnother (Header) then filling with specified Entities,
    /// themselves copied
    /// </summary>
    public abstract Interface_InterfaceModel? NewEmptyModel();

    /// <summary>
    /// Records a category number for an entity number Returns True when done, False if &lt;num&gt; is out of
    /// range
    /// </summary>
    public bool SetCategoryNumber(int num, int val)
    {
        int i, nb = NbEntities();
        if (num < 1 || num > nb)
        {
            return false;
        }
        if (thecategory is null)
        {
            thecategory = new TCollection_HAsciiString(nb, ' ');
        }
        else if (thecategory.Length() < nb)
        {
            TCollection_HAsciiString c = new TCollection_HAsciiString(nb, ' ');
            for (i = thecategory.Length(); i > 0; i--)
            {
                c.SetValue(i, thecategory.Value(i));
            }
            thecategory = c;
        }
        char cval = (char)(val + 32);
        thecategory.SetValue(num, cval);
        return true;
    }

    /// <summary>
    /// Returns the recorded category number for a given entity number 0 if none was defined for this entity
    /// </summary>
    public int CategoryNumber(int num)
    {
        if (thecategory is null)
        {
            return 0;
        }
        if (num < 1 || num > thecategory.Length())
        {
            return 0;
        }
        int val = thecategory.Value(num);
        return val - 32;
    }

    /// <summary>
    /// Allows an EntityIterator to get a list of Entities
    /// </summary>
    public void FillIterator(Interface_EntityIterator iter)
    {
        int nb = NbEntities();
        for (int i = 1; i <= nb; i++)
        {
            iter.GetOneItem(theentities.FindKey(i));
        }
    }

    /// <summary>
    /// Returns the list of all Entities, as an Iterator on Entities (the Entities themselves, not the Reports)
    /// </summary>
    public Interface_EntityIterator Entities()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        FillIterator(iter);
        return iter;
    }

    /// <summary>
    /// Returns the list of all ReportEntities, i.e. data about Entities read with Error or Warning information
    /// (each item has to be casted to Report Entity then it can be queried for Concerned Entity, Content, Check
    /// ...) By default, returns the main reports, is &lt;semantic&gt; is True it returns the list for semantic
    /// checks
    /// </summary>
    public Interface_EntityIterator Reports(bool semantic = false)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        if (semantic)
        {
            NCollection_IntDataMap<Standard_Transient>.Iterator itmap = new NCollection_IntDataMap<Standard_Transient>.Iterator(therepch);
            for (; itmap.More(); itmap.Next())
            {
                iter.AddItem(itmap.Value());
            }
        }
        else
        {
            NCollection_IntDataMap<Standard_Transient>.Iterator itmap = new NCollection_IntDataMap<Standard_Transient>.Iterator(thereports);
            for (; itmap.More(); itmap.Next())
            {
                iter.AddItem(itmap.Value());
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns the list of ReportEntities which redefine data (generally, if concerned entity is "Error", a
    /// literal content is added to it : this is a "redefined entity"
    /// </summary>
    public Interface_EntityIterator Redefineds()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        NCollection_IntDataMap<Standard_Transient>.Iterator itmap = new NCollection_IntDataMap<Standard_Transient>.Iterator(thereports);
        for (; itmap.More(); itmap.Next())
        {
            Interface_ReportEntity rep = (itmap.Value() as Interface_ReportEntity);
            if (rep is null)
            {
                continue;
            }
            if (!rep.HasNewContent())
            {
                continue;
            }
            iter.AddItem(rep);
        }
        return iter;
    }

    /// <summary>
    /// Returns the GlobalCheck, which memorizes messages global to the file (not specific to an Entity),
    /// especially Header
    /// </summary>
    public Interface_Check? GlobalCheck(bool syntactic = true)
    {
        if (syntactic)
        {
            return thecheckstx;
        }
        else
        {
            return thechecksem;
        }
    }

    /// <summary>
    /// Allows to modify GlobalCheck, after getting then completing it Remark : it is SYNTACTIC check. Semantics,
    /// see FillChecks
    /// </summary>
    public void SetGlobalCheck(Interface_Check? ach)
    {
        thecheckstx = ach;
    }

    /// <summary>
    /// Minimum Semantic Global Check on data in model (header) Can only check basic Data. See also GlobalCheck
    /// from Protocol for a check which takes the Graph into account Default does nothing, can be redefined
    /// </summary>
    public virtual void VerifyCheck(ref Interface_Check? ach)
    {

    }

    /// <summary>
    /// Dumps Header in a short, easy to read, form, onto a Stream &lt;level&gt; allows to print more or less
    /// parts of the header, if necessary. 0 for basic print
    /// </summary>
    public abstract void DumpHeader(TextWriter S, int level = 0);

    /// <summary>
    /// Prints identification of a given entity in &lt;me&gt;, in order to be printed in a list or phrase
    /// &lt;mode&gt; &lt; 0 : prints only its number &lt;mode&gt; = 1 : just calls PrintLabel &lt;mode&gt; = 0
    /// (D) : prints its number plus '/' plus PrintLabel If &lt;ent&gt; == &lt;me&gt;, simply prints "Global" If
    /// &lt;ent&gt; is unknown, prints "??/its type"
    /// </summary>
    public void Print(Standard_Transient? ent, TextWriter S, int mode = 0)
    {
        if (ent is null)
        {
            S.Write("null");
            return;
        }
        int num = Number(ent);
        if (mode <= 0)
        {
            S.Write(num);
        }
        if (mode == 0)
        {
            S.Write(":");
        }
        if (mode >= 0)
        {
            if (num > 0)
            {
                PrintToLog(ent, S);
                //      PrintLabel (ent,S);
            }
            else
            {
                S.Write("??");
            }
        }
    }

    /// <summary>
    /// Prints label specific to each norm, for a given entity. Must only print label itself, in order to be
    /// included in a phrase. Can call the result of StringLabel, but not obliged.
    /// </summary>
    public abstract void PrintLabel(Standard_Transient? ent, TextWriter S);

    /// <summary>
    /// Prints label specific to each norm in log format, for a given entity. By default, just calls PrintLabel,
    /// can be redefined
    /// </summary>
    public virtual void PrintToLog(Standard_Transient? ent, TextWriter S)
    {
        PrintLabel(ent, S);
    }

    /// <summary>
    /// Returns a string with the label attached to a given entity. Warning : While this string may be edited on
    /// the spot, if it is a read field, the returned value must be copied before.
    /// </summary>
    public abstract TCollection_HAsciiString? StringLabel(Standard_Transient? ent);

    /// <summary>
    /// Searches a label which matches with one entity. Begins from &lt;lastnum&gt;+1 (default:1) and scans the
    /// entities until &lt;NbEntities&gt;. For the first which matches &lt;label&gt;, this method returns its
    /// Number. Returns 0 if nothing found Can be called recursively (labels are not specified as unique)
    /// &lt;exact&gt; : if True (default), exact match is required else, checks the END of entity label This
    /// method is virtual, hence it can be redefined for a more efficient search (if exact is true).
    /// </summary>
    public virtual int NextNumberForLabel(string label, int fromnum = 0, bool exact = true)
    {
        int n = NbEntities();
        TCollection_HAsciiString labs = new TCollection_HAsciiString(label);
        int lnb = labs.Length();
        labs.LowerCase();

        int i; // svv #1
        for (i = fromnum + 1; i <= n; i++)
        {
            TCollection_HAsciiString lab = StringLabel(Value(i));
            if (lab is null)
            {
                continue;
            }
            if (exact)
            {
                if (lab.IsSameString(labs, false))
                {
                    return i;
                }
            }
            else
            {
                if (lab.Length() < lnb)
                {
                    continue;
                }
                lab.LowerCase();
                if (lab.SearchFromEnd(labs) == lab.Length() - lnb + 1)
                {
                    return i;
                }
            }
        }

        //   In "non exact", we admit to receive the number between 1 and n
        if (exact)
        {
            return 0;
        }
        i = 0;
        if (labs.IsIntegerValue())
        {
            i = atoi(labs.ToCString());
        }
        if (i <= 0 || i > n)
        {
            i = 0;
        }
        return i;
    }

    protected internal Interface_InterfaceModel()
    {
        haschecksem = false;
        isdispatch = false;
        thecheckstx = new Interface_Check();
        thechecksem = new Interface_Check();
    }


    // Entities by reference, numbered in insertion order (OCCT's map of handles); the reports in OCCT's order of
    // integer maps, which Reports() and Redefineds() follow (decision 784's rule)
    private readonly NCollection_IndexedMap<Standard_Transient> theentities = new(ReferenceEqualityComparer.Instance);
    private readonly NCollection_IntDataMap<Standard_Transient> thereports = new();
    private readonly NCollection_IntDataMap<Standard_Transient> therepch = new();
    private Interface_Check? thecheckstx;
    private Interface_Check? thechecksem;
    private bool haschecksem;
    private bool isdispatch;
    private TCollection_HAsciiString? thecategory;
    private Interface_GTool? thegtool;
}
