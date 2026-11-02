// Port of: src/DataExchange/TKXSBase/Interface/Interface_CopyTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_CopyTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Performs Deep Copies of sets of Entities Allows to perform Copy of Interface Entities from a Model to
/// another one. Works by calling general services GetFromAnother and GetImplied. Uses a CopyMap to bind a
/// unique Result to each Copied Entity It is possible to command Copies of Entities (and those they
/// reference) by call to the General Service Library, or to enforce results for transfer of some Entities
/// (calling Bind) A Same CopyTool can be used for several successive Copies from the same Model : either by
/// restarting from scratch (e.g. to copy different parts of a starting Model to several Targets), or
/// incremental : in that case, it is possible to know what is the content of the last increment (defined by
/// last call to ClearLastFlags and queried by call to LastCopiedAfter) Works in two times : first, create
/// the list of copied Entities second, pushes them to a target Model (manages also Model's Header) or
/// returns the Result as an Iterator, as desired The core action (Copy) works by using ShallowCopy (method
/// attached to each class) and Copy from GeneralLib (itself using dedicated tools). It can be redefined for
/// specific actions.
/// </summary>
public class Interface_CopyTool
{

    /// <summary>
    /// Creates a CopyTool adapted to work from a Model. Works with a General Service Library, given as an
    /// argument
    /// </summary>
    public Interface_CopyTool(Interface_InterfaceModel? amodel, Interface_GeneralLib lib)
    {
        thelib = new Interface_GeneralLib(lib);
        thelst = new Interface_BitMap(amodel.NbEntities());
        thelst.Init(false);
        themod = amodel;
        themap = new Interface_CopyMap(amodel);
        therep = new Interface_CopyMap(amodel);
        thelev = 0;
        theimp = false;
    }

    /// <summary>
    /// Same as above, but Library is defined through a Protocol
    /// </summary>
    public Interface_CopyTool(Interface_InterfaceModel? amodel, Interface_Protocol? protocol)
    {
        thelib = new Interface_GeneralLib(protocol);
        thelst = new Interface_BitMap(amodel.NbEntities());
        thelst.Init(false);
        themod = amodel;
        themap = new Interface_CopyMap(amodel);
        therep = new Interface_CopyMap(amodel);
        thelev = 0;
        theimp = false;
    }

    /// <summary>
    /// Same as above, but works with the Active Protocol: the model's protocol in the port (OCCT's process-wide active
    /// protocol is left out, decision 12103)
    /// </summary>
    public Interface_CopyTool(Interface_InterfaceModel? amodel)
    {
        thelib = new Interface_GeneralLib(amodel.Protocol());
        thelst = new Interface_BitMap(amodel.NbEntities());
        if (amodel.Protocol() is null)
        {
            throw new Interface_InterfaceError("Interface CopyTool : Create with Active Protocol undefined");
        }

        thelst.Init(false);
        themod = amodel;
        themap = new Interface_CopyMap(amodel);
        therep = new Interface_CopyMap(amodel);
        thelev = 0;
        theimp = false;
    }

    /// <summary>
    /// Returns the Model on which the CopyTool works
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return themod;
    }

    /// <summary>
    /// Changes the Map of Result for another one. This allows to work with a more sophisticated Mapping Control
    /// than the Standard one which is CopyMap (e.g. TransferProcess from Transfer)
    /// </summary>
    public void SetControl(Interface_CopyControl? othermap)
    {
        themap = othermap;
    }

    /// <summary>
    /// Returns the object used for Control
    /// </summary>
    public Interface_CopyControl? Control()
    {
        return themap;
    }

    /// <summary>
    /// Clears Transfer List. Gets Ready to begin another Transfer
    /// </summary>
    public virtual void Clear()
    {
        themap.Clear();
        therep.Clear();
        thelev = 0;
        theimp = false;
        therts.Clear();
        ClearLastFlags();
    }

    /// <summary>
    /// Creates the CounterPart of an Entity (by ShallowCopy), Binds it, then Copies the content of the former
    /// Entity to the other one (same Type), by call to the General Service Library It may command the Copy of
    /// Referenced Entities Then, its returns True. If &lt;mapped&gt; is True, the Map is used to store the
    /// Result Else, the Result is simply produced : it can be used to Copy internal sub-parts of Entities, which
    /// are not intended to be shared (Strings, Arrays, etc...) If &lt;errstat&gt; is True, this means that the
    /// Entity is recorded in the Model as Erroneous : in this case, the General Service for Deep Copy is not
    /// called (this could be dangerous) : hence the Counter-Part is produced but empty, it can be referenced.
    /// This method does nothing and returns False if the Protocol does not recognize &lt;ent&gt;. It basically
    /// makes a Deep Copy without changing the Types. It can be redefined for special uses.
    /// </summary>
    public virtual bool Copy(Standard_Transient? entfrom, ref Standard_Transient? entto, bool mapped, bool errstat)
    {
        bool res = true;
        if (entfrom == theent)
        {
            if (themdu is null)
            {
                res = false;
            }
        }
        else
        {
            theent = entfrom;
            res = thelib.Select(entfrom, out themdu, out theCN);
        }
        if (!res)
        {
            //  Built-in :
            if (entfrom is null)
            {
                return res;
            }
            if (entfrom.DynamicType() == typeof(TCollection_HAsciiString))
            {
                entto = new TCollection_HAsciiString(
                    (entfrom as TCollection_HAsciiString));
                res = true;
            }
            return res;
        }
        //  Create the empty Entity (NewVoid), the Copy remains to be done
        res = NewVoid(entfrom, ref entto);
        if (mapped)
        {
            themap.Bind(entfrom, entto); // Map before continuing ...
        }

        //  Now, perform the Copy (depending on case; if ShallowCopy is not enough:
        //  it is <themdu> who decides)

        //    An Entity in Error is not copied (no sense and it's risky ...)
        //    However, it is "Copied Empty (NewVoid)" so referenceable
        if (!errstat)
        {
            themdu.CopyCase(theCN, entfrom, entto, this);
        }
        return res;
    }

    /// <summary>
    /// Transfers one Entity, if not yet bound to a result Remark : For an Entity which is reported in the
    /// Starting Model, the ReportEntity will also be copied with its Content if it has one (at least
    /// ShallowCopy; Complete Copy if the Protocol recognizes the Content : see method Copy)
    /// </summary>
    public Standard_Transient? Transferred(Standard_Transient? ent)
    {
        Standard_Transient res = null;
        if (ent is null)
        {
            return res; // Copy of a Null : very simple ...
        }
        int nument = themod.Number(ent);

        //  <nument> == 0 . May be a non-shared sub-part ...
        //  We accept but we protect against a loop
        if (nument == 0 && thelev > 100)
        {
            throw new Interface_InterfaceError(
                "CopyTool : Transferred, Entity is not contained in Starting Model");
        }
        if (!themap.Search(ent, out res))
        { // already transferred ? if not, do it

            //  We perform the Copy (finally, we try)
            //  In case of failure, nothing is recorded
            if (!Copy(ent, ref res, (nument != 0), themod.IsRedefinedContent(nument)))
            {
                return res;
            }

            thelev++;
            if (nument != 0)
            {
                thelst.SetTrue(nument);
            }
            Interface_ReportEntity rep = null;
            if (nument != 0)
            {
                rep = themod.ReportEntity(nument);
            }
            if (rep is not null)
            {
                //  WARNING WARNING, if ReportEntity : Also copy Content and remake a
                //  ReportEntity with the initial terms
                if (rep.IsUnknown())
                {
                    therep.Bind(ent, new Interface_ReportEntity(res));
                }
                else
                {
                    Standard_Transient contfrom, contto = null;
                    contfrom = rep.Content();
                    Interface_ReportEntity repto = new Interface_ReportEntity(rep.Check(), res);
                    if (contfrom is not null)
                    {
                        if (contfrom == ent)
                        {
                            contto = res;
                        }
                        else
                        {
                            Copy(contfrom, ref contto, themod.Contains(contfrom), false);
                        }
                        repto.SetContent(contto);
                    }
                    therep.Bind(ent, repto);
                }
            }
            //    Manage the nesting level (0 = root of transfer)
            thelev--;
        }
        if (thelev == 0 && nument > 0)
        {
            therts.Append(nument);
        }
        return res;
    }

    /// <summary>
    /// Defines a Result for the Transfer of a Starting object. Used by method Transferred (which performs a
    /// normal Copy), but can also be called to enforce a result : in the latter case, the enforced result must
    /// be compatible with the other Transfers which are performed
    /// </summary>
    public void Bind(Standard_Transient? ent, Standard_Transient? res)
    {
        int num = themod.Number(ent);
        themap.Bind(ent, res);
        thelst.SetTrue(num);
    }

    /// <summary>
    /// Search for the result of a Starting Object (i.e. an Entity) Returns True if a Result is Bound (and fills
    /// "result") Returns False if no result is Bound
    /// </summary>
    public bool Search(Standard_Transient? ent, out Standard_Transient? res)
    {
        return themap.Search(ent, out res);
    }

    /// <summary>
    /// Clears LastFlags only. This allows to know what Entities are copied after its call (see method
    /// LastCopiedAfter). It can be used when copies are done by increments, which must be distinguished.
    /// ClearLastFlags is also called by Clear.
    /// </summary>
    public void ClearLastFlags()
    {
        thelst.Init(false);
    }

    /// <summary>
    /// Returns an copied Entity and its Result which were operated after last call to ClearLastFlags. It returns
    /// the first "Last Copied Entity" which Number follows &lt;numfrom&gt;, Zero if none. It is used in a loop
    /// as follow : Integer num = 0; while ( (num = CopyTool.LastCopiedAfter(num,ent,res)) ) { .. Process
    /// Starting &lt;ent&gt; and its Result &lt;res&gt; }
    /// </summary>
    public int LastCopiedAfter(int numfrom, out Standard_Transient? ent, out Standard_Transient? res)
    {
        int nb = thelst.Length();
        ent = null; // OCCT leaves them as they were when none follows
        res = null;
        for (int num = numfrom + 1; num <= nb; num++)
        {
            if (thelst.Value(num))
            {
                ent = themod.Value(num);
                if (themap.Search(ent, out res))
                {
                    return num;
                }
            }
        }
        return 0;
    }

    /// <summary>
    /// Transfers one Entity and records result into the Transfer List Calls method Transferred
    /// </summary>
    public void TransferEntity(Standard_Transient? ent)
    {
        Standard_Transient res = Transferred(ent);
    }

    /// <summary>
    /// Renews the Implied References. These References do not involve Copying of referenced Entities. For such a
    /// Reference, if the Entity which defines it AND the referenced Entity are both copied, then this Reference
    /// is renewed. Else it is deleted in the copied Entities. Remark : this concerns only some specific
    /// references, such as "back pointers".
    /// </summary>
    public void RenewImpliedRefs()
    {
        if (theimp)
        {
            return; // already done
        }
        theimp = true;

        //  Transfer Pass 2 : recovery of non "Share" relations (but "Imply")
        //  i.e. concerning entities that may or may not have been transferred
        //  (And that the 1st pass did not copy but left as Null)
        //  N.B. : we should forbid commanding new transfers ...

        int nb = themod.NbEntities();
        for (int i = 1; i <= nb; i++)
        {
            Standard_Transient ent = themod.Value(i);
            Standard_Transient res = null;
            if (!themap.Search(ent, out res))
            {
                continue; // entity not transferred
            }
            //    Renewal of "Imply" references.  Warning, do not copy if not loaded
            Standard_Transient aRep = null;
            if (!therep.Search(ent, out aRep))
            {
                Implied(ent, res);
            }
            else
            {
                Interface_ReportEntity rep = (aRep as Interface_ReportEntity);
                if (rep is not null && !rep.HasNewContent())
                {
                    Implied(ent, res);
                }
            }
        }
    }

    /// <summary>
    /// Fills a Model with the result of the transfer (TransferList) Commands copy of Header too, and calls
    /// RenewImpliedRefs
    /// </summary>
    public void FillModel(Interface_InterfaceModel? bmodel)
    {
        //  Preparatory work concerning the models
        //  We start : this involves the Header
        bmodel.Clear();
        bmodel.GetFromAnother(themod);

        //  Transfer Pass 1 : We take the Entities previously copied
        Interface_EntityIterator list = CompleteResult(true);
        bmodel.GetFromTransfer(list);

        //  Transfer Pass 2 : recovery of non "Share" relations (but "Imply")
        RenewImpliedRefs();
    }

    /// <summary>
    /// Returns the complete list of copied Entities If &lt;withreports&gt; is given True, the entities which
    /// were reported in the Starting Model are replaced in the list by the copied ReportEntities
    /// </summary>
    public Interface_EntityIterator CompleteResult(bool withreports = false)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int nb = themod.NbEntities();
        for (int i = 1; i <= nb; i++)
        {
            Standard_Transient ent = themod.Value(i);
            Standard_Transient res = null;
            if (!themap.Search(ent, out res))
            {
                continue;
            }
            if (withreports)
            {
                Standard_Transient rep = null;
                if (therep.Search(ent, out rep))
                {
                    res = rep;
                }
            }
            iter.GetOneItem(res);
        }
        return iter;
    }

    /// <summary>
    /// Returns the list of Root copied Entities (those which were asked for copy by the user of CopyTool, not by
    /// copying another Entity)
    /// </summary>
    public Interface_EntityIterator RootResult(bool withreports = false)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        int nb = therts.Length();
        for (int i = 1; i <= nb; i++)
        {
            int j = therts.Value(i);
            Standard_Transient ent = themod.Value(j);
            Standard_Transient res = null;
            if (!themap.Search(ent, out res))
            {
                continue;
            }
            if (withreports)
            {
                Standard_Transient rep = null;
                if (therep.Search(ent, out rep))
                {
                    res = rep;
                }
            }
            iter.GetOneItem(res);
        }
        return iter;
    }


    /// <summary>
    /// Creates a new void instance (just created) of the same class as &lt;entfrom&gt;. Uses the general service
    /// GeneralModule:NewVoid Returns True if OK (Recognize has succeeded), False else (in such a case, the
    /// standard method ShallowCopy is called to produce &lt;ento&gt; from &lt;entfrom&gt; : hence it is not
    /// void) No mapping is managed by this method
    /// </summary>
    protected internal virtual bool NewVoid(Standard_Transient? entfrom, ref Standard_Transient? entto)
    {
        if (entfrom == theent)
        {
            if (themdu is null)
            {
                return false;
            }
            return themdu.NewVoid(theCN, out entto);
        }
        theent = entfrom;
        bool res = thelib.Select(entfrom, out themdu, out theCN);
        if (res)
        {
            res = themdu.NewVoid(theCN, out entto);
        }
        if (!res)
        {
            res = themdu.NewCopiedCase(theCN, entfrom, ref entto, this);
        }
        //  if (res == 0) entto = entfrom.ShallowCopy();   sorry, nothing more possible
        return res;
    }

    /// <summary>
    /// Renews the Implied References of one already Copied Entity
    /// </summary>
    protected internal virtual void Implied(Standard_Transient? entfrom, Standard_Transient? entto)
    {
        Interface_GeneralModule module = null;
        int CN;
        if (thelib.Select(entfrom, out module, out CN))
        {
            module.RenewImpliedCase(CN, entfrom, entto, this);
        }
    }

    protected internal Interface_GeneralLib thelib = new Interface_GeneralLib();
    private Interface_InterfaceModel? themod;
    private Interface_CopyControl? themap;
    private Interface_CopyMap? therep;
    private Interface_BitMap thelst = new Interface_BitMap();
    private int thelev;
    private NCollection_Sequence<int> therts = new NCollection_Sequence<int>();
    private bool theimp;
    private Standard_Transient? theent;
    private Interface_GeneralModule? themdu;
    private int theCN;
}
