// Port of: src/DataExchange/TKXSBase/Interface/Interface_CopyTool.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_CopyTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

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
internal class CopyTool
{

    /// <summary>
    /// Creates a CopyTool adapted to work from a Model. Works with a General Service Library, given as an
    /// argument
    /// </summary>
    public CopyTool(InterfaceModel? amodel, GeneralLib lib)
    {
        _thelib = new GeneralLib(lib);
        _thelst = new BitMap(amodel.EntityCount);
        _thelst.Init(false);
        _themod = amodel;
        _themap = new CopyMap(amodel);
        _therep = new CopyMap(amodel);
        _thelev = 0;
        _theimp = false;
    }

    /// <summary>
    /// Same as above, but Library is defined through a Protocol
    /// </summary>
    public CopyTool(InterfaceModel? amodel, Protocol? protocol)
    {
        _thelib = new GeneralLib(protocol);
        _thelst = new BitMap(amodel.EntityCount);
        _thelst.Init(false);
        _themod = amodel;
        _themap = new CopyMap(amodel);
        _therep = new CopyMap(amodel);
        _thelev = 0;
        _theimp = false;
    }

    /// <summary>
    /// Same as above, but works with the Active Protocol: the model's protocol in the port (OCCT's process-wide active
    /// protocol is left out, decision 1183)
    /// </summary>
    public CopyTool(InterfaceModel? amodel)
    {
        _thelib = new GeneralLib(amodel.Protocol());
        _thelst = new BitMap(amodel.EntityCount);
        if (amodel.Protocol() is null)
        {
            throw new InterfaceErrorException("Interface CopyTool : Create with Active Protocol undefined");
        }

        _thelst.Init(false);
        _themod = amodel;
        _themap = new CopyMap(amodel);
        _therep = new CopyMap(amodel);
        _thelev = 0;
        _theimp = false;
    }

    /// <summary>
    /// Returns the Model on which the CopyTool works
    /// </summary>
    public InterfaceModel? Model()
    {
        return _themod;
    }

    /// <summary>
    /// Changes the Map of Result for another one. This allows to work with a more sophisticated Mapping Control
    /// than the Standard one which is CopyMap (e.g. TransferProcess from Transfer)
    /// </summary>
    public void SetControl(CopyControl? othermap)
    {
        _themap = othermap;
    }

    /// <summary>
    /// Returns the object used for Control
    /// </summary>
    public CopyControl? Control()
    {
        return _themap;
    }

    /// <summary>
    /// Clears Transfer List. Gets Ready to begin another Transfer
    /// </summary>
    public virtual void Clear()
    {
        _themap.Clear();
        _therep.Clear();
        _thelev = 0;
        _theimp = false;
        _therts.Clear();
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
    public virtual bool Copy(Transient? entfrom, ref Transient? entto, bool mapped, bool errstat)
    {
        bool res = true;
        if (entfrom == _theent)
        {
            if (_themdu is null)
            {
                res = false;
            }
        }
        else
        {
            _theent = entfrom;
            res = _thelib.Select(entfrom, out _themdu, out _cn);
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
            _themap.Bind(entfrom, entto); // Map before continuing ...
        }

        //  Now, perform the Copy (depending on case; if ShallowCopy is not enough:
        //  it is <themdu> who decides)

        //    An Entity in Error is not copied (no sense and it's risky ...)
        //    However, it is "Copied Empty (NewVoid)" so referenceable
        if (!errstat)
        {
            _themdu.CopyCase(_cn, entfrom, entto, this);
        }
        return res;
    }

    /// <summary>
    /// Transfers one Entity, if not yet bound to a result Remark : For an Entity which is reported in the
    /// Starting Model, the ReportEntity will also be copied with its Content if it has one (at least
    /// ShallowCopy; Complete Copy if the Protocol recognizes the Content : see method Copy)
    /// </summary>
    public Transient? Transferred(Transient? ent)
    {
        Transient res = null;
        if (ent is null)
        {
            return res; // Copy of a Null : very simple ...
        }
        int nument = _themod.Number(ent);

        //  <nument> == 0 . May be a non-shared sub-part ...
        //  We accept but we protect against a loop
        if (nument == 0 && _thelev > 100)
        {
            throw new InterfaceErrorException(
                "CopyTool : Transferred, Entity is not contained in Starting Model");
        }
        if (!_themap.Search(ent, out res))
        { // already transferred ? if not, do it

            //  We perform the Copy (finally, we try)
            //  In case of failure, nothing is recorded
            if (!Copy(ent, ref res, (nument != 0), _themod.IsRedefinedContent(nument)))
            {
                return res;
            }

            _thelev++; // OCCT: after the copy: the entities the copy transfers count at level 0 (roots)
            if (nument != 0)
            {
                _thelst.SetTrue(nument);
            }
            ReportEntity rep = null;
            if (nument != 0)
            {
                rep = _themod.ReportEntity(nument);
            }
            if (rep is not null)
            {
                //  WARNING WARNING, if ReportEntity : Also copy Content and remake a
                //  ReportEntity with the initial terms
                if (rep.IsUnknown)
                {
                    _therep.Bind(ent, new ReportEntity(res));
                }
                else
                {
                    Transient contfrom, contto = null;
                    contfrom = rep.Content();
                    ReportEntity repto = new ReportEntity(rep.Check(), res);
                    if (contfrom is not null)
                    {
                        if (contfrom == ent)
                        {
                            contto = res;
                        }
                        else
                        {
                            Copy(contfrom, ref contto, _themod.Contains(contfrom), false);
                        }
                        repto.SetContent(contto);
                    }
                    _therep.Bind(ent, repto);
                }
            }
            //    Manage the nesting level (0 = root of transfer)
            _thelev--;
        }
        if (_thelev == 0 && nument > 0)
        {
            _therts.Append(nument);
        }
        return res;
    }

    /// <summary>
    /// Defines a Result for the Transfer of a Starting object. Used by method Transferred (which performs a
    /// normal Copy), but can also be called to enforce a result : in the latter case, the enforced result must
    /// be compatible with the other Transfers which are performed
    /// </summary>
    public void Bind(Transient? ent, Transient? res)
    {
        int num = _themod.Number(ent);
        _themap.Bind(ent, res);
        _thelst.SetTrue(num);
    }

    /// <summary>
    /// Search for the result of a Starting Object (i.e. an Entity) Returns True if a Result is Bound (and fills
    /// "result") Returns False if no result is Bound
    /// </summary>
    public bool Search(Transient? ent, out Transient? res)
    {
        return _themap.Search(ent, out res);
    }

    /// <summary>
    /// Clears LastFlags only. This allows to know what Entities are copied after its call (see method
    /// LastCopiedAfter). It can be used when copies are done by increments, which must be distinguished.
    /// ClearLastFlags is also called by Clear.
    /// </summary>
    public void ClearLastFlags()
    {
        _thelst.Init(false);
    }

    /// <summary>
    /// Returns an copied Entity and its Result which were operated after last call to ClearLastFlags. It returns
    /// the first "Last Copied Entity" which Number follows &lt;numfrom&gt;, Zero if none. It is used in a loop
    /// as follow : Integer num = 0; while ( (num = CopyTool.LastCopiedAfter(num,ent,res)) ) { .. Process
    /// Starting &lt;ent&gt; and its Result &lt;res&gt; }
    /// </summary>
    public int LastCopiedAfter(int numfrom, out Transient? ent, out Transient? res)
    {
        int nb = _thelst.Length();
        ent = null; // OCCT leaves them as they were when none follows
        res = null;
        for (int num = numfrom + 1; num <= nb; num++)
        {
            if (_thelst.Value(num))
            {
                ent = _themod.Value(num);
                if (_themap.Search(ent, out res))
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
    public void TransferEntity(Transient? ent)
    {
        Transient res = Transferred(ent);
    }

    /// <summary>
    /// Renews the Implied References. These References do not involve Copying of referenced Entities. For such a
    /// Reference, if the Entity which defines it AND the referenced Entity are both copied, then this Reference
    /// is renewed. Else it is deleted in the copied Entities. Remark : this concerns only some specific
    /// references, such as "back pointers".
    /// </summary>
    public void RenewImpliedRefs()
    {
        if (_theimp)
        {
            return; // already done
        }
        _theimp = true;

        //  Transfer Pass 2 : recovery of non "Share" relations (but "Imply")
        //  i.e. concerning entities that may or may not have been transferred
        //  (And that the 1st pass did not copy but left as Null)
        //  N.B. : we should forbid commanding new transfers ...

        int nb = _themod.EntityCount;
        for (int i = 1; i <= nb; i++)
        {
            Transient ent = _themod.Value(i);
            Transient res = null;
            if (!_themap.Search(ent, out res))
            {
                continue; // entity not transferred
            }
            //    Renewal of "Imply" references.  Warning, do not copy if not loaded
            Transient aRep = null;
            if (!_therep.Search(ent, out aRep))
            {
                Implied(ent, res);
            }
            else
            {
                ReportEntity rep = (aRep as ReportEntity);
                if (rep is not null && !rep.HasNewContent)
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
    public void FillModel(InterfaceModel? bmodel)
    {
        //  Preparatory work concerning the models
        //  We start : this involves the Header
        bmodel.Clear();
        bmodel.GetFromAnother(_themod);

        //  Transfer Pass 1 : We take the Entities previously copied
        EntityIterator list = CompleteResult(true);
        bmodel.GetFromTransfer(list);

        //  Transfer Pass 2 : recovery of non "Share" relations (but "Imply")
        RenewImpliedRefs();
    }

    /// <summary>
    /// Returns the complete list of copied Entities If &lt;withreports&gt; is given True, the entities which
    /// were reported in the Starting Model are replaced in the list by the copied ReportEntities
    /// </summary>
    public EntityIterator CompleteResult(bool withreports = false)
    {
        EntityIterator iter = new EntityIterator();
        int nb = _themod.EntityCount;
        for (int i = 1; i <= nb; i++)
        {
            Transient ent = _themod.Value(i);
            Transient res = null;
            if (!_themap.Search(ent, out res))
            {
                continue;
            }
            if (withreports)
            {
                Transient rep = null;
                if (_therep.Search(ent, out rep))
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
    public EntityIterator RootResult(bool withreports = false)
    {
        EntityIterator iter = new EntityIterator();
        int nb = _therts.Length();
        for (int i = 1; i <= nb; i++)
        {
            int j = _therts.Value(i);
            Transient ent = _themod.Value(j);
            Transient res = null;
            if (!_themap.Search(ent, out res))
            {
                continue;
            }
            if (withreports)
            {
                Transient rep = null;
                if (_therep.Search(ent, out rep))
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
    protected internal virtual bool NewVoid(Transient? entfrom, ref Transient? entto)
    {
        if (entfrom == _theent)
        {
            if (_themdu is null)
            {
                return false;
            }
            return _themdu.NewVoid(_cn, out entto);
        }
        _theent = entfrom;
        bool res = _thelib.Select(entfrom, out _themdu, out _cn);
        if (res)
        {
            res = _themdu.NewVoid(_cn, out entto);
        }
        if (!res)
        {
            res = _themdu.NewCopiedCase(_cn, entfrom, ref entto, this);
        }
        //  if (res == 0) entto = entfrom.ShallowCopy();   sorry, nothing more possible
        return res;
    }

    /// <summary>
    /// Renews the Implied References of one already Copied Entity
    /// </summary>
    protected internal virtual void Implied(Transient? entfrom, Transient? entto)
    {
        GeneralModule module = null;
        int cn;
        if (_thelib.Select(entfrom, out module, out cn))
        {
            module.RenewImpliedCase(cn, entfrom, entto, this);
        }
    }

    protected internal GeneralLib _thelib = new GeneralLib();
    private InterfaceModel? _themod;
    private CopyControl? _themap;
    private CopyMap? _therep;
    private BitMap _thelst = new BitMap();
    private int _thelev;
    private NCollection_Sequence<int> _therts = new NCollection_Sequence<int>();
    private bool _theimp;
    private Transient? _theent;
    private GeneralModule? _themdu;
    private int _cn;
}
