// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ShareOut.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ShareOut.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class gathers the information required to produce one or several file(s) from the content of an
/// InterfaceModel (passing through the creation of intermediate Models). It can correspond to a complete
/// Divide up of a set of Entities intended to be exhaustive and to limit duplications. Or to a simple
/// Extraction of some Entities, in order to work on them. A ShareOut is composed of a list of Dispatches. To
/// Each Dispatch in the ShareOut, is bound an Id. Number This Id. Number allows to identify a Display inside
/// the ShareOut in a stable way (for instance, to attach file names) ShareOut can be seen as a "passive"
/// description, activated through a ShareOutResult, which gives the InterfaceModel on which to work, as a
/// unique source. Thus it is easy to change it without coherence problems Services about it are provided by
/// the class ShareOutResult which is a service class : simulation (list of files and of entities per file;
/// "forgotten" entities; duplicated entities), exploitation (generation of derivated Models, each of them
/// generating an output file)
/// </summary>
public class IFSelect_ShareOut : Standard_Transient
{

    /// <summary>
    /// Creates an empty ShareOut
    /// </summary>
    public IFSelect_ShareOut()
    {
        thedefrt = new TCollection_HAsciiString("Default");
        thenbdefs = thelastrun = 0;
    }

    /// <summary>
    /// Removes in one operation all the Dispatches with their Idents Also clears all information about Names,
    /// and all Results but naming information which are : - kept if &lt;onlydisp&gt; is True. - cleared if
    /// &lt;onlydisp&gt; is False (complete clearing) If &lt;onlydisp&gt; is True, that's all. Else, clears also
    /// Modifiers
    /// </summary>
    public void Clear(bool onlydisp)
    {
        thedisps.Clear();
        ClearResult(!onlydisp);
        if (onlydisp)
        {
            return;
        }
        themodelmodifiers.Clear();
        thefilemodifiers.Clear();
    }

    /// <summary>
    /// Clears all data produced (apart from Dispatches, etc...) if &lt;alsoname&gt; is True, all is cleared.
    /// Else, information about produced Names are kept (to maintain unicity of naming across clearings)
    /// </summary>
    public void ClearResult(bool alsoname)
    {
        thelastrun = 0;
        if (alsoname)
        {
            thenbdefs = 0;
        }
    }

    /// <summary>
    /// Removes an item, which can be, either a Dispatch (removed from the list of Dispatches), or a
    /// GeneralModifier (removed from the list of Model Modifiers or from the list of File Modifiers according to
    /// its type). Returns True if done, False if has not been found or if it is neither a Dispatch, nor a
    /// Modifier.
    /// </summary>
    public bool RemoveItem(Standard_Transient? item)
    {
        IFSelect_GeneralModifier modifier = item as IFSelect_GeneralModifier;
        if (modifier is not null)
        {
            bool formodel = modifier.IsKind(typeof(IFSelect_Modifier));
            int atnum = ModifierRank(modifier);
            return RemoveModifier(formodel, atnum);
        }
        IFSelect_Dispatch disp = item as IFSelect_Dispatch;
        if (disp is not null)
        {
            int atnum = DispatchRank(disp);
            return RemoveDispatch(atnum);
        }
        return false;
    }

    /// <summary>
    /// Returns the rank of last run item (ClearResult resets it to 0)
    /// </summary>
    public int LastRun()
    {
        return thelastrun;
    }

    /// <summary>
    /// Records a new value for the rank of last run item
    /// </summary>
    public void SetLastRun(int lastrun)
    {
        thelastrun = lastrun;
    }

    /// <summary>
    /// Returns the count of Dispatches
    /// </summary>
    public int NbDispatches()
    {
        return thedisps.Length();
    }

    /// <summary>
    /// Returns the Rank of a Dispatch, given its Value (Handle). Returns 0 if the Dispatch is unknown in the
    /// ShareOut
    /// </summary>
    public int DispatchRank(IFSelect_Dispatch? disp)
    {
        if (disp is null)
        {
            return 0;
        }
        for (int i = thedisps.Length(); i >= 1; i--)
        {
            if (disp == thedisps.Value(i))
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>
    /// Returns a Dispatch, given its rank in the list
    /// </summary>
    public IFSelect_Dispatch? Dispatch(int num)
    {
        return thedisps.Value(num);
    }

    /// <summary>
    /// Adds a Dispatch to the list
    /// </summary>
    public void AddDispatch(IFSelect_Dispatch? disp)
    {
        if (disp is null)
        {
            return;
        }
        thedisps.Append(disp);
    }

    /// <summary>
    /// Removes a Dispatch, given its rank in the list Returns True if done, False if rank is not between
    /// (LastRun + 1) and (NbDispatches)
    /// </summary>
    public bool RemoveDispatch(int rank)
    {
        if (rank <= thelastrun || rank > thedisps.Length())
        {
            return false;
        }
        thedisps.Remove(rank);
        return true;
    }

    /// <summary>
    /// Sets a Modifier to be applied on all Dispatches to be run If &lt;modifier&gt; is a ModelModifier, adds it
    /// to the list of Model Modifiers; else to the list of File Modifiers By default (atnum = 0) at the end of
    /// the list, else at &lt;atnum&gt; Each Modifier is used, after each copy of a packet of Entities into a
    /// Model : its criteria are checked and if they are OK, the method Perform of this Modifier is run.
    /// </summary>
    public void AddModifier(IFSelect_GeneralModifier? modifier, int atnum)
    {
        bool formodel = modifier.IsKind(typeof(IFSelect_Modifier));
        if (ModifierRank(modifier) == 0)
        {
            AddModif(modifier, formodel, atnum);
        }
        IFSelect_Dispatch nuldisp = null;
        modifier.SetDispatch(nuldisp);
    }

    /// <summary>
    /// Sets a Modifier to be applied on the Dispatch &lt;dispnum&gt; If &lt;modifier&gt; is a ModelModifier,
    /// adds it to the list of Model Modifiers; else to the list of File Modifiers This is the same list as for
    /// all Dispatches, but the Modifier is qualified to be applied to one Dispatch only Then, &lt;atnum&gt;
    /// refers to the entire list By default (atnum = 0) at the end of the list, else at &lt;atnum&gt; Remark :
    /// if the Modifier was already in the list and if &lt;atnum&gt; = 0, the Modifier is not moved, but only
    /// qualified for a Dispatch
    /// </summary>
    public void AddModifier(IFSelect_GeneralModifier? modifier, int dispnum, int atnum)
    {
        bool formodel = modifier.IsKind(typeof(IFSelect_Modifier));
        if (ModifierRank(modifier) == 0)
        {
            AddModif(modifier, formodel, atnum);
        }
        IFSelect_Dispatch disp = Dispatch(dispnum);
        modifier.SetDispatch(disp);
    }

    /// <summary>
    /// Adds a Modifier to the list of Modifiers : Model Modifiers if &lt;formodel&gt; is True, File Modifiers
    /// else (internal).
    /// </summary>
    public void AddModif(IFSelect_GeneralModifier? modifier, bool formodel, int atnum = 0)
    {
        if (formodel)
        {
            if (atnum > 0 && atnum <= themodelmodifiers.Length())
            {
                themodelmodifiers.InsertBefore(atnum, modifier);
            }
            else
            {
                themodelmodifiers.Append(modifier);
            }
        }
        else
        {
            if (atnum > 0 && atnum <= thefilemodifiers.Length())
            {
                thefilemodifiers.InsertBefore(atnum, modifier);
            }
            else
            {
                thefilemodifiers.Append(modifier);
            }
        }
    }

    /// <summary>
    /// Returns count of Modifiers (which apply to complete Models) : Model Modifiers if &lt;formodel&gt; is
    /// True, File Modifiers else
    /// </summary>
    public int NbModifiers(bool formodel)
    {
        if (formodel)
        {
            return themodelmodifiers.Length();
        }
        else
        {
            return thefilemodifiers.Length();
        }
    }

    /// <summary>
    /// Returns a Modifier of the list, given its rank : Model Modifiers if &lt;formodel&gt; is True, File
    /// Modifiers else
    /// </summary>
    public IFSelect_GeneralModifier? GeneralModifier(bool formodel, int atnum)
    {
        if (formodel)
        {
            return themodelmodifiers.Value(atnum);
        }
        else
        {
            return thefilemodifiers.Value(atnum);
        }
    }

    /// <summary>
    /// Returns a Modifier of the list of Model Modifiers, duely casted
    /// </summary>
    public IFSelect_Modifier? ModelModifier(int num)
    {
        return (themodelmodifiers.Value(num) as IFSelect_Modifier);
    }

    /// <summary>
    /// Gives the rank of a Modifier in the list, 0 if not in the list Model Modifiers if &lt;modifier&gt; is
    /// kind of ModelModifer, File Modifiers else
    /// </summary>
    public int ModifierRank(IFSelect_GeneralModifier? modifier)
    {
        int i;
        bool formodel = modifier.IsKind(typeof(IFSelect_Modifier));
        if (formodel)
        {
            for (i = themodelmodifiers.Length(); i >= 1; i--)
            {
                if (modifier == themodelmodifiers.Value(i))
                {
                    return i;
                }
            }
        }
        else
        {
            for (i = thefilemodifiers.Length(); i >= 1; i--)
            {
                if (modifier == thefilemodifiers.Value(i))
                {
                    return i;
                }
            }
        }
        return 0;
    }

    /// <summary>
    /// Removes a Modifier, given it rank in the list : Model Modifiers if &lt;formodel&gt; is True, File
    /// Modifiers else Returns True if done, False if &lt;num&gt; is out of range
    /// </summary>
    public bool RemoveModifier(bool formodel, int atnum)
    {
        if (atnum <= 0)
        {
            return false;
        }
        if (formodel)
        {
            if (atnum > themodelmodifiers.Length())
            {
                return false;
            }
            themodelmodifiers.Remove(atnum);
        }
        else
        {
            if (atnum > thefilemodifiers.Length())
            {
                return false;
            }
            thefilemodifiers.Remove(atnum);
        }
        return true;
    }

    /// <summary>
    /// Changes the rank of a modifier in the list : Model Modifiers if &lt;formodel&gt; is True, File Modifiers
    /// else from &lt;before&gt; to &lt;after&gt; Returns True if done, False else (before or after out of range)
    /// </summary>
    public bool ChangeModifierRank(bool formodel, int before, int after)
    {
        int nb;
        if (before <= 0 || after <= 0)
        {
            return false;
        }
        if (before == after)
        {
            return true;
        }
        if (formodel)
        {
            nb = themodelmodifiers.Length();
            if (before > nb || after > nb)
            {
                return false;
            }
            IFSelect_GeneralModifier bef = themodelmodifiers.Value(before);
            themodelmodifiers.Remove(before);
            if (after == nb)
            {
                themodelmodifiers.Append(bef);
            }
            else
            {
                themodelmodifiers.InsertBefore(after, bef);
            }
        }
        else
        {
            nb = thefilemodifiers.Length();
            if (before > nb || after > nb)
            {
                return false;
            }
            IFSelect_GeneralModifier bef = thefilemodifiers.Value(before);
            thefilemodifiers.Remove(before);
            if (after == nb)
            {
                thefilemodifiers.Append(bef);
            }
            else
            {
                thefilemodifiers.InsertBefore(after, bef);
            }
        }
        return true;
    }

    /// <summary>
    /// Attaches a Root Name to a Dispatch given its rank, as an HAsciiString (standard form). A Null Handle
    /// resets this name. Returns True if OK, False if this Name is already attached, for a Dispatch or for
    /// Default, or &lt;num&gt; out of range
    /// </summary>
    public bool SetRootName(int num, TCollection_HAsciiString? name)
    {
        if (num < 1 || num > thedisps.Length())
        {
            return false;
        }
        if (RootNumber(name) != 0)
        {
            return false;
        }
        Dispatch(num).SetRootName(name);
        return true;
    }

    /// <summary>
    /// Returns True if the Dispatch of rank &lt;num&gt; has an attached Root Name. False else, or if num is out
    /// of range
    /// </summary>
    public bool HasRootName(int num)
    {
        if (num < 1 || num > thedisps.Length())
        {
            return false;
        }
        return Dispatch(num).HasRootName();
    }

    /// <summary>
    /// Returns the Root bound to a Dispatch, given its rank Returns a Null Handle if not defined
    /// </summary>
    public TCollection_HAsciiString? RootName(int num)
    {
        TCollection_HAsciiString nulname = null;
        if (num < 1 || num > thedisps.Length())
        {
            return nulname;
        }
        return Dispatch(num).RootName();
    }

    /// <summary>
    /// Returns an integer value about a given root name : - positive : it's the rank of the Dispatch which has
    /// this name - null : this root name is unknown - negative (-1) : this root name is the default root name
    /// </summary>
    public int RootNumber(TCollection_HAsciiString? name)
    {
        if (name is null)
        {
            return 0;
        }
        if (thedefrt is not null)
        {
            if (thedefrt.IsSameString(name))
            {
                return -1;
            }
        }
        for (int i = 1; i <= thedisps.Length(); i++)
        {
            TCollection_HAsciiString root = thedisps.Value(i).RootName();
            if (root is null)
            {
                continue;
            }
            if (root.IsSameString(name))
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>
    /// Defines or Changes the general Prefix (which is prepended to complete file name generated). If this
    /// method is not call, Prefix remains empty
    /// </summary>
    public void SetPrefix(TCollection_HAsciiString? pref)
    {
        thepref = pref;
        thenbdefs = 0;
    }

    /// <summary>
    /// Defines or Changes the Default Root Name to a new value (which is used for dispatches which have no
    /// attached root name). If this method is not called, DefaultRootName remains empty Returns True if OK,
    /// False if this Name is already attached, for a Dispatch or for Default
    /// </summary>
    public bool SetDefaultRootName(TCollection_HAsciiString? defrt)
    {
        if (RootNumber(defrt) != 0)
        {
            return false;
        }
        if (thedefrt is null || !thedefrt.IsSameString(defrt))
        {
            thenbdefs = 0;
        }
        thedefrt = defrt;
        return true;
    }

    /// <summary>
    /// Defines or Changes the general Extension (which is appended to complete file name generated). If this
    /// method is not call, Extension remains empty
    /// </summary>
    public void SetExtension(TCollection_HAsciiString? ext)
    {
        theext = ext;
        thenbdefs = 0;
    }

    /// <summary>
    /// Returns the general Prefix. Can be empty.
    /// </summary>
    public TCollection_HAsciiString? Prefix()
    {
        if (thepref is null)
        {
            return new TCollection_HAsciiString("");
        }
        return thepref;
    }

    /// <summary>
    /// Returns the Default Root Name. Can be empty.
    /// </summary>
    public TCollection_HAsciiString? DefaultRootName()
    {
        if (thedefrt is null)
        {
            return new TCollection_HAsciiString("");
        }
        return thedefrt;
    }

    /// <summary>
    /// Returns the general Extension. Can be empty (not recommended)
    /// </summary>
    public TCollection_HAsciiString? Extension()
    {
        if (theext is null)
        {
            return new TCollection_HAsciiString("");
        }
        return theext;
    }

    /// <summary>
    /// Computes the complete file name for a Packet of a Dispatch, given Dispatch Number (Rank), Packet Number,
    /// and Count of Packets generated by this Dispatch (0 if unknown) File Name is made of following strings,
    /// concatenated : General Prefix, Root Name for Dispatch, Packet Suffix, and General Extension. If no Root
    /// Name is specified for a Dispatch, DefaultRootName is considered (and pnum is not used, but
    /// &lt;thenbdefs&gt; is incremented and used Error if no Root is defined for this &lt;idnum&gt;
    /// </summary>
    public string FileName(int dnum, int pnum, int nbpack = 0)
    {
        TCollection_HAsciiString rot = RootName(dnum);
        int num = pnum;
        int npac = nbpack;
        bool sufnum = (npac > 1 || num > 1);
        if (rot is null)
        {
            rot = thedefrt;
            thenbdefs++;
            num = thenbdefs;
            npac = 0;
            sufnum = true; // number on default names, from the 1st without 0
        }

        string res = "";
        if (thepref is not null)
        {
            res += thepref;
        }
        if (rot is not null)
        {
            res += rot;
        }

        //  Suffixe numerique
        if (sufnum)
        { // otherwise, no numeric suffix
            //  PacketSuffix name : _ followed by number <num>
            //  If nbpack non-zero, then we have an upper bound and we can precede with zeros
            //  Ex.: nbpack = 50 (so 2 digits), num = 3, this will give _03
            //  idnum not used : this method can be redefined and use idnum ...
            //  If nbpack = 0 or 1, num = 1 no suffix, otherwise suffix "_num" as is
            //  MODIF of 3-NOV-1995 . to avoid any confusion, num = 1 also gives _1
            int nbch = 0;
            if (npac >= num)
            {
                int nbpa = 1;
                while (nbpa <= npac)
                {
                    nbpa *= 10;
                    nbch++;
                }
            }
            if (nbch > 1)
            {
                // "_%<nbch>.<nbch>d"
                res += "_" + num.ToString("D" + nbch, System.Globalization.CultureInfo.InvariantCulture);
            }
            else if (npac >= num || num >= 1)
            {
                // "_%d"
                res += "_" + num.ToString(System.Globalization.CultureInfo.InvariantCulture);
            }
        }

        if (theext is not null)
        {
            res += theext;
        }
        return res;
    }

    private NCollection_Sequence<IFSelect_Dispatch> thedisps = new NCollection_Sequence<IFSelect_Dispatch>();
    private NCollection_Sequence<IFSelect_GeneralModifier> themodelmodifiers = new NCollection_Sequence<IFSelect_GeneralModifier>();
    private NCollection_Sequence<IFSelect_GeneralModifier> thefilemodifiers = new NCollection_Sequence<IFSelect_GeneralModifier>();
    private TCollection_HAsciiString? thepref;
    private TCollection_HAsciiString? thedefrt;
    private TCollection_HAsciiString? theext;
    private int thenbdefs;
    private int thelastrun;
}
