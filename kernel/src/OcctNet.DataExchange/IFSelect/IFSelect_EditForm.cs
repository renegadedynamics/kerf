// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_EditForm.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_EditForm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_EditValue;
using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// An EditForm is the way to apply an Editor on an Entity or on the Model It gives read-only or read-write
/// access, with or without undo It can be complete (all the values of the Editor are present) or partial (a
/// sub-list of these value are present) Anyway, all references to Number (argument &lt;num&gt;) refer to
/// Number of Value for the Editor While references to Rank are for rank in the EditForm, which may differ if
/// it is not Complete Two methods give the correspondence between this Number and the Rank in the EditForm :
/// RankFromNumber and NumberFromRank
/// </summary>
public class IFSelect_EditForm : Standard_Transient
{
    private static void PrintList(NCollection_HSequence<TCollection_HAsciiString>? list, TextWriter S, bool alsolist)
    {
        if (list is null)
        {
            S.Write("(null LIST)");
            S.Write('\n');
            return;
        }

        int i, nb = list.Length();
        S.Write("(List : ");
        S.Write(nb);
        S.Write(" Items)");
        S.Write('\n');
        if (!alsolist)
        {
            return;
        }

        for (i = 1; i <= nb; i++)
        {
            TCollection_HAsciiString str = list.Value(i);
            S.Write("  [");
            S.Write(i);
            S.Write("]\t");
            S.Write(str is null ? "(null)" : str.ToCString());
            S.Write('\n');
        }
    }

    /// <summary>
    /// Creates a complete EditForm from an Editor A specific Label can be given
    /// </summary>
    public IFSelect_EditForm(IFSelect_Editor? editor, bool @readonly, bool undoable, string label = "")
    {
        thecomplete = true;
        theloaded = false;
        thekeepst = false;
        thelabel = label;
        thenums = new NCollection_Array1<int>(0, 1);
        theorigs = new NCollection_Array1<object>(0, (undoable ? editor.NbValues() : 0));
        themodifs = new NCollection_Array1<object>(0, (@readonly ? 0 : editor.NbValues()));
        thestatus = new NCollection_Array1<int>(0, (@readonly ? 0 : editor.NbValues()));
        theeditor = editor;
        thetouched = 0;
    }

    /// <summary>
    /// Creates an extracted EditForm from an Editor, limited to the values identified in &lt;nums&gt; A specific
    /// Label can be given
    /// </summary>
    public IFSelect_EditForm(IFSelect_Editor? editor, NCollection_Sequence<int> nums, bool @readonly, bool undoable, string label = "")
    {
        thecomplete = false;
        theloaded = false;
        thekeepst = false;
        thelabel = label;
        thenums = new NCollection_Array1<int>(0, nums.Length());
        theorigs = new NCollection_Array1<object>(0, (undoable ? nums.Length() : 0));
        themodifs = new NCollection_Array1<object>(0, (@readonly ? 0 : nums.Length()));
        thestatus = new NCollection_Array1<int>(0, (@readonly ? 0 : nums.Length()));
        theeditor = editor;
        thetouched = 0;
        int i, nb = nums.Length();
        for (i = 1; i <= nb; i++)
        {
            thenums.SetValue(i, nums.Value(i));
        }
    }

    /// <summary>
    /// Returns and may change the keep status on modif It starts as False If it is True, Apply does not clear
    /// modification status and the EditForm can be loaded again, modified value remain and may be applied again
    /// Remark that ApplyData does not clear the modification status, a call to ClearEdit does
    /// </summary>
    public ref bool EditKeepStatus()
    {
        return ref thekeepst;
    }

    public string Label()
    {
        return thelabel;
    }

    /// <summary>
    /// Tells if the EditForm is loaded now
    /// </summary>
    public bool IsLoaded()
    {
        return theloaded;
    }

    public void ClearData()
    {
        theent = null;
        themodel = null;
        theloaded = false;
    }

    public void SetData(Standard_Transient? ent, Interface_InterfaceModel? model)
    {
        theent = ent;
        themodel = model;
    }

    public void SetEntity(Standard_Transient? ent)
    {
        theent = ent;
    }

    public void SetModel(Interface_InterfaceModel? model)
    {
        themodel = model;
    }

    public Standard_Transient? Entity()
    {
        return theent;
    }

    public Interface_InterfaceModel? Model()
    {
        return themodel;
    }

    public IFSelect_Editor? Editor()
    {
        return theeditor;
    }

    /// <summary>
    /// Tells if an EditForm is complete or is an extract from Editor
    /// </summary>
    public bool IsComplete()
    {
        return thecomplete;
    }

    /// <summary>
    /// Returns the count of values &lt;editable&gt; True : count of editable values, i.e. For a complete
    /// EditForm, it is given by the Editor Else, it is the length of the extraction map &lt;editable&gt; False :
    /// all the values from the Editor
    /// </summary>
    public int NbValues(bool editable)
    {
        if (!editable || thecomplete)
        {
            return theeditor.NbValues();
        }
        return thenums.Upper();
    }

    /// <summary>
    /// Returns the Value Number in the Editor from a given Rank in the EditForm For a complete EditForm, both
    /// are equal Else, it is given by the extraction map Returns 0 if &lt;rank&gt; exceeds the count of editable
    /// values,
    /// </summary>
    public int NumberFromRank(int rank)
    {
        if (thecomplete)
        {
            return rank;
        }
        if (rank < 1 || rank > thenums.Upper())
        {
            return 0;
        }
        return thenums.Value(rank);
    }

    /// <summary>
    /// Returns the Rank in the EditForm from a given Number of Value for the Editor For a complete EditForm,
    /// both are equal Else, it is given by the extraction map Returns 0 if &lt;number&gt; is not forecast to be
    /// edited, or is out of range
    /// </summary>
    public int RankFromNumber(int num)
    {
        if (thecomplete)
        {
            return num;
        }
        int i, n = thenums.Upper();
        for (i = 1; i <= n; i++)
        {
            if (thenums.Value(i) == num)
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>
    /// Returns the Value Number in the Editor for a given Name i.e. the true ValueNumber which can be used in
    /// various methods of EditForm If it is not complete, for a recorded (in the Editor) but non-loaded name,
    /// returns negative value (- number)
    /// </summary>
    public int NameNumber(string name)
    {
        int res = theeditor.NameNumber(name);
        if (thecomplete || res == 0)
        {
            return res;
        }
        //   Otherwise, search res in thenums
        int i, nb = thenums.Length();
        for (i = 1; i <= nb; i++)
        {
            if (res == thenums.Value(i))
            {
                return res;
            }
        }
        return -res;
    }

    /// <summary>
    /// Returns the Rank of Value in the EditForm for a given Name i.e. if it is not complete, for a recorded (in
    /// the Editor) but non-loaded name, returns 0
    /// </summary>
    public int NameRank(string name)
    {
        int res = theeditor.NameNumber(name);
        if (thecomplete || res == 0)
        {
            return res;
        }
        //   Otherwise, search res in thenums
        int i, nb = thenums.Length();
        for (i = 1; i <= nb; i++)
        {
            if (res == thenums.Value(i))
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>
    /// For a read-write undoable EditForm, loads original values from defaults stored in the Editor
    /// </summary>
    public void LoadDefault()
    {
        theloaded = true;
        thetouched = 0;
        int i, nb = theorigs.Upper();
        if (nb == 0)
        {
            return;
        }
        for (i = 1; i <= nb; i++)
        {
            int num = NumberFromRank(i);
            if (num == 0)
            {
                continue;
            }
            TCollection_HAsciiString str = theeditor.StringValue(this, num);
            theorigs.SetValue(i, str);
        }
    }

    /// <summary>
    /// Loads modifications to data Default uses Editor. Can be redefined Remark that &lt;ent&gt; and/or
    /// &lt;model&gt; may be null, according to the kind of Editor. Shortcuts are available for these cases, but
    /// they finally call LoadData (hence, just ignore non-used args)
    /// </summary>
    public virtual bool LoadData(Standard_Transient? ent, Interface_InterfaceModel? model)
    {
        thetouched = 0;
        if (!theeditor.Load(this, ent, model))
        {
            return false;
        }
        SetData(ent, model);
        theloaded = true;
        return true;
    }

    /// <summary>
    /// Shortcut for LoadData when &lt;model&gt; is not used
    /// </summary>
    public bool LoadEntity(Standard_Transient? ent)
    {
        thetouched = 0;
        Interface_InterfaceModel model = null;
        if (!theeditor.Load(this, ent, model))
        {
            return false;
        }
        SetEntity(ent);
        theloaded = true;
        return true;
    }

    /// <summary>
    /// Shortcut for LoadData when only the model is concerned
    /// </summary>
    public bool LoadModel(Interface_InterfaceModel? model)
    {
        thetouched = 0;
        Standard_Transient ent = null;
        if (!theeditor.Load(this, ent, model))
        {
            return false;
        }
        SetData(ent, model);
        theloaded = true;
        return true;
    }

    /// <summary>
    /// Shortcut when both &lt;ent&gt; and &lt;model&gt; are not used (when the Editor works on fully static or
    /// global data)
    /// </summary>
    public bool LoadData()
    {
        thetouched = 0;
        Interface_InterfaceModel model = null;
        Standard_Transient ent = null;
        if (!theeditor.Load(this, ent, model))
        {
            return false;
        }
        theloaded = true;
        return true;
    }

    /// <summary>
    /// Loads an original value (single). Called by the Editor only
    /// </summary>
    public void LoadValue(int num, TCollection_HAsciiString? val)
    {
        int n = RankFromNumber(num);
        if (n <= 0 || n > theorigs.Upper())
        {
            return;
        }
        theorigs.SetValue(n, val);
    }

    /// <summary>
    /// Loads an original value as a list. Called by the Editor only
    /// </summary>
    public void LoadList(int num, NCollection_HSequence<TCollection_HAsciiString>? list)
    {
        int n = RankFromNumber(num);
        if (n <= 0 || n > theorigs.Upper())
        {
            return;
        }
        theorigs.SetValue(n, list);
    }

    /// <summary>
    /// From an edited value, returns its ... value (original one) Null means that this value is not defined
    /// &lt;num&gt; is for the EditForm, not the Editor It is for a single parameter. For a list, gives a Null
    /// Handle
    /// </summary>
    public TCollection_HAsciiString? OriginalValue(int num)
    {
        int n = RankFromNumber(num);
        if (theorigs.Upper() == 0)
        {
            return theeditor.StringValue(this, num);
        }
        else
        {
            return (theorigs.Value(n) as TCollection_HAsciiString);
        }
    }

    /// <summary>
    /// Returns an original value, as a list &lt;num&gt; is for the EditForm, not the Editor For a single
    /// parameter, gives a Null Handle
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? OriginalList(int num)
    {
        int n = RankFromNumber(num);
        if (theorigs.Upper() == 0)
        {
            return theeditor.ListValue(this, num);
        }
        else
        {
            return (theorigs.Value(n) as NCollection_HSequence<TCollection_HAsciiString>);
        }
    }

    /// <summary>
    /// Returns the Edited (i.e. Modified) Value (string for single) &lt;num&gt; reports to the EditForm If
    /// IsModified is False, returns OriginalValue Null with IsModified True : means that this value is not
    /// defined or has been removed It is for a single parameter. For a list, gives a Null Handle
    /// </summary>
    public TCollection_HAsciiString? EditedValue(int num)
    {
        if (themodifs.Upper() == 0)
        {
            return OriginalValue(num);
        }
        if (!IsModified(num))
        {
            return OriginalValue(num);
        }
        int n = RankFromNumber(num);
        return (themodifs.Value(n) as TCollection_HAsciiString);
    }

    /// <summary>
    /// Returns the Edited Value as a list If IsModified is False, returns OriginalValue Null with IsModified
    /// True : means that this value is not defined or has been removed For a single parameter, gives a Null
    /// Handle
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? EditedList(int num)
    {
        if (themodifs.Upper() == 0)
        {
            return OriginalList(num);
        }
        if (!IsModified(num))
        {
            return OriginalList(num);
        }
        int n = RankFromNumber(num);
        return (themodifs.Value(n) as NCollection_HSequence<TCollection_HAsciiString>);
    }

    /// <summary>
    /// Tells if a Value (of the EditForm) is modified (directly or through touching by Update)
    /// </summary>
    public bool IsModified(int num)
    {
        if (thestatus.Upper() == 0)
        {
            return false;
        }
        int n = RankFromNumber(num);
        return (thestatus.Value(n) != 0);
    }

    /// <summary>
    /// Tells if a Value (of the EditForm) has been touched, i.e. not modified directly but by the modification
    /// of another one (by method Update from the Editor)
    /// </summary>
    public bool IsTouched(int num)
    {
        if (thestatus.Upper() == 0)
        {
            return false;
        }
        int n = RankFromNumber(num);
        return (thestatus.Value(n) == 2);
    }

    /// <summary>
    /// Gives a new value for the item &lt;num&gt; of the EditForm, if it is a single parameter (for a list, just
    /// returns False) Null means to Remove it &lt;enforce&gt; True to overpass Protected or Computed Access Mode
    /// Calls the method Update from the Editor, which can touch other parameters (see NbTouched) Returns True if
    /// well recorded, False if this value is not allowed Warning : Does not apply immediately : will be applied
    /// by the method Apply
    /// </summary>
    public bool Modify(int num, TCollection_HAsciiString? newval, bool enforce = false)
    {
        //  Peut-on editer
        thetouched = 0;
        if (themodifs.Upper() == 0)
        {
            return false;
        }
        int tnum = RankFromNumber(num);
        if (tnum == 0)
        {
            return false;
        }
        IFSelect_EditValue acc = theeditor.EditMode(num);
        if (newval is null && acc != IFSelect_Optional)
        {
            return false;
        }
        if (!enforce && (acc == IFSelect_EditProtected || acc == IFSelect_EditComputed))
        {
            return false;
        }

        //  Satisfies ?
        Interface_TypedValue typval = theeditor.TypedValue(num);
        if (!typval.Satisfies(newval))
        {
            return false;
        }
        Interface_ParamType pty = typval.Type();
        if (pty == Interface_ParamIdent && newval is not null)
        {
            if (themodel is null)
            {
                return false;
            }
            if (themodel.NextNumberForLabel(newval.ToCString(), 0, false) <= 0)
            {
                return false;
            }
        }

        //  Update ?
        if (!theeditor.Update(this, num, newval, enforce))
        {
            return false;
        }

        thestatus.SetValue(tnum, 1);
        themodifs.SetValue(tnum, newval);
        return true;
    }

    /// <summary>
    /// Gives a new value computed by the Editor, if another parameter commands the value of &lt;num&gt; It is
    /// generally the case for a Computed Parameter for instance Increments the counter of touched parameters
    /// Warning : it gives no protection for ReadOnly etc... while it is the internal way of touching parameters
    /// Does not work (returns False) if &lt;num&gt; is for a list
    /// </summary>
    public bool Touch(int num, TCollection_HAsciiString? newval)
    {
        if (themodifs.Upper() == 0)
        {
            return false;
        }
        int tnum = RankFromNumber(num);
        if (tnum == 0)
        {
            return false;
        }

        thestatus.SetValue(tnum, 2);
        themodifs.SetValue(tnum, newval);
        thetouched++;
        return true;
    }

    /// <summary>
    /// Acts as Touch but for a list Does not work (returns False) if &lt;num&gt; is for a single param
    /// </summary>
    public bool TouchList(int num, NCollection_HSequence<TCollection_HAsciiString>? newlist)
    {
        if (themodifs.Upper() == 0)
        {
            return false;
        }
        int tnum = RankFromNumber(num);
        if (tnum == 0)
        {
            return false;
        }

        thestatus.SetValue(tnum, 2);
        themodifs.SetValue(tnum, newlist);
        thetouched++;
        return true;
    }

    /// <summary>
    /// Clears modification status : by default all, or one by its numbers (in the Editor)
    /// </summary>
    public void ClearEdit(int num = 0)
    {
        int i, nb = thestatus.Upper();
        if (num == 0)
        {
            for (i = 1; i <= nb; i++)
            {
                thestatus.SetValue(i, 0);
            }
        }
        else
        {
            int tnum = RankFromNumber(num);
            if (tnum > 0 && num <= nb)
            {
                thestatus.SetValue(tnum, 0);
            }
        }
    }

    /// <summary>
    /// Prints Definitions, relative to the Editor
    /// </summary>
    public void PrintDefs(TextWriter S)
    {
        int iv, nbv = NbValues(true);
        S.Write("***** EditForm,  Label : ");
        S.Write(Label());
        S.Write('\n');
        if (IsComplete())
        {
            S.Write("Complete, ");
            S.Write(nbv);
            S.Write(" Values");
            S.Write('\n');
        }
        else
        {
            S.Write("Extraction on ");
            S.Write(nbv);
            S.Write(" Values : (extracted<-editor)");
            S.Write('\n');
            for (iv = 1; iv <= nbv; iv++)
            {
                S.Write("  ");
                S.Write(iv);
                S.Write("<-");
                S.Write(NumberFromRank(iv));
            }
            S.Write('\n');
        }
        S.Write("*****");
        S.Write('\n');
    }

    /// <summary>
    /// Prints Values, according to what and alsolist &lt;names&gt; True : prints Long Names; False : prints
    /// Short Names &lt;what&gt; &lt; 0 : prints Original Values (+ flag Modified) &lt;what&gt; &gt; 0 : prints
    /// Final Values (+flag Modified) &lt;what&gt; = 0 : prints Modified Values (Original + Edited)
    /// &lt;alsolist&gt; False (D) : lists are printed only as their count &lt;alsolist&gt; True : lists are
    /// printed for all their items
    /// </summary>
    public void PrintValues(TextWriter S, int what, bool names, bool alsolist = false)
    {
        int iv, nbv = NbValues(true);
        S.Write("****************************************************");
        S.Write('\n');
        S.Write("*****  ");
        S.Write(Label());
        S.Write(Interface_MSG.Blanks(Label(), 40));
        S.Write("*****");
        S.Write('\n');
        S.Write("*****                                          *****");
        S.Write('\n');
        if (!theloaded)
        {
            S.Write("*****         Values are NOT loaded            *****");
            S.Write('\n');
        }
        else
        {
            //  Data on which we worked
            if (themodel is null)
            {
                if (theent is null)
                {
                    S.Write("*****  No loaded data");
                }
                else
                {
                    S.Write("*****  No loaded Model. Loaded object : type ");
                    S.Write(theent.DynamicType().Name);
                }
            }
            else
            {
                if (theent is null)
                {
                    S.Write("*****  No loaded entity");
                }
                else
                {
                    S.Write("*****  Loaded entity : ");
                    themodel.PrintLabel(theent, S);
                }
            }
        }
        S.Write('\n');
        S.Write("****************************************************");
        S.Write('\n');
        S.Write('\n');

        //  Display of values
        bool nams = names;
        int maxnam = theeditor.MaxNameLength(names ? 0 : -1);
        if (maxnam == 0)
        {
            maxnam = theeditor.MaxNameLength(0);
            nams = true;
        }
        int nbmod = 0;
        if (what != 0)
        {
            S.Write("Mod N0 Name               Value");
            S.Write('\n');
        }
        else
        {
            S.Write(" N0 Name               Value");
            S.Write('\n');
        }

        for (iv = 1; iv <= nbv; iv++)
        {
            int jv = NumberFromRank(iv);
            string name = theeditor.Name(jv, !nams);

            //     Original or Final
            if (what != 0)
            {
                TCollection_HAsciiString str = null;
                if (IsModified(jv))
                {
                    S.Write("* ");
                }
                else
                {
                    S.Write("  ");
                }
                S.Write(Interface_MSG.Blanks(iv, 3));
                S.Write(iv);
                S.Write(" ");
                S.Write(name);
                S.Write(Interface_MSG.Blanks(name, maxnam));
                S.Write("  ");

                if (theeditor.IsList(jv))
                {
                    NCollection_HSequence<TCollection_HAsciiString> list = null;
                    if (what < 0)
                    {
                        list = OriginalList(jv);
                    }
                    if (what > 0)
                    {
                        list = EditedList(jv);
                    }
                    PrintList(list, S, alsolist);
                    continue;
                }

                if (what < 0)
                {
                    str = OriginalValue(jv);
                }
                if (what > 0)
                {
                    str = EditedValue(jv);
                }

                S.Write(str is null ? "(null)" : str.ToCString());
                S.Write('\n');

                //    Modified only
            }
            else
            {
                if (!IsModified(jv))
                {
                    continue;
                }
                nbmod++;
                if (theeditor.IsList(jv))
                {
                    NCollection_HSequence<TCollection_HAsciiString> list =
                        OriginalList(jv);
                    S.Write(Interface_MSG.Blanks(iv, 3));
                    S.Write(iv);
                    S.Write(" ");
                    S.Write(name);
                    S.Write(Interface_MSG.Blanks(name, maxnam));
                    S.Write(" ORIG:");
                    PrintList(list, S, alsolist);

                    list = EditedList(jv);
                    S.Write(Interface_MSG.Blanks("", maxnam + 5));
                    S.Write("MOD :");
                    PrintList(list, S, alsolist);

                    continue;
                }

                TCollection_HAsciiString str = OriginalValue(jv);
                S.Write(Interface_MSG.Blanks(iv, 3));
                S.Write(iv);
                S.Write(" ");
                S.Write(name);
                S.Write(Interface_MSG.Blanks(name, maxnam));
                S.Write(" ORIG:");
                S.Write(str is null ? "(null)" : str.ToCString());
                S.Write('\n');
                str = EditedValue(jv);
                S.Write(Interface_MSG.Blanks("", maxnam + 4));
                S.Write(" MOD :");
                S.Write(str is null ? "(null)" : str.ToCString());
                S.Write('\n');
            }
        }
        if (what == 0)
        {
            S.Write("On ");
            S.Write(nbv);
            S.Write(" Values, ");
            S.Write(nbmod);
            S.Write(" Modified");
            S.Write('\n');
        }
    }

    /// <summary>
    /// Applies modifications to own data Calls ApplyData then Clears Status according EditKeepStatus
    /// </summary>
    public bool Apply()
    {
        bool stat = ApplyData(theent, themodel);
        if (stat && !thekeepst)
        {
            ClearEdit();
        }
        return stat;
    }

    /// <summary>
    /// Tells if this EditForm can work with its Editor and its actual Data (Entity and Model) Default uses
    /// Editor. Can be redefined
    /// </summary>
    public virtual bool Recognize()
    {
        return theeditor.Recognize(this);
    }

    /// <summary>
    /// Applies modifications to data Default uses Editor. Can be redefined
    /// </summary>
    public virtual bool ApplyData(Standard_Transient? ent, Interface_InterfaceModel? model)
    {
        return theeditor.Apply(this, ent, model);
    }

    /// <summary>
    /// For an undoable EditForm, Applies ... origibal values ! and clears modified ones Can be run only once
    /// </summary>
    public bool Undo()
    {
        if (thestatus.Upper() == 0 || theorigs.Upper() == 0)
        {
            return false;
        }
        int i, nb = thestatus.Upper();
        for (i = 1; i <= nb; i++)
        {
            if (thestatus.Value(i) != 0)
            {
                themodifs.SetValue(i, theorigs.Value(i));
            }
        }
        return Apply();
    }

    private bool thecomplete;
    private bool theloaded;
    private bool thekeepst;
    private string thelabel = "";
    private NCollection_Array1<int> thenums;
    private NCollection_Array1<object> theorigs;
    private NCollection_Array1<object> themodifs;
    private NCollection_Array1<int> thestatus;
    private IFSelect_Editor? theeditor;
    private Standard_Transient? theent;
    private Interface_InterfaceModel? themodel;
    private int thetouched;
}
