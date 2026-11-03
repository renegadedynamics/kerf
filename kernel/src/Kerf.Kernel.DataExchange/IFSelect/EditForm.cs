// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_EditForm.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_EditForm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.Selection.EditValue;
using static Kerf.Engine.Exchange.Framework.ParamType;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// An EditForm is the way to apply an Editor on an Entity or on the Model It gives read-only or read-write
/// access, with or without undo It can be complete (all the values of the Editor are present) or partial (a
/// sub-list of these value are present) Anyway, all references to Number (argument &lt;num&gt;) refer to
/// Number of Value for the Editor While references to Rank are for rank in the EditForm, which may differ if
/// it is not Complete Two methods give the correspondence between this Number and the Rank in the EditForm :
/// RankFromNumber and NumberFromRank
/// </summary>
public class EditForm : Transient
{
    private static void PrintList(NCollection_HSequence<TCollection_HAsciiString>? list, TextWriter s, bool alsolist)
    {
        if (list is null)
        {
            s.Write("(null LIST)");
            s.Write('\n');
            return;
        }

        int i, nb = list.Length();
        s.Write("(List : ");
        s.Write(nb);
        s.Write(" Items)");
        s.Write('\n');
        if (!alsolist)
        {
            return;
        }

        for (i = 1; i <= nb; i++)
        {
            TCollection_HAsciiString str = list.Value(i);
            s.Write("  [");
            s.Write(i);
            s.Write("]\t");
            s.Write(str is null ? "(null)" : str.ToCString());
            s.Write('\n');
        }
    }

    /// <summary>
    /// Creates a complete EditForm from an Editor A specific Label can be given
    /// </summary>
    public EditForm(Editor? editor, bool @readonly, bool undoable, string label = "")
    {
        _thecomplete = true;
        _theloaded = false;
        _thekeepst = false;
        _thelabel = label;
        _thenums = new NCollection_Array1<int>(0, 1);
        _theorigs = new NCollection_Array1<Transient>(0, (undoable ? editor.ValueCount : 0));
        _themodifs = new NCollection_Array1<Transient>(0, (@readonly ? 0 : editor.ValueCount));
        _thestatus = new NCollection_Array1<int>(0, (@readonly ? 0 : editor.ValueCount));
        _theeditor = editor;
        _thetouched = 0;
    }

    /// <summary>
    /// Creates an extracted EditForm from an Editor, limited to the values identified in &lt;nums&gt; A specific
    /// Label can be given
    /// </summary>
    public EditForm(Editor? editor, NCollection_Sequence<int> nums, bool @readonly, bool undoable, string label = "")
    {
        _thecomplete = false;
        _theloaded = false;
        _thekeepst = false;
        _thelabel = label;
        _thenums = new NCollection_Array1<int>(0, nums.Length());
        _theorigs = new NCollection_Array1<Transient>(0, (undoable ? nums.Length() : 0));
        _themodifs = new NCollection_Array1<Transient>(0, (@readonly ? 0 : nums.Length()));
        _thestatus = new NCollection_Array1<int>(0, (@readonly ? 0 : nums.Length()));
        _theeditor = editor;
        _thetouched = 0;
        int i, nb = nums.Length();
        for (i = 1; i <= nb; i++)
        {
            _thenums.SetValue(i, nums.Value(i));
        }
    }

    /// <summary>
    /// Returns and may change the keep status on modif It starts as False If it is True, Apply does not clear
    /// modification status and the EditForm can be loaded again, modified value remain and may be applied again
    /// Remark that ApplyData does not clear the modification status, a call to ClearEdit does
    /// </summary>
    public ref bool EditKeepStatus()
    {
        return ref _thekeepst;
    }

    public string Label()
    {
        return _thelabel;
    }

    /// <summary>
    /// Tells if the EditForm is loaded now
    /// </summary>
    public bool IsLoaded
    {
        get
        {
            return _theloaded;
        }
    }

    public void ClearData()
    {
        _theent = null;
        _themodel = null;
        _theloaded = false;
    }

    public void SetData(Transient? ent, InterfaceModel? model)
    {
        _theent = ent;
        _themodel = model;
    }

    public void SetEntity(Transient? ent)
    {
        _theent = ent;
    }

    public void SetModel(InterfaceModel? model)
    {
        _themodel = model;
    }

    public Transient? Entity()
    {
        return _theent;
    }

    public InterfaceModel? Model()
    {
        return _themodel;
    }

    public Editor? Editor()
    {
        return _theeditor;
    }

    /// <summary>
    /// Tells if an EditForm is complete or is an extract from Editor
    /// </summary>
    public bool IsComplete
    {
        get
        {
            return _thecomplete;
        }
    }

    /// <summary>
    /// Returns the count of values &lt;editable&gt; True : count of editable values, i.e. For a complete
    /// EditForm, it is given by the Editor Else, it is the length of the extraction map &lt;editable&gt; False :
    /// all the values from the Editor
    /// </summary>
    public int NbValues(bool editable)
    {
        if (!editable || _thecomplete)
        {
            return _theeditor.ValueCount;
        }
        return _thenums.Upper();
    }

    /// <summary>
    /// Returns the Value Number in the Editor from a given Rank in the EditForm For a complete EditForm, both
    /// are equal Else, it is given by the extraction map Returns 0 if &lt;rank&gt; exceeds the count of editable
    /// values,
    /// </summary>
    public int NumberFromRank(int rank)
    {
        if (_thecomplete)
        {
            return rank;
        }
        if (rank < 1 || rank > _thenums.Upper())
        {
            return 0;
        }
        return _thenums.Value(rank);
    }

    /// <summary>
    /// Returns the Rank in the EditForm from a given Number of Value for the Editor For a complete EditForm,
    /// both are equal Else, it is given by the extraction map Returns 0 if &lt;number&gt; is not forecast to be
    /// edited, or is out of range
    /// </summary>
    public int RankFromNumber(int num)
    {
        if (_thecomplete)
        {
            return num;
        }
        int i, n = _thenums.Upper();
        for (i = 1; i <= n; i++)
        {
            if (_thenums.Value(i) == num)
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
        int res = _theeditor.NameNumber(name);
        if (_thecomplete || res == 0)
        {
            return res;
        }
        //   Otherwise, search res in thenums
        int i, nb = _thenums.Length();
        for (i = 1; i <= nb; i++)
        {
            if (res == _thenums.Value(i))
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
        int res = _theeditor.NameNumber(name);
        if (_thecomplete || res == 0)
        {
            return res;
        }
        //   Otherwise, search res in thenums
        int i, nb = _thenums.Length();
        for (i = 1; i <= nb; i++)
        {
            if (res == _thenums.Value(i))
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
        _theloaded = true;
        _thetouched = 0;
        int i, nb = _theorigs.Upper();
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
            TCollection_HAsciiString str = _theeditor.StringValue(this, num);
            _theorigs.SetValue(i, str);
        }
    }

    /// <summary>
    /// Loads modifications to data Default uses Editor. Can be redefined Remark that &lt;ent&gt; and/or
    /// &lt;model&gt; may be null, according to the kind of Editor. Shortcuts are available for these cases, but
    /// they finally call LoadData (hence, just ignore non-used args)
    /// </summary>
    public virtual bool LoadData(Transient? ent, InterfaceModel? model)
    {
        _thetouched = 0;
        if (!_theeditor.Load(this, ent, model))
        {
            return false;
        }
        SetData(ent, model);
        _theloaded = true;
        return true;
    }

    /// <summary>
    /// Shortcut for LoadData when &lt;model&gt; is not used
    /// </summary>
    public bool LoadEntity(Transient? ent)
    {
        _thetouched = 0;
        InterfaceModel model = null;
        if (!_theeditor.Load(this, ent, model))
        {
            return false;
        }
        SetEntity(ent);
        _theloaded = true;
        return true;
    }

    /// <summary>
    /// Shortcut for LoadData when only the model is concerned
    /// </summary>
    public bool LoadModel(InterfaceModel? model)
    {
        _thetouched = 0;
        Transient ent = null;
        if (!_theeditor.Load(this, ent, model))
        {
            return false;
        }
        SetData(ent, model);
        _theloaded = true;
        return true;
    }

    /// <summary>
    /// Shortcut when both &lt;ent&gt; and &lt;model&gt; are not used (when the Editor works on fully static or
    /// global data)
    /// </summary>
    public bool LoadData()
    {
        _thetouched = 0;
        InterfaceModel model = null;
        Transient ent = null;
        if (!_theeditor.Load(this, ent, model))
        {
            return false;
        }
        _theloaded = true;
        return true;
    }

    /// <summary>
    /// Loads an original value (single). Called by the Editor only
    /// </summary>
    public void LoadValue(int num, TCollection_HAsciiString? val)
    {
        int n = RankFromNumber(num);
        if (n <= 0 || n > _theorigs.Upper())
        {
            return;
        }
        _theorigs.SetValue(n, val);
    }

    /// <summary>
    /// Loads an original value as a list. Called by the Editor only
    /// </summary>
    public void LoadList(int num, NCollection_HSequence<TCollection_HAsciiString>? list)
    {
        int n = RankFromNumber(num);
        if (n <= 0 || n > _theorigs.Upper())
        {
            return;
        }
        _theorigs.SetValue(n, list);
    }

    /// <summary>
    /// From an edited value, returns its ... value (original one) Null means that this value is not defined
    /// &lt;num&gt; is for the EditForm, not the Editor It is for a single parameter. For a list, gives a Null
    /// Handle
    /// </summary>
    public TCollection_HAsciiString? OriginalValue(int num)
    {
        int n = RankFromNumber(num);
        if (_theorigs.Upper() == 0)
        {
            return _theeditor.StringValue(this, num);
        }
        else
        {
            return (_theorigs.Value(n) as TCollection_HAsciiString);
        }
    }

    /// <summary>
    /// Returns an original value, as a list &lt;num&gt; is for the EditForm, not the Editor For a single
    /// parameter, gives a Null Handle
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? OriginalList(int num)
    {
        int n = RankFromNumber(num);
        if (_theorigs.Upper() == 0)
        {
            return _theeditor.ListValue(this, num);
        }
        else
        {
            return (_theorigs.Value(n) as NCollection_HSequence<TCollection_HAsciiString>);
        }
    }

    /// <summary>
    /// Returns the Edited (i.e. Modified) Value (string for single) &lt;num&gt; reports to the EditForm If
    /// IsModified is False, returns OriginalValue Null with IsModified True : means that this value is not
    /// defined or has been removed It is for a single parameter. For a list, gives a Null Handle
    /// </summary>
    public TCollection_HAsciiString? EditedValue(int num)
    {
        if (_themodifs.Upper() == 0)
        {
            return OriginalValue(num);
        }
        if (!IsModified(num))
        {
            return OriginalValue(num);
        }
        int n = RankFromNumber(num);
        return (_themodifs.Value(n) as TCollection_HAsciiString);
    }

    /// <summary>
    /// Returns the Edited Value as a list If IsModified is False, returns OriginalValue Null with IsModified
    /// True : means that this value is not defined or has been removed For a single parameter, gives a Null
    /// Handle
    /// </summary>
    public NCollection_HSequence<TCollection_HAsciiString>? EditedList(int num)
    {
        if (_themodifs.Upper() == 0)
        {
            return OriginalList(num);
        }
        if (!IsModified(num))
        {
            return OriginalList(num);
        }
        int n = RankFromNumber(num);
        return (_themodifs.Value(n) as NCollection_HSequence<TCollection_HAsciiString>);
    }

    /// <summary>
    /// Tells if a Value (of the EditForm) is modified (directly or through touching by Update)
    /// </summary>
    public bool IsModified(int num)
    {
        if (_thestatus.Upper() == 0)
        {
            return false;
        }
        int n = RankFromNumber(num);
        return (_thestatus.Value(n) != 0);
    }

    /// <summary>
    /// Tells if a Value (of the EditForm) has been touched, i.e. not modified directly but by the modification
    /// of another one (by method Update from the Editor)
    /// </summary>
    public bool IsTouched(int num)
    {
        if (_thestatus.Upper() == 0)
        {
            return false;
        }
        int n = RankFromNumber(num);
        return (_thestatus.Value(n) == 2);
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
        _thetouched = 0;
        if (_themodifs.Upper() == 0)
        {
            return false;
        }
        int tnum = RankFromNumber(num);
        if (tnum == 0)
        {
            return false;
        }
        EditValue acc = _theeditor.EditMode(num);
        if (newval is null && acc != EditValue.Optional)
        {
            return false;
        }
        if (!enforce && (acc == EditValue.EditProtected || acc == EditValue.EditComputed))
        {
            return false;
        }

        //  Satisfies ?
        InterfaceTypedValue typval = _theeditor.TypedValue(num);
        if (!typval.Satisfies(newval))
        {
            return false;
        }
        ParamType pty = typval.Type();
        if (pty == ParamType.ParamIdent && newval is not null)
        {
            if (_themodel is null)
            {
                return false;
            }
            if (_themodel.NextNumberForLabel(newval.ToCString(), 0, false) <= 0)
            {
                return false;
            }
        }

        //  Update ?
        if (!_theeditor.Update(this, num, newval, enforce))
        {
            return false;
        }

        _thestatus.SetValue(tnum, 1);
        _themodifs.SetValue(tnum, newval);
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
        if (_themodifs.Upper() == 0)
        {
            return false;
        }
        int tnum = RankFromNumber(num);
        if (tnum == 0)
        {
            return false;
        }

        _thestatus.SetValue(tnum, 2);
        _themodifs.SetValue(tnum, newval);
        _thetouched++;
        return true;
    }

    /// <summary>
    /// Acts as Touch but for a list Does not work (returns False) if &lt;num&gt; is for a single param
    /// </summary>
    public bool TouchList(int num, NCollection_HSequence<TCollection_HAsciiString>? newlist)
    {
        if (_themodifs.Upper() == 0)
        {
            return false;
        }
        int tnum = RankFromNumber(num);
        if (tnum == 0)
        {
            return false;
        }

        _thestatus.SetValue(tnum, 2);
        _themodifs.SetValue(tnum, newlist);
        _thetouched++;
        return true;
    }

    /// <summary>
    /// Clears modification status : by default all, or one by its numbers (in the Editor)
    /// </summary>
    public void ClearEdit(int num = 0)
    {
        int i, nb = _thestatus.Upper();
        if (num == 0)
        {
            for (i = 1; i <= nb; i++)
            {
                _thestatus.SetValue(i, 0);
            }
        }
        else
        {
            int tnum = RankFromNumber(num);
            if (tnum > 0 && num <= nb)
            {
                _thestatus.SetValue(tnum, 0);
            }
        }
    }

    /// <summary>
    /// Prints Definitions, relative to the Editor
    /// </summary>
    public void PrintDefs(TextWriter s)
    {
        int iv, nbv = NbValues(true);
        s.Write("***** EditForm,  Label : ");
        s.Write(Label());
        s.Write('\n');
        if (IsComplete)
        {
            s.Write("Complete, ");
            s.Write(nbv);
            s.Write(" Values");
            s.Write('\n');
        }
        else
        {
            s.Write("Extraction on ");
            s.Write(nbv);
            s.Write(" Values : (extracted<-editor)");
            s.Write('\n');
            for (iv = 1; iv <= nbv; iv++)
            {
                s.Write("  ");
                s.Write(iv);
                s.Write("<-");
                s.Write(NumberFromRank(iv));
            }
            s.Write('\n');
        }
        s.Write("*****");
        s.Write('\n');
    }

    /// <summary>
    /// Prints Values, according to what and alsolist &lt;names&gt; True : prints Long Names; False : prints
    /// Short Names &lt;what&gt; &lt; 0 : prints Original Values (+ flag Modified) &lt;what&gt; &gt; 0 : prints
    /// Final Values (+flag Modified) &lt;what&gt; = 0 : prints Modified Values (Original + Edited)
    /// &lt;alsolist&gt; False (D) : lists are printed only as their count &lt;alsolist&gt; True : lists are
    /// printed for all their items
    /// </summary>
    public void PrintValues(TextWriter s, int what, bool names, bool alsolist = false)
    {
        int iv, nbv = NbValues(true);
        s.Write("****************************************************");
        s.Write('\n');
        s.Write("*****  ");
        s.Write(Label());
        s.Write(FrameworkMsg.Blanks(Label(), 40));
        s.Write("*****");
        s.Write('\n');
        s.Write("*****                                          *****");
        s.Write('\n');
        if (!_theloaded)
        {
            s.Write("*****         Values are NOT loaded            *****");
            s.Write('\n');
        }
        else
        {
            //  Data on which we worked
            if (_themodel is null)
            {
                if (_theent is null)
                {
                    s.Write("*****  No loaded data");
                }
                else
                {
                    s.Write("*****  No loaded Model. Loaded object : type ");
                    s.Write(_theent.DynamicTypeName());
                }
            }
            else
            {
                if (_theent is null)
                {
                    s.Write("*****  No loaded entity");
                }
                else
                {
                    s.Write("*****  Loaded entity : ");
                    _themodel.PrintLabel(_theent, s);
                }
            }
        }
        s.Write('\n');
        s.Write("****************************************************");
        s.Write('\n');
        s.Write('\n');

        //  Display of values
        bool nams = names;
        int maxnam = _theeditor.MaxNameLength(names ? 0 : -1);
        if (maxnam == 0)
        {
            maxnam = _theeditor.MaxNameLength(0);
            nams = true;
        }
        int nbmod = 0;
        if (what != 0)
        {
            s.Write("Mod N0 Name               Value");
            s.Write('\n');
        }
        else
        {
            s.Write(" N0 Name               Value");
            s.Write('\n');
        }

        for (iv = 1; iv <= nbv; iv++)
        {
            int jv = NumberFromRank(iv);
            string name = _theeditor.Name(jv, !nams);

            //     Original or Final
            if (what != 0)
            {
                TCollection_HAsciiString str = null;
                if (IsModified(jv))
                {
                    s.Write("* ");
                }
                else
                {
                    s.Write("  ");
                }
                s.Write(FrameworkMsg.Blanks(iv, 3));
                s.Write(iv);
                s.Write(" ");
                s.Write(name);
                s.Write(FrameworkMsg.Blanks(name, maxnam));
                s.Write("  ");

                if (_theeditor.IsList(jv))
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
                    PrintList(list, s, alsolist);
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

                s.Write(str is null ? "(null)" : str.ToCString());
                s.Write('\n');

                //    Modified only
            }
            else
            {
                if (!IsModified(jv))
                {
                    continue;
                }
                nbmod++;
                if (_theeditor.IsList(jv))
                {
                    NCollection_HSequence<TCollection_HAsciiString> list =
                        OriginalList(jv);
                    s.Write(FrameworkMsg.Blanks(iv, 3));
                    s.Write(iv);
                    s.Write(" ");
                    s.Write(name);
                    s.Write(FrameworkMsg.Blanks(name, maxnam));
                    s.Write(" ORIG:");
                    PrintList(list, s, alsolist);

                    list = EditedList(jv);
                    s.Write(FrameworkMsg.Blanks("", maxnam + 5));
                    s.Write("MOD :");
                    PrintList(list, s, alsolist);

                    continue;
                }

                TCollection_HAsciiString str = OriginalValue(jv);
                s.Write(FrameworkMsg.Blanks(iv, 3));
                s.Write(iv);
                s.Write(" ");
                s.Write(name);
                s.Write(FrameworkMsg.Blanks(name, maxnam));
                s.Write(" ORIG:");
                s.Write(str is null ? "(null)" : str.ToCString());
                s.Write('\n');
                str = EditedValue(jv);
                s.Write(FrameworkMsg.Blanks("", maxnam + 4));
                s.Write(" MOD :");
                s.Write(str is null ? "(null)" : str.ToCString());
                s.Write('\n');
            }
        }
        if (what == 0)
        {
            s.Write("On ");
            s.Write(nbv);
            s.Write(" Values, ");
            s.Write(nbmod);
            s.Write(" Modified");
            s.Write('\n');
        }
    }

    /// <summary>
    /// Applies modifications to own data Calls ApplyData then Clears Status according EditKeepStatus
    /// </summary>
    public bool Apply()
    {
        bool stat = ApplyData(_theent, _themodel);
        if (stat && !_thekeepst)
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
        return _theeditor.Recognize(this);
    }

    /// <summary>
    /// Applies modifications to data Default uses Editor. Can be redefined
    /// </summary>
    public virtual bool ApplyData(Transient? ent, InterfaceModel? model)
    {
        return _theeditor.Apply(this, ent, model);
    }

    /// <summary>
    /// For an undoable EditForm, Applies ... origibal values ! and clears modified ones Can be run only once
    /// </summary>
    public bool Undo()
    {
        if (_thestatus.Upper() == 0 || _theorigs.Upper() == 0)
        {
            return false;
        }
        int i, nb = _thestatus.Upper();
        for (i = 1; i <= nb; i++)
        {
            if (_thestatus.Value(i) != 0)
            {
                _themodifs.SetValue(i, _theorigs.Value(i));
            }
        }
        return Apply();
    }

    private bool _thecomplete;
    private bool _theloaded;
    private bool _thekeepst;
    private string _thelabel = "";
    private NCollection_Array1<int> _thenums;
    private NCollection_Array1<Transient> _theorigs;
    private NCollection_Array1<Transient> _themodifs;
    private NCollection_Array1<int> _thestatus;
    private Editor? _theeditor;
    private Transient? _theent;
    private InterfaceModel? _themodel;
    private int _thetouched;
}
