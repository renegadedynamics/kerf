// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Editor.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Editor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// An Editor defines a set of values and a way to edit them, on an entity or on the model (e.g. on its
/// header) Each Value is controlled by a TypedValue, with a number (it is an Integer) and a name under two
/// forms (complete and short) and an edit mode
/// </summary>
internal abstract class Editor : Transient
{

    /// <summary>
    /// Sets a Typed Value for a given ident and short name, with an Edit Mode
    /// </summary>
    public void SetValue(int num, InterfaceTypedValue? typval, string shortname = "", EditValue editmode = EditValue.Editable)
    {
        if (num < 1 || num > _thenbval)
        {
            return;
        }
        string shn = shortname;
        int lng = shn.Length;
        if (lng > 0)
        {
            _thenames[shortname] = num;
        }
        if (lng > _themaxsh)
        {
            _themaxsh = lng;
        }
        lng = typval.Name().Length;
        if (lng > _themaxco)
        {
            _themaxco = lng;
        }
        lng = typval.Label().Length;
        if (lng > _themaxla)
        {
            _themaxla = lng;
        }

        _thenames[typval.Name()] = num;
        int edm = (int)editmode;
        _thevalues.SetValue(num, typval);
        _theshorts.SetValue(num, shn);
        _themodes.SetValue(num, edm);
    }

    /// <summary>
    /// Sets a parameter to be a List max &lt; 0 : not for a list (set when starting) max = 0 : list with no
    /// length limit (default for SetList) max &gt; 0 : list limited to &lt;max&gt; items
    /// </summary>
    public void SetList(int num, int max = 0)
    {
        if (num < 1 || num > _thenbval)
        {
            return;
        }
        _thelists.SetValue(num, max);
    }

    /// <summary>
    /// Returns the count of Typed Values
    /// </summary>
    public int ValueCount
    {
        get
        {
            return _thenbval;
        }
    }

    /// <summary>
    /// Returns a Typed Value from its ident
    /// </summary>
    public InterfaceTypedValue? TypedValue(int num)
    {
        return _thevalues.Value(num);
    }

    /// <summary>
    /// Tells if a parameter is a list
    /// </summary>
    public bool IsList(int num)
    {
        if (num < 1 || num > _thenbval)
        {
            return false;
        }
        return (_thelists.Value(num) >= 0);
    }

    /// <summary>
    /// Returns max length allowed for a list = 0 means : list with no limit &lt; 0 means : not a list
    /// </summary>
    public int MaxList(int num)
    {
        if (num < 1 || num > _thenbval)
        {
            return -1;
        }
        return _thelists.Value(num);
    }

    /// <summary>
    /// Returns the name of a Value (complete or short) from its ident Short Name can be empty
    /// </summary>
    public string Name(int num, bool isshort = false)
    {
        if (num < 1 || num > _thenbval)
        {
            return "";
        }
        if (isshort)
        {
            return _theshorts.Value(num);
        }
        else
        {
            return TypedValue(num).Name();
        }
    }

    /// <summary>
    /// Returns the edit mode of a Value
    /// </summary>
    public EditValue EditMode(int num)
    {
        if (num < 1 || num > _thenbval)
        {
            return EditValue.EditDynamic;
        }
        int edm = _themodes.Value(num);
        return (EditValue)edm;
    }

    /// <summary>
    /// Returns the number (ident) of a Value, from its name, short or complete. If not found, returns 0
    /// </summary>
    public int NameNumber(string name)
    {
        if (_thenames.TryGetValue(name, out int res))
        {
            return res;
        }
        res = Atoi(name); // if it's an integer, we try it
        if (res < 1 || res > ValueCount)
        {
            res = 0;
        }
        return res;
    }

    public void PrintNames(TextWriter s)
    {
        int i, nb = ValueCount;
        s.Write("****    Editor : ");
        s.Write(Label());
        s.Write('\n');
        s.Write("****    Nb Values = ");
        s.Write(nb);
        s.Write("    ****    Names / Labels");
        s.Write('\n');
        s.Write(" Num ");
        if (_themaxsh > 0)
        {
            s.Write("Short");
            s.Write(FrameworkMsg.Blanks("Short", _themaxsh));
            s.Write(" ");
        }
        s.Write("Complete");
        s.Write(FrameworkMsg.Blanks("Complete", _themaxco));
        s.Write("  Label");
        s.Write('\n');

        for (i = 1; i <= nb; i++)
        {
            InterfaceTypedValue tv = TypedValue(i);
            if (tv is null)
            {
                continue;
            }
            s.Write(FrameworkMsg.Blanks(i, 3));
            s.Write(i);
            s.Write(" ");
            if (_themaxsh > 0)
            {
                string sho = _theshorts.Value(i);
                s.Write(sho);
                s.Write(FrameworkMsg.Blanks(sho, _themaxsh));
                s.Write(" ");
            }
            s.Write(tv.Name());
            s.Write(FrameworkMsg.Blanks(tv.Name(), _themaxco));
            s.Write("  ");
            s.Write(tv.Label());
            s.Write('\n');
        }
    }

    public void PrintDefs(TextWriter s, bool labels = false)
    {
        int i, nb = ValueCount;
        s.Write("****    Editor : ");
        s.Write(Label());
        s.Write('\n');
        s.Write("****    Nb Values = ");
        s.Write(nb);
        s.Write("    ****    ");
        s.Write((labels ? "Labels" : "Names"));
        s.Write("  /  Definitions");
        s.Write('\n');
        s.Write(" Num ");
        if (labels)
        {
            s.Write("Label");
            s.Write(FrameworkMsg.Blanks("Label", _themaxla));
        }
        else
        {
            if (_themaxsh > 0)
            {
                s.Write("Short");
                s.Write(FrameworkMsg.Blanks("Short", _themaxsh + 1));
            }
            s.Write("Complete");
            s.Write(FrameworkMsg.Blanks("Complete", _themaxco));
        }
        s.Write("  Edit Mode  &  Definition");
        s.Write('\n');

        for (i = 1; i <= nb; i++)
        {
            InterfaceTypedValue tv = TypedValue(i);
            if (tv is null)
            {
                continue;
            }
            s.Write(" ");
            s.Write(FrameworkMsg.Blanks(i, 3));
            s.Write(i);
            s.Write(" ");
            if (labels)
            {
                s.Write(tv.Label());
                s.Write(FrameworkMsg.Blanks(tv.Label(), _themaxla));
            }
            else
            {
                if (_themaxsh > 0)
                {
                    string sho = _theshorts.Value(i);
                    s.Write(sho);
                    s.Write(FrameworkMsg.Blanks(sho, _themaxsh));
                    s.Write(" ");
                }
                s.Write(tv.Name());
                s.Write(FrameworkMsg.Blanks(tv.Name(), _themaxco));
            }

            s.Write(" ");
            int maxls = MaxList(i);
            if (maxls == 0)
            {
                s.Write(" (List) ");
            }
            else if (maxls > 0)
            {
                s.Write(" (List <= ");
                s.Write(maxls);
                s.Write(" Items) ");
            }
            else
            {
                s.Write(" ");
            }
            EditValue edm = EditMode(i);
            switch (edm)
            {
                case EditValue.Optional:
                    s.Write("Optional ");
                    break;
                case EditValue.Editable:
                    s.Write("Editable ");
                    break;
                case EditValue.EditProtected:
                    s.Write("Protected");
                    break;
                case EditValue.EditComputed:
                    s.Write("Computed ");
                    break;
                case EditValue.EditRead:
                    s.Write("ReadOnly ");
                    break;
                case EditValue.EditDynamic:
                    s.Write("Dynamic  ");
                    break;
                default:
                    s.Write("?????????");
                    break;
            }

            s.Write(" ");
            s.Write(tv.Definition());
            s.Write('\n');
        }
    }

    /// <summary>
    /// Returns the MaxLength of, according to what : &lt;what&gt; = -1 : length of short names &lt;what&gt; = 0
    /// : length of complete names &lt;what&gt; = 1 : length of values labels
    /// </summary>
    public int MaxNameLength(int what)
    {
        if (what == -1)
        {
            return _themaxsh;
        }
        if (what == 0)
        {
            return _themaxco;
        }
        if (what == 1)
        {
            return _themaxla;
        }
        return 0;
    }

    /// <summary>
    /// Returns the specific label
    /// </summary>
    public abstract string Label();

    /// <summary>
    /// Builds and Returns an EditForm, empty (no data yet) Can be redefined to return a specific type of
    /// EditForm
    /// </summary>
    public virtual EditForm? Form(bool @readonly, bool undoable = true)
    {
        return new EditForm(this, @readonly, undoable, Label());
    }

    /// <summary>
    /// Tells if this Editor can work on this EditForm and its content (model, entity ?)
    /// </summary>
    public abstract bool Recognize(EditForm? form);

    /// <summary>
    /// Returns the value of an EditForm, for a given item (if not a list. for a list, a Null String may be
    /// returned)
    /// </summary>
    public abstract TCollection_HAsciiString? StringValue(EditForm? form, int num);

    /// <summary>
    /// Returns the value of an EditForm as a List, for a given item If not a list, a Null Handle should be
    /// returned Default returns a Null Handle, because many Editors have no list to edit. To be redefined as
    /// required
    /// </summary>
    public virtual NCollection_HSequence<TCollection_HAsciiString>? ListValue(EditForm? form, int num)
    {
        NCollection_HSequence<TCollection_HAsciiString> list = null;
        return list;
    }

    /// <summary>
    /// Loads original values from some data, to an EditForm Remark: &lt;ent&gt; may be Null, this means all
    /// &lt;model&gt; is concerned Also &lt;model&gt; may be Null, if no context applies for &lt;ent&gt; And both
    /// &lt;ent&gt; and &lt;model&gt; may be Null, for a full static editor
    /// </summary>
    public abstract bool Load(EditForm? form, Transient? ent, InterfaceModel? model);

    /// <summary>
    /// Updates the EditForm when a parameter is modified I.E. default does nothing, can be redefined, as follows
    /// : Returns True when done (even if does nothing), False in case of refuse (for instance, if the new value
    /// is not suitable) &lt;num&gt; is the rank of the parameter for the EDITOR itself &lt;enforce&gt; True
    /// means that protected parameters can be touched If a parameter commands the value of other ones, when it
    /// is modified, it is necessary to touch them by Touch from EditForm
    /// </summary>
    public virtual bool Update(EditForm? form, int num, TCollection_HAsciiString? newval, bool enforce)
    {
        return true;
    }

    /// <summary>
    /// Acts as Update, but when the value is a list
    /// </summary>
    public virtual bool UpdateList(EditForm? form, int num, NCollection_HSequence<TCollection_HAsciiString>? newlist, bool enforce)
    {
        return true;
    }

    /// <summary>
    /// Applies modified values of the EditForm with some data Remark: &lt;ent&gt; may be Null, this means all
    /// &lt;model&gt; is concerned Also &lt;model&gt; may be Null, if no context applies for &lt;ent&gt; And both
    /// &lt;ent&gt; and &lt;model&gt; may be Null, for a full static editor
    /// </summary>
    public abstract bool Apply(EditForm? form, Transient? ent, InterfaceModel? model);

    /// <summary>
    /// Prepares the list of Typed Values (gives its count) This count can be tuned later, to a LOWER value, this
    /// allows to initialize with a "maximum reservation" then cut the extra
    /// </summary>
    protected Editor(int nbval)
    {
        _thenbval = nbval;
        _themaxsh = 0;
        _themaxco = 0;
        _themaxla = 0;
        _thevalues = new NCollection_Array1<InterfaceTypedValue>(1, nbval);
        _theshorts = new NCollection_Array1<string>(1, nbval);
        _themodes = new NCollection_Array1<int>(1, nbval);
        _thelists = new NCollection_Array1<int>(1, nbval);
        _thelists.Init(-1);
    }

    /// <summary>
    /// Adjusts the true count of values. It can be LOWER or equal to the initial size (which then acts as a
    /// reservation), but never greater
    /// </summary>
    protected void SetNbValues(int nbval)
    {
        if (nbval > _thevalues.Upper())
        {
            throw new OutOfRangeException("IFSelect_Editor:SetNbValues");
        }
        _thenbval = nbval;
    }

    private int _thenbval;
    private int _themaxsh;
    private int _themaxco;
    private int _themaxla;
    private Dictionary<string, int> _thenames = new Dictionary<string, int>();
    private NCollection_Array1<InterfaceTypedValue> _thevalues;
    private NCollection_Array1<string> _theshorts;
    private NCollection_Array1<int> _themodes;
    private NCollection_Array1<int> _thelists;
}
