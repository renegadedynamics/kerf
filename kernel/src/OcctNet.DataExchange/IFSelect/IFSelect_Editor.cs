// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Editor.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_Editor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_EditValue;

namespace Occt;

/// <summary>
/// An Editor defines a set of values and a way to edit them, on an entity or on the model (e.g. on its
/// header) Each Value is controlled by a TypedValue, with a number (it is an Integer) and a name under two
/// forms (complete and short) and an edit mode
/// </summary>
public abstract class IFSelect_Editor : Standard_Transient
{

    /// <summary>
    /// Sets a Typed Value for a given ident and short name, with an Edit Mode
    /// </summary>
    public void SetValue(int num, Interface_TypedValue? typval, string shortname = "", IFSelect_EditValue editmode = IFSelect_Editable)
    {
        if (num < 1 || num > thenbval)
        {
            return;
        }
        string shn = shortname;
        int lng = shn.Length;
        if (lng > 0)
        {
            thenames.Bind(shortname, num);
        }
        if (lng > themaxsh)
        {
            themaxsh = lng;
        }
        lng = typval.Name().Length;
        if (lng > themaxco)
        {
            themaxco = lng;
        }
        lng = typval.Label().Length;
        if (lng > themaxla)
        {
            themaxla = lng;
        }

        thenames.Bind(typval.Name(), num);
        int edm = (int)editmode;
        thevalues.SetValue(num, typval);
        theshorts.SetValue(num, shn);
        themodes.SetValue(num, edm);
    }

    /// <summary>
    /// Sets a parameter to be a List max &lt; 0 : not for a list (set when starting) max = 0 : list with no
    /// length limit (default for SetList) max &gt; 0 : list limited to &lt;max&gt; items
    /// </summary>
    public void SetList(int num, int max = 0)
    {
        if (num < 1 || num > thenbval)
        {
            return;
        }
        thelists.SetValue(num, max);
    }

    /// <summary>
    /// Returns the count of Typed Values
    /// </summary>
    public int NbValues()
    {
        return thenbval;
    }

    /// <summary>
    /// Returns a Typed Value from its ident
    /// </summary>
    public Interface_TypedValue? TypedValue(int num)
    {
        return thevalues.Value(num);
    }

    /// <summary>
    /// Tells if a parameter is a list
    /// </summary>
    public bool IsList(int num)
    {
        if (num < 1 || num > thenbval)
        {
            return false;
        }
        return (thelists.Value(num) >= 0);
    }

    /// <summary>
    /// Returns max length allowed for a list = 0 means : list with no limit &lt; 0 means : not a list
    /// </summary>
    public int MaxList(int num)
    {
        if (num < 1 || num > thenbval)
        {
            return -1;
        }
        return thelists.Value(num);
    }

    /// <summary>
    /// Returns the name of a Value (complete or short) from its ident Short Name can be empty
    /// </summary>
    public string Name(int num, bool isshort = false)
    {
        if (num < 1 || num > thenbval)
        {
            return "";
        }
        if (isshort)
        {
            return theshorts.Value(num);
        }
        else
        {
            return TypedValue(num).Name();
        }
    }

    /// <summary>
    /// Returns the edit mode of a Value
    /// </summary>
    public IFSelect_EditValue EditMode(int num)
    {
        if (num < 1 || num > thenbval)
        {
            return IFSelect_EditDynamic;
        }
        int edm = themodes.Value(num);
        return (IFSelect_EditValue)edm;
    }

    /// <summary>
    /// Returns the number (ident) of a Value, from its name, short or complete. If not found, returns 0
    /// </summary>
    public int NameNumber(string name)
    {
        if (thenames.Find(name, out int res))
        {
            return res;
        }
        res = atoi(name); // if it's an integer, we try it
        if (res < 1 || res > NbValues())
        {
            res = 0;
        }
        return res;
    }

    public void PrintNames(TextWriter S)
    {
        int i, nb = NbValues();
        S.Write("****    Editor : ");
        S.Write(Label());
        S.Write('\n');
        S.Write("****    Nb Values = ");
        S.Write(nb);
        S.Write("    ****    Names / Labels");
        S.Write('\n');
        S.Write(" Num ");
        if (themaxsh > 0)
        {
            S.Write("Short");
            S.Write(Interface_MSG.Blanks("Short", themaxsh));
            S.Write(" ");
        }
        S.Write("Complete");
        S.Write(Interface_MSG.Blanks("Complete", themaxco));
        S.Write("  Label");
        S.Write('\n');

        for (i = 1; i <= nb; i++)
        {
            Interface_TypedValue tv = TypedValue(i);
            if (tv is null)
            {
                continue;
            }
            S.Write(Interface_MSG.Blanks(i, 3));
            S.Write(i);
            S.Write(" ");
            if (themaxsh > 0)
            {
                string sho = theshorts.Value(i);
                S.Write(sho);
                S.Write(Interface_MSG.Blanks(sho, themaxsh));
                S.Write(" ");
            }
            S.Write(tv.Name());
            S.Write(Interface_MSG.Blanks(tv.Name(), themaxco));
            S.Write("  ");
            S.Write(tv.Label());
            S.Write('\n');
        }
    }

    public void PrintDefs(TextWriter S, bool labels = false)
    {
        int i, nb = NbValues();
        S.Write("****    Editor : ");
        S.Write(Label());
        S.Write('\n');
        S.Write("****    Nb Values = ");
        S.Write(nb);
        S.Write("    ****    ");
        S.Write((labels ? "Labels" : "Names"));
        S.Write("  /  Definitions");
        S.Write('\n');
        S.Write(" Num ");
        if (labels)
        {
            S.Write("Label");
            S.Write(Interface_MSG.Blanks("Label", themaxla));
        }
        else
        {
            if (themaxsh > 0)
            {
                S.Write("Short");
                S.Write(Interface_MSG.Blanks("Short", themaxsh + 1));
            }
            S.Write("Complete");
            S.Write(Interface_MSG.Blanks("Complete", themaxco));
        }
        S.Write("  Edit Mode  &  Definition");
        S.Write('\n');

        for (i = 1; i <= nb; i++)
        {
            Interface_TypedValue tv = TypedValue(i);
            if (tv is null)
            {
                continue;
            }
            S.Write(" ");
            S.Write(Interface_MSG.Blanks(i, 3));
            S.Write(i);
            S.Write(" ");
            if (labels)
            {
                S.Write(tv.Label());
                S.Write(Interface_MSG.Blanks(tv.Label(), themaxla));
            }
            else
            {
                if (themaxsh > 0)
                {
                    string sho = theshorts.Value(i);
                    S.Write(sho);
                    S.Write(Interface_MSG.Blanks(sho, themaxsh));
                    S.Write(" ");
                }
                S.Write(tv.Name());
                S.Write(Interface_MSG.Blanks(tv.Name(), themaxco));
            }

            S.Write(" ");
            int maxls = MaxList(i);
            if (maxls == 0)
            {
                S.Write(" (List) ");
            }
            else if (maxls > 0)
            {
                S.Write(" (List <= ");
                S.Write(maxls);
                S.Write(" Items) ");
            }
            else
            {
                S.Write(" ");
            }
            IFSelect_EditValue edm = EditMode(i);
            switch (edm)
            {
                case IFSelect_Optional:
                    S.Write("Optional ");
                    break;
                case IFSelect_Editable:
                    S.Write("Editable ");
                    break;
                case IFSelect_EditProtected:
                    S.Write("Protected");
                    break;
                case IFSelect_EditComputed:
                    S.Write("Computed ");
                    break;
                case IFSelect_EditRead:
                    S.Write("ReadOnly ");
                    break;
                case IFSelect_EditDynamic:
                    S.Write("Dynamic  ");
                    break;
                default:
                    S.Write("?????????");
                    break;
            }

            S.Write(" ");
            S.Write(tv.Definition());
            S.Write('\n');
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
            return themaxsh;
        }
        if (what == 0)
        {
            return themaxco;
        }
        if (what == 1)
        {
            return themaxla;
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
    public virtual IFSelect_EditForm? Form(bool @readonly, bool undoable = true)
    {
        return new IFSelect_EditForm(this, @readonly, undoable, Label());
    }

    /// <summary>
    /// Tells if this Editor can work on this EditForm and its content (model, entity ?)
    /// </summary>
    public abstract bool Recognize(IFSelect_EditForm? form);

    /// <summary>
    /// Returns the value of an EditForm, for a given item (if not a list. for a list, a Null String may be
    /// returned)
    /// </summary>
    public abstract TCollection_HAsciiString? StringValue(IFSelect_EditForm? form, int num);

    /// <summary>
    /// Returns the value of an EditForm as a List, for a given item If not a list, a Null Handle should be
    /// returned Default returns a Null Handle, because many Editors have no list to edit. To be redefined as
    /// required
    /// </summary>
    public virtual NCollection_HSequence<TCollection_HAsciiString>? ListValue(IFSelect_EditForm? form, int num)
    {
        NCollection_HSequence<TCollection_HAsciiString> list = null;
        return list;
    }

    /// <summary>
    /// Loads original values from some data, to an EditForm Remark: &lt;ent&gt; may be Null, this means all
    /// &lt;model&gt; is concerned Also &lt;model&gt; may be Null, if no context applies for &lt;ent&gt; And both
    /// &lt;ent&gt; and &lt;model&gt; may be Null, for a full static editor
    /// </summary>
    public abstract bool Load(IFSelect_EditForm? form, Standard_Transient? ent, Interface_InterfaceModel? model);

    /// <summary>
    /// Updates the EditForm when a parameter is modified I.E. default does nothing, can be redefined, as follows
    /// : Returns True when done (even if does nothing), False in case of refuse (for instance, if the new value
    /// is not suitable) &lt;num&gt; is the rank of the parameter for the EDITOR itself &lt;enforce&gt; True
    /// means that protected parameters can be touched If a parameter commands the value of other ones, when it
    /// is modified, it is necessary to touch them by Touch from EditForm
    /// </summary>
    public virtual bool Update(IFSelect_EditForm? form, int num, TCollection_HAsciiString? newval, bool enforce)
    {
        return true;
    }

    /// <summary>
    /// Acts as Update, but when the value is a list
    /// </summary>
    public virtual bool UpdateList(IFSelect_EditForm? form, int num, NCollection_HSequence<TCollection_HAsciiString>? newlist, bool enforce)
    {
        return true;
    }

    /// <summary>
    /// Applies modified values of the EditForm with some data Remark: &lt;ent&gt; may be Null, this means all
    /// &lt;model&gt; is concerned Also &lt;model&gt; may be Null, if no context applies for &lt;ent&gt; And both
    /// &lt;ent&gt; and &lt;model&gt; may be Null, for a full static editor
    /// </summary>
    public abstract bool Apply(IFSelect_EditForm? form, Standard_Transient? ent, Interface_InterfaceModel? model);

    /// <summary>
    /// Prepares the list of Typed Values (gives its count) This count can be tuned later, to a LOWER value, this
    /// allows to initialize with a "maximum reservation" then cut the extra
    /// </summary>
    protected IFSelect_Editor(int nbval)
    {
        thenbval = nbval;
        themaxsh = 0;
        themaxco = 0;
        themaxla = 0;
        thevalues = new NCollection_Array1<Interface_TypedValue>(1, nbval);
        theshorts = new NCollection_Array1<string>(1, nbval);
        themodes = new NCollection_Array1<int>(1, nbval);
        thelists = new NCollection_Array1<int>(1, nbval);
        thelists.Init(-1);
    }

    /// <summary>
    /// Adjusts the true count of values. It can be LOWER or equal to the initial size (which then acts as a
    /// reservation), but never greater
    /// </summary>
    protected void SetNbValues(int nbval)
    {
        if (nbval > thevalues.Upper())
        {
            throw new Standard_OutOfRange("IFSelect_Editor:SetNbValues");
        }
        thenbval = nbval;
    }

    private int thenbval;
    private int themaxsh;
    private int themaxco;
    private int themaxla;
    private NCollection_DataMap<string, int> thenames = new NCollection_DataMap<string, int>();
    private NCollection_Array1<Interface_TypedValue> thevalues;
    private NCollection_Array1<string> theshorts;
    private NCollection_Array1<int> themodes;
    private NCollection_Array1<int> thelists;
}
