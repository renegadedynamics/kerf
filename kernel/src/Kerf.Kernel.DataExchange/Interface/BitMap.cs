// Port of: src/DataExchange/TKXSBase/Interface/Interface_BitMap.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_BitMap.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// A bit map simply allows to associate a boolean flag to each item of a list, such as a list of entities,
/// etc... numbered between 1 and a positive count nbitems The BitMap class allows to associate several
/// binary flags, each of one is identified by a number from 0 to a count which can remain at zero or be
/// positive : nbflags Flags lists over than numflag=0 are added after creation Each of one can be named,
/// hence the user can identify it either by its flag number or by a name which gives a flag n0 (flag n0 0
/// has no name)
/// </summary>
public class BitMap
{

    /// <summary>
    /// Creates a empty BitMap
    /// </summary>
    public BitMap()
    {
        Initialize(0);
    }

    /// <summary>
    /// Creates a BitMap for &lt;nbitems&gt; items One flag is defined, n0 0 &lt;resflags&gt; prepares allocation
    /// for &lt;resflags&gt; more flags Flags values start at false
    /// </summary>
    public BitMap(int nbitems, int resflags = 0)
    {
        Initialize(nbitems, resflags);
    }

    /// <summary>
    /// Initialize empty bit by &lt;nbitems&gt; items One flag is defined, n0 0 &lt;resflags&gt; prepares
    /// allocation for &lt;resflags&gt; more flags Flags values start at false
    /// </summary>
    public void Initialize(int nbitems, int resflags = 0)
    {
        _thenbitems = nbitems;
        _thenbwords = nbitems / 32 + 1;
        _thenbflags = 0;
        _theflags = new NCollection_HArray1<int>(0, _thenbwords * (resflags + 1), 0);
    }

    /// <summary>
    /// Creates a BitMap from another one if &lt;copied&gt; is True, copies data else, data are not copied, only
    /// the header object is
    /// </summary>
    public BitMap(BitMap other, bool copied = false)
    {
        Initialize(other, copied);
    }

    /// <summary>
    /// Initialize a BitMap from another one
    /// </summary>
    public void Initialize(BitMap other, bool copied = false)
    {
        _thenbitems = other._thenbitems;
        _thenbwords = other._thenbwords;
        _thenbflags = other._thenbflags;
        if (!copied)
        {
            _theflags = other._theflags;
            _thenames = other._thenames;
        }
        else
        {
            _theflags = new NCollection_HArray1<int>(other._theflags.Array1());
            if (other._thenames is not null)
            {
                _thenames = new NCollection_HSequence<string>(other._thenames.Sequence());
            }
        }
    }

    /// <summary>
    /// Reservates for a count of more flags
    /// </summary>
    public void Reservate(int moreflags)
    {
        int nb = _theflags.Upper();
        int nbflags = nb / _thenbwords - 1; // flag 0 not counted ...
        if (nbflags >= _thenbflags + moreflags)
        {
            return;
        }
        int nbw = _thenbwords * (_thenbflags + moreflags + 2);
        NCollection_HArray1<int> flags = new NCollection_HArray1<int>(0, nbw);
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 0; i <= nb; i++)
        {
            flags.SetValue(i, _theflags.Value(i));
        }
        for (i = nb + 1; i <= nbw; i++)
        {
            flags.SetValue(i, 0);
        }
        _theflags = flags;
    }

    /// <summary>
    /// Sets for a new count of items, which can be either less or greater than the former one For new items,
    /// their flags start at false
    /// </summary>
    public void SetLength(int nbitems)
    {
        int nbw = nbitems / 32 + 1;
        if (nbw == _thenbwords)
        {
            return;
        }
        NCollection_HArray1<int> flags =
            new NCollection_HArray1<int>(0, nbw * (_thenbflags + 1));
        if (nbw > _thenbwords)
        {
            flags.Init(0);
        }
        int nbmots = (nbw > _thenbwords ? _thenbwords : nbw);
        int i0 = 0, i1 = 0;
        for (int nf = 0; nf <= _thenbflags; nf++)
        {
            for (int i = 0; i < nbmots; i++)
            {
                flags.SetValue(i1 + i, _theflags.Value(i0 + i));
            }
            i0 += _thenbwords;
            i1 += nbw;
        }
        _theflags = flags;
        _thenbitems = nbitems;
        _thenbwords = nbw;
    }

    /// <summary>
    /// Adds a flag, a name can be attached to it Returns its flag number Makes required reservation
    /// </summary>
    public int AddFlag(string name = "")
    {
        Reservate(1);
        int deja = 0;
        if (_thenames is null)
        {
            _thenames = new NCollection_HSequence<string>();
        }
        else
        {
            int i, nb = _thenames.Length();
            for (i = 1; i <= nb; i++)
            {
                if (_thenames.Value(i) == ".")
                {
                    _thenames.ChangeValue(i) += name; // OCCT: appended, not replaced
                    deja = i;
                }
            }
        }
        if (deja == 0)
        {
            _thenames.Append((name));
        }
        _thenbflags++;
        return (deja != 0 ? deja : _thenbflags);
    }

    /// <summary>
    /// Adds several flags (&lt;more&gt;) with no name Returns the number of last added flag
    /// </summary>
    public int AddSomeFlags(int more)
    {
        Reservate(more);
        if (_thenames is null)
        {
            _thenames = new NCollection_HSequence<string>();
        }
        for (int i = 1; i <= more; i++)
        {
            _thenames.Append((""));
        }
        _thenbflags += more;
        return _thenbflags;
    }

    /// <summary>
    /// Removes a flag given its number. Returns True if done, false if num is out of range
    /// </summary>
    public bool RemoveFlag(int num)
    {
        if (num < 1 || num > _thenames.Length())
        {
            return false;
        }
        if (num == _thenames.Length())
        {
            _thenames.Remove(_thenames.Length());
        }
        else
        {
            _thenames.ChangeValue(num) += "."; // OCCT: appended, not replaced
        }
        _thenbflags--;
        return true;
    }

    /// <summary>
    /// Sets a name for a flag, given its number name can be empty (to erase the name of a flag) Returns True if
    /// done, false if : num is out of range, or name non-empty already set to another flag
    /// </summary>
    public bool SetFlagName(int num, string name)
    {
        if (num < 1 || num > _thenames.Length())
        {
            return false;
        }
        int deja = (name.Length == 0 ? 0 : FlagNumber(name));
        if (deja != 0 && deja != num)
        {
            return false;
        }
        _thenames.ChangeValue(num) += name; // OCCT: appended, not replaced
        return true;
    }

    /// <summary>
    /// Returns the count of flags (flag 0 not included)
    /// </summary>
    public int FlagCount
    {
        get
        {
            return _thenbflags;
        }
    }

    /// <summary>
    /// Returns the count of items (i.e. the length of the bitmap)
    /// </summary>
    public int Length()
    {
        return _thenbitems;
    }

    /// <summary>
    /// Returns the name recorded for a flag, or an empty string
    /// </summary>
    public string FlagName(int num)
    {
        if (_theflags is null)
        {
            return "";
        }
        if (num < 1 || num > _thenames.Length())
        {
            return "";
        }
        return _thenames.Value(num);
    }

    /// <summary>
    /// Returns the number or a flag given its name, or zero
    /// </summary>
    public int FlagNumber(string name)
    {
        if (name.Length == 0)
        {
            return 0;
        }
        if (_thenames is null)
        {
            return 0;
        }
        int i, nb = _thenames.Length();
        for (i = 1; i <= nb; i++)
        {
            if (_thenames.Value(i) == name)
            {
                return i;
            }
        }
        return 0;
    }

    /// <summary>
    /// Returns the value (true/false) of a flag, from : - the number of the item - the flag number, by default 0
    /// </summary>
    public bool Value(int item, int flag = 0)
    {
        int numw = (_thenbwords * flag) + (item >> 5);
        int val = _theflags.Value(numw);
        if (val == 0)
        {
            return false;
        }
        if (val == ~(0))
        {
            return true;
        }
        int numb = item & 31;
        return (((1 << numb) & val) != 0);
    }

    /// <summary>
    /// Sets a new value for a flag
    /// </summary>
    public void SetValue(int item, bool val, int flag = 0)
    {
        if (val)
        {
            SetTrue(item, flag);
        }
        else
        {
            SetFalse(item, flag);
        }
    }

    /// <summary>
    /// Sets a flag to True
    /// </summary>
    public void SetTrue(int item, int flag = 0)
    {
        int numw = (_thenbwords * flag) + (item >> 5);
        int numb = item & 31;
        _theflags.ChangeValue(numw) |= (1 << numb);
    }

    /// <summary>
    /// Sets a flag to False
    /// </summary>
    public void SetFalse(int item, int flag = 0)
    {
        int numw = (_thenbwords * flag) + (item >> 5);
        ref int val = ref _theflags.ChangeValue(numw);
        if (val == 0)
        {
            return;
        }
        int numb = item & 31;
        _theflags.ChangeValue(numw) &= ~(1 << numb);
    }

    /// <summary>
    /// Returns the former value for a flag and sets it to True (before : value returned; after : True)
    /// </summary>
    public bool CTrue(int item, int flag = 0)
    {
        int numw = (_thenbwords * flag) + (item >> 5);
        int numb = item & 31;
        ref int val = ref _theflags.ChangeValue(numw);
        int res, mot = (1 << numb);

        if (val == 0)
        {
            val = mot;
            return false;
        }
        else
        {
            res = val & mot;
            val |= mot;
        }
        return (res != 0);
    }

    /// <summary>
    /// Returns the former value for a flag and sets it to False (before : value returned; after : False)
    /// </summary>
    public bool CFalse(int item, int flag = 0)
    {
        int numw = (_thenbwords * flag) + (item >> 5);
        int numb = item & 31;
        ref int val = ref _theflags.ChangeValue(numw);
        int res, mot = ~(1 << numb);

        if (val == ~(0))
        {
            val = mot;
            return false;
        }
        else
        {
            res = val | mot;
            val &= mot;
        }
        return (res != 0);
    }

    /// <summary>
    /// Initialises all the values of Flag Number &lt;flag&gt; to a given value &lt;val&gt;
    /// </summary>
    public void Init(bool val, int flag = 0)
    {
        int i, ii = _thenbwords, i1 = _thenbwords * flag;
        if (flag < 0)
        {
            i1 = 0;
            ii = _thenbwords * (_thenbflags + 1);
        }
        if (val)
        {
            for (i = 0; i < ii; i++)
            {
                _theflags.SetValue(i1 + i, ~(0));
            }
        }
        else
        {
            for (i = 0; i < ii; i++)
            {
                _theflags.SetValue(i1 + i, 0);
            }
        }
    }

    /// <summary>
    /// Clear all field of bit map
    /// </summary>
    public void Clear()
    {
        _theflags = null;
        Initialize(0);
    }

    private int _thenbitems;
    private int _thenbwords;
    private int _thenbflags;
    private NCollection_HArray1<int>? _theflags;
    private NCollection_HSequence<string>? _thenames;
}
