// Port of: src/DataExchange/TKXSBase/Interface/Interface_LineBuffer.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_LineBuffer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// Simple management of a line buffer, used by the interface file writers (STEP's writer cuts its lines with it):
/// a maximum size, initial blanks, and a "keep" mark to carry the end of a line to the next one.
/// </summary>
/// <remarks>
/// The buffer is OCCT's array of characters, 1-based, with its null terminators, so the line cutting is the same
/// character for character.
/// </remarks>
internal class LineBuffer
{
    /// <summary>Creates a line buffer with an absolute maximum size (the default only satisfies the compiler).</summary>
    public LineBuffer(int size = 10)
    {
        _line = new NCollection_Array1<char>(1, size + 1);
        _line.SetValue(1, '\0');
        _max = size;
        _init = _len = _get = _keep = _friz = 0;
    }

    /// <summary>Changes the maximum allowed size of the buffer; zero sets it to the initial size.</summary>
    /// <exception cref="OutOfRangeException">More than the initial size.</exception>
    public void SetMax(int max)
    {
        if (max > _line.Length())
        {
            throw new OutOfRangeException("Interface LineBuffer : SetMax");
        }
        if (max <= 0)
        {
            _max = _line.Length();
        }
        else
        {
            _max = max;
        }
    }

    /// <summary>Sets an initial reservation for blank characters (counted in the size of the current line).</summary>
    /// <exception cref="OutOfRangeException">Not less than the maximum size.</exception>
    public void SetInitial(int initial)
    {
        if (_friz > 0)
        {
            return;
        }
        if (initial >= _max)
        {
            throw new OutOfRangeException("Interface LineBuffer : SetInitial");
        }
        if (initial <= 0)
        {
            _init = 0;
        }
        else
        {
            _init = initial;
        }
    }

    /// <summary>
    /// Sets a keep status at the current length: at the next Move, the new line will begin with the characters
    /// between keep + 1 and the current length.
    /// </summary>
    public void SetKeep()
    {
        _keep = -_len;
    }

    /// <summary>
    /// True if there is room enough to add <paramref name="more"/> characters; else the buffer must be dumped
    /// before refilling it. <paramref name="more"/> is recorded to manage the keep status.
    /// </summary>
    public bool CanGet(int more)
    {
        _get = more;
        if ((_len + _init + more) <= _max)
        {
            return true;
        }
        if (_keep < 0)
        {
            _keep = -_keep;
        }
        return false;
    }

    /// <summary>The content of the line buffer (up to its first null character).</summary>
    public string Content()
    {
        int length = 0;
        while (length < _line.Length() && _line.Value(length + 1) != '\0')
        {
            length++;
        }
        return new string(_line.AsSpan().Slice(0, length));
    }

    /// <summary>The length of the line buffer (with the initial blanks).</summary>
    public int Length() => _len + _init;

    /// <summary>Clears the line buffer completely.</summary>
    public void Clear()
    {
        _get = _keep = _len = _friz = 0;
        _line.SetValue(1, '\0');
    }

    /// <summary>
    /// Inhibits the effect of SetInitial until the next Move (i.e. Keep): Prepare will not insert initial blanks,
    /// but further ones will. This allows to cancel initial blanks on an internal split.
    /// </summary>
    public void FreezeInitial()
    {
        _friz = _init + 1;
        _init = 0;
    }

    /// <summary>Appends the content of the line buffer to a string, then clears the buffer (keeping the kept part).</summary>
    public void Move(ref string str)
    {
        Prepare();
        str += Content();
        Keep();
    }

    /// <summary>Same as above, with the string known through a handle.</summary>
    public void Move(TCollection_HAsciiString str)
    {
        Prepare();
        str.AssignCat(Content());
        Keep();
    }

    /// <summary>Same as above, but generates the handled string.</summary>
    public TCollection_HAsciiString Moved()
    {
        Prepare();
        TCollection_HAsciiString r = new TCollection_HAsciiString(Content());
        Keep();
        return r;
    }

    /// <summary>Adds a text (OCCT's C string: up to its first null character).</summary>
    public void Add(string text)
    {
        int length = text.IndexOf('\0');
        Add(text, length < 0 ? text.Length : length);
    }

    /// <summary>Adds the first <paramref name="lntext"/> characters of a text (cut at the maximum size).</summary>
    public void Add(string text, int lntext)
    {
        int lnt = (lntext > (_max - _len - _init) ? (_max - _len - _init) : lntext);
        for (int i = 1; i <= lnt; ++i)
        {
            _line.SetValue(_len + i, text[i - 1]);
        }
        _len += lnt;
        _line.SetValue(_len + 1, '\0');
    }

    /// <summary>Adds one character (not checked against the maximum size).</summary>
    public void Add(char text)
    {
        _line.SetValue(_len + 1, text);
        ++_len;
        _line.SetValue(_len + 1, '\0');
    }

    /// <summary>
    /// Prepares Move: inserts the initial blanks if required, and determines if SetKeep can be supported (it cannot be
    /// if the length plus the next string to get, see CanGet, overpasses the maximum size).
    /// </summary>
    private void Prepare()
    {
        //  ATTENTION aux blanx initiaux
        if (_init > 0)
        {
            if ((_len + _init) > _max)
            {
                return;
            }

            for (int i = _len + 1; i > 0; --i)
            {
                _line.SetValue(i + _init, _line.Value(i));
            }
            for (int i = 1; i <= _init; ++i)
            {
                _line.SetValue(i, ' ');
            }
        }

        //  MANAGE KEEP: is it playable? otherwise, cancel. if yes, note the junction
        if (_keep > 0)
        {
            _keep += (_init + 1); // myInit, and +1 because Keep INCLUDED
        }
        if (_keep > 0)
        {
            if ((_len + _get + _init - _keep) >= _max)
            {
                _keep = 0;
            }
        }
        if (_keep > 0)
        {
            _kept = _line.Value(_keep);
            _line.SetValue(_keep, '\0');
        }
    }

    /// <summary>Keeps the characters from SetKeep; if SetKeep is zero, equivalent to Clear.</summary>
    private void Keep()
    {
        //  If Keep, save from myKeep + 1 to myLen (+1 for final 0)
        if (_keep > 0)
        {
            _line.SetValue(1, _kept);
            for (int i = _keep + 1; i <= _len + _init + 1; ++i)
            {
                _line.SetValue(i - _keep + 1, _line.Value(i));
            }
            _len = _len + _init - _keep + 1;
        }
        else
        {
            Clear();
        }
        _get = _keep = 0;
        if (_friz > 0)
        {
            _init = _friz - 1;
            _friz = 0;
        }
    }

    private readonly NCollection_Array1<char> _line;
    private int _max;
    private int _init;
    private int _keep;
    private int _get;
    private int _len;
    private int _friz;
    private char _kept;
}
