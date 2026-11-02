// Port of: src/DataExchange/TKXSBase/Interface/Interface_LineBuffer.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_LineBuffer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Simple management of a line buffer, used by the interface file writers (STEP's writer cuts its lines with it):
/// a maximum size, initial blanks, and a "keep" mark to carry the end of a line to the next one.
/// </summary>
/// <remarks>
/// The buffer is OCCT's array of characters, 1-based, with its null terminators, so the line cutting is the same
/// character for character.
/// </remarks>
public class Interface_LineBuffer
{
    /// <summary>Creates a line buffer with an absolute maximum size (the default only satisfies the compiler).</summary>
    public Interface_LineBuffer(int size = 10)
    {
        myLine = new NCollection_Array1<char>(1, size + 1);
        myLine.SetValue(1, '\0');
        myMax = size;
        myInit = myLen = myGet = myKeep = myFriz = 0;
    }

    /// <summary>Changes the maximum allowed size of the buffer; zero sets it to the initial size.</summary>
    /// <exception cref="Standard_OutOfRange">More than the initial size.</exception>
    public void SetMax(int theMax)
    {
        if (theMax > myLine.Length())
        {
            throw new Standard_OutOfRange("Interface LineBuffer : SetMax");
        }
        if (theMax <= 0)
        {
            myMax = myLine.Length();
        }
        else
        {
            myMax = theMax;
        }
    }

    /// <summary>Sets an initial reservation for blank characters (counted in the size of the current line).</summary>
    /// <exception cref="Standard_OutOfRange">Not less than the maximum size.</exception>
    public void SetInitial(int theInitial)
    {
        if (myFriz > 0)
        {
            return;
        }
        if (theInitial >= myMax)
        {
            throw new Standard_OutOfRange("Interface LineBuffer : SetInitial");
        }
        if (theInitial <= 0)
        {
            myInit = 0;
        }
        else
        {
            myInit = theInitial;
        }
    }

    /// <summary>
    /// Sets a keep status at the current length: at the next Move, the new line will begin with the characters
    /// between keep + 1 and the current length.
    /// </summary>
    public void SetKeep()
    {
        myKeep = -myLen;
    }

    /// <summary>
    /// True if there is room enough to add <paramref name="theMore"/> characters; else the buffer must be dumped
    /// before refilling it. <paramref name="theMore"/> is recorded to manage the keep status.
    /// </summary>
    public bool CanGet(int theMore)
    {
        myGet = theMore;
        if ((myLen + myInit + theMore) <= myMax)
        {
            return true;
        }
        if (myKeep < 0)
        {
            myKeep = -myKeep;
        }
        return false;
    }

    /// <summary>The content of the line buffer (up to its first null character).</summary>
    public string Content()
    {
        int aLength = 0;
        while (aLength < myLine.Length() && myLine.Value(aLength + 1) != '\0')
        {
            aLength++;
        }
        return new string(myLine.AsSpan().Slice(0, aLength));
    }

    /// <summary>The length of the line buffer (with the initial blanks).</summary>
    public int Length() => myLen + myInit;

    /// <summary>Clears the line buffer completely.</summary>
    public void Clear()
    {
        myGet = myKeep = myLen = myFriz = 0;
        myLine.SetValue(1, '\0');
    }

    /// <summary>
    /// Inhibits the effect of SetInitial until the next Move (i.e. Keep): Prepare will not insert initial blanks,
    /// but further ones will. This allows to cancel initial blanks on an internal split.
    /// </summary>
    public void FreezeInitial()
    {
        myFriz = myInit + 1;
        myInit = 0;
    }

    /// <summary>Appends the content of the line buffer to a string, then clears the buffer (keeping the kept part).</summary>
    public void Move(ref string theStr)
    {
        Prepare();
        theStr += Content();
        Keep();
    }

    /// <summary>Same as above, with the string known through a handle.</summary>
    public void Move(TCollection_HAsciiString theStr)
    {
        Prepare();
        theStr.AssignCat(Content());
        Keep();
    }

    /// <summary>Same as above, but generates the handled string.</summary>
    public TCollection_HAsciiString Moved()
    {
        Prepare();
        TCollection_HAsciiString R = new TCollection_HAsciiString(Content());
        Keep();
        return R;
    }

    /// <summary>Adds a text (OCCT's C string: up to its first null character).</summary>
    public void Add(string theText)
    {
        int aLength = theText.IndexOf('\0');
        Add(theText, aLength < 0 ? theText.Length : aLength);
    }

    /// <summary>Adds the first <paramref name="lntext"/> characters of a text (cut at the maximum size).</summary>
    public void Add(string text, int lntext)
    {
        int lnt = (lntext > (myMax - myLen - myInit) ? (myMax - myLen - myInit) : lntext);
        for (int i = 1; i <= lnt; ++i)
        {
            myLine.SetValue(myLen + i, text[i - 1]);
        }
        myLen += lnt;
        myLine.SetValue(myLen + 1, '\0');
    }

    /// <summary>Adds one character (not checked against the maximum size).</summary>
    public void Add(char theText)
    {
        myLine.SetValue(myLen + 1, theText);
        ++myLen;
        myLine.SetValue(myLen + 1, '\0');
    }

    /// <summary>
    /// Prepares Move: inserts the initial blanks if required, and determines if SetKeep can be supported (it cannot be
    /// if the length plus the next string to get, see CanGet, overpasses the maximum size).
    /// </summary>
    private void Prepare()
    {
        //  ATTENTION aux blanx initiaux
        if (myInit > 0)
        {
            if ((myLen + myInit) > myMax)
            {
                return;
            }

            for (int i = myLen + 1; i > 0; --i)
            {
                myLine.SetValue(i + myInit, myLine.Value(i));
            }
            for (int i = 1; i <= myInit; ++i)
            {
                myLine.SetValue(i, ' ');
            }
        }

        //  MANAGE KEEP: is it playable? otherwise, cancel. if yes, note the junction
        if (myKeep > 0)
        {
            myKeep += (myInit + 1); // myInit, and +1 because Keep INCLUDED
        }
        if (myKeep > 0)
        {
            if ((myLen + myGet + myInit - myKeep) >= myMax)
            {
                myKeep = 0;
            }
        }
        if (myKeep > 0)
        {
            myKept = myLine.Value(myKeep);
            myLine.SetValue(myKeep, '\0');
        }
    }

    /// <summary>Keeps the characters from SetKeep; if SetKeep is zero, equivalent to Clear.</summary>
    private void Keep()
    {
        //  If Keep, save from myKeep + 1 to myLen (+1 for final 0)
        if (myKeep > 0)
        {
            myLine.SetValue(1, myKept);
            for (int i = myKeep + 1; i <= myLen + myInit + 1; ++i)
            {
                myLine.SetValue(i - myKeep + 1, myLine.Value(i));
            }
            myLen = myLen + myInit - myKeep + 1;
        }
        else
        {
            Clear();
        }
        myGet = myKeep = 0;
        if (myFriz > 0)
        {
            myInit = myFriz - 1;
            myFriz = 0;
        }
    }

    private readonly NCollection_Array1<char> myLine;
    private int myMax;
    private int myInit;
    private int myKeep;
    private int myGet;
    private int myLen;
    private int myFriz;
    private char myKept;
}
