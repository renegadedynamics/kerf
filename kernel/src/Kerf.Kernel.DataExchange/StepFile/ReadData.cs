// Port of: src/DataExchange/TKDESTEP/StepFile/StepFile_ReadData.hxx
// Port of: src/DataExchange/TKDESTEP/StepFile/StepFile_ReadData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Occt;
using System.Globalization;
using static Kerf.Engine.Exchange.Framework.ParamType;

namespace Kerf.Engine.Exchange.Step.Parser;

/// <summary>
/// Provides data structures and tools to collect and store the data read from the STEP file. This class is designed
/// to work in collaboration with the STEP parser (<see cref="Scanner"/> and <see cref="StepParser"/>,
/// which replace OCCT's flex and bison code and drive this class exactly as it does).
/// </summary>
/// <remarks>
/// All text received in the parsing process is kept, and the last received text value is held as the current text.
/// This value is used to initialize a new record (representation of the STEP entity) and its arguments (parameters
/// of the STEP entity). Arguments are chained in their record, records in the list of records.
/// <para>
/// Texts are the file's bytes, one <see cref="char"/> per byte (Latin-1): nothing is decoded here. OCCT keeps the
/// file's chars; StepData_StepReaderData decodes <c>\X2\</c>, <c>\S\</c> and the code pages later. A text ends at
/// its first NUL byte, as OCCT's C strings do.
/// </para>
/// <para>
/// Reading of several STEP files simultaneously is possible (e.g. in multiple threads) provided that each file is
/// read using its own instances of the scanner, the parser and this class.
/// </para>
/// </remarks>
internal sealed class ReadData
{
    // Constant litterales
    private static class TextValue
    {
        internal const string SubList = "/* (SUB) */";
        internal const string Scope = "SCOPE";
        internal const string Nil = " ";
        internal const string Sub1 = "$1"; /* optimisation ... */
        internal const string Sub2 = "$2";
        internal const string ArgType1 = "(IF#TnEHBx"; /* types arguments (2 1es lettres) */
        internal const string ArgType2 = ")nlIxdnxix";
        internal const string IdZero = "#0";
    }

    /// <summary>List of arguments, contains all argument descriptions.</summary>
    private sealed class Argument
    {
        internal Argument? _next; //!< Next argument in the list for this record
        internal string? _value; //!< Character value of the argument
        internal ParamType _type = ParamType.ParamSub; //!< Type of the argument
    }

    /// <summary>List of records, contains all text processed by the parser.</summary>
    private sealed class Record
    {
        internal Record? _next; //!< Next record in the list
        internal Argument? _first; //!< First argument in the record
        internal Argument? _last; //!< Last argument in the record
        internal string? _ident; //!< Record identifier (Example: "#12345") or scope-end
        internal string? _type; //!< Type of the record
    }

    /// <summary>List of scopes pages, contains all records for external processing.</summary>
    private sealed class Scope
    {
        internal Scope? _previous; //!< Previous scope, to which it will be necessary to return
        internal Record? _record; //!< Record interrupted by the scope (to resume)
    }

    /// <summary>List of errors messages, contains all errors.</summary>
    private sealed class ErrorsPage
    {
        private ErrorsPage? _next; //!< Chaining of records pages
        private readonly string _error; //!< Own error message

        internal ErrorsPage(string error) => _error = error;

        //! Returns point to the next ErrorsPage
        internal ErrorsPage? NextErrorPage() => _next;

        //! Sets the next ErrorPage
        internal void SetNextErrorPage(ErrorsPage nextErrorsPage) => _next = nextErrorsPage;

        //! Returns an error message
        internal string ErrorMessage() => _error;
    }

    private int _modePrint; //!< Control print output (for call from yacc)
    private int _nbRec; //!< Total number of data records read
    private int _nbHead; //!< Number of records taken by the Header
    private int _nbPar; //!< Total number of parameters read
    private int _yaRec; //!< Presence record already created (after 1 Ident)
    private int _numSub; //!< Number of current sublist
    private bool _errorArg; //!< Control of error argument (true - error argument was created)
    private string? _resText; //!< Text value written by the scanner and passed to the parser to create record
    private string? _currType = TextValue.SubList; //!< Type of last record read
    private string? _subArg; //!< Ident last record (possible sub-list)
    private ParamType _typeArg = ParamType.ParamSub; //!< Type of last argument read
    private Argument? _currArg; //!< Current node of the arguments list
    private Record? _firstRec; //!< First node of the records list
    private Record? _curRec; //!< Current node of the records list
    private Record? _lastRec; //!< Last node of the records list
    private Scope? _curScope; //!< Current node of the scopes list
    private ErrorsPage? _firstError; //!< First node of the errors pages list
    private ErrorsPage? _curError; //!< Current node of the errors pages list

    /// <summary>Constructs an uninitialized tool.</summary>
    public ReadData() { }

    /// <summary>
    /// Prepares the text value for analysis. It is the main tool for transferring data from the scanner to the
    /// parser.
    /// </summary>
    internal void CreateNewText(string newText, int lenText)
    {
        // OCCT: the text is a C string, so it ends at its first NUL (a NUL byte in a quoted string cuts it there)
        int nul = newText.IndexOf('\0');
        if (nul >= 0)
        {
            newText = newText[..nul];
        }

        // Optimization for most frequent entity, CARTESIAN_POINT
        // OCCT: it comes before the error argument's concatenation, which it skips
        const string textOfCarPnt = "CARTESIAN_POINT";
        if (newText == textOfCarPnt)
        {
            _resText = textOfCarPnt;
            return;
        }

        // If error argument exists - append new text to old result text
        // Else create new result text
        if (_errorArg)
        {
            _resText = string.Concat(_resText, newText);
            return;
        }

        _resText = newText;
    }

    /// <summary>Adds the current record to the list.</summary>
    internal void RecordNewEntity()
    {
        _errorArg = false; // Reset error argument mod
        AddNewRecord(_curRec!);
        SetTypeArg(ParamType.ParamSub);
        _subArg = _curRec!._ident;
        _curRec = _curRec._next;
        _lastRec!._next = null;
    }

    /// <summary>Creates a new record and sets Ident from the current text.</summary>
    internal void RecordIdent()
    {
        _curRec = CreateNewRecord();
        GetResultText(out _curRec._ident);
        _curRec._next = null;
        _curRec._first = null;
        _curRec._last = null;
        _yaRec = 1;
    }

    /// <summary>Starts reading of the type (entity).</summary>
    internal void RecordType()
    {
        if (_yaRec == 0)
        {
            _curRec = CreateNewRecord();
            _curRec._ident = TextValue.IdZero;
            _curRec._next = null;
            _curRec._first = null;
            _curRec._last = null;
        }

        GetResultText(out _curRec!._type);
        _yaRec = _numSub = 0;
    }

    /// <summary>Prepares and saves a record or sub-record.</summary>
    internal void RecordListStart()
    {
        if (_numSub > 0)
        {
            Record subRec;
            subRec = CreateNewRecord();
            switch (_numSub)
            {
                case 1:
                    subRec._ident = TextValue.Sub1;
                    break;
                case 2:
                    subRec._ident = TextValue.Sub2;
                    break;
                default:
                {
                    string bufSub;
                    if (_numSub > 9)
                    {
                        bufSub = "$" + _numSub.ToString(CultureInfo.InvariantCulture);
                    }
                    else
                    {
                        bufSub = "$" + (char)(_numSub + 48);
                    }

                    subRec._ident = RecordNewText(bufSub);
                    break;
                }
            }

            subRec._type = _currType;
            _currType = TextValue.SubList;
            subRec._next = _curRec;
            subRec._first = null;
            subRec._last = null;
            _curRec = subRec;
        }

        _errorArg = false; // Reset error arguments mode
        _numSub++;
    }

    /// <summary>Prepares new arguments. Type and value already known.</summary>
    internal void CreateNewArg()
    {
        Argument newArg = new();
        _nbPar++;
        newArg._type = _typeArg;
        if (_typeArg == ParamType.ParamSub)
        {
            newArg._value = _subArg;
        }
        else
        {
            GetResultText(out newArg._value);
        }

        if (_typeArg == ParamType.ParamMisc)
        {
            _errorArg = true;
        }

        if (_curRec!._first == null)
        {
            _curRec._first = newArg;
            _curRec._last = newArg;
        }
        else if (_curRec._last == null)
        {
            _curRec._first._next = newArg;
            _curRec._last = newArg;
        }
        else
        {
            Argument nextArg = _curRec._last;
            nextArg._next = newArg;
            _curRec._last = newArg;
        }

        newArg._next = null;
    }

    /// <summary>
    /// Prepares error arguments, controls count of error arguments. If the parser handles a sequence of error
    /// types, creates only one argument and updates text value.
    /// </summary>
    internal void CreateErrorArg()
    {
        // If already exists - update text value
        if (!_errorArg)
        {
            SetTypeArg(ParamType.ParamMisc);
            CreateNewArg();
            _errorArg = true;
            return;
        }

        Argument currArg = _curRec!._last!;
        GetResultText(out currArg._value);
    }

    /// <summary>Creates a new scope, containing the current record.</summary>
    internal void AddNewScope()
    {
        Scope newScope;
        Record aRecord;
        newScope = new Scope();
        newScope._record = _curRec;
        newScope._previous = _curScope;
        _curScope = newScope;
        aRecord = CreateNewRecord();
        aRecord._ident = TextValue.Scope;
        aRecord._type = TextValue.Nil;
        aRecord._first = null;
        aRecord._last = null;
        AddNewRecord(aRecord);
    }

    /// <summary>Ends the scope.</summary>
    internal void FinalOfScope()
    {
        Scope oldScope;
        Record aRecord;
        if (_curScope == null)
        {
            return;
        }

        aRecord = CreateNewRecord();
        aRecord._ident = TextValue.Scope;
        aRecord._type = TextValue.Nil;
        aRecord._first = null;
        aRecord._last = null;

        // OCCT: mySubArg is null until a first list is closed (a file with an empty header whose first entity is a
        // scope): OCCT reads through the null pointer; the port takes it for no export list
        if (_subArg is { Length: > 0 } && _subArg[0] == '$')
        {
            if (_modePrint > 0)
            {
                Printf(string.Create(CultureInfo.InvariantCulture, $"Export List : (List in Record n0 {_nbRec}) -- "));
                PrintRecord(_lastRec);
            }

            _curRec = aRecord;
            _typeArg = ParamType.ParamSub;
            CreateNewArg();
        }

        AddNewRecord(aRecord);

        _curRec = _curScope._record;
        _yaRec = 1;
        oldScope = _curScope;
        _curScope = oldScope._previous;
    }

    /// <summary>Releases memory.</summary>
    /// <param name="mode">1 - clear the records and arguments; 2 - clear the texts; 3 - clear all data.</param>
    public void ClearRecorder(int mode)
    {
        if ((mode & 1) != 0)
        {
            _currType = null;
            _subArg = null;
            _currArg = null;
            _firstRec = null;
            _curRec = null;
            _lastRec = null;
            _curScope = null;
        }

        if ((mode & 2) != 0)
        {
            _resText = null;
        }
    }

    /// <summary>Returns a value of fields of current argument.</summary>
    public bool GetArgDescription(out ParamType type, out string? value)
    {
        if (_currArg == null)
        {
            type = default;
            value = null;
            return false;
        }

        type = _currArg._type;
        value = _currArg._value;
        _currArg = _currArg._next;
        return true;
    }

    /// <summary>Returns a value of all file counters (and starts the walk over the records).</summary>
    public void GetFileNbR(out int nbHead, out int nbRec, out int nbPage)
    {
        _curRec = _firstRec;
        nbHead = _nbHead;
        nbRec = _nbRec;
        nbPage = _nbPar;
    }

    /// <summary>Returns a value of fields of current record (theNbArg is 1 if it has arguments, else 0).</summary>
    public bool GetRecordDescription(out string? ident, out string? type, out int nbArg)
    {
        if (_curRec == null)
        {
            ident = null;
            type = null;
            nbArg = 0;
            return false;
        }

        ident = _curRec._ident;
        type = _curRec._type;
        nbArg = _curRec._first != null ? 1 : 0;
        _currArg = _curRec._first;
        return true;
    }

    /// <summary>Initializes the record type with the current text.</summary>
    internal void RecordTypeText() => GetResultText(out _currType);

    /// <summary>Skips to next record.</summary>
    public void NextRecord() => _curRec = _curRec?._next;

    /// <summary>Prints data of current record according to the modeprint.</summary>
    public void PrintCurrentRecord() => PrintRecord(_curRec);

    /// <summary>Controls the correct argument count for the record. Resets error argument mode.</summary>
    internal void PrepareNewArg() => _errorArg = false;

    /// <summary>Prepares the end of the head section.</summary>
    internal void FinalOfHead() => _nbHead = _nbRec;

    /// <summary>Sets type of the current argument.</summary>
    internal void SetTypeArg(ParamType argType) => _typeArg = argType;

    /// <summary>
    /// Initializes the print mode: 0 - don't print descriptions; 1 - print only descriptions of record; 2 - print
    /// descriptions of records and its arguments.
    /// </summary>
    public void SetModePrint(int mode) => _modePrint = mode;

    /// <summary>Returns mode print.</summary>
    public int GetModePrint() => _modePrint;

    /// <summary>Returns number of records.</summary>
    public int GetNbRecord() => _nbRec;

    /// <summary>Adds an error message.</summary>
    public void AddError(string errorMessage)
    {
        if (_firstError == null)
        {
            _firstError = new ErrorsPage(errorMessage);
            _curError = _firstError;
        }
        else
        {
            _curError!.SetNextErrorPage(new ErrorsPage(errorMessage));
            _curError = _curError.NextErrorPage();
        }
    }

    /// <summary>
    /// Transfers error messages to a check: each one as a fail, with the original "Undefined Parsing". Returns
    /// True if there was no error.
    /// </summary>
    public bool ErrorHandle(FrameworkCheck check)
    {
        if (_firstError != null)
        {
            ErrorsPage? current = _firstError;
            while (current != null)
            {
                check.AddFail(current.ErrorMessage(), "Undefined Parsing");
                current = current.NextErrorPage();
            }
        }
        return _firstError == null;
    }

    /// <summary>The error messages, in order (what ErrorHandle hands to its Interface_Check).</summary>
    internal IEnumerable<string> InternalErrors()
    {
        for (ErrorsPage? current = _firstError; current != null; current = current.NextErrorPage())
        {
            yield return current.ErrorMessage();
        }
    }

    /// <summary>Returns the message of the last error.</summary>
    public string? GetLastError() => _curError?.ErrorMessage();

    /// <summary>The text written to the console in the print modes (OCCT's Printf: the standard output).</summary>
    internal TextWriter PrintOutput { get; set; } = Console.Out;

    //! Prepare text to analyze
    private string RecordNewText(string text)
    {
        string? savResText;
        string? newText;
        savResText = _resText;
        CreateNewText(text, text.Length);
        newText = _resText;
        _resText = savResText;
        return newText!;
    }

    //! Get current text value
    private void GetResultText(out string? text) => text = _resText;

    //! Add a record to the current records page
    private void AddNewRecord(Record newRecord)
    {
        _nbRec++;
        if (_firstRec == null)
        {
            _firstRec = newRecord;
        }

        if (_lastRec != null)
        {
            _lastRec._next = newRecord;
        }

        _lastRec = newRecord;
    }

    //! Create new empty record
    // OCCT allocates it uninitialized: a record whose type is never set (an identifier the parser never gives a
    // type, reached only through syntax errors) has a garbage type there and a null one here
    private static Record CreateNewRecord() => new();

    private void Printf(string text) => PrintOutput.Write(text);

    //! Print data of the record according to the modeprint
    private void PrintRecord(Record? theRecord)
    {
        int numArg = 0;
        int numLen = 0;
        int argLen = 0;
        if (theRecord == null)
        {
            Printf("Not defined\n");
            return;
        }

        Printf($"Ident : {theRecord._ident}  Type : {theRecord._type}  Nb.Arg.s : {(theRecord._first != null ? theRecord._first._value : "")}\n");
        if (_modePrint < 2)
        {
            return;
        }

        _currArg = theRecord._first;
        while (_currArg != null)
        {
            numArg++;
            argLen = (_currArg._value?.Length ?? 0) + 18;
            numLen += argLen;
            if (numLen > 132)
            {
                Printf("\n");
                numLen = argLen;
            }

            Printf(string.Create(CultureInfo.InvariantCulture,
                $"  - Arg.{numArg}[{TextValue.ArgType1[(int)_currArg._type]}{TextValue.ArgType2[(int)_currArg._type]}] : {_currArg._value}"));
            _currArg = _currArg._next;
        }

        if (argLen > 0)
        {
            Printf("\n");
        }
    }
}
