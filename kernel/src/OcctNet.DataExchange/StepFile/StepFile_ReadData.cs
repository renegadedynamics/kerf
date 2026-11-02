// Port of: src/DataExchange/TKDESTEP/StepFile/StepFile_ReadData.hxx
// Port of: src/DataExchange/TKDESTEP/StepFile/StepFile_ReadData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// Provides data structures and tools to collect and store the data read from the STEP file. This class is designed
/// to work in collaboration with the STEP parser (<see cref="StepFile_Scanner"/> and <see cref="StepFile_Parser"/>,
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
public sealed class StepFile_ReadData
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
        internal Argument? myNext; //!< Next argument in the list for this record
        internal string? myValue; //!< Character value of the argument
        internal Interface_ParamType myType = Interface_ParamSub; //!< Type of the argument
    }

    /// <summary>List of records, contains all text processed by the parser.</summary>
    private sealed class Record
    {
        internal Record? myNext; //!< Next record in the list
        internal Argument? myFirst; //!< First argument in the record
        internal Argument? myLast; //!< Last argument in the record
        internal string? myIdent; //!< Record identifier (Example: "#12345") or scope-end
        internal string? myType; //!< Type of the record
    }

    /// <summary>List of scopes pages, contains all records for external processing.</summary>
    private sealed class Scope
    {
        internal Scope? myPrevious; //!< Previous scope, to which it will be necessary to return
        internal Record? myRecord; //!< Record interrupted by the scope (to resume)
    }

    /// <summary>List of errors messages, contains all errors.</summary>
    private sealed class ErrorsPage
    {
        private ErrorsPage? myNext; //!< Chaining of records pages
        private readonly string myError; //!< Own error message

        internal ErrorsPage(string theError) => myError = theError;

        //! Returns point to the next ErrorsPage
        internal ErrorsPage? NextErrorPage() => myNext;

        //! Sets the next ErrorPage
        internal void SetNextErrorPage(ErrorsPage theNextErrorsPage) => myNext = theNextErrorsPage;

        //! Returns an error message
        internal string ErrorMessage() => myError;
    }

    private int myModePrint; //!< Control print output (for call from yacc)
    private int myNbRec; //!< Total number of data records read
    private int myNbHead; //!< Number of records taken by the Header
    private int myNbPar; //!< Total number of parameters read
    private int myYaRec; //!< Presence record already created (after 1 Ident)
    private int myNumSub; //!< Number of current sublist
    private bool myErrorArg; //!< Control of error argument (true - error argument was created)
    private string? myResText; //!< Text value written by the scanner and passed to the parser to create record
    private string? myCurrType = TextValue.SubList; //!< Type of last record read
    private string? mySubArg; //!< Ident last record (possible sub-list)
    private Interface_ParamType myTypeArg = Interface_ParamSub; //!< Type of last argument read
    private Argument? myCurrArg; //!< Current node of the arguments list
    private Record? myFirstRec; //!< First node of the records list
    private Record? myCurRec; //!< Current node of the records list
    private Record? myLastRec; //!< Last node of the records list
    private Scope? myCurScope; //!< Current node of the scopes list
    private ErrorsPage? myFirstError; //!< First node of the errors pages list
    private ErrorsPage? myCurError; //!< Current node of the errors pages list

    /// <summary>Constructs an uninitialized tool.</summary>
    public StepFile_ReadData() { }

    /// <summary>
    /// Prepares the text value for analysis. It is the main tool for transferring data from the scanner to the
    /// parser.
    /// </summary>
    internal void CreateNewText(string theNewText, int theLenText)
    {
        // OCCT: the text is a C string, so it ends at its first NUL (a NUL byte in a quoted string cuts it there)
        int aNul = theNewText.IndexOf('\0');
        if (aNul >= 0)
        {
            theNewText = theNewText[..aNul];
        }

        // Optimization for most frequent entity, CARTESIAN_POINT
        // OCCT: it comes before the error argument's concatenation, which it skips
        const string aTextOfCarPnt = "CARTESIAN_POINT";
        if (theNewText == aTextOfCarPnt)
        {
            myResText = aTextOfCarPnt;
            return;
        }

        // If error argument exists - append new text to old result text
        // Else create new result text
        if (myErrorArg)
        {
            myResText = string.Concat(myResText, theNewText);
            return;
        }

        myResText = theNewText;
    }

    /// <summary>Adds the current record to the list.</summary>
    internal void RecordNewEntity()
    {
        myErrorArg = false; // Reset error argument mod
        AddNewRecord(myCurRec!);
        SetTypeArg(Interface_ParamSub);
        mySubArg = myCurRec!.myIdent;
        myCurRec = myCurRec.myNext;
        myLastRec!.myNext = null;
    }

    /// <summary>Creates a new record and sets Ident from the current text.</summary>
    internal void RecordIdent()
    {
        myCurRec = CreateNewRecord();
        GetResultText(out myCurRec.myIdent);
        myCurRec.myNext = null;
        myCurRec.myFirst = null;
        myCurRec.myLast = null;
        myYaRec = 1;
    }

    /// <summary>Starts reading of the type (entity).</summary>
    internal void RecordType()
    {
        if (myYaRec == 0)
        {
            myCurRec = CreateNewRecord();
            myCurRec.myIdent = TextValue.IdZero;
            myCurRec.myNext = null;
            myCurRec.myFirst = null;
            myCurRec.myLast = null;
        }

        GetResultText(out myCurRec!.myType);
        myYaRec = myNumSub = 0;
    }

    /// <summary>Prepares and saves a record or sub-record.</summary>
    internal void RecordListStart()
    {
        if (myNumSub > 0)
        {
            Record aSubRec;
            aSubRec = CreateNewRecord();
            switch (myNumSub)
            {
                case 1:
                    aSubRec.myIdent = TextValue.Sub1;
                    break;
                case 2:
                    aSubRec.myIdent = TextValue.Sub2;
                    break;
                default:
                {
                    string aBufSub;
                    if (myNumSub > 9)
                    {
                        aBufSub = "$" + myNumSub.ToString(CultureInfo.InvariantCulture);
                    }
                    else
                    {
                        aBufSub = "$" + (char)(myNumSub + 48);
                    }

                    aSubRec.myIdent = RecordNewText(aBufSub);
                    break;
                }
            }

            aSubRec.myType = myCurrType;
            myCurrType = TextValue.SubList;
            aSubRec.myNext = myCurRec;
            aSubRec.myFirst = null;
            aSubRec.myLast = null;
            myCurRec = aSubRec;
        }

        myErrorArg = false; // Reset error arguments mode
        myNumSub++;
    }

    /// <summary>Prepares new arguments. Type and value already known.</summary>
    internal void CreateNewArg()
    {
        Argument aNewArg = new();
        myNbPar++;
        aNewArg.myType = myTypeArg;
        if (myTypeArg == Interface_ParamSub)
        {
            aNewArg.myValue = mySubArg;
        }
        else
        {
            GetResultText(out aNewArg.myValue);
        }

        if (myTypeArg == Interface_ParamMisc)
        {
            myErrorArg = true;
        }

        if (myCurRec!.myFirst == null)
        {
            myCurRec.myFirst = aNewArg;
            myCurRec.myLast = aNewArg;
        }
        else if (myCurRec.myLast == null)
        {
            myCurRec.myFirst.myNext = aNewArg;
            myCurRec.myLast = aNewArg;
        }
        else
        {
            Argument aNextArg = myCurRec.myLast;
            aNextArg.myNext = aNewArg;
            myCurRec.myLast = aNewArg;
        }

        aNewArg.myNext = null;
    }

    /// <summary>
    /// Prepares error arguments, controls count of error arguments. If the parser handles a sequence of error
    /// types, creates only one argument and updates text value.
    /// </summary>
    internal void CreateErrorArg()
    {
        // If already exists - update text value
        if (!myErrorArg)
        {
            SetTypeArg(Interface_ParamMisc);
            CreateNewArg();
            myErrorArg = true;
            return;
        }

        Argument aCurrArg = myCurRec!.myLast!;
        GetResultText(out aCurrArg.myValue);
    }

    /// <summary>Creates a new scope, containing the current record.</summary>
    internal void AddNewScope()
    {
        Scope aNewScope;
        Record aRecord;
        aNewScope = new Scope();
        aNewScope.myRecord = myCurRec;
        aNewScope.myPrevious = myCurScope;
        myCurScope = aNewScope;
        aRecord = CreateNewRecord();
        aRecord.myIdent = TextValue.Scope;
        aRecord.myType = TextValue.Nil;
        aRecord.myFirst = null;
        aRecord.myLast = null;
        AddNewRecord(aRecord);
    }

    /// <summary>Ends the scope.</summary>
    internal void FinalOfScope()
    {
        Scope anOldScope;
        Record aRecord;
        if (myCurScope == null)
        {
            return;
        }

        aRecord = CreateNewRecord();
        aRecord.myIdent = TextValue.Scope;
        aRecord.myType = TextValue.Nil;
        aRecord.myFirst = null;
        aRecord.myLast = null;

        // OCCT: mySubArg is null until a first list is closed (a file with an empty header whose first entity is a
        // scope): OCCT reads through the null pointer; the port takes it for no export list
        if (mySubArg is { Length: > 0 } && mySubArg[0] == '$')
        {
            if (myModePrint > 0)
            {
                Printf(string.Create(CultureInfo.InvariantCulture, $"Export List : (List in Record n0 {myNbRec}) -- "));
                PrintRecord(myLastRec);
            }

            myCurRec = aRecord;
            myTypeArg = Interface_ParamSub;
            CreateNewArg();
        }

        AddNewRecord(aRecord);

        myCurRec = myCurScope.myRecord;
        myYaRec = 1;
        anOldScope = myCurScope;
        myCurScope = anOldScope.myPrevious;
    }

    /// <summary>Releases memory.</summary>
    /// <param name="theMode">1 - clear the records and arguments; 2 - clear the texts; 3 - clear all data.</param>
    public void ClearRecorder(int theMode)
    {
        if ((theMode & 1) != 0)
        {
            myCurrType = null;
            mySubArg = null;
            myCurrArg = null;
            myFirstRec = null;
            myCurRec = null;
            myLastRec = null;
            myCurScope = null;
        }

        if ((theMode & 2) != 0)
        {
            myResText = null;
        }
    }

    /// <summary>Returns a value of fields of current argument.</summary>
    public bool GetArgDescription(out Interface_ParamType theType, out string? theValue)
    {
        if (myCurrArg == null)
        {
            theType = default;
            theValue = null;
            return false;
        }

        theType = myCurrArg.myType;
        theValue = myCurrArg.myValue;
        myCurrArg = myCurrArg.myNext;
        return true;
    }

    /// <summary>Returns a value of all file counters (and starts the walk over the records).</summary>
    public void GetFileNbR(out int theNbHead, out int theNbRec, out int theNbPage)
    {
        myCurRec = myFirstRec;
        theNbHead = myNbHead;
        theNbRec = myNbRec;
        theNbPage = myNbPar;
    }

    /// <summary>Returns a value of fields of current record (theNbArg is 1 if it has arguments, else 0).</summary>
    public bool GetRecordDescription(out string? theIdent, out string? theType, out int theNbArg)
    {
        if (myCurRec == null)
        {
            theIdent = null;
            theType = null;
            theNbArg = 0;
            return false;
        }

        theIdent = myCurRec.myIdent;
        theType = myCurRec.myType;
        theNbArg = myCurRec.myFirst != null ? 1 : 0;
        myCurrArg = myCurRec.myFirst;
        return true;
    }

    /// <summary>Initializes the record type with the current text.</summary>
    internal void RecordTypeText() => GetResultText(out myCurrType);

    /// <summary>Skips to next record.</summary>
    public void NextRecord() => myCurRec = myCurRec?.myNext;

    /// <summary>Prints data of current record according to the modeprint.</summary>
    public void PrintCurrentRecord() => PrintRecord(myCurRec);

    /// <summary>Controls the correct argument count for the record. Resets error argument mode.</summary>
    internal void PrepareNewArg() => myErrorArg = false;

    /// <summary>Prepares the end of the head section.</summary>
    internal void FinalOfHead() => myNbHead = myNbRec;

    /// <summary>Sets type of the current argument.</summary>
    internal void SetTypeArg(Interface_ParamType theArgType) => myTypeArg = theArgType;

    /// <summary>
    /// Initializes the print mode: 0 - don't print descriptions; 1 - print only descriptions of record; 2 - print
    /// descriptions of records and its arguments.
    /// </summary>
    public void SetModePrint(int theMode) => myModePrint = theMode;

    /// <summary>Returns mode print.</summary>
    public int GetModePrint() => myModePrint;

    /// <summary>Returns number of records.</summary>
    public int GetNbRecord() => myNbRec;

    /// <summary>Adds an error message.</summary>
    public void AddError(string theErrorMessage)
    {
        if (myFirstError == null)
        {
            myFirstError = new ErrorsPage(theErrorMessage);
            myCurError = myFirstError;
        }
        else
        {
            myCurError!.SetNextErrorPage(new ErrorsPage(theErrorMessage));
            myCurError = myCurError.NextErrorPage();
        }
    }

    // ErrorHandle (transfers error messages to an Interface_Check, each as a fail with the original
    // "Undefined Parsing") comes with Interface_Check, in StepData's unit (8B): it reads InternalErrors.

    /// <summary>The error messages, in order (what ErrorHandle hands to its Interface_Check).</summary>
    internal IEnumerable<string> InternalErrors()
    {
        for (ErrorsPage? aCurrent = myFirstError; aCurrent != null; aCurrent = aCurrent.NextErrorPage())
        {
            yield return aCurrent.ErrorMessage();
        }
    }

    /// <summary>Returns the message of the last error.</summary>
    public string? GetLastError() => myCurError?.ErrorMessage();

    /// <summary>The text written to the console in the print modes (OCCT's Printf: the standard output).</summary>
    internal TextWriter PrintOutput { get; set; } = Console.Out;

    //! Prepare text to analyze
    private string RecordNewText(string theText)
    {
        string? aSavResText;
        string? aNewText;
        aSavResText = myResText;
        CreateNewText(theText, theText.Length);
        aNewText = myResText;
        myResText = aSavResText;
        return aNewText!;
    }

    //! Get current text value
    private void GetResultText(out string? theText) => theText = myResText;

    //! Add a record to the current records page
    private void AddNewRecord(Record theNewRecord)
    {
        myNbRec++;
        if (myFirstRec == null)
        {
            myFirstRec = theNewRecord;
        }

        if (myLastRec != null)
        {
            myLastRec.myNext = theNewRecord;
        }

        myLastRec = theNewRecord;
    }

    //! Create new empty record
    // OCCT allocates it uninitialized: a record whose type is never set (an identifier the parser never gives a
    // type, reached only through syntax errors) has a garbage type there and a null one here
    private static Record CreateNewRecord() => new();

    private void Printf(string theText) => PrintOutput.Write(theText);

    //! Print data of the record according to the modeprint
    private void PrintRecord(Record? theRecord)
    {
        int aNumArg = 0;
        int aNumLen = 0;
        int anArgLen = 0;
        if (theRecord == null)
        {
            Printf("Not defined\n");
            return;
        }

        Printf($"Ident : {theRecord.myIdent}  Type : {theRecord.myType}  Nb.Arg.s : {(theRecord.myFirst != null ? theRecord.myFirst.myValue : "")}\n");
        if (myModePrint < 2)
        {
            return;
        }

        myCurrArg = theRecord.myFirst;
        while (myCurrArg != null)
        {
            aNumArg++;
            anArgLen = (myCurrArg.myValue?.Length ?? 0) + 18;
            aNumLen += anArgLen;
            if (aNumLen > 132)
            {
                Printf("\n");
                aNumLen = anArgLen;
            }

            Printf(string.Create(CultureInfo.InvariantCulture,
                $"  - Arg.{aNumArg}[{TextValue.ArgType1[(int)myCurrArg.myType]}{TextValue.ArgType2[(int)myCurrArg.myType]}] : {myCurrArg.myValue}"));
            myCurrArg = myCurrArg.myNext;
        }

        if (anArgLen > 0)
        {
            Printf("\n");
        }
    }
}
