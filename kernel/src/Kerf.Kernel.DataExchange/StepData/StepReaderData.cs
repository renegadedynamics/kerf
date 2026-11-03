// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepReaderData.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepReaderData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Resources;
using Occt;
using System.Text;
using static Kerf.Engine.Exchange.Framework.ParamType;
using static Kerf.Engine.Foundation.Resources.FormatType;
using static Kerf.Engine.Exchange.Step.Data.Logical;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// Specific FileReaderData for Step. Contains literal description of entities (for each one: type as a string,
/// ident, parameter list), provides references evaluation, plus access to literal data and specific access methods
/// (Boolean, XY, XYZ).
/// </summary>
/// <remarks>
/// <para>
/// The parameters keep the file's text, one char per byte (decision 1174). The strings the <c>Read*</c> methods return
/// (<see cref="ReadString"/> and the texts of <see cref="ReadAny"/>, <see cref="ReadSub"/>, <see cref="ReadField"/>)
/// are decoded as OCCT's <c>cleanText</c> does (code page, <c>\X\</c>, <c>\X2\</c>, <c>\X4\</c>, <c>\S\</c>,
/// <c>\P?\</c>, <c>\N\</c>, <c>\T\</c>, doubled quotes and backslashes): OCCT holds the result in UTF-8, the port as
/// the .NET string of the same text (decision 1192).
/// </para>
/// <para>
/// The handles OCCT passes by non-const reference without reassigning them (the checks) are passed by value; the
/// outputs a read may leave unchanged (on an error) are <c>ref</c>, as OCCT's references (decision 1193).
/// </para>
/// </remarks>
public class StepReaderData : FileReaderData
{
    // The Header consists of entities analogous in principle to those
    // of the Data, except that they are without identifier, and can neither
    // reference, nor be referenced (whether with Header or with Data)
    // Thus, in StepReaderData, the Header consists of the first "thenbhead" Entities
    // This separation allows STEP files to have metadata and structural information
    // separate from the main geometric and semantic data

    private readonly NCollection_Array1<int> _theidents;
    private readonly NCollection_Array1<int> _thetypes;
    private readonly IndexedSet<string> _thenametypes = new();
    private readonly Dictionary<int, int> _themults = new();
    private int _thenbents;
    private int _thelastn;
    private readonly int _thenbhead;
    private int _thenbscop;
    private readonly FrameworkCheck _thecheck;
    private readonly FormatType _sourceCodePage;

    // ----------  Fonctions Utilitaires  ----------

    /// <summary>Converts a character to its hexadecimal value; 0 if it has no representation in this system.</summary>
    private static int ConvertCharacterTo16bit(char character)
    {
        switch (character)
        {
            case '0':
                return 0;
            case '1':
                return 1;
            case '2':
                return 2;
            case '3':
                return 3;
            case '4':
                return 4;
            case '5':
                return 5;
            case '6':
                return 6;
            case '7':
                return 7;
            case '8':
                return 8;
            case '9':
                return 9;
            case 'A':
            case 'a':
                return 10;
            case 'B':
            case 'b':
                return 11;
            case 'C':
            case 'c':
                return 12;
            case 'D':
            case 'd':
                return 13;
            case 'E':
            case 'e':
                return 14;
            case 'F':
            case 'f':
                return 15;
            default:
                return 0;
        }
    }

    // TCollection_ExtendedString's checked accessors, 1-based, on the text being cleaned
    private static char ExtValue(StringBuilder str, int theWhere)
    {
        if (theWhere > 0 && theWhere <= str.Length)
        {
            return str[theWhere - 1];
        }
        throw new OutOfRangeException("TCollection_ExtendedString::Value : parameter theWhere");
    }

    private static void ExtSetValue(StringBuilder str, int theWhere, char what)
    {
        if (theWhere > 0 && theWhere <= str.Length)
        {
            str[theWhere - 1] = what;
            return;
        }
        throw new OutOfRangeException("TCollection_ExtendedString::SetValue : parameter theWhere");
    }

    private static void ExtSetValue(StringBuilder str, int theWhere, string what)
    {
        if (theWhere < 1 || theWhere > str.Length + 1)
        {
            throw new OutOfRangeException("TCollection_ExtendedString::SetValue : parameter theWhere");
        }
        if (what.Length <= 0)
        {
            return;
        }
        int newEnd = theWhere - 1 + what.Length;
        if (newEnd > str.Length)
        {
            str.Length = newEnd;
        }
        for (int i = 0; i < what.Length; i++)
        {
            str[theWhere - 1 + i] = what[i];
        }
    }

    // aResString.ToExtString() + theFrom: the rest of the text from a 0-based index, as a C string (up to its first
    // zero char), copied as C++'s temporary TCollection_ExtendedString is
    private static string ExtTail(StringBuilder str, int theFrom)
    {
        int end = theFrom;
        while (end < str.Length && str[end] != '\0')
        {
            end++;
        }
        return theFrom >= end ? "" : str.ToString(theFrom, end - theFrom);
    }

    private static void ExtTrunc(StringBuilder str, int howMany)
    {
        if (howMany < 0 || howMany > str.Length)
        {
            throw new OutOfRangeException("TCollection_ExtendedString::Trunc : parameter 'theHowMany'");
        }
        str.Length = howMany;
    }

    // TCollection_ExtendedString::AssignCat(char16_t): a zero char is not added
    private static void ExtAssignCat(StringBuilder str, char theChar)
    {
        if (theChar != '\0')
        {
            str.Append(theChar);
        }
    }

    /// <summary>
    /// Prepares a string to use in OCCT exchange structure. If code page is Resource_FormatType_NoConversion, cleans
    /// only special characters without conversion; else converts the string using the code page and handles the
    /// control directives.
    /// </summary>
    private void CleanText(TCollection_HAsciiString val)
    {
        if (val.Length() == 2)
        {
            val.Clear();
            return;
        }
        StringBuilder resString = new StringBuilder();
        bool toConversion = _sourceCodePage != FormatType.FormatType_NoConversion;
        Unicode.ConvertFormatToUnicode(_sourceCodePage, val.ToCString().Substring(1), resString);
        int resStringSize = resString.Length - 1; // skip the last apostrophe
        StringBuilder tempExtString = new StringBuilder(); // string for characters within control directives
        int setCharInd = 1; // index to set value to result string
        FormatType localFormatType = FormatType.FormatType_iso8859_1; // a code page for a "\S\" control directive
        for (int stringInd = 1; stringInd <= resStringSize; ++stringInd)
        {
            char aChar = ExtValue(resString, stringInd);
            setCharInd = stringInd;
            if (aChar == '\\' && stringInd <= resStringSize - 3) // can contains the control directive
            {
                bool isConverted = false;
                char dirChar = ExtValue(resString, stringInd + 1);
                bool isSecSlash = ExtValue(resString, stringInd + 2) == '\\';
                bool isThirdSlash = ExtValue(resString, stringInd + 3) == '\\';
                // Encoding ISO 8859 characters within a string;
                // ("\P{N}\") control directive;
                // indicates code page for ("\S\") control directive;
                // {N}: "A", "B", "C", "D", "E", "F", "G", "H", "I";
                // "A" identifies ISO 8859-1; "B" identifies ISO 8859-2, etc.
                if (dirChar == 'P' && isThirdSlash)
                {
                    char pageId = (char)(ExtValue(resString, stringInd + 2) & 255);
                    if (pageId >= 'a' && pageId <= 'z')
                    {
                        pageId = (char)(pageId - 'a' + 'A'); // C's toupper (the "C" locale)
                    }
                    if (pageId >= 'A' && pageId <= 'I')
                    {
                        localFormatType = (FormatType)((int)FormatType.FormatType_iso8859_1 + (pageId - 'A'));
                    }
                    else
                    {
                        _thecheck.AddWarning("String control directive \\P*\\ with an unsupported symbol in place of *");
                    }
                    isConverted = true;
                    stringInd += 3;
                }
                // Encoding ISO 8859 characters within a string;
                // ("\S\") control directive;
                // converts followed a LATIN CODEPOINT character.
                else if (dirChar == 'S' && isSecSlash)
                {
                    char resChar = (char)((ExtValue(resString, stringInd + 3) | 0x80) & 0xFF);
                    string strForCovert = resChar.ToString();
                    Unicode.ConvertFormatToUnicode(localFormatType, strForCovert, tempExtString);
                    isConverted = true;
                    stringInd += 3;
                }
                // Encoding U+0000 to U+00FF in a string
                // ("\X\") control directive;
                // converts followed two hexadecimal character.
                else if (dirChar == 'X' && stringInd <= resStringSize - 4 && isSecSlash)
                {
                    int resChar = ConvertCharacterTo16bit(ExtValue(resString, stringInd + 3));
                    resChar = ((resChar << 4) | ConvertCharacterTo16bit(ExtValue(resString, stringInd + 4))) & 0xFF;
                    // pass through without conversion: TCollection_ExtendedString(aStrForConvert, false)
                    tempExtString.Clear();
                    if (resChar != 0)
                    {
                        tempExtString.Append((char)resChar);
                    }
                    isConverted = true;
                    stringInd += 4;
                }
                // Encoding ISO 10646 characters within a string
                // ("\X{N}\") control directive;
                // {N}: "0", "2", "4";
                // "\X2\" or "\X4\" converts followed a hexadecimal character sequence;
                // "\X0\" indicate the end of the "\X2\" or "\X4\".
                else if (dirChar == 'X' && isThirdSlash)
                {
                    int firstInd = stringInd + 3;
                    int lastInd = stringInd;
                    bool isClosed = false;
                    // find the end of the "\X2\" or "\X4\" by an external "aStringInd"
                    for (; stringInd <= resStringSize && !isClosed; ++stringInd)
                    {
                        if (resStringSize - stringInd > 2 && ExtValue(resString, stringInd) == '\\'
                            && ExtValue(resString, stringInd + 1) == 'X' && ExtValue(resString, stringInd + 2) == '0'
                            && ExtValue(resString, stringInd + 3) == '\\')
                        {
                            lastInd = stringInd - 1;
                            stringInd = stringInd + 2;
                            isClosed = true;
                        }
                    }
                    if (!isClosed) // "\X0\" not exists
                    {
                        lastInd = stringInd = resStringSize;
                    }
                    int strLen = lastInd - firstInd;
                    // "\X2\" control directive;
                    // followed by multiples of four or three hexadecimal characters.
                    // Encoding in UTF-16
                    if (ExtValue(resString, firstInd - 1) == '2' && resStringSize - firstInd > 3)
                    {
                        int iterStep = (strLen % 4 == 0) ? 4 : 3;
                        if (strLen % iterStep != 0)
                        {
                            ExtAssignCat(tempExtString, '?');
                            _thecheck.AddWarning("String control directive \\X2\\ is followed by number of digits not multiple of 4");
                        }
                        else
                        {
                            int utfCharacter = '\0';
                            for (int charInd = 1; charInd <= strLen; ++charInd)
                            {
                                utfCharacter |= ConvertCharacterTo16bit(ExtValue(resString, charInd + firstInd));
                                if (charInd % iterStep == 0)
                                {
                                    ExtAssignCat(tempExtString, (char)utfCharacter);
                                    utfCharacter = '\0';
                                }
                                utfCharacter = (utfCharacter << 4) & 0xFFFF; // a char16_t
                            }
                        }
                    }
                    // "\X4\" control directive;
                    // followed by multiples of eight hexadecimal characters.
                    // Encoding in UTF-32
                    else if (ExtValue(resString, firstInd - 1) == '4' && resStringSize - firstInd > 7)
                    {
                        if (strLen % 8 != 0)
                        {
                            ExtAssignCat(tempExtString, '?');
                            _thecheck.AddWarning("String control directive \\X4\\ is followed by number of digits not multiple of 8");
                        }
                        else
                        {
                            uint utfCharacter = '\0';
                            for (int charInd = 1; charInd <= strLen; ++charInd)
                            {
                                utfCharacter |= (uint)ConvertCharacterTo16bit(ExtValue(resString, charInd + firstInd));
                                if (charInd % 8 == 0)
                                {
                                    // NCollection_UtfIterator<char32_t> on the one character, written as UTF-16 then
                                    // appended as a C string (it ends at a zero character)
                                    var utfString = new StringBuilder();
                                    if (utfCharacter != 0)
                                    {
                                        Unicode.AppendUtf16(utfCharacter, utfString);
                                    }
                                    tempExtString.Append(utfString);
                                    utfCharacter = '\0';
                                }
                                utfCharacter = utfCharacter << 4;
                            }
                        }
                    }
                    isConverted = true;
                }
                if (isConverted) // find the control directive
                {
                    if (toConversion) // else skip moving
                    {
                        resStringSize -= stringInd - setCharInd - tempExtString.Length + 1; // change the string size to remove unused symbols
                        ExtSetValue(resString, setCharInd, tempExtString.ToString());
                        setCharInd += tempExtString.Length; // move to the new position
                        ExtSetValue(resString, setCharInd, ExtTail(resString, stringInd));
                        stringInd = setCharInd - 1;
                        ExtTrunc(resString, resStringSize);
                    }
                    tempExtString.Clear();
                    continue;
                }
            }
            if (stringInd <= resStringSize - 1)
            {
                char charNext = ExtValue(resString, stringInd + 1);
                if (charNext == aChar && (aChar == '\'' || aChar == '\\'))
                {
                    ExtSetValue(resString, setCharInd, ExtTail(resString, stringInd)); // move the string,removing one symbol
                    resStringSize--; // change the string size to remove unused symbol
                    ExtTrunc(resString, resStringSize);
                }
                else if (aChar == '\\')
                {
                    bool isDirective = stringInd <= resStringSize - 2 && ExtValue(resString, stringInd + 2) == '\\';
                    if (isDirective)
                    {
                        if (charNext == 'N')
                        {
                            ExtSetValue(resString, setCharInd++, '\n');
                            ExtSetValue(resString, setCharInd, ExtTail(resString, stringInd + 2)); // move the string,removing two symbols
                            resStringSize -= 2; // change the string size to remove unused symbols
                            ExtTrunc(resString, resStringSize);
                            continue;
                        }
                        else if (charNext == 'T')
                        {
                            ExtSetValue(resString, setCharInd++, '\t');
                            ExtSetValue(resString, setCharInd, ExtTail(resString, stringInd + 2)); // move the string,removing two symbols
                            resStringSize -= 2; // change the string size to remove unused symbols
                            ExtTrunc(resString, resStringSize);
                            continue;
                        }
                    }
                }
            }
            if (aChar == '\n' || aChar == '\r')
            {
                ExtSetValue(resString, setCharInd, ExtTail(resString, stringInd));
                resStringSize--;
                ExtTrunc(resString, resStringSize);
                stringInd--;
            }
        }
        val.Clear();
        ExtTrunc(resString, resStringSize); // trunc the last apostrophe
        val.AssignCat(Unicode.ToUtf8Equivalent(resString));
    }

    //  -------------  METHODES  -------------

    /// <summary>
    /// Creates StepReaderData correctly dimensioned (necessary at creation time, because it contains arrays):
    /// nbheader is nb of records for Header, nbtotal for Header+Data and nbpar gives the total count of parameters.
    /// </summary>
    public StepReaderData(int nbheader, int nbtotal, int nbpar, FormatType sourceCodePage = FormatType.FormatType_UTF8)
        : base(nbtotal, nbpar)
    {
        _theidents = new NCollection_Array1<int>(1, nbtotal);
        _thetypes = new NCollection_Array1<int>(1, nbtotal);
        _sourceCodePage = sourceCodePage;
        _thenbscop = 0;
        _thenbents = 0;
        _thelastn = 0;
        _thenbhead = nbheader;
        _thecheck = new FrameworkCheck();
    }

    /// <summary>Fills the fields of a record.</summary>
    public void SetRecord(int num, string ident, string type, int nbpar)
    {
        int numlst;
        type ??= ""; // OCCT.NET: a record whose type the parser never set (decision 1178)
        ident ??= "";
        if (global::Kerf.Engine.Foundation.Standard.CStringChar(type, 0) != '(')
        {
            _thenbents++; // total number of proper file terms
        }
        _thetypes.ChangeValue(num) = _thenametypes.Add(type) + 1;
        if (global::Kerf.Engine.Foundation.Standard.CStringChar(ident, 0) == '$')
        {
            if (ident.Length > 2)
            {
                numlst = global::Kerf.Engine.Foundation.Standard.Atoi(ident.Substring(1));
            }
            else
            {
                numlst = global::Kerf.Engine.Foundation.Standard.CStringChar(ident, 1) - 48;
            }
            if (_thelastn < numlst)
            {
                _thelastn = numlst; // highest sub-list number
            }
            _theidents.SetValue(num, -2 - numlst);
        }
        else if (global::Kerf.Engine.Foundation.Standard.CStringChar(ident, 0) == '#')
        {
            numlst = global::Kerf.Engine.Foundation.Standard.Atoi(ident.Substring(1));
            _theidents.SetValue(num, numlst);
            if (numlst == 0 && num > _thenbhead)
            {
                //    Header, or Complex Type ...
                //    If Complex Type, find Previous Type (we consider this is rare)
                //    Chain the previous type to the next one
                //    VERIFICATION that types are in alphabetical order
                for (int prev = num - 1; prev > _thenbhead; prev--)
                {
                    if (_theidents[prev] >= 0)
                    {
                        _themults[prev] = num;
                        if (string.CompareOrdinal(_thenametypes[_thetypes.Value(num) - 1], _thenametypes[_thetypes.Value(prev) - 1]) < 0)
                        {
                            //  Warning: components in complex entity are not in alphabetical order.
                            var errm = new StringBuilder("Complex Type incorrect : ");
                            errm.Append(_thenametypes[_thetypes.Value(prev) - 1]);
                            errm.Append(" / ");
                            errm.Append(_thenametypes[_thetypes.Value(num) - 1]);
                            errm.Append(" ... ");
                            while (_theidents[prev] <= 0)
                            {
                                prev--;
                                if (prev <= 0)
                                {
                                    break;
                                }
                            }
                            using (Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendTrace())
                            {
                                sout.Append("  ***  Incorrect record ").Append(num).Append(" (on ").Append(RecordCount).Append(" -> ")
                                    .Append(num * 100 / RecordCount).Append(" % in File)  ***");
                                if (prev > 0)
                                {
                                    sout.Append("  Ident #").Append(_theidents[prev]);
                                }
                                sout.Append("\n").Append(errm.ToString()).Append('\n');
                            }
                            _thecheck.AddWarning(errm.ToString(), "Complex Type incorrect : ");
                        }
                        break;
                    }
                }
            }
        }
        else if (ident == "SCOPE")
        {
            _theidents.SetValue(num, -1); // SCOPE
            _thenbscop++;
        }
        else if (ident == "ENDSCOPE")
        {
            _theidents.SetValue(num, -2); // ENDSCOPE
        }
        //      Reste 0
    }

    /// <summary>Fills the fields of a parameter of a record. This is a variant of AddParam, adapted to STEP (optimized for specific values).</summary>
    public void AddStepParam(int num, string aval, ParamType atype, int nument = 0)
    {
        if (atype == global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub)
        {
            int numid = 0;
            if (global::Kerf.Engine.Foundation.Standard.CStringChar(aval, 2) != '\0')
            {
                numid = global::Kerf.Engine.Foundation.Standard.Atoi(aval.Substring(1));
                AddParam(num, aval, atype, numid);
            }
            else
            {
                numid = global::Kerf.Engine.Foundation.Standard.CStringChar(aval, 1) - 48; // -48 ('0') -1 (adresse [] depuis 0)
                AddParam(num, aval, atype, numid);
            }
        }
        else if (atype == global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent)
        {
            int numid = global::Kerf.Engine.Foundation.Standard.Atoi(aval.Substring(1));
            AddParam(num, aval, atype, numid);
        }
        else
        {
            AddParam(num, aval, atype, nument);
        }
    }

    /// <summary>Returns Record Type.</summary>
    public string RecordType(int num)
    {
        return _thenametypes[_thetypes.Value(num) - 1];
    }

    /// <summary>Returns Record Type as a CString.</summary>
    public string CType(int num)
    {
        return _thenametypes[_thetypes.Value(num) - 1];
    }

    /// <summary>Returns record identifier (Positive number). If returned ident is not positive: Sub-List or Scope mark.</summary>
    public int RecordIdent(int num)
    {
        return _theidents[num];
    }

    //  ########################################################################
    //  ....       Parameter reading aids, adapted for STEP       ....

    /// <summary>
    /// Returns SubList numero designated by a parameter (nump) in a record (num), or zero if the parameter does not
    /// exist or is not a SubList address. Zero too if aslast is True and nump is not for the last parameter.
    /// </summary>
    public int SubListNumber(int num, int nump, bool aslast)
    {
        if (nump == 0 || nump > NbParams(num))
        {
            return 0;
        }
        FileParameter fp = Param(num, nump);
        if (fp.ParamType() != global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub)
        {
            return 0;
        }
        if (aslast)
        {
            if (nump != NbParams(num))
            {
                return 0;
            }
        }
        return fp.EntityNumber();
    }

    /// <summary>Returns True if num corresponds to a Complex Type Entity (as can be defined by ANDOR Express clause).</summary>
    public bool IsComplex(int num)
    {
        return _themults.ContainsKey(num);
    }

    /// <summary>
    /// Returns the List of Types which correspond to a Complex Type Entity. If not Complex, there is just one Type in
    /// it. For a SubList or a Scope mark, types remains empty.
    /// </summary>
    public void ComplexType(int num, NCollection_Sequence<string> types)
    {
        if (_theidents[num] < 0)
        {
            return;
        }
        for (int i = num; i > 0; i = NextForComplex(i))
        {
            types.Append(RecordType(i));
        }
    }

    /// <summary>
    /// Returns the Next "Component" for a Complex Type Entity, of which num is already a Component (the first one or a
    /// next one). Returns 0 for a Simple Type or for the last Component.
    /// </summary>
    public int NextForComplex(int num)
    {
        int next = 0;
        if (_themults.ContainsKey(num))
        {
            next = _themults.GetValue(num);
        }
        return next;
    }

    /// <summary>
    /// Determines the first component which brings a given name, for a Complex Type Entity. num0 is the very first
    /// record of this entity; num is given the last NextNamedForComplex, starts at zero; it is returned as the newly
    /// found number. If the alphabetic order is not respected, it restarts from num0 and loops on NextForComplex until
    /// finding name; ach is then filled with a Warning; if not found at all, with a Fail, and num is returned as zero.
    /// Returns True if alphabetic order, False else.
    /// </summary>
    public bool NamedForComplex(string name, int num0, ref int num, FrameworkCheck ach)
    {
        int n = (num <= 0 ? num0 : NextForComplex(num));
        // sln 04,10.2001. BUC61003. if(n==0) the next  function is not called in order to avoid exception
        if ((n != 0) && RecordType(n) == name)
        {
            num = n;
            return true;
        }
        if (n == 0)
        {
            NamedForComplex(name, num0, ref n, ach); // on a rembobine
        }
        //  Not in alphabetical order: loop
        string txtmes;
        string errmess = "Parameter n0.%d (%s) not a LIST";
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, name);
        for (n = num0; n > 0; n = NextForComplex(n))
        {
            if (RecordType(n) == name)
            {
                num = n;
                errmess = "Complex Record n0.%d, member type %s not in alphabetic order";
                txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, name);
                ach.AddWarning(txtmes, errmess);
                return false;
            }
        }
        num = 0;
        errmess = "Complex Record n0.%d, member type %s not found";
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, name);
        ach.AddFail(txtmes, errmess);
        return false;
    }

    /// <summary>
    /// Determines the first component which brings a given name, or short name for a Complex Type Entity (as the
    /// form with one name).
    /// </summary>
    public bool NamedForComplex(string name, string shortName, int num0, ref int num, FrameworkCheck ach)
    {
        int n = (num <= 0 ? num0 : NextForComplex(num));
        if ((n != 0) && (RecordType(n) == name || RecordType(n) == shortName))
        {
            num = n;
            return true;
        }
        // entities are not in alphabetical order
        string txtmes;
        string errmess = "Parameter n0.%d (%s) not a LIST";
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, name);
        for (n = num0; n > 0; n = NextForComplex(n))
        {
            if (RecordType(n) == name || RecordType(n) == shortName)
            {
                num = n;
                errmess = "Complex Record n0.%d, member type %s not in alphabetic order";
                txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, name);
                ach.AddWarning(txtmes, errmess);
                return false;
            }
        }
        num = 0;
        errmess = "Complex Record n0.%d, member type %s not found";
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, name);
        ach.AddFail(txtmes, errmess);
        return false;
    }

    //  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##

    /// <summary>
    /// Checks Count of Parameters of record num to equate nbreq. If this Check is successful, returns True. Else,
    /// fills ach with an Error Message then returns False; mess is included in the Error message if given non empty.
    /// </summary>
    public bool CheckNbParams(int num, int nbreq, FrameworkCheck ach, string mess = "")
    {
        if (NbParams(num) == nbreq)
        {
            return true;
        }
        string txtmes;
        string errmess;
        if (string.IsNullOrEmpty(mess))
        {
            errmess = "Count of Parameters is not %d";
        }
        else
        {
            errmess = "Count of Parameters is not %d for %s";
        }
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, nbreq, mess);
        ach.AddFail(txtmes, errmess);
        return false;
    }

    /// <summary>
    /// Reads parameter nump of record num as a sub-list (may be typed, see ReadTypedParameter in this case). Returns
    /// True if OK. Else (not a LIST), returns false and feeds Check with appropriate check. If optional is True and
    /// Param is not defined, returns True with ach not filled and numsub returned as 0.
    /// </summary>
    public bool ReadSubList(int num, int nump, string mess, FrameworkCheck ach, ref int numsub, bool optional = false, int lenmin = 0, int lenmax = 0)
    {
        string txtmes;
        numsub = SubListNumber(num, nump, false);
        if (numsub > 0)
        {
            int nbParams = NbParams(numsub);
            if (nbParams == 0)
            {
                string errMess = "Parameter n0.%d (%s) is an empty LIST";
                txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errMess, nump, mess);
                ach.AddWarning(txtmes, errMess);
            }
            return true;
        }
        //  Si optionel indefini, on passe l eponge
        numsub = 0;
        bool isvoid = (Param(num, nump).ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid);
        if (isvoid && optional)
        {
            return false;
        }
        string errmess = "Parameter n0.%d (%s) not a LIST";
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, nump, mess);
        ach.AddFail(txtmes, errmess);
        return false;
    }

    //  ...   Utilities for LateBinding

    /// <summary>
    /// Reads the content of a sub-list into a transient: SelectNamed, or HArray1 of Integer, Real, String,
    /// Transient... (recursive call if list of list). If a sub-list has mixed types, an HArray1 of Transient is
    /// produced, it may contain SelectMember. Intended to be called by ReadField. The returned status is: negative if
    /// failed, 0 if empty, else the kind to be recorded in the field.
    /// </summary>
    /// <remarks>The value is an <c>object</c>: the port's HArray1 are not Standard_Transient (decision 17).</remarks>
    public int ReadSub(int numsub, string mess, FrameworkCheck ach, PDescr? descr, ref object? val)
    {
        int nbp = NbParams(numsub);
        if (nbp == 0)
        {
            return 0; // empty list = Handle Null
        }
        string rectyp = RecordType(numsub);
        if (nbp == 1 && global::Kerf.Engine.Foundation.Standard.CStringChar(rectyp, 0) != '(')
        {
            //  it's a type with one parameter -> SelectNamed
            //  cf ReadSelect but here, we are already on the parameter content
            SelectNamed sn = new SelectNamed();
            val = sn;
            sn.SetName(rectyp);
            Transient aSN = sn;
            if (ReadAny(numsub, 1, mess, ach, descr, ref aSN))
            {
                return sn.Kind();
            }
            else
            {
                return 0;
            }
        }
        //  common case: make an HArray1 of ... of ... of what exactly
        FileParameter fp0 = Param(numsub, 1);
        ParamType ft, ft0 = fp0.ParamType();
        string str = fp0.CValue();
        NCollection_HArray1<Transient> htr = null;
        NCollection_HArray1<int> hin = null;
        NCollection_HArray1<double> hre = null;
        NCollection_HArray1<TCollection_HAsciiString> hst = null;
        int kod = 0;
        switch (ft0)
        {
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamMisc:
                return -1;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamInteger:
                kod = 1;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal:
                kod = 5;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent:
                kod = 7;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid:
                kod = 0;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamText:
                kod = 6;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum:
                kod = 4;
                break; // a confirmer(logical)
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamLogical:
                return -1;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub:
                kod = 0;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamHexa:
                return -1;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamBinary:
                return -1;
            default:
                return -1;
        }
        if (kod == 1 || kod == 3)
        {
            hin = new NCollection_HArray1<int>(1, nbp);
            val = hin;
        }
        else if (kod == 5)
        {
            hre = new NCollection_HArray1<double>(1, nbp);
            val = hre;
        }
        else if (kod == 6)
        {
            hst = new NCollection_HArray1<TCollection_HAsciiString>(1, nbp);
            val = hst;
        }
        else
        {
            htr = new NCollection_HArray1<Transient>(1, nbp);
            val = htr;
        }
        //  Attention: if variable type, will need to change approach -> htr
        for (int ip = 1; ip <= nbp; ip++)
        {
            FileParameter fp = Param(numsub, ip);
            str = fp.CValue();
            ft = fp.ParamType();
            switch (kod)
            {
                case 1:
                {
                    if (ft != global::Kerf.Engine.Exchange.Framework.ParamType.ParamInteger)
                    {
                        kod = 0;
                        break;
                    }
                    hin.SetValue(ip, global::Kerf.Engine.Foundation.Standard.Atoi(str));
                    break;
                }
                case 2:
                case 3:
                {
                    if (ft != global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum)
                    {
                        kod = 0;
                        break;
                    }
                    if (str == ".F.")
                    {
                        hin.SetValue(ip, 0);
                    }
                    else if (str == ".T.")
                    {
                        hin.SetValue(ip, 1);
                    }
                    else if (str == ".U.")
                    {
                        hin.SetValue(ip, 2);
                    }
                    else
                    {
                        kod = 0;
                    }
                    break;
                }
                case 4:
                {
                    if (ft != global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum)
                    {
                        kod = 0;
                        break;
                    }
                    SelectNamed sn = new SelectNamed();
                    sn.SetEnum(-1, str);
                    htr.SetValue(ip, sn);
                    break;
                }
                case 5:
                {
                    if (ft != global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
                    {
                        kod = 0;
                        break;
                    }
                    hre.SetValue(ip, Fastof(str));
                    break;
                }
                case 6:
                {
                    if (ft != global::Kerf.Engine.Exchange.Framework.ParamType.ParamText)
                    {
                        kod = 0;
                        break;
                    }
                    TCollection_HAsciiString txt = new TCollection_HAsciiString(str);
                    CleanText(txt);
                    hst.SetValue(ip, txt);
                    break;
                }
                case 7:
                {
                    Transient ent = BoundEntity(fp.EntityNumber());
                    htr.SetValue(ip, ent);
                    break;
                }
                default:
                    break;
            }
            //    Remaining other cases ... everything is possible. cf the Param type
            if (kod > 0)
            {
                continue;
            }
            //    Need to pass to transient ...
            if (htr is null)
            {
                htr = new NCollection_HArray1<Transient>(1, nbp);
                val = htr;
                int jp;
                if (hin is not null)
                {
                    for (jp = 1; jp < ip; jp++)
                    {
                        SelectInt sin = new SelectInt();
                        sin.SetInt(hin.Value(jp));
                        htr.SetValue(jp, sin);
                    }
                }
                if (hre is not null)
                {
                    for (jp = 1; jp < ip; jp++)
                    {
                        SelectReal sre = new SelectReal();
                        sre.SetReal(hre.Value(jp));
                        htr.SetValue(jp, sre);
                    }
                }
                if (hst is not null)
                {
                    for (jp = 1; jp < ip; jp++)
                    {
                        htr.SetValue(jp, hst.Value(jp));
                    }
                }
            }
            //    Now, let's go: read the field and put it in place
            //    What follows strongly resembles ReadAny ...
            switch (ft)
            {
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamMisc:
                    break;
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamInteger:
                {
                    SelectInt sin = new SelectInt();
                    sin.SetInteger(global::Kerf.Engine.Foundation.Standard.Atoi(str));
                    htr.SetValue(ip, sin);
                    break;
                }
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal:
                {
                    SelectReal sre = new SelectReal();
                    sre.SetReal(Fastof(str));
                    break;
                    // htr->SetValue (ip,sre); break; svv #2: unreachable
                }
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent:
                    htr.SetValue(ip, BoundEntity(fp.EntityNumber()));
                    break;
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid:
                    break;
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum:
                {
                    SelectInt sin;
                    SelectNamed sna;
                    int logic = -1;
                    // PTV 16.09.2000
                    // set the default value of StepData_Logical
                    Logical slog = Logical.LUnknown;
                    if (global::Kerf.Engine.Foundation.Standard.CStringChar(str, 0) == '.' && global::Kerf.Engine.Foundation.Standard.CStringChar(str, 2) == '.' && global::Kerf.Engine.Foundation.Standard.CStringChar(str, 3) == '\0')
                    {
                        if (str[1] == 'F')
                        {
                            slog = Logical.LFalse;
                            logic = 0;
                        }
                        else if (str[1] == 'T')
                        {
                            slog = Logical.LTrue;
                            logic = 1;
                        }
                        else if (str[1] == 'U')
                        {
                            slog = Logical.LUnknown;
                            logic = 2;
                        }
                    }
                    if (logic >= 0)
                    {
                        sin = new SelectInt();
                        sin.SetLogical(slog);
                        htr.SetValue(ip, sin);
                    }
                    else
                    {
                        sna = new SelectNamed();
                        sna.SetEnum(logic, str);
                        htr.SetValue(ip, sna);
                    }
                    break;
                }
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamLogical:
                    break;
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamText:
                {
                    TCollection_HAsciiString txt = new TCollection_HAsciiString(str);
                    CleanText(txt);
                    htr.SetValue(ip, txt);
                    break;
                }
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub:
                {
                    object sub = null;
                    int nent = fp.EntityNumber();
                    int kind = ReadSub(nent, mess, ach, descr, ref sub);
                    if (kind < 0)
                    {
                        break;
                    }
                    // OCCT: a sub-list's value is a handle; the port's lists are not Standard_Transient (decision 17),
                    // so a list of lists keeps only its select members and strings
                    htr.SetValue(ip, sub as Transient);
                    break;
                }
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamHexa:
                    break;
                case global::Kerf.Engine.Exchange.Framework.ParamType.ParamBinary:
                    break;
                default:
                    break;
            }
            return -1;
        }
        return 8; // for Any
    }

    /// <summary>
    /// Reads parameter nump of record num into a SelectMember, self-sufficient (no Description needed). If val is
    /// already created, it will be filled, as possible; and if reading does not match its own description, the result
    /// will be False. If val is not yet created, it will be (SelectNamed).
    /// </summary>
    public bool ReadMember(int num, int nump, string mess, FrameworkCheck ach, ref SelectMember? val)
    {
        Transient v = val;
        PDescr nuldescr = null;
        if (v is null)
        {
            return ReadAny(num, nump, mess, ach, nuldescr, ref v) && (val = v as SelectMember) is not null;
        }
        bool res = ReadAny(num, nump, mess, ach, nuldescr, ref v);
        if (v == val)
        {
            return res;
        }
        //   changement -> refus
        string txtmes;
        string errmess = "Parameter n0.%d (%s) : does not match SELECT clause";
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, nump, mess);
        ach.AddFail(txtmes, errmess);
        return false;
    }

    /// <summary>Safe variant for arbitrary type of argument.</summary>
    public bool ReadMember<T>(int num, int nump, string mess, FrameworkCheck ach, ref T? val) where T : SelectMember
    {
        SelectMember aVal = val;
        return ReadMember(num, nump, mess, ach, ref aVal) && (val = aVal as T) is not null;
    }

    /// <summary>
    /// Reads parameter nump of record num into a Field, controlled by a Parameter Descriptor (PDescr), which controls
    /// its allowed type(s) and value. ach is filled if the read parameter does not match its description (but the
    /// field is read anyway). If the description is not defined, no control is done. Returns True when done.
    /// </summary>
    public bool ReadField(int num, int nump, string mess, FrameworkCheck ach, PDescr? descr, Field fild)
    {
        FileParameter fp = Param(num, nump);
        string str = fp.CValue();
        bool ok = true;
        int nent, kind;
        object sub = null;
        ParamType ft = fp.ParamType();
        switch (ft)
        {
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamMisc:
                ok = false;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamInteger:
                fild.SetInteger(global::Kerf.Engine.Foundation.Standard.Atoi(str));
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal:
                fild.SetReal(Fastof(str));
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent:
                nent = fp.EntityNumber();
                if (nent > 0)
                {
                    fild.SetEntity(BoundEntity(nent));
                }
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid:
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamText:
            {
                TCollection_HAsciiString txt = new TCollection_HAsciiString(str);
                CleanText(txt);
                fild.Set(txt);
                break;
            }
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum:
                if (str == ".T.")
                {
                    fild.SetLogical(Logical.LTrue);
                }
                else if (str == ".F.")
                {
                    fild.SetLogical(Logical.LFalse);
                }
                else if (str == ".U.")
                {
                    fild.SetLogical(Logical.LUnknown);
                }
                else
                {
                    fild.SetEnum(-1, str);
                }
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamLogical:
                ok = false;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub:
                nent = fp.EntityNumber();
                kind = ReadSub(nent, mess, ach, descr, ref sub);
                if (kind < 0)
                {
                    break;
                }
                fild.Clear(kind);
                fild.Set(sub);
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamHexa:
                ok = false;
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamBinary:
                ok = false;
                break;
            default:
                ok = false;
                break;
        }
        if (!ok)
        {
            if (str == "*")
            {
                fild.SetDerived();
            }
        }
        return true;
    }

    /// <summary>Reads a list of fields controlled by an ESDescr.</summary>
    public bool ReadList(int num, FrameworkCheck ach, ESDescr descr, FieldList list)
    {
        // controler nbs egaux
        int i, nb = list.FieldCount;
        if (!CheckNbParams(num, nb, ach, descr.TypeName()))
        {
            return false;
        }
        for (i = 1; i <= nb; i++)
        {
            PDescr pde = descr.Field(i);
            Field fild = list.CField(i);
            ReadField(num, i, pde.Name(), ach, pde, fild);
        }
        return true;
    }

    /// <summary>
    /// Reads parameter nump of record num into a Transient Value according to the type of the parameter: Named for
    /// Integer, Boolean, Logical, Enum, Real: SelectNamed; Immediate Integer, Boolean, Logical, Enum, Real:
    /// SelectInt/Real; Text: HAsciiString; Ident: the referenced Entity; Sub-List not processed, see ReadSub. val is in
    /// out, hence it is possible to predefine a specific SelectMember then to fill it. If val is Null or if the result
    /// is not a SelectMember, val itself is returned a new ref. For a Select with a Name, val must then be a
    /// SelectNamed.
    /// </summary>
    public bool ReadAny(int num, int nump, string mess, FrameworkCheck ach, PDescr? descr, ref Transient? val)
    {
        FileParameter fp = Param(num, nump);
        string str = fp.CValue();
        ParamType ft = fp.ParamType();
        //    Now, let's go: read the field and put it in place
        switch (ft)
        {
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamMisc:
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamInteger:
            {
                if (val is not null)
                {
                    SelectMember sm = val as SelectMember;
                    sm.SetReal(Fastof(str)); // OCCT: an integer is set as a real into a given member
                    return true;
                }
                SelectInt sin = new SelectInt();
                sin.SetInteger(global::Kerf.Engine.Foundation.Standard.Atoi(str));
                val = sin;
                return true;
            }
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal:
            {
                if (val is not null)
                {
                    SelectMember sm = val as SelectMember;
                    sm.SetReal(Fastof(str));
                    return true;
                }
                SelectReal sre = new SelectReal();
                sre.SetReal(Fastof(str));
                val = sre;
                return true;
            }
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent:
            {
                int nent = fp.EntityNumber();
                if (nent > 0)
                {
                    val = BoundEntity(nent);
                }
                return (val is not null);
            }
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid:
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum:
            {
                SelectMember sm = null;
                if (val is not null)
                {
                    sm = val as SelectMember;
                }
                SelectInt sin;
                SelectNamed sna;
                int logic = -1;
                // PTV 16.09.2000
                // set the default value of StepData_Logical
                Logical slog = Logical.LUnknown;
                if (global::Kerf.Engine.Foundation.Standard.CStringChar(str, 0) == '.' && global::Kerf.Engine.Foundation.Standard.CStringChar(str, 2) == '.' && global::Kerf.Engine.Foundation.Standard.CStringChar(str, 3) == '\0')
                {
                    if (str[1] == 'F')
                    {
                        slog = Logical.LFalse;
                        logic = 0;
                    }
                    else if (str[1] == 'T')
                    {
                        slog = Logical.LTrue;
                        logic = 1;
                    }
                    else if (str[1] == 'U')
                    {
                        slog = Logical.LUnknown;
                        logic = 2;
                    }
                }
                if (logic >= 0)
                {
                    if (sm is not null)
                    {
                        sm.SetLogical(slog);
                    }
                    else
                    {
                        sin = new SelectInt();
                        val = sin;
                        sin.SetLogical(slog);
                    }
                }
                else
                {
                    if (sm is not null)
                    {
                        sm.SetEnum(logic, str);
                    }
                    else
                    {
                        sna = new SelectNamed();
                        val = sna; // Named sans nom...
                        sna.SetEnum(logic, str);
                    }
                } // -> Select general
                return true;
            }
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamLogical:
                break;
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamText:
            {
                TCollection_HAsciiString txt = new TCollection_HAsciiString(str);
                CleanText(txt);
                // PDN May 2000: for reading SOURCE_ITEM (external references)
                if (val is not null)
                {
                    SelectMember sm = val as SelectMember;
                    sm.SetString(txt.ToCString());
                    return true;
                }
                val = txt;
                return true;
            }
            case global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub:
            {
                int numsub = SubListNumber(num, nump, false);
                int nbp = NbParams(numsub);
                if (nbp == 0)
                {
                    return false; // empty list = Handle Null
                }
                string rectyp = RecordType(numsub);
                if (nbp == 1 && global::Kerf.Engine.Foundation.Standard.CStringChar(rectyp, 0) != '(')
                {
                    //  SelectNamed because Field !!!
                    // skl 15.01.2003 (for members with array of real)
                    if (val is SelectArrReal sma)
                    {
                        int numsub2 = SubListNumber(numsub, 1, false);
                        int nbp2 = NbParams(numsub2);
                        if (nbp2 > 1)
                        {
                            if (Param(numsub2, 1).ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
                            {
                                if (!sma.SetName(rectyp))
                                {
                                    return false;
                                }
                                NCollection_HSequence<double> seq = new NCollection_HSequence<double>();
                                for (int i = 1; i <= nbp2; i++)
                                {
                                    if (Param(numsub2, i).ParamType() != global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
                                    {
                                        continue;
                                    }
                                    Transient asr = new SelectReal();
                                    if (!ReadAny(numsub2, i, mess, ach, descr, ref asr))
                                    {
                                        continue;
                                    }
                                    if (asr is SelectReal sm1)
                                    {
                                        seq.Append(sm1.Real());
                                    }
                                }
                                NCollection_HArray1<double> arr = new NCollection_HArray1<double>(1, seq.Length());
                                for (int nr = 1; nr <= seq.Length(); nr++)
                                {
                                    arr.SetValue(nr, seq.Value(nr));
                                }
                                sma.SetArrReal(arr);
                                return true;
                            }
                        }
                    }
                    SelectMember sm = val as SelectMember;
                    if (sm is null)
                    {
                        sm = new SelectNamed();
                        val = sm;
                    }
                    if (!sm.SetName(rectyp))
                    {
                        return false; // loupe
                    }
                    return ReadAny(numsub, 1, mess, ach, descr, ref val);
                }
                break;
            }
            default:
                break;
        }
        return false;
    }

    //  ....

    /// <summary>
    /// Reads parameter nump of record num as a sub-list of two Reals X,Y. Returns True if OK. Else, returns false and
    /// feeds Check with appropriate Fails (parameter not a sub-list, not two Reals in the sub-list) composed with mess
    /// which gives the name of the parameter.
    /// </summary>
    public bool ReadXY(int num, int nump, string mess, FrameworkCheck ach, ref double x, ref double y)
    {
        string errmess = null; // Null if no error
        int numsub = SubListNumber(num, nump, false);
        if (numsub != 0)
        {
            if (NbParams(numsub) == 2)
            {
                FileParameter fpx = Param(numsub, 1);
                if (fpx.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
                {
                    x = Fastof(fpx.CValue());
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : (X,Y) X not a Real";
                }
                FileParameter fpy = Param(numsub, 2);
                if (fpy.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
                {
                    y = Fastof(fpy.CValue());
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : (X,Y) Y not a Real";
                }
            }
            else
            {
                errmess = "Parameter n0.%d (%s) : (X,Y) has not 2 params";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) : (X,Y) not a SubList";
        }
        return Report(errmess, nump, mess, ach);
    }

    /// <summary>Reads parameter nump of record num as a sub-list of three Reals X,Y,Z. Return value and Check managed as by ReadXY.</summary>
    public bool ReadXYZ(int num, int nump, string mess, FrameworkCheck ach, ref double x, ref double y, ref double z)
    {
        string errmess = null; // Null if no error
        int numsub = SubListNumber(num, nump, false);
        if (numsub != 0)
        {
            if (NbParams(numsub) == 3)
            {
                FileParameter fpx = Param(numsub, 1);
                if (fpx.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
                {
                    x = Fastof(fpx.CValue());
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : (X,Y,Z) X not a Real";
                }
                FileParameter fpy = Param(numsub, 2);
                if (fpy.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
                {
                    y = Fastof(fpy.CValue());
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : (X,Y,Z) Y not a Real";
                }
                FileParameter fpz = Param(numsub, 3);
                if (fpz.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
                {
                    z = Fastof(fpz.CValue());
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : (X,Y,Z) Z not a Real";
                }
            }
            else
            {
                errmess = "Parameter n0.%d (%s) : (X,Y,Z) has not 3 params";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) : (X,Y,Z) not a SubList";
        }
        return Report(errmess, nump, mess, ach);
    }

    // The common end of the Read* methods: no message, true; else a fail with the message, false
    private static bool Report(string errmess, int nump, string mess, FrameworkCheck ach)
    {
        if (errmess is null)
        {
            return true;
        }
        string txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, nump, mess);
        ach.AddFail(txtmes, errmess);
        return false;
    }

    /// <summary>Reads parameter nump of record num as a single Real value. Return value and Check managed as by ReadXY (demands a Real).</summary>
    public bool ReadReal(int num, int nump, string mess, FrameworkCheck ach, ref double val)
    {
        string errmess = null; // Null si pas d erreur
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal || fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamInteger)
            {
                val = Fastof(fp.CValue());
            }
            else
            {
                errmess = "Parameter n0.%d (%s) not a Real";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    //  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##

    /// <summary>
    /// Reads parameter nump of record num as a single Entity. Return value and Check managed as by ReadReal (demands a
    /// reference to an Entity). In Addition, demands read Entity to be Kind of a required Type atype. Remark that
    /// returned status is False and ent is Null if parameter is not an Entity, ent remains Not Null is parameter is an
    /// Entity but is not Kind of required type.
    /// </summary>
    public bool ReadEntity(int num, int nump, string mess, FrameworkCheck ach, Type atype, ref Transient? ent)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            int nent = fp.EntityNumber();
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent)
            {
                if (nent > 0)
                {
                    Transient entent = BoundEntity(nent);
                    if (entent is null || !atype.IsInstanceOfType(entent))
                    {
                        errmess = "Parameter n0.%d (%s) : Entity has illegal type";
                        if (entent is UndefinedEntity)
                        {
                            ent = entent;
                        }
                    }
                    else
                    {
                        ent = entent;
                    }
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : Unresolved reference";
                }
            }
            else
            {
                errmess = "Parameter n0.%d (%s) not an Entity";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    /// <summary>Safe variant for arbitrary type of argument.</summary>
    public bool ReadEntity<T>(int num, int nump, string mess, FrameworkCheck ach, Type atype, ref T? ent) where T : Transient
    {
        Transient anEnt = ent;
        return ReadEntity(num, nump, mess, ach, atype, ref anEnt) && (ent = anEnt as T) is not null;
    }

    /// <summary>Same as above, but a SelectType checks Type Matching, and records the read Entity (see method Value from SelectType).</summary>
    public bool ReadEntity(int num, int nump, string mess, FrameworkCheck ach, SelectType sel)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            int nent = fp.EntityNumber();
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent)
            {
                if (nent > 0)
                {
                    Transient entent = BoundEntity(nent);
                    if (!sel.Matches(entent))
                    {
                        errmess = "Parameter n0.%d (%s) : Entity has illegal type";
                        // fot not supported STEP entity
                        if (entent is UndefinedEntity)
                        {
                            sel.SetValue(entent);
                        }
                    }
                    else
                    {
                        sel.SetValue(entent);
                    }
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : Unresolved reference";
                }
            }
            else if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid)
            {
                errmess = "Parameter n0.%d (%s) not an Entity";
            }
            else
            {
                // Remaining case: we are actually interested in the SelectMember ...
                Transient sm = sel.NewMember();
                // SelectMember which performs this role. Can be specialized
                if (!ReadAny(num, nump, mess, ach, sel.Description(), ref sm))
                {
                    errmess = "Parameter n0.%d (%s) : could not be read";
                }
                if (!sel.Matches(sm))
                {
                    errmess = "Parameter n0.%d (%s) : illegal parameter type";
                }
                else
                {
                    sel.SetValue(sm);
                }
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    //  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##

    /// <summary>Reads parameter nump of record num as a single Integer. Return value and Check managed as by ReadXY (demands an Integer).</summary>
    public bool ReadInteger(int num, int nump, string mess, FrameworkCheck ach, ref int val)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamInteger)
            {
                val = global::Kerf.Engine.Foundation.Standard.Atoi(fp.CValue());
            }
            else if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
            {
                val = RoundToInt(Fastof(fp.CValue()));
                errmess = "Parameter n0.%d (%s) was rounded";
            }
            if (fp.ParamType() != global::Kerf.Engine.Exchange.Framework.ParamType.ParamInteger && fp.ParamType() != global::Kerf.Engine.Exchange.Framework.ParamType.ParamReal)
            {
                errmess = "Parameter n0.%d (%s) not an Integer";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    // static_cast<int>(std::round(x)): halves away from zero, then the conversion (out of range: MSVC's 0x80000000)
    private static int RoundToInt(double value)
    {
        double rounded = Math.Round(value, MidpointRounding.AwayFromZero);
        return rounded >= int.MinValue && rounded < 2147483648.0 ? (int)rounded : int.MinValue;
    }

    /// <summary>
    /// Reads parameter nump of record num as a Boolean. Return value and Check managed as by ReadReal (demands a
    /// Boolean enum, i.e. text ".T." for True or ".F." for False).
    /// </summary>
    public bool ReadBoolean(int num, int nump, string mess, FrameworkCheck ach, ref bool flag)
    {
        flag = true;
        string errmess = null; // Null si pas d erreur
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum)
            {
                string txt = fp.CValue();
                if (txt == ".T.")
                {
                    flag = true;
                }
                else if (txt == ".F.")
                {
                    flag = false;
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : Incorrect Boolean Value. It was set to true";
                }
            }
            else
            {
                errmess = "Parameter n0.%d (%s) not a Boolean. It was set to true";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent.It was set to true";
        }
        return Report(errmess, nump, mess, ach);
    }

    /// <summary>
    /// Reads parameter nump of record num as a Logical. Return value and Check managed as by ReadBoolean (demands a
    /// Logical enum, i.e. text ".T.", ".F.", or ".U.").
    /// </summary>
    public bool ReadLogical(int num, int nump, string mess, FrameworkCheck ach, ref Logical flag)
    {
        string errmess = null; // Null si pas d erreur
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum)
            {
                string txt = fp.CValue();
                if (txt == ".T.")
                {
                    flag = Logical.LTrue;
                }
                else if (txt == ".F.")
                {
                    flag = Logical.LFalse;
                }
                else if (txt == ".U.")
                {
                    flag = Logical.LUnknown;
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : Incorrect Logical Value";
                }
            }
            else
            {
                errmess = "Parameter n0.%d (%s) not a Logical";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    /// <summary>
    /// Reads parameter nump of record num as a String (text between quotes, quotes are removed by the Read
    /// operation). Return value and Check managed as by ReadXY (demands a String).
    /// </summary>
    public bool ReadString(int num, int nump, string mess, FrameworkCheck ach, ref TCollection_HAsciiString? val)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamText)
            {
                val = new TCollection_HAsciiString(fp.CValue());
                CleanText(val);
            }
            else
            {
                errmess = "Parameter n0.%d (%s) not a quoted String";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    /// <summary>Reads parameter nump of record num as an enumeration text (between dots, dots kept).</summary>
    public bool ReadEnumParam(int num, int nump, string mess, FrameworkCheck ach, ref string text)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum)
            {
                text = fp.CValue();
            }
            else if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid)
            {
                errmess = "Parameter n0.%d (%s) : Undefined Enumeration not allowed";
            }
            else
            {
                errmess = "Parameter n0.%d (%s) not an Enumeration";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    /// <summary>Fills a check with a fail message if enumeration value does match parameter definition. Just a help to centralize message definitions.</summary>
    public void FailEnumValue(int num, int nump, string mess, FrameworkCheck ach)
    {
        string errmess = "Parameter n0.%d (%s) : Incorrect Enumeration Value";
        string txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, nump, mess);
        ach.AddFail(txtmes, errmess);
    }

    /// <summary>
    /// Reads parameter nump of record num as an Enumeration (text between dots) and converts it to an integer value,
    /// by an EnumTool. Returns True if OK, false if: this parameter is not enumeration, or is not recognized by the
    /// EnumTool (with fail).
    /// </summary>
    public bool ReadEnum(int num, int nump, string mess, FrameworkCheck ach, EnumTool enumtool, ref int val)
    {
        //  resume with ReadEnumParam?
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamEnum)
            {
                val = enumtool.Value(fp.CValue());
                if (val >= 0)
                {
                    return true;
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : Incorrect Enumeration Value";
                }
            }
            else if (fp.ParamType() == global::Kerf.Engine.Exchange.Framework.ParamType.ParamVoid)
            {
                val = enumtool.NullValue();
                if (val < 0)
                {
                    errmess = "Parameter n0.%d (%s) : Undefined Enumeration not allowed";
                }
            }
            else
            {
                errmess = "Parameter n0.%d (%s) not an Enumeration";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    /// <summary>
    /// Resolves a parameter which can be enclosed in a type def., as TYPE(val). The parameter must then be read
    /// normally according its type. mustbetyped True demands a typed parameter, False accepts a non-typed parameter as
    /// option. numr, numrp are the resolved record and parameter numbers (num, nump if no type, else numrp = 1); typ
    /// returns the recorded type, or empty string. Remark: a non-typed list is considered as "non-typed".
    /// </summary>
    public bool ReadTypedParam(int num, int nump, bool mustbetyped, string mess, FrameworkCheck ach, ref int numr, ref int numrp, ref string typ)
    {
        string txtmes;
        string errmess = null; // Null si pas d erreur
        if (nump > 0 && nump <= NbParams(num))
        {
            FileParameter fp = Param(num, nump);
            if (fp.ParamType() != global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub)
            {
                //    Not a sub-list: OK if allowed
                numr = num;
                numrp = nump;
                typ = "";
                if (mustbetyped)
                {
                    errmess = "Parameter n0.%d (%s) : single, not typed";
                    txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, nump, mess);
                    ach.AddFail(txtmes, errmess);
                    return false;
                }
                return true;
            }
            numr = fp.EntityNumber();
            numrp = 1;
            if (NbParams(numr) != 1)
            {
                errmess = "Parameter n0.%d (%s) : SubList, not typed";
            }
            typ = RecordType(numr);
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        return Report(errmess, nump, mess, ach);
    }

    /// <summary>
    /// Checks if parameter nump of record num is given as Derived. If this Check is successful (i.e. Param = "*"),
    /// returns True. Else, fills ach with a Message which contains mess and returns False. According to errstat, this
    /// message is Warning if errstat is False (Default), Fail if errstat is True.
    /// </summary>
    public bool CheckDerived(int num, int nump, string mess, FrameworkCheck ach, bool errstat = false)
    {
        string errmess = null; // Null if no error
        bool warn = !errstat;
        if (nump > 0 && nump <= NbParams(num))
        {
            if (Param(num, nump).CValue() == "*")
            {
                return true;
            }
            else
            {
                errmess = "Parameter n0.%d (%s) not Derived";
            }
        }
        else
        {
            errmess = "Parameter n0.%d (%s) absent";
        }
        string txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, nump, mess);
        if (warn)
        {
            ach.AddWarning(txtmes, errmess);
        }
        else
        {
            ach.AddFail(txtmes, errmess);
        }
        return false;
    }

    //  #########################################################################
    // ....     Specific methods (requested by FileReaderData)     .... //

    /// <summary>Returns total count of Entities (including Header).</summary>
    public override int NbEntities() // redefined
    {
        return _thenbents;
    }

    /// <summary>
    /// Determines the first suitable record following a given one, that is, skips SCOPE, ENDSCOPE and SUBLIST
    /// records. Note: skips Header records, which are accessed separately.
    /// </summary>
    public override int FindNextRecord(int num)
    {
        // returns, for a given record number (by num), the next one which
        // defines an entity, or 0 if finished:
        // passes the Header (first nbhend records) and
        // skips SCOPE and ENDSCOPE records and SUB-LISTS
        if (num < 0)
        {
            return 0;
        }
        int num1 = num + 1;
        if (num == 0)
        {
            num1 = _thenbhead + 1;
        }
        int max = RecordCount;
        while (num1 <= max)
        {
            if (_theidents[num1] > 0)
            {
                return num1;
            }
            // SCOPE,ENDSCOPE and Sub-List have a fictitious identifier: -1,-2 respectively
            // and SUBLIST have a negative one. Only a real entity has a positive Ident
            num1++;
        }
        return 0;
    }

    /// <summary>Searches for a Parameter of the record num, which refers to the Ident id (form #nnn). If found, returns its EntityNumber, else returns Zero.</summary>
    private int FindEntityNumber(int num, int id)
    {
        //  Given an "Id": search in the Ident type Parameters of <num>,
        //  if one of them designates #Id precisely. If yes, return its EntityNumber
        if (num == 0)
        {
            return 0;
        }
        int nb = NbParams(num);
        for (int i = 1; i <= nb; i++)
        {
            FileParameter fp = Param(num, i);
            if (fp.ParamType() != global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent)
            {
                continue;
            }
            int ixp = global::Kerf.Engine.Foundation.Standard.Atoi(fp.CValue().Substring(1));
            if (ixp == id)
            {
                return fp.EntityNumber();
            }
        }
        return 0; // ici, pas trouve
    }

    //  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
    // ....         The following function deserves special attention        ....
    //  This method preloads the EntityNumbers in the Params: they designate
    //  the Entities properly said in the list read by BoundEntity
    //  Interest: also addresses sub-lists (Num->record number in the Directory)
    //  result exploited by ParamEntity and ParamNumber
    //
    //  In the absence of SCOPE, or if the "ident" are strictly ordered, for sure
    //  they are not duplicated, we can use an IndexedMap with full
    //  confidence. Otherwise, we must scan the file, but with SCOPES
    //  this goes much faster (if they are big enough): we find our way around.
    // For the search by scanning, We operate in several steps
    // Before anything, the loading has already done preparation: the idents
    // (Entity, SubList) are already in integers (reading speed), in particular
    // in the EntityNumber: thus, we read this ident, we process it, and we put back
    // in its place a real Record number
    //
    // First, we pass the directory to integer table, sub-lists purged
    // in parallel, inverse table towards this table, because sub-lists can on the
    // contrary designate objects ...
    // For sub-lists, we exploit their construction mode: they are
    // recorded BEFORE being referenced. A "subn" array thus notes for
    // each sub-list number (relative to an entity that follows, and referenced
    // by it or another sub-list that also follows), its record number
    // NOTE: this also works for the Header, processed on the occasion

    /// <summary>
    /// Determines reference numbers in EntityNumber fields, called by Prepare from StepReaderTool to prepare later
    /// using by a StepModel. If withmap is given False, the basic exploration algorithm is activated, otherwise a map
    /// is used as far as it is possible (this option can be used only to test this algorithm).
    /// </summary>
    public void SetEntityNumbers(bool withmap = true)
    {
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendTrace();
        //   Initial pass: Direct resolution by Map
        //   if everything passes (no collision), OK. Otherwise, other passes to plan
        //   We resolve sub-lists at the same time
        //   The Map approach is O(1) lookup but requires unique identifiers
        //   If identifiers collide (due to SCOPE sections), we fall back to linear search
        int nbdirec = RecordCount;
        NCollection_Array1<int> subn = new NCollection_Array1<int>(0, _thelastn);
        bool pbmap = false; // at least one conflict
        int nbmap = 0;
        IndexedSet<int> imap = new IndexedSet<int>();
        NCollection_Array1<int> indm = new NCollection_Array1<int>(0, nbdirec); // Index Map -> Record Number (only if map)
        int num; // svv Jan11 2000 : porting on DEC
        for (num = 1; num <= nbdirec; num++)
        {
            int ident = _theidents[num];
            if (ident > 0)
            { // Ident normal -> Map ?
                //  Map: if Overlap, inhibit it. Otherwise, note index
                int indmap = imap.Add(ident) + 1;
                if (indmap <= nbmap)
                {
                    indmap = imap.IndexOf(ident) + 1; // safer
                    indm[indmap] = -1; // Map -> pb
                    pbmap = true;
                    //  pbmap means another pass will be necessary ...
                }
                else
                {
                    nbmap = indmap;
                    indm[indmap] = num; // Map ->ident
                }
            }
        }
        for (num = 1; num <= nbdirec; num++)
        {
            int ident = _theidents[num];
            if (ident < -2)
            {
                subn[-(ident + 2)] = num; // toujours a jour ...
            }
            int nba = NbParams(num);
            int nda = (num == 1 ? 0 : ParamFirstRank(num - 1));
            for (int na = nba; na > 0; na--)
            {
                //    We process: sub-lists (except subn), idents (if Map says OK ...)
                FileParameter fp = ChangeParameter(nda + na);
                ParamType letype = fp.ParamType();
                if (letype == global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub)
                {
                    int numsub = fp.EntityNumber();
                    if (numsub > _thelastn)
                    {
                        using (Messenger.StreamBuffer info = global::Kerf.Engine.Foundation.Message.SendInfo())
                        {
                            info.Append("Bad Sub.N0, Record ").Append(num).Append(" Param ").Append(na).Append(":$").Append(numsub).Append('\n');
                        }
                        continue;
                    }
                    fp.SetEntityNumber(subn[numsub]);
                }
                else if (letype == global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent)
                {
                    int id = fp.EntityNumber();
                    int indmap = imap.IndexOf(id) + 1;
                    if (indmap > 0)
                    { // the map found it
                        int num0 = indm[indmap];
                        if (num0 > 0)
                        {
                            fp.SetEntityNumber(num0); // AND THERE, we have resolved
                        }
                        else
                        {
                            fp.SetEntityNumber(-id); // CONFLICT -> will need to resolve ...
                        }
                    }
                    else
                    { // NOT RESOLVED, if no pbmap, say it
                        if (pbmap)
                        {
                            fp.SetEntityNumber(-id);
                            continue; // pbmap: we will find ourselves again
                        }
                        //  ...  Build the Check  ...
                        string failmess = global::Kerf.Engine.Foundation.Standard.Sprintf("Unresolved Reference, Ent.Id.#%d Param.n0 %d (Id.#%d)", ident, na, id);
                        _thecheck.AddFail(failmess, "Unresolved Reference");
                        //  ...  And output a more complete message
                        sout.Append("*** ERR StepReaderData *** Entite #").Append(ident).Append("\n    Type:").Append(RecordType(num))
                            .Append("  Param.n0 ").Append(na).Append(": #").Append(id).Append(" Not found").Append('\n');
                    } // END  Mapping
                } // END  Reference Processing
            } // END  Parameters Loop
        } // END  Directory Loop
        if (!pbmap)
        {
            return;
        }
        sout.Append(" --  2nd pass required --");
        int nbseq = _thenbents + 2 * _thenbscop;
        NCollection_Array1<int> inds = new NCollection_Array1<int>(0, nbseq); // n0 Record/Entite
        NCollection_Array1<int> indi = new NCollection_Array1<int>(0, nbseq); // Idents/scopes
        NCollection_Array1<int> indr = new NCollection_Array1<int>(0, nbdirec); // inverse of nds
        NCollection_HArray1<int> indx = null; // for EXPORT (if any)
        imap.Clear();
        bool iamap = withmap; // (default True)
        nbmap = 0;
        NCollection_Sequence<int> scopile = new NCollection_Sequence<int>(); // scope chaining noted by stack
        int nr = 0;
        for (num = 1; num <= nbdirec; num++)
        {
            int ident = _theidents[num];
            if (ident < -2)
            { // SUB-LIST (most common case)
                indr[num] = nr + 1; // search based on nr (following object)
            }
            else if (ident >= 0)
            { // Ident normal
                nr++;
                inds[nr] = num;
                indi[nr] = ident;
                indr[num] = nr;
                if (ident > 0)
                { // and not (iamap && ident > 0)
                    //  Map: if Overlap, inhibit it. Otherwise, note index
                    int indmap = imap.Add(ident) + 1;
                    if (indmap <= nbmap)
                    {
                        bool errorscope = false;
                        indmap = imap.IndexOf(ident) + 1; // plus sur
                        pbmap = true;
                        if (_thenbscop == 0)
                        {
                            errorscope = true;
                            //  Identical numbers when there is no SCOPE? ERROR!
                            //  (Of course, if there are SCOPES, we pass through, but still...)
                        }
                        else
                        {
                            //  If there are SCOPES, let's look more closely to report a problem
                            //  Error if SAME SCOPE group
                            //  ATTENTION, we search, not in all records, but in the records
                            //    CHAINED, cf nr and not num (no sub-list, scope-endscope chaining)
                            int fromscope = nr;
                            int toscope = indm[indmap];
                            if (toscope < 0)
                            {
                                toscope = -toscope;
                            }
                            for (; ; )
                            {
                                fromscope--; // basic iteration
                                if (fromscope <= toscope)
                                {
                                    errorscope = true; // BANG, we are on it
                                    break;
                                }
                                int idtest = indi[fromscope];
                                if (idtest >= 0)
                                {
                                    continue; // the next one (well, the previous one)
                                }
                                if (idtest == -1)
                                {
                                    break; // not same level, so it's OK
                                }
                                if (idtest == -3)
                                {
                                    fromscope = inds[fromscope];
                                    if (fromscope < toscope)
                                    {
                                        break; // we exit, not on same level
                                    }
                                }
                            }
                        }
                        if (errorscope)
                        {
                            //  We are inside: report it
                            string ligne = global::Kerf.Engine.Foundation.Standard.Sprintf("Ident defined SEVERAL TIMES : #%d", ident);
                            _thecheck.AddFail(ligne, "Ident defined SEVERAL TIMES : #%d");
                            sout.Append("StepReaderData : SetEntityNumbers, ").Append(ligne).Append('\n');
                        }
                        if (indm[indmap] > 0)
                        {
                            indm[indmap] = -indm[indmap]; // Not for Map
                        }
                        //  Normal case for the Map
                    }
                    else
                    {
                        nbmap = indmap;
                        indm[indmap] = nr; // Map ->(indm)->inds
                    }
                }
            }
            else if (ident == -1)
            { // SCOPE
                nr++;
                inds[nr] = num;
                indi[nr] = -1;
                indr[num] = 0;
                scopile.Append(nr);
            }
            else if (ident == -2)
            { // ENDSCOPE
                int nscop = scopile.Last(); // chainage SCOPE-ENDSCOPE
                scopile.Remove(scopile.Length());
                nr++;
                inds[nr] = nscop;
                indi[nr] = -3;
                indr[num] = 0;
                inds[nscop] = nr;
                if (NbParams(num) > 0)
                {
                    //  EXPORT : traitement special greffe sur celui de SCOPE (sans le perturber)
                    if (indx is null)
                    {
                        indx = new NCollection_HArray1<int>(0, nbseq);
                        for (int ixp = 0; ixp <= nbseq; ixp++)
                        {
                            indx.ChangeValue(ixp) = 0;
                        }
                    }
                    indx.ChangeValue(nr) = num;
                    indx.ChangeValue(nscop) = num;
                }
            }
            else if (ident == 0)
            { // HEADER
                indr[num] = 0;
            }
        }
        //  ..    EXPORT resolution, if any and if needed    ..
        //  For each EXPORT value that has not been resolved by the MAP,
        //  determine its local position by backward search from ENDSCOPE
        if ((!iamap || pbmap) && indx is not null)
        {
            for (nr = 0; nr <= nbseq; nr++)
            {
                if (indx.Value(nr) == 0 && indi[nr] != -3)
                {
                    continue; // ENDSCOPE + EXPORT
                }
                num = indx.Value(nr);
                int nba = NbParams(num);
                for (int na = 1; na <= nba; na++)
                {
                    FileParameter fp = ChangeParam(num, na);
                    if (fp.ParamType() != global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent)
                    {
                        continue;
                    }
                    int id = -fp.EntityNumber();
                    if (id < 0)
                    {
                        continue; // already resolved at head
                    }
                    //  Search for the requested Id: if nested EXPORT, already resolved but we must
                    //  look! (useless however to go see: it's already done, because
                    //  a nested EXPORT has been processed BEFORE the one that nests)
                    int n0 = nr - 1;
                    if (indi[n0] == -3)
                    {
                        n0--; // if we just follow an ENDSCOPE
                    }
                    while (n0 > 0)
                    {
                        int irec = indi[n0];
                        if (irec == id)
                        { // found
                            fp.SetEntityNumber(inds[n0]);
                            break;
                        }
                        if (irec == -1)
                        {
                            break; // SCOPE: end of this SCOPE/ENDSCOPE
                        }
                        if (irec == -3)
                        {
                            //  beware of EXPORT: if an EXPORT holds Id, note its already calculated Number
                            //  Attention: Id to be read from CValue because EntityNumber already resolved
                            int nok = FindEntityNumber(indx.Value(n0), id);
                            if (nok > 0)
                            {
                                fp.SetEntityNumber(nok);
                                break;
                            }
                            n0 = inds[n0]; // ENDSCOPE or unsuccessful EXPORT: skip it
                        } // end processing on an ENDSCOPE or EXPORT
                        n0--;
                    } // end resolution of an EXPORT Parameter
                } // end resolution of an EXPORT list
            } // end looping on EXPORTs
        }
        //  Table exploitation: looping operates on the table
        //  Sub-lists processing: done on the fly, by managing a stack
        //  based on the constitution of sub-lists
        int maxsubpil = 30; // simulated stack with an Array: very strong
        NCollection_HArray1<int> subpile = new NCollection_HArray1<int>(1, maxsubpil); // ... saves memory ...
        int nbsubpil = 0; // ... and so much faster!
        for (num = 1; num <= nbdirec; num++)
        {
            nr = indr[num];
            if (nr == 0)
            {
                continue; //    not an object or a sub-list
            }
            int nba = NbParams(num);
            for (int na = nba; na > 0; na--)
            {
                //  We read from the end: this allows processing sub-lists on the fly
                //  Otherwise, we should note that there were sub-lists and resume afterwards
                //  Reverse processing ensures that nested sub-lists are resolved before their containers
                //  This is critical for maintaining referential integrity in complex STEP structures
                FileParameter fp = ChangeParam(num, na);
                ParamType letype = fp.ParamType();
                if (letype == global::Kerf.Engine.Exchange.Framework.ParamType.ParamSub)
                {
                    //  sub-list type parameter: sub-list number read by unstacking
                    fp.SetEntityNumber(subpile.Value(nbsubpil));
                    nbsubpil--; //	subpile->Remove(nbsubpil);
                }
                else if (letype == global::Kerf.Engine.Exchange.Framework.ParamType.ParamIdent)
                {
                    //  ident type parameter (references an entity): search for requested ident
                    int id = -fp.EntityNumber();
                    if (id < 0)
                    {
                        continue; // already resolved at head
                    }
                    // Here we go: we will search for id in ndi; scanning algorithm
                    // This implements a bidirectional search strategy: first backward from current position
                    // to file beginning, then forward to file end. This optimizes for locality of references.
                    int pass, sens, nok, n0, irec;
                    pass = sens = nok = 0;
                    if (!iamap)
                    {
                        pass = 1; // if map not available
                    }
                    while (pass < 3)
                    {
                        pass++;
                        //    MAP available
                        if (pass == 1)
                        { // MAP AVAILABLE
                            int indmap = imap.IndexOf(id) + 1;
                            if (indmap > 0)
                            { // the map found it
                                nok = indm[indmap];
                                if (nok < 0)
                                {
                                    continue; // CONFLICT -> need to resolve ...
                                }
                                break;
                            }
                            else
                            {
                                continue;
                            }
                        }
                        //    1st Pass: BACKWARD -> File beginning
                        if (sens == 0 && nr > 1)
                        {
                            n0 = nr - 1;
                            if (indi[n0] == -3)
                            {
                                n0--; // if we just follow an ENDSCOPE
                            }
                            while (n0 > 0)
                            {
                                irec = indi[n0];
                                if (irec == id)
                                { // found
                                    nok = n0;
                                    break;
                                }
                                //    ENDSCOPE : Attention a EXPORT sinon sauter
                                if (irec == -3)
                                {
                                    if (indx is null)
                                    {
                                        n0 = inds[n0];
                                    }
                                    else
                                    {
                                        //    EXPORT, we must look
                                        nok = FindEntityNumber(indx.Value(n0), id);
                                        if (nok > 0)
                                        {
                                            break;
                                        }
                                        n0 = inds[n0]; // ENDSCOPE: skip it
                                    }
                                }
                                n0--;
                            }
                            //    2nd Pass: DESCENT -> End of file
                        }
                        else if (nr < nbseq)
                        { // descent -> end of file
                            n0 = nr + 1;
                            while (n0 <= nbseq)
                            {
                                irec = indi[n0];
                                if (irec == id)
                                { // found
                                    nok = n0;
                                    break;
                                }
                                //    SCOPE: Attention to EXPORT otherwise skip
                                if (irec == -1)
                                {
                                    if (indx is null)
                                    {
                                        n0 = inds[n0];
                                    }
                                    else
                                    {
                                        //    EXPORT, we must look
                                        nok = FindEntityNumber(indx.Value(n0), id);
                                        if (nok > 0)
                                        {
                                            break;
                                        }
                                        n0 = inds[n0]; // SCOPE: skip it
                                    }
                                }
                                n0++;
                            }
                        }
                        if (nok > 0)
                        {
                            break;
                        }
                        sens = 1 - sens; // next pass
                    }
                    // here we have nok, number found
                    if (nok > 0)
                    {
                        int num0 = inds[nok];
                        fp.SetEntityNumber(num0); // AND THERE, we have resolved
                        // not found: report it
                    }
                    else
                    {
                        //  Feed the Check ... For this, determine Entity n0 and Ident
                        int nument = 0;
                        int n0ent; // svv Jan11 2000 : porting on DEC
                        for (n0ent = 1; n0ent <= nr; n0ent++)
                        {
                            if (indi[n0ent] > 0)
                            {
                                nument++;
                            }
                        }
                        int ident = RecordIdent(num);
                        if (ident < 0)
                        {
                            for (n0ent = num + 1; n0ent <= nbdirec; n0ent++)
                            {
                                ident = RecordIdent(n0ent);
                                if (ident > 0)
                                {
                                    break;
                                }
                            }
                        }
                        //  ...  Build the Check  ...
                        string failmess = global::Kerf.Engine.Foundation.Standard.Sprintf("Unresolved Reference, Ent.n0 %d (Id.#%d) Param.n0 %d (Id.#%d)", nument, ident, na, id);
                        _thecheck.AddFail(failmess, "Unresolved Reference");
                        //  ...  And output a more complete message
                        sout.Append("*** ERR StepReaderData *** Entite ").Append(nument).Append(", a ").Append((nr * 100) / nbseq)
                            .Append("% de DATA : #").Append(ident).Append("\n    Type:").Append(RecordType(num)).Append("  Param.n0 ")
                            .Append(na).Append(": #").Append(id).Append(" Not found").Append('\n');
                        fp.SetEntityNumber(0); // -> Unresolved reference
                    }
                }
            }
            //  If this record is itself a sub-list, stack it!
            if (inds[nr] != num)
            {
                if (nbsubpil >= maxsubpil)
                {
                    maxsubpil = maxsubpil + 30;
                    NCollection_HArray1<int> newsubpil = new NCollection_HArray1<int>(1, maxsubpil);
                    for (int bidpil = 1; bidpil <= maxsubpil - 30; bidpil++)
                    {
                        newsubpil.SetValue(bidpil, subpile.Value(bidpil));
                    }
                    subpile = newsubpil;
                }
                nbsubpil++;
                subpile.SetValue(nbsubpil, num); // Append(num);
            }
        }
    }

    //  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##
    //  ....             Header Management: Preparation, reading             ....

    /// <summary>Determines first suitable record of Header; works as FindNextRecord, but treats only Header records.</summary>
    public int FindNextHeaderRecord(int num)
    {
        // returns, for a given record number (by num), the next one which
        // defines an entity, or 0 if finished:
        // Operates like FindNextRecord but only scans the Header
        if (num < 0)
        {
            return 0;
        }
        int num1 = num + 1;
        int max = _thenbhead;
        while (num1 <= max)
        {
            // SCOPE,ENDSCOPE and Sub-List have a negative identifier
            // Only retain positive or null Idents (null: no Ident in Header)
            if (RecordIdent(num1) >= 0)
            {
                return num1;
            }
            num1++;
        }
        return 0;
    }

    /// <summary>Works as SetEntityNumbers but for Header: more simple because there are no Reference, only Sub-Lists.</summary>
    public void PrepareHeader()
    {
        // Reference resolution: only concerns sub-lists
        //  already done by SetEntityNumbers so no need to worry
    }

    /// <summary>Returns the Global Check. It can record Fail messages about Undefined References (detected by SetEntityNumbers).</summary>
    public FrameworkCheck GlobalCheck()
    {
        return _thecheck;
    }
}
