// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepReaderData.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepReaderData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using System.Text;
using static Occt.Interface_ParamType;
using static Occt.Resource_FormatType;
using static Occt.StepData_Logical;

namespace Occt;

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
public class StepData_StepReaderData : Interface_FileReaderData
{
    // The Header consists of entities analogous in principle to those
    // of the Data, except that they are without identifier, and can neither
    // reference, nor be referenced (whether with Header or with Data)
    // Thus, in StepReaderData, the Header consists of the first "thenbhead" Entities
    // This separation allows STEP files to have metadata and structural information
    // separate from the main geometric and semantic data

    private readonly NCollection_Array1<int> theidents;
    private readonly NCollection_Array1<int> thetypes;
    private readonly NCollection_IndexedMap<string> thenametypes = new();
    private readonly NCollection_DataMap<int, int> themults = new();
    private int thenbents;
    private int thelastn;
    private readonly int thenbhead;
    private int thenbscop;
    private readonly Interface_Check thecheck;
    private readonly Resource_FormatType mySourceCodePage;

    // ----------  Fonctions Utilitaires  ----------

    /// <summary>Converts a character to its hexadecimal value; 0 if it has no representation in this system.</summary>
    private static int convertCharacterTo16bit(char theCharacter)
    {
        switch (theCharacter)
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
    private static char ExtValue(StringBuilder theStr, int theWhere)
    {
        if (theWhere > 0 && theWhere <= theStr.Length)
        {
            return theStr[theWhere - 1];
        }
        throw new OutOfRangeException("TCollection_ExtendedString::Value : parameter theWhere");
    }

    private static void ExtSetValue(StringBuilder theStr, int theWhere, char theWhat)
    {
        if (theWhere > 0 && theWhere <= theStr.Length)
        {
            theStr[theWhere - 1] = theWhat;
            return;
        }
        throw new OutOfRangeException("TCollection_ExtendedString::SetValue : parameter theWhere");
    }

    private static void ExtSetValue(StringBuilder theStr, int theWhere, string theWhat)
    {
        if (theWhere < 1 || theWhere > theStr.Length + 1)
        {
            throw new OutOfRangeException("TCollection_ExtendedString::SetValue : parameter theWhere");
        }
        if (theWhat.Length <= 0)
        {
            return;
        }
        int aNewEnd = theWhere - 1 + theWhat.Length;
        if (aNewEnd > theStr.Length)
        {
            theStr.Length = aNewEnd;
        }
        for (int i = 0; i < theWhat.Length; i++)
        {
            theStr[theWhere - 1 + i] = theWhat[i];
        }
    }

    // aResString.ToExtString() + theFrom: the rest of the text from a 0-based index, as a C string (up to its first
    // zero char), copied as C++'s temporary TCollection_ExtendedString is
    private static string ExtTail(StringBuilder theStr, int theFrom)
    {
        int anEnd = theFrom;
        while (anEnd < theStr.Length && theStr[anEnd] != '\0')
        {
            anEnd++;
        }
        return theFrom >= anEnd ? "" : theStr.ToString(theFrom, anEnd - theFrom);
    }

    private static void ExtTrunc(StringBuilder theStr, int theHowMany)
    {
        if (theHowMany < 0 || theHowMany > theStr.Length)
        {
            throw new OutOfRangeException("TCollection_ExtendedString::Trunc : parameter 'theHowMany'");
        }
        theStr.Length = theHowMany;
    }

    // TCollection_ExtendedString::AssignCat(char16_t): a zero char is not added
    private static void ExtAssignCat(StringBuilder theStr, char theChar)
    {
        if (theChar != '\0')
        {
            theStr.Append(theChar);
        }
    }

    /// <summary>
    /// Prepares a string to use in OCCT exchange structure. If code page is Resource_FormatType_NoConversion, cleans
    /// only special characters without conversion; else converts the string using the code page and handles the
    /// control directives.
    /// </summary>
    private void cleanText(TCollection_HAsciiString theVal)
    {
        if (theVal.Length() == 2)
        {
            theVal.Clear();
            return;
        }
        StringBuilder aResString = new StringBuilder();
        bool toConversion = mySourceCodePage != Resource_FormatType.Resource_FormatType_NoConversion;
        Resource_Unicode.ConvertFormatToUnicode(mySourceCodePage, theVal.ToCString().Substring(1), aResString);
        int aResStringSize = aResString.Length - 1; // skip the last apostrophe
        StringBuilder aTempExtString = new StringBuilder(); // string for characters within control directives
        int aSetCharInd = 1; // index to set value to result string
        Resource_FormatType aLocalFormatType = Resource_FormatType.Resource_FormatType_iso8859_1; // a code page for a "\S\" control directive
        for (int aStringInd = 1; aStringInd <= aResStringSize; ++aStringInd)
        {
            char aChar = ExtValue(aResString, aStringInd);
            aSetCharInd = aStringInd;
            if (aChar == '\\' && aStringInd <= aResStringSize - 3) // can contains the control directive
            {
                bool isConverted = false;
                char aDirChar = ExtValue(aResString, aStringInd + 1);
                bool isSecSlash = ExtValue(aResString, aStringInd + 2) == '\\';
                bool isThirdSlash = ExtValue(aResString, aStringInd + 3) == '\\';
                // Encoding ISO 8859 characters within a string;
                // ("\P{N}\") control directive;
                // indicates code page for ("\S\") control directive;
                // {N}: "A", "B", "C", "D", "E", "F", "G", "H", "I";
                // "A" identifies ISO 8859-1; "B" identifies ISO 8859-2, etc.
                if (aDirChar == 'P' && isThirdSlash)
                {
                    char aPageId = (char)(ExtValue(aResString, aStringInd + 2) & 255);
                    if (aPageId >= 'a' && aPageId <= 'z')
                    {
                        aPageId = (char)(aPageId - 'a' + 'A'); // C's toupper (the "C" locale)
                    }
                    if (aPageId >= 'A' && aPageId <= 'I')
                    {
                        aLocalFormatType = (Resource_FormatType)((int)Resource_FormatType.Resource_FormatType_iso8859_1 + (aPageId - 'A'));
                    }
                    else
                    {
                        thecheck.AddWarning("String control directive \\P*\\ with an unsupported symbol in place of *");
                    }
                    isConverted = true;
                    aStringInd += 3;
                }
                // Encoding ISO 8859 characters within a string;
                // ("\S\") control directive;
                // converts followed a LATIN CODEPOINT character.
                else if (aDirChar == 'S' && isSecSlash)
                {
                    char aResChar = (char)((ExtValue(aResString, aStringInd + 3) | 0x80) & 0xFF);
                    string aStrForCovert = aResChar.ToString();
                    Resource_Unicode.ConvertFormatToUnicode(aLocalFormatType, aStrForCovert, aTempExtString);
                    isConverted = true;
                    aStringInd += 3;
                }
                // Encoding U+0000 to U+00FF in a string
                // ("\X\") control directive;
                // converts followed two hexadecimal character.
                else if (aDirChar == 'X' && aStringInd <= aResStringSize - 4 && isSecSlash)
                {
                    int aResChar = convertCharacterTo16bit(ExtValue(aResString, aStringInd + 3));
                    aResChar = ((aResChar << 4) | convertCharacterTo16bit(ExtValue(aResString, aStringInd + 4))) & 0xFF;
                    // pass through without conversion: TCollection_ExtendedString(aStrForConvert, false)
                    aTempExtString.Clear();
                    if (aResChar != 0)
                    {
                        aTempExtString.Append((char)aResChar);
                    }
                    isConverted = true;
                    aStringInd += 4;
                }
                // Encoding ISO 10646 characters within a string
                // ("\X{N}\") control directive;
                // {N}: "0", "2", "4";
                // "\X2\" or "\X4\" converts followed a hexadecimal character sequence;
                // "\X0\" indicate the end of the "\X2\" or "\X4\".
                else if (aDirChar == 'X' && isThirdSlash)
                {
                    int aFirstInd = aStringInd + 3;
                    int aLastInd = aStringInd;
                    bool isClosed = false;
                    // find the end of the "\X2\" or "\X4\" by an external "aStringInd"
                    for (; aStringInd <= aResStringSize && !isClosed; ++aStringInd)
                    {
                        if (aResStringSize - aStringInd > 2 && ExtValue(aResString, aStringInd) == '\\'
                            && ExtValue(aResString, aStringInd + 1) == 'X' && ExtValue(aResString, aStringInd + 2) == '0'
                            && ExtValue(aResString, aStringInd + 3) == '\\')
                        {
                            aLastInd = aStringInd - 1;
                            aStringInd = aStringInd + 2;
                            isClosed = true;
                        }
                    }
                    if (!isClosed) // "\X0\" not exists
                    {
                        aLastInd = aStringInd = aResStringSize;
                    }
                    int aStrLen = aLastInd - aFirstInd;
                    // "\X2\" control directive;
                    // followed by multiples of four or three hexadecimal characters.
                    // Encoding in UTF-16
                    if (ExtValue(aResString, aFirstInd - 1) == '2' && aResStringSize - aFirstInd > 3)
                    {
                        int anIterStep = (aStrLen % 4 == 0) ? 4 : 3;
                        if (aStrLen % anIterStep != 0)
                        {
                            ExtAssignCat(aTempExtString, '?');
                            thecheck.AddWarning("String control directive \\X2\\ is followed by number of digits not multiple of 4");
                        }
                        else
                        {
                            int aUtfCharacter = '\0';
                            for (int aCharInd = 1; aCharInd <= aStrLen; ++aCharInd)
                            {
                                aUtfCharacter |= convertCharacterTo16bit(ExtValue(aResString, aCharInd + aFirstInd));
                                if (aCharInd % anIterStep == 0)
                                {
                                    ExtAssignCat(aTempExtString, (char)aUtfCharacter);
                                    aUtfCharacter = '\0';
                                }
                                aUtfCharacter = (aUtfCharacter << 4) & 0xFFFF; // a char16_t
                            }
                        }
                    }
                    // "\X4\" control directive;
                    // followed by multiples of eight hexadecimal characters.
                    // Encoding in UTF-32
                    else if (ExtValue(aResString, aFirstInd - 1) == '4' && aResStringSize - aFirstInd > 7)
                    {
                        if (aStrLen % 8 != 0)
                        {
                            ExtAssignCat(aTempExtString, '?');
                            thecheck.AddWarning("String control directive \\X4\\ is followed by number of digits not multiple of 8");
                        }
                        else
                        {
                            uint aUtfCharacter = '\0';
                            for (int aCharInd = 1; aCharInd <= aStrLen; ++aCharInd)
                            {
                                aUtfCharacter |= (uint)convertCharacterTo16bit(ExtValue(aResString, aCharInd + aFirstInd));
                                if (aCharInd % 8 == 0)
                                {
                                    // NCollection_UtfIterator<char32_t> on the one character, written as UTF-16 then
                                    // appended as a C string (it ends at a zero character)
                                    var aUtfString = new StringBuilder();
                                    if (aUtfCharacter != 0)
                                    {
                                        Resource_Unicode.AppendUtf16(aUtfCharacter, aUtfString);
                                    }
                                    aTempExtString.Append(aUtfString);
                                    aUtfCharacter = '\0';
                                }
                                aUtfCharacter = aUtfCharacter << 4;
                            }
                        }
                    }
                    isConverted = true;
                }
                if (isConverted) // find the control directive
                {
                    if (toConversion) // else skip moving
                    {
                        aResStringSize -= aStringInd - aSetCharInd - aTempExtString.Length + 1; // change the string size to remove unused symbols
                        ExtSetValue(aResString, aSetCharInd, aTempExtString.ToString());
                        aSetCharInd += aTempExtString.Length; // move to the new position
                        ExtSetValue(aResString, aSetCharInd, ExtTail(aResString, aStringInd));
                        aStringInd = aSetCharInd - 1;
                        ExtTrunc(aResString, aResStringSize);
                    }
                    aTempExtString.Clear();
                    continue;
                }
            }
            if (aStringInd <= aResStringSize - 1)
            {
                char aCharNext = ExtValue(aResString, aStringInd + 1);
                if (aCharNext == aChar && (aChar == '\'' || aChar == '\\'))
                {
                    ExtSetValue(aResString, aSetCharInd, ExtTail(aResString, aStringInd)); // move the string,removing one symbol
                    aResStringSize--; // change the string size to remove unused symbol
                    ExtTrunc(aResString, aResStringSize);
                }
                else if (aChar == '\\')
                {
                    bool isDirective = aStringInd <= aResStringSize - 2 && ExtValue(aResString, aStringInd + 2) == '\\';
                    if (isDirective)
                    {
                        if (aCharNext == 'N')
                        {
                            ExtSetValue(aResString, aSetCharInd++, '\n');
                            ExtSetValue(aResString, aSetCharInd, ExtTail(aResString, aStringInd + 2)); // move the string,removing two symbols
                            aResStringSize -= 2; // change the string size to remove unused symbols
                            ExtTrunc(aResString, aResStringSize);
                            continue;
                        }
                        else if (aCharNext == 'T')
                        {
                            ExtSetValue(aResString, aSetCharInd++, '\t');
                            ExtSetValue(aResString, aSetCharInd, ExtTail(aResString, aStringInd + 2)); // move the string,removing two symbols
                            aResStringSize -= 2; // change the string size to remove unused symbols
                            ExtTrunc(aResString, aResStringSize);
                            continue;
                        }
                    }
                }
            }
            if (aChar == '\n' || aChar == '\r')
            {
                ExtSetValue(aResString, aSetCharInd, ExtTail(aResString, aStringInd));
                aResStringSize--;
                ExtTrunc(aResString, aResStringSize);
                aStringInd--;
            }
        }
        theVal.Clear();
        ExtTrunc(aResString, aResStringSize); // trunc the last apostrophe
        theVal.AssignCat(Resource_Unicode.ToUtf8Equivalent(aResString));
    }

    //  -------------  METHODES  -------------

    /// <summary>
    /// Creates StepReaderData correctly dimensioned (necessary at creation time, because it contains arrays):
    /// nbheader is nb of records for Header, nbtotal for Header+Data and nbpar gives the total count of parameters.
    /// </summary>
    public StepData_StepReaderData(int nbheader, int nbtotal, int nbpar, Resource_FormatType theSourceCodePage = Resource_FormatType.Resource_FormatType_UTF8)
        : base(nbtotal, nbpar)
    {
        theidents = new NCollection_Array1<int>(1, nbtotal);
        thetypes = new NCollection_Array1<int>(1, nbtotal);
        mySourceCodePage = theSourceCodePage;
        thenbscop = 0;
        thenbents = 0;
        thelastn = 0;
        thenbhead = nbheader;
        thecheck = new Interface_Check();
    }

    /// <summary>Fills the fields of a record.</summary>
    public void SetRecord(int num, string ident, string type, int nbpar)
    {
        int numlst;
        type ??= ""; // OCCT.NET: a record whose type the parser never set (decision 1178)
        ident ??= "";
        if (global::Kerf.Engine.Foundation.Standard.CStringChar(type, 0) != '(')
        {
            thenbents++; // total number of proper file terms
        }
        thetypes.ChangeValue(num) = thenametypes.Add(type);
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
            if (thelastn < numlst)
            {
                thelastn = numlst; // highest sub-list number
            }
            theidents.SetValue(num, -2 - numlst);
        }
        else if (global::Kerf.Engine.Foundation.Standard.CStringChar(ident, 0) == '#')
        {
            numlst = global::Kerf.Engine.Foundation.Standard.Atoi(ident.Substring(1));
            theidents.SetValue(num, numlst);
            if (numlst == 0 && num > thenbhead)
            {
                //    Header, or Complex Type ...
                //    If Complex Type, find Previous Type (we consider this is rare)
                //    Chain the previous type to the next one
                //    VERIFICATION that types are in alphabetical order
                for (int prev = num - 1; prev > thenbhead; prev--)
                {
                    if (theidents[prev] >= 0)
                    {
                        themults.Bind(prev, num);
                        if (string.CompareOrdinal(thenametypes.FindKey(thetypes.Value(num)), thenametypes.FindKey(thetypes.Value(prev))) < 0)
                        {
                            //  Warning: components in complex entity are not in alphabetical order.
                            var errm = new StringBuilder("Complex Type incorrect : ");
                            errm.Append(thenametypes.FindKey(thetypes.Value(prev)));
                            errm.Append(" / ");
                            errm.Append(thenametypes.FindKey(thetypes.Value(num)));
                            errm.Append(" ... ");
                            while (theidents[prev] <= 0)
                            {
                                prev--;
                                if (prev <= 0)
                                {
                                    break;
                                }
                            }
                            using (Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendTrace())
                            {
                                sout.Append("  ***  Incorrect record ").Append(num).Append(" (on ").Append(NbRecords()).Append(" -> ")
                                    .Append(num * 100 / NbRecords()).Append(" % in File)  ***");
                                if (prev > 0)
                                {
                                    sout.Append("  Ident #").Append(theidents[prev]);
                                }
                                sout.Append("\n").Append(errm.ToString()).Append('\n');
                            }
                            thecheck.AddWarning(errm.ToString(), "Complex Type incorrect : ");
                        }
                        break;
                    }
                }
            }
        }
        else if (ident == "SCOPE")
        {
            theidents.SetValue(num, -1); // SCOPE
            thenbscop++;
        }
        else if (ident == "ENDSCOPE")
        {
            theidents.SetValue(num, -2); // ENDSCOPE
        }
        //      Reste 0
    }

    /// <summary>Fills the fields of a parameter of a record. This is a variant of AddParam, adapted to STEP (optimized for specific values).</summary>
    public void AddStepParam(int num, string aval, Interface_ParamType atype, int nument = 0)
    {
        if (atype == Interface_ParamType.Interface_ParamSub)
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
        else if (atype == Interface_ParamType.Interface_ParamIdent)
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
        return thenametypes.FindKey(thetypes.Value(num));
    }

    /// <summary>Returns Record Type as a CString.</summary>
    public string CType(int num)
    {
        return thenametypes.FindKey(thetypes.Value(num));
    }

    /// <summary>Returns record identifier (Positive number). If returned ident is not positive: Sub-List or Scope mark.</summary>
    public int RecordIdent(int num)
    {
        return theidents[num];
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
        Interface_FileParameter FP = Param(num, nump);
        if (FP.ParamType() != Interface_ParamType.Interface_ParamSub)
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
        return FP.EntityNumber();
    }

    /// <summary>Returns True if num corresponds to a Complex Type Entity (as can be defined by ANDOR Express clause).</summary>
    public bool IsComplex(int num)
    {
        return themults.IsBound(num);
    }

    /// <summary>
    /// Returns the List of Types which correspond to a Complex Type Entity. If not Complex, there is just one Type in
    /// it. For a SubList or a Scope mark, types remains empty.
    /// </summary>
    public void ComplexType(int num, NCollection_Sequence<string> types)
    {
        if (theidents[num] < 0)
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
        if (themults.IsBound(num))
        {
            next = themults.Find(num);
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
    public bool NamedForComplex(string name, int num0, ref int num, Interface_Check ach)
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
    public bool NamedForComplex(string theName, string theShortName, int num0, ref int num, Interface_Check ach)
    {
        int n = (num <= 0 ? num0 : NextForComplex(num));
        if ((n != 0) && (RecordType(n) == theName || RecordType(n) == theShortName))
        {
            num = n;
            return true;
        }
        // entities are not in alphabetical order
        string txtmes;
        string errmess = "Parameter n0.%d (%s) not a LIST";
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, theName);
        for (n = num0; n > 0; n = NextForComplex(n))
        {
            if (RecordType(n) == theName || RecordType(n) == theShortName)
            {
                num = n;
                errmess = "Complex Record n0.%d, member type %s not in alphabetic order";
                txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, theName);
                ach.AddWarning(txtmes, errmess);
                return false;
            }
        }
        num = 0;
        errmess = "Complex Record n0.%d, member type %s not found";
        txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(errmess, num0, theName);
        ach.AddFail(txtmes, errmess);
        return false;
    }

    //  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##  ##

    /// <summary>
    /// Checks Count of Parameters of record num to equate nbreq. If this Check is successful, returns True. Else,
    /// fills ach with an Error Message then returns False; mess is included in the Error message if given non empty.
    /// </summary>
    public bool CheckNbParams(int num, int nbreq, Interface_Check ach, string mess = "")
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
    public bool ReadSubList(int num, int nump, string mess, Interface_Check ach, ref int numsub, bool optional = false, int lenmin = 0, int lenmax = 0)
    {
        string txtmes;
        numsub = SubListNumber(num, nump, false);
        if (numsub > 0)
        {
            int aNbParams = NbParams(numsub);
            if (aNbParams == 0)
            {
                string anErrMess = "Parameter n0.%d (%s) is an empty LIST";
                txtmes = global::Kerf.Engine.Foundation.Standard.Sprintf(anErrMess, nump, mess);
                ach.AddWarning(txtmes, anErrMess);
            }
            return true;
        }
        //  Si optionel indefini, on passe l eponge
        numsub = 0;
        bool isvoid = (Param(num, nump).ParamType() == Interface_ParamType.Interface_ParamVoid);
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
    public int ReadSub(int numsub, string mess, Interface_Check ach, StepData_PDescr? descr, ref object? val)
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
            StepData_SelectNamed sn = new StepData_SelectNamed();
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
        Interface_FileParameter FP0 = Param(numsub, 1);
        Interface_ParamType FT, FT0 = FP0.ParamType();
        string str = FP0.CValue();
        NCollection_HArray1<Transient> htr = null;
        NCollection_HArray1<int> hin = null;
        NCollection_HArray1<double> hre = null;
        NCollection_HArray1<TCollection_HAsciiString> hst = null;
        int kod = 0;
        switch (FT0)
        {
            case Interface_ParamType.Interface_ParamMisc:
                return -1;
            case Interface_ParamType.Interface_ParamInteger:
                kod = 1;
                break;
            case Interface_ParamType.Interface_ParamReal:
                kod = 5;
                break;
            case Interface_ParamType.Interface_ParamIdent:
                kod = 7;
                break;
            case Interface_ParamType.Interface_ParamVoid:
                kod = 0;
                break;
            case Interface_ParamType.Interface_ParamText:
                kod = 6;
                break;
            case Interface_ParamType.Interface_ParamEnum:
                kod = 4;
                break; // a confirmer(logical)
            case Interface_ParamType.Interface_ParamLogical:
                return -1;
            case Interface_ParamType.Interface_ParamSub:
                kod = 0;
                break;
            case Interface_ParamType.Interface_ParamHexa:
                return -1;
            case Interface_ParamType.Interface_ParamBinary:
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
            Interface_FileParameter FP = Param(numsub, ip);
            str = FP.CValue();
            FT = FP.ParamType();
            switch (kod)
            {
                case 1:
                {
                    if (FT != Interface_ParamType.Interface_ParamInteger)
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
                    if (FT != Interface_ParamType.Interface_ParamEnum)
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
                    if (FT != Interface_ParamType.Interface_ParamEnum)
                    {
                        kod = 0;
                        break;
                    }
                    StepData_SelectNamed sn = new StepData_SelectNamed();
                    sn.SetEnum(-1, str);
                    htr.SetValue(ip, sn);
                    break;
                }
                case 5:
                {
                    if (FT != Interface_ParamType.Interface_ParamReal)
                    {
                        kod = 0;
                        break;
                    }
                    hre.SetValue(ip, Fastof(str));
                    break;
                }
                case 6:
                {
                    if (FT != Interface_ParamType.Interface_ParamText)
                    {
                        kod = 0;
                        break;
                    }
                    TCollection_HAsciiString txt = new TCollection_HAsciiString(str);
                    cleanText(txt);
                    hst.SetValue(ip, txt);
                    break;
                }
                case 7:
                {
                    Transient ent = BoundEntity(FP.EntityNumber());
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
                        StepData_SelectInt sin = new StepData_SelectInt();
                        sin.SetInt(hin.Value(jp));
                        htr.SetValue(jp, sin);
                    }
                }
                if (hre is not null)
                {
                    for (jp = 1; jp < ip; jp++)
                    {
                        StepData_SelectReal sre = new StepData_SelectReal();
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
            switch (FT)
            {
                case Interface_ParamType.Interface_ParamMisc:
                    break;
                case Interface_ParamType.Interface_ParamInteger:
                {
                    StepData_SelectInt sin = new StepData_SelectInt();
                    sin.SetInteger(global::Kerf.Engine.Foundation.Standard.Atoi(str));
                    htr.SetValue(ip, sin);
                    break;
                }
                case Interface_ParamType.Interface_ParamReal:
                {
                    StepData_SelectReal sre = new StepData_SelectReal();
                    sre.SetReal(Fastof(str));
                    break;
                    // htr->SetValue (ip,sre); break; svv #2: unreachable
                }
                case Interface_ParamType.Interface_ParamIdent:
                    htr.SetValue(ip, BoundEntity(FP.EntityNumber()));
                    break;
                case Interface_ParamType.Interface_ParamVoid:
                    break;
                case Interface_ParamType.Interface_ParamEnum:
                {
                    StepData_SelectInt sin;
                    StepData_SelectNamed sna;
                    int logic = -1;
                    // PTV 16.09.2000
                    // set the default value of StepData_Logical
                    StepData_Logical slog = StepData_Logical.StepData_LUnknown;
                    if (global::Kerf.Engine.Foundation.Standard.CStringChar(str, 0) == '.' && global::Kerf.Engine.Foundation.Standard.CStringChar(str, 2) == '.' && global::Kerf.Engine.Foundation.Standard.CStringChar(str, 3) == '\0')
                    {
                        if (str[1] == 'F')
                        {
                            slog = StepData_Logical.StepData_LFalse;
                            logic = 0;
                        }
                        else if (str[1] == 'T')
                        {
                            slog = StepData_Logical.StepData_LTrue;
                            logic = 1;
                        }
                        else if (str[1] == 'U')
                        {
                            slog = StepData_Logical.StepData_LUnknown;
                            logic = 2;
                        }
                    }
                    if (logic >= 0)
                    {
                        sin = new StepData_SelectInt();
                        sin.SetLogical(slog);
                        htr.SetValue(ip, sin);
                    }
                    else
                    {
                        sna = new StepData_SelectNamed();
                        sna.SetEnum(logic, str);
                        htr.SetValue(ip, sna);
                    }
                    break;
                }
                case Interface_ParamType.Interface_ParamLogical:
                    break;
                case Interface_ParamType.Interface_ParamText:
                {
                    TCollection_HAsciiString txt = new TCollection_HAsciiString(str);
                    cleanText(txt);
                    htr.SetValue(ip, txt);
                    break;
                }
                case Interface_ParamType.Interface_ParamSub:
                {
                    object sub = null;
                    int nent = FP.EntityNumber();
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
                case Interface_ParamType.Interface_ParamHexa:
                    break;
                case Interface_ParamType.Interface_ParamBinary:
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
    public bool ReadMember(int num, int nump, string mess, Interface_Check ach, ref StepData_SelectMember? val)
    {
        Transient v = val;
        StepData_PDescr nuldescr = null;
        if (v is null)
        {
            return ReadAny(num, nump, mess, ach, nuldescr, ref v) && (val = v as StepData_SelectMember) is not null;
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
    public bool ReadMember<T>(int num, int nump, string mess, Interface_Check ach, ref T? val) where T : StepData_SelectMember
    {
        StepData_SelectMember aVal = val;
        return ReadMember(num, nump, mess, ach, ref aVal) && (val = aVal as T) is not null;
    }

    /// <summary>
    /// Reads parameter nump of record num into a Field, controlled by a Parameter Descriptor (PDescr), which controls
    /// its allowed type(s) and value. ach is filled if the read parameter does not match its description (but the
    /// field is read anyway). If the description is not defined, no control is done. Returns True when done.
    /// </summary>
    public bool ReadField(int num, int nump, string mess, Interface_Check ach, StepData_PDescr? descr, StepData_Field fild)
    {
        Interface_FileParameter FP = Param(num, nump);
        string str = FP.CValue();
        bool OK = true;
        int nent, kind;
        object sub = null;
        Interface_ParamType FT = FP.ParamType();
        switch (FT)
        {
            case Interface_ParamType.Interface_ParamMisc:
                OK = false;
                break;
            case Interface_ParamType.Interface_ParamInteger:
                fild.SetInteger(global::Kerf.Engine.Foundation.Standard.Atoi(str));
                break;
            case Interface_ParamType.Interface_ParamReal:
                fild.SetReal(Fastof(str));
                break;
            case Interface_ParamType.Interface_ParamIdent:
                nent = FP.EntityNumber();
                if (nent > 0)
                {
                    fild.SetEntity(BoundEntity(nent));
                }
                break;
            case Interface_ParamType.Interface_ParamVoid:
                break;
            case Interface_ParamType.Interface_ParamText:
            {
                TCollection_HAsciiString txt = new TCollection_HAsciiString(str);
                cleanText(txt);
                fild.Set(txt);
                break;
            }
            case Interface_ParamType.Interface_ParamEnum:
                if (str == ".T.")
                {
                    fild.SetLogical(StepData_Logical.StepData_LTrue);
                }
                else if (str == ".F.")
                {
                    fild.SetLogical(StepData_Logical.StepData_LFalse);
                }
                else if (str == ".U.")
                {
                    fild.SetLogical(StepData_Logical.StepData_LUnknown);
                }
                else
                {
                    fild.SetEnum(-1, str);
                }
                break;
            case Interface_ParamType.Interface_ParamLogical:
                OK = false;
                break;
            case Interface_ParamType.Interface_ParamSub:
                nent = FP.EntityNumber();
                kind = ReadSub(nent, mess, ach, descr, ref sub);
                if (kind < 0)
                {
                    break;
                }
                fild.Clear(kind);
                fild.Set(sub);
                break;
            case Interface_ParamType.Interface_ParamHexa:
                OK = false;
                break;
            case Interface_ParamType.Interface_ParamBinary:
                OK = false;
                break;
            default:
                OK = false;
                break;
        }
        if (!OK)
        {
            if (str == "*")
            {
                fild.SetDerived();
            }
        }
        return true;
    }

    /// <summary>Reads a list of fields controlled by an ESDescr.</summary>
    public bool ReadList(int num, Interface_Check ach, StepData_ESDescr descr, StepData_FieldList list)
    {
        // controler nbs egaux
        int i, nb = list.NbFields();
        if (!CheckNbParams(num, nb, ach, descr.TypeName()))
        {
            return false;
        }
        for (i = 1; i <= nb; i++)
        {
            StepData_PDescr pde = descr.Field(i);
            StepData_Field fild = list.CField(i);
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
    public bool ReadAny(int num, int nump, string mess, Interface_Check ach, StepData_PDescr? descr, ref Transient? val)
    {
        Interface_FileParameter FP = Param(num, nump);
        string str = FP.CValue();
        Interface_ParamType FT = FP.ParamType();
        //    Now, let's go: read the field and put it in place
        switch (FT)
        {
            case Interface_ParamType.Interface_ParamMisc:
                break;
            case Interface_ParamType.Interface_ParamInteger:
            {
                if (val is not null)
                {
                    StepData_SelectMember sm = val as StepData_SelectMember;
                    sm.SetReal(Fastof(str)); // OCCT: an integer is set as a real into a given member
                    return true;
                }
                StepData_SelectInt sin = new StepData_SelectInt();
                sin.SetInteger(global::Kerf.Engine.Foundation.Standard.Atoi(str));
                val = sin;
                return true;
            }
            case Interface_ParamType.Interface_ParamReal:
            {
                if (val is not null)
                {
                    StepData_SelectMember sm = val as StepData_SelectMember;
                    sm.SetReal(Fastof(str));
                    return true;
                }
                StepData_SelectReal sre = new StepData_SelectReal();
                sre.SetReal(Fastof(str));
                val = sre;
                return true;
            }
            case Interface_ParamType.Interface_ParamIdent:
            {
                int nent = FP.EntityNumber();
                if (nent > 0)
                {
                    val = BoundEntity(nent);
                }
                return (val is not null);
            }
            case Interface_ParamType.Interface_ParamVoid:
                break;
            case Interface_ParamType.Interface_ParamEnum:
            {
                StepData_SelectMember sm = null;
                if (val is not null)
                {
                    sm = val as StepData_SelectMember;
                }
                StepData_SelectInt sin;
                StepData_SelectNamed sna;
                int logic = -1;
                // PTV 16.09.2000
                // set the default value of StepData_Logical
                StepData_Logical slog = StepData_Logical.StepData_LUnknown;
                if (global::Kerf.Engine.Foundation.Standard.CStringChar(str, 0) == '.' && global::Kerf.Engine.Foundation.Standard.CStringChar(str, 2) == '.' && global::Kerf.Engine.Foundation.Standard.CStringChar(str, 3) == '\0')
                {
                    if (str[1] == 'F')
                    {
                        slog = StepData_Logical.StepData_LFalse;
                        logic = 0;
                    }
                    else if (str[1] == 'T')
                    {
                        slog = StepData_Logical.StepData_LTrue;
                        logic = 1;
                    }
                    else if (str[1] == 'U')
                    {
                        slog = StepData_Logical.StepData_LUnknown;
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
                        sin = new StepData_SelectInt();
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
                        sna = new StepData_SelectNamed();
                        val = sna; // Named sans nom...
                        sna.SetEnum(logic, str);
                    }
                } // -> Select general
                return true;
            }
            case Interface_ParamType.Interface_ParamLogical:
                break;
            case Interface_ParamType.Interface_ParamText:
            {
                TCollection_HAsciiString txt = new TCollection_HAsciiString(str);
                cleanText(txt);
                // PDN May 2000: for reading SOURCE_ITEM (external references)
                if (val is not null)
                {
                    StepData_SelectMember sm = val as StepData_SelectMember;
                    sm.SetString(txt.ToCString());
                    return true;
                }
                val = txt;
                return true;
            }
            case Interface_ParamType.Interface_ParamSub:
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
                    if (val is StepData_SelectArrReal sma)
                    {
                        int numsub2 = SubListNumber(numsub, 1, false);
                        int nbp2 = NbParams(numsub2);
                        if (nbp2 > 1)
                        {
                            if (Param(numsub2, 1).ParamType() == Interface_ParamType.Interface_ParamReal)
                            {
                                if (!sma.SetName(rectyp))
                                {
                                    return false;
                                }
                                NCollection_HSequence<double> aSeq = new NCollection_HSequence<double>();
                                for (int i = 1; i <= nbp2; i++)
                                {
                                    if (Param(numsub2, i).ParamType() != Interface_ParamType.Interface_ParamReal)
                                    {
                                        continue;
                                    }
                                    Transient asr = new StepData_SelectReal();
                                    if (!ReadAny(numsub2, i, mess, ach, descr, ref asr))
                                    {
                                        continue;
                                    }
                                    if (asr is StepData_SelectReal sm1)
                                    {
                                        aSeq.Append(sm1.Real());
                                    }
                                }
                                NCollection_HArray1<double> anArr = new NCollection_HArray1<double>(1, aSeq.Length());
                                for (int nr = 1; nr <= aSeq.Length(); nr++)
                                {
                                    anArr.SetValue(nr, aSeq.Value(nr));
                                }
                                sma.SetArrReal(anArr);
                                return true;
                            }
                        }
                    }
                    StepData_SelectMember sm = val as StepData_SelectMember;
                    if (sm is null)
                    {
                        sm = new StepData_SelectNamed();
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
    public bool ReadXY(int num, int nump, string mess, Interface_Check ach, ref double X, ref double Y)
    {
        string errmess = null; // Null if no error
        int numsub = SubListNumber(num, nump, false);
        if (numsub != 0)
        {
            if (NbParams(numsub) == 2)
            {
                Interface_FileParameter FPX = Param(numsub, 1);
                if (FPX.ParamType() == Interface_ParamType.Interface_ParamReal)
                {
                    X = Fastof(FPX.CValue());
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : (X,Y) X not a Real";
                }
                Interface_FileParameter FPY = Param(numsub, 2);
                if (FPY.ParamType() == Interface_ParamType.Interface_ParamReal)
                {
                    Y = Fastof(FPY.CValue());
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
    public bool ReadXYZ(int num, int nump, string mess, Interface_Check ach, ref double X, ref double Y, ref double Z)
    {
        string errmess = null; // Null if no error
        int numsub = SubListNumber(num, nump, false);
        if (numsub != 0)
        {
            if (NbParams(numsub) == 3)
            {
                Interface_FileParameter FPX = Param(numsub, 1);
                if (FPX.ParamType() == Interface_ParamType.Interface_ParamReal)
                {
                    X = Fastof(FPX.CValue());
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : (X,Y,Z) X not a Real";
                }
                Interface_FileParameter FPY = Param(numsub, 2);
                if (FPY.ParamType() == Interface_ParamType.Interface_ParamReal)
                {
                    Y = Fastof(FPY.CValue());
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : (X,Y,Z) Y not a Real";
                }
                Interface_FileParameter FPZ = Param(numsub, 3);
                if (FPZ.ParamType() == Interface_ParamType.Interface_ParamReal)
                {
                    Z = Fastof(FPZ.CValue());
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
    private static bool Report(string errmess, int nump, string mess, Interface_Check ach)
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
    public bool ReadReal(int num, int nump, string mess, Interface_Check ach, ref double val)
    {
        string errmess = null; // Null si pas d erreur
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            if (FP.ParamType() == Interface_ParamType.Interface_ParamReal || FP.ParamType() == Interface_ParamType.Interface_ParamInteger)
            {
                val = Fastof(FP.CValue());
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
    public bool ReadEntity(int num, int nump, string mess, Interface_Check ach, Type atype, ref Transient? ent)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            int nent = FP.EntityNumber();
            if (FP.ParamType() == Interface_ParamType.Interface_ParamIdent)
            {
                if (nent > 0)
                {
                    Transient entent = BoundEntity(nent);
                    if (entent is null || !atype.IsInstanceOfType(entent))
                    {
                        errmess = "Parameter n0.%d (%s) : Entity has illegal type";
                        if (entent is StepData_UndefinedEntity)
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
    public bool ReadEntity<T>(int num, int nump, string mess, Interface_Check ach, Type atype, ref T? ent) where T : Transient
    {
        Transient anEnt = ent;
        return ReadEntity(num, nump, mess, ach, atype, ref anEnt) && (ent = anEnt as T) is not null;
    }

    /// <summary>Same as above, but a SelectType checks Type Matching, and records the read Entity (see method Value from SelectType).</summary>
    public bool ReadEntity(int num, int nump, string mess, Interface_Check ach, StepData_SelectType sel)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            int nent = FP.EntityNumber();
            if (FP.ParamType() == Interface_ParamType.Interface_ParamIdent)
            {
                if (nent > 0)
                {
                    Transient entent = BoundEntity(nent);
                    if (!sel.Matches(entent))
                    {
                        errmess = "Parameter n0.%d (%s) : Entity has illegal type";
                        // fot not supported STEP entity
                        if (entent is StepData_UndefinedEntity)
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
            else if (FP.ParamType() == Interface_ParamType.Interface_ParamVoid)
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
    public bool ReadInteger(int num, int nump, string mess, Interface_Check ach, ref int val)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            if (FP.ParamType() == Interface_ParamType.Interface_ParamInteger)
            {
                val = global::Kerf.Engine.Foundation.Standard.Atoi(FP.CValue());
            }
            else if (FP.ParamType() == Interface_ParamType.Interface_ParamReal)
            {
                val = RoundToInt(Fastof(FP.CValue()));
                errmess = "Parameter n0.%d (%s) was rounded";
            }
            if (FP.ParamType() != Interface_ParamType.Interface_ParamInteger && FP.ParamType() != Interface_ParamType.Interface_ParamReal)
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
    private static int RoundToInt(double theValue)
    {
        double aRounded = Math.Round(theValue, MidpointRounding.AwayFromZero);
        return aRounded >= int.MinValue && aRounded < 2147483648.0 ? (int)aRounded : int.MinValue;
    }

    /// <summary>
    /// Reads parameter nump of record num as a Boolean. Return value and Check managed as by ReadReal (demands a
    /// Boolean enum, i.e. text ".T." for True or ".F." for False).
    /// </summary>
    public bool ReadBoolean(int num, int nump, string mess, Interface_Check ach, ref bool flag)
    {
        flag = true;
        string errmess = null; // Null si pas d erreur
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            if (FP.ParamType() == Interface_ParamType.Interface_ParamEnum)
            {
                string txt = FP.CValue();
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
    public bool ReadLogical(int num, int nump, string mess, Interface_Check ach, ref StepData_Logical flag)
    {
        string errmess = null; // Null si pas d erreur
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            if (FP.ParamType() == Interface_ParamType.Interface_ParamEnum)
            {
                string txt = FP.CValue();
                if (txt == ".T.")
                {
                    flag = StepData_Logical.StepData_LTrue;
                }
                else if (txt == ".F.")
                {
                    flag = StepData_Logical.StepData_LFalse;
                }
                else if (txt == ".U.")
                {
                    flag = StepData_Logical.StepData_LUnknown;
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
    public bool ReadString(int num, int nump, string mess, Interface_Check ach, ref TCollection_HAsciiString? val)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            if (FP.ParamType() == Interface_ParamType.Interface_ParamText)
            {
                val = new TCollection_HAsciiString(FP.CValue());
                cleanText(val);
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
    public bool ReadEnumParam(int num, int nump, string mess, Interface_Check ach, ref string text)
    {
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            if (FP.ParamType() == Interface_ParamType.Interface_ParamEnum)
            {
                text = FP.CValue();
            }
            else if (FP.ParamType() == Interface_ParamType.Interface_ParamVoid)
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
    public void FailEnumValue(int num, int nump, string mess, Interface_Check ach)
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
    public bool ReadEnum(int num, int nump, string mess, Interface_Check ach, StepData_EnumTool enumtool, ref int val)
    {
        //  resume with ReadEnumParam?
        string errmess = null; // Null if no error
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            if (FP.ParamType() == Interface_ParamType.Interface_ParamEnum)
            {
                val = enumtool.Value(FP.CValue());
                if (val >= 0)
                {
                    return true;
                }
                else
                {
                    errmess = "Parameter n0.%d (%s) : Incorrect Enumeration Value";
                }
            }
            else if (FP.ParamType() == Interface_ParamType.Interface_ParamVoid)
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
    public bool ReadTypedParam(int num, int nump, bool mustbetyped, string mess, Interface_Check ach, ref int numr, ref int numrp, ref string typ)
    {
        string txtmes;
        string errmess = null; // Null si pas d erreur
        if (nump > 0 && nump <= NbParams(num))
        {
            Interface_FileParameter FP = Param(num, nump);
            if (FP.ParamType() != Interface_ParamType.Interface_ParamSub)
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
            numr = FP.EntityNumber();
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
    public bool CheckDerived(int num, int nump, string mess, Interface_Check ach, bool errstat = false)
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
        return thenbents;
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
            num1 = thenbhead + 1;
        }
        int max = NbRecords();
        while (num1 <= max)
        {
            if (theidents[num1] > 0)
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
            Interface_FileParameter FP = Param(num, i);
            if (FP.ParamType() != Interface_ParamType.Interface_ParamIdent)
            {
                continue;
            }
            int ixp = global::Kerf.Engine.Foundation.Standard.Atoi(FP.CValue().Substring(1));
            if (ixp == id)
            {
                return FP.EntityNumber();
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
        int nbdirec = NbRecords();
        NCollection_Array1<int> subn = new NCollection_Array1<int>(0, thelastn);
        bool pbmap = false; // at least one conflict
        int nbmap = 0;
        NCollection_IndexedMap<int> imap = new NCollection_IndexedMap<int>();
        NCollection_Array1<int> indm = new NCollection_Array1<int>(0, nbdirec); // Index Map -> Record Number (only if map)
        int num; // svv Jan11 2000 : porting on DEC
        for (num = 1; num <= nbdirec; num++)
        {
            int ident = theidents[num];
            if (ident > 0)
            { // Ident normal -> Map ?
                //  Map: if Overlap, inhibit it. Otherwise, note index
                int indmap = imap.Add(ident);
                if (indmap <= nbmap)
                {
                    indmap = imap.FindIndex(ident); // safer
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
            int ident = theidents[num];
            if (ident < -2)
            {
                subn[-(ident + 2)] = num; // toujours a jour ...
            }
            int nba = NbParams(num);
            int nda = (num == 1 ? 0 : ParamFirstRank(num - 1));
            for (int na = nba; na > 0; na--)
            {
                //    We process: sub-lists (except subn), idents (if Map says OK ...)
                Interface_FileParameter FP = ChangeParameter(nda + na);
                Interface_ParamType letype = FP.ParamType();
                if (letype == Interface_ParamType.Interface_ParamSub)
                {
                    int numsub = FP.EntityNumber();
                    if (numsub > thelastn)
                    {
                        using (Messenger.StreamBuffer anInfo = global::Kerf.Engine.Foundation.Message.SendInfo())
                        {
                            anInfo.Append("Bad Sub.N0, Record ").Append(num).Append(" Param ").Append(na).Append(":$").Append(numsub).Append('\n');
                        }
                        continue;
                    }
                    FP.SetEntityNumber(subn[numsub]);
                }
                else if (letype == Interface_ParamType.Interface_ParamIdent)
                {
                    int id = FP.EntityNumber();
                    int indmap = imap.FindIndex(id);
                    if (indmap > 0)
                    { // the map found it
                        int num0 = indm[indmap];
                        if (num0 > 0)
                        {
                            FP.SetEntityNumber(num0); // AND THERE, we have resolved
                        }
                        else
                        {
                            FP.SetEntityNumber(-id); // CONFLICT -> will need to resolve ...
                        }
                    }
                    else
                    { // NOT RESOLVED, if no pbmap, say it
                        if (pbmap)
                        {
                            FP.SetEntityNumber(-id);
                            continue; // pbmap: we will find ourselves again
                        }
                        //  ...  Build the Check  ...
                        string failmess = global::Kerf.Engine.Foundation.Standard.Sprintf("Unresolved Reference, Ent.Id.#%d Param.n0 %d (Id.#%d)", ident, na, id);
                        thecheck.AddFail(failmess, "Unresolved Reference");
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
        int nbseq = thenbents + 2 * thenbscop;
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
            int ident = theidents[num];
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
                    int indmap = imap.Add(ident);
                    if (indmap <= nbmap)
                    {
                        bool errorscope = false;
                        indmap = imap.FindIndex(ident); // plus sur
                        pbmap = true;
                        if (thenbscop == 0)
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
                            thecheck.AddFail(ligne, "Ident defined SEVERAL TIMES : #%d");
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
                    Interface_FileParameter FP = ChangeParam(num, na);
                    if (FP.ParamType() != Interface_ParamType.Interface_ParamIdent)
                    {
                        continue;
                    }
                    int id = -FP.EntityNumber();
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
                            FP.SetEntityNumber(inds[n0]);
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
                                FP.SetEntityNumber(nok);
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
                Interface_FileParameter FP = ChangeParam(num, na);
                Interface_ParamType letype = FP.ParamType();
                if (letype == Interface_ParamType.Interface_ParamSub)
                {
                    //  sub-list type parameter: sub-list number read by unstacking
                    FP.SetEntityNumber(subpile.Value(nbsubpil));
                    nbsubpil--; //	subpile->Remove(nbsubpil);
                }
                else if (letype == Interface_ParamType.Interface_ParamIdent)
                {
                    //  ident type parameter (references an entity): search for requested ident
                    int id = -FP.EntityNumber();
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
                            int indmap = imap.FindIndex(id);
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
                        FP.SetEntityNumber(num0); // AND THERE, we have resolved
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
                        thecheck.AddFail(failmess, "Unresolved Reference");
                        //  ...  And output a more complete message
                        sout.Append("*** ERR StepReaderData *** Entite ").Append(nument).Append(", a ").Append((nr * 100) / nbseq)
                            .Append("% de DATA : #").Append(ident).Append("\n    Type:").Append(RecordType(num)).Append("  Param.n0 ")
                            .Append(na).Append(": #").Append(id).Append(" Not found").Append('\n');
                        FP.SetEntityNumber(0); // -> Unresolved reference
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
        int max = thenbhead;
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
    public Interface_Check GlobalCheck()
    {
        return thecheck;
    }
}
