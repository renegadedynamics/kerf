// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Ap214;
using Kerf.Engine.Exchange.Step.Ap214.ReadWrite;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using System.Globalization;
using System.Text;

namespace Kerf.Kernel.DataExchange.Tests.StepGen;

/// <summary>
/// Round trips of STEP files through the generated readers and writers (units 8C1 and 8C2): read with
/// <see cref="Ap214Protocol"/> and its modules, written back with the files' own labels, and each record the
/// protocol recognises compared with the input's, token by token. Differences OCCT's own reading and writing make are
/// counted under their causes; anything else is unexplained.
/// </summary>
internal sealed class StepRoundTrip
{
    public long Files, Unread, Records, Compared, Same, WrappedStrings;
    /// <summary>The records the protocol doesn't recognise, by type (read as unknown entities, not compared).</summary>
    public readonly SortedDictionary<string, long> Unknown = new(StringComparer.Ordinal);
    private readonly Ap214ReadWriteModule myModule = new();
    public readonly SortedDictionary<string, (long Count, string Example)> Explained = new(StringComparer.Ordinal);
    public readonly SortedDictionary<string, (long Count, string Example)> Unexplained = new(StringComparer.Ordinal);
    public readonly HashSet<string> Types = new(StringComparer.Ordinal);

    // the causes of the differences OCCT makes (decision 1213)
    public const string ParameterValue = "an untyped trimming parameter is written PARAMETER_VALUE(...) (the member is named)";
    public const string IntegerAsReal = "an integer in a real SELECT member is written as a real";
    public const string Text = "a text with escapes or non-ASCII characters (decoded and encoded by StepData, unit 8B)";
    public const string LostReference = "a reference to a record the parser dropped (a syntax error in the file)";
    public const string ComplexUncertainty = "CATIA's complex LENGTH_MEASURE_WITH_UNIT and UNCERTAINTY_MEASURE_WITH_UNIT is written simple (RWStepBasic_RWUncertaintyMeasureWithUnit)";
    public const string DerivedDimensions = "an SI unit's dimensions are written derived (*): StepBasic_SiUnit computes them";
    public const string SizeMember = "an untyped size is written POSITIVE_LENGTH_MEASURE(...) (StepBasic_SizeMember always names itself)";
    public const string NotComplex = "OCCT's IsComplex misses the complex cases 800 and 802: written as a record of type '?' followed by the components";
    public const string BoolAsInt = "a boolean written as an integer (C++ promotes the bool passed to Send(int), RWStepRepr_RWBooleanRepresentationItem)";
    public const string StyledItemTarget = "a styled item's target that isn't a representation item is written $: the writer sends Item(), which only returns representation items";
    public const string RenderingMethod = "a rendering method read through inverted strcmp tests: anything but .CONSTANT_SHADING. becomes .CONSTANT_SHADING. (and it .COLOUR_SHADING.)";
    public const string ElementPurpose = "APPLICATION_DEFINED_ELEMENT_PURPOSE read as the enumerated purpose (CompareNames gives both case 1), written ENUMERATED_..._ELEMENT_PURPOSE";
    public const string NestedSize = "a doubly typed size, POSITIVE_LENGTH_MEASURE(LENGTH_MEASURE(x)), keeps its outer type only (StepBasic_SizeMember)";
    public const string DerivedCharacterized = "CHARACTERIZED_OBJECT's name and description written derived (*,*) by RWStepVisual_RWCharacterizedObjAndRepresentationAndDraughtingModel";
    public const string NormalsOutOfRange = "not written: OCCT's writer reads the normals (or control triangles) with rows and columns exchanged, out of range (occt-issues.md, 8C2)";

    /// <summary>The entities whose OCCT writer can throw (rows and columns exchanged): tried on a writer of their own first.</summary>
    private static readonly HashSet<Type> MayThrow = [typeof(ComplexTriangulatedFace), typeof(CubicBezierTriangulatedFace)];

    /// <summary>Reads a file with the protocol and writes it back (the DATA records, with the file's labels).</summary>
    public static string? ReadWrite(string thePath, StepProtocol theProtocol) => ReadWrite(thePath, theProtocol, []);

    public static string? ReadWrite(string thePath, StepProtocol theProtocol, HashSet<int> theNotWritten)
    {
        var aModel = new StepModel();
        aModel.SetProtocol(theProtocol);
        using (var aStream = File.OpenRead(thePath))
        {
            if (StepFileReader.Read(thePath, aStream, aModel, theProtocol) != 0)
            {
                return null;
            }
        }

        var aWriter = new StepWriter(aModel);
        aWriter.LabelMode() = 1;
        var aLib = new WriterLib(theProtocol);
        for (int i = 1; i <= aModel.EntityCount; i++)
        {
            if (aModel.Value(i) is { } anEnt && MayThrow.Contains(anEnt.GetType()))
            {
                // a throwing WriteStep leaves the writer inside the record: tried on a writer of its own first
                try
                {
                    new StepWriter(aModel).SendEntity(i, aLib);
                }
                catch (Exception caught) when (Refusal.Of(caught) is OutOfRangeException)
                {
                    theNotWritten.Add(aModel.IdentLabel(anEnt));
                    continue;
                }
            }

            aWriter.SendEntity(i, aLib);
        }

        var aText = new StringWriter(CultureInfo.InvariantCulture);
        aWriter.Print(aText);
        return aText.ToString();
    }

    /// <summary>True if StepAP214's module recognises the record's type (or its complex type's components).</summary>
    private bool Recognised(StepRecord theRec)
    {
        if (theRec.Type is not null)
        {
            return myModule.CaseStep(theRec.Type) > 0;
        }

        var aTypes = new NCollection_Sequence<string>();
        foreach (string aType in theRec.Complex ?? [])
        {
            aTypes.Append(aType);
        }

        return aTypes.Length() > 0 && myModule.CaseStep(aTypes) > 0;
    }

    public void Run(string thePath, StepProtocol theProtocol)
    {
        Files++;
        var aNotWritten = new HashSet<int>();
        string? aWritten = ReadWrite(thePath, theProtocol, aNotWritten);
        if (aWritten is null)
        {
            Unread++;
            return;
        }

        var anIn = StepRecords.Parse(File.ReadAllText(thePath, Encoding.Latin1), false);
        var anOut = StepRecords.Parse(aWritten, true);
        foreach (var (anId, aRec) in anIn)
        {
            Records++;
            if (!Recognised(aRec))
            {
                string aName = aRec.Type ?? "(" + string.Join(" ", aRec.Complex ?? []) + ")";
                Unknown[aName] = Unknown.GetValueOrDefault(aName) + 1;
                continue;
            }

            Compared++;
            Types.Add(aRec.Type ?? "(" + string.Join(" ", aRec.Complex!) + ")");
            anOut.TryGetValue(anId, out var aWrittenRec);
            if (aWrittenRec is not null && aWrittenRec.Wrapped)
            {
                WrappedStrings++;
            }

            string? aCause = aWrittenRec is null ? aNotWritten.Contains(anId) ? NormalsOutOfRange : "(not written)" : Compare(aRec, aWrittenRec);
            if (aCause is null)
            {
                Same++;
                continue;
            }

            // NotComplex, BoolAsInt, StyledItemTarget and RenderingMethod were OCCT's too, fixed by Kerf unit 10K3
            // (decisions 1501 to 1504): they would be unexplained now
            bool isExplained = aCause is ParameterValue or IntegerAsReal or Text or LostReference or DerivedDimensions or ComplexUncertainty or NormalsOutOfRange
                               or SizeMember or NestedSize or DerivedCharacterized or ElementPurpose;
            var aDict = isExplained ? Explained : Unexplained;
            string aKey = isExplained ? aCause : (aRec.Type ?? "(complex)") + ": " + aCause;
            var e = aDict.GetValueOrDefault(aKey);
            aDict[aKey] = (e.Count + 1, e.Example ?? $"{Path.GetFileName(thePath)} #{anId}\n  in:  {Clip(aRec.Text)}\n  out: {Clip(aWrittenRec?.Text)}");
        }
    }

    private static string Clip(string? s) => s is null ? "(missing)" : s.Length > 400 ? s[..400] + "..." : s;

    private static bool SameReal(string a, string b)
    {
        double x = double.Parse(a.AsSpan(1), CultureInfo.InvariantCulture), y = double.Parse(b.AsSpan(1), CultureInfo.InvariantCulture);
        // the writer keeps 12 or 13 significant digits (Interface_FloatWriter: %.12E, or %.12f between 0.1 and 1000)
        return x == y || Math.Abs(x - y) <= 1e-11 * Math.Max(Math.Abs(x), Math.Abs(y));
    }

    private static bool Same2(List<string> a, List<string> b)
    {
        if (a.Count != b.Count)
        {
            return false;
        }

        for (int i = 0; i < a.Count; i++)
        {
            if (a[i] != b[i] && !(a[i][0] == 'R' && b[i][0] == 'R' && SameReal(a[i], b[i])))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>A complex record's tokens with its components in alphabetic order (OCCT reads any order).</summary>
    private static List<string> Sorted(StepRecord theRec)
    {
        if (theRec.Complex is null)
        {
            return theRec.Tokens;
        }

        var aParts = new List<(string Name, List<string> Tokens)>();
        var t = theRec.Tokens;
        int i = 1; // after the outer "("
        while (i < t.Count - 1)
        {
            int j = i + 1, aDepth = 0;
            for (; j < t.Count; j++)
            {
                if (t[j] == "(")
                {
                    aDepth++;
                }
                else if (t[j] == ")" && --aDepth == 0)
                {
                    break;
                }
            }

            aParts.Add((t[i], t.GetRange(i, j - i + 1)));
            i = j + 1;
        }

        var aResult = new List<string> { "(" };
        foreach (var p in aParts.OrderBy(x => x.Name, StringComparer.Ordinal))
        {
            aResult.AddRange(p.Tokens);
        }

        aResult.Add(")");
        return aResult;
    }

    /// <summary>Null if the records are the same; otherwise the cause of the difference.</summary>
    private static string? Compare(StepRecord theInRec, StepRecord theOutRec)
    {
        var theIn = Sorted(theInRec);
        var theOut = Sorted(theOutRec);
        if (Same2(theIn, theOut))
        {
            return null;
        }

        if (theOutRec.Text.Contains("UNKNOWN REF", StringComparison.Ordinal))
        {
            return LostReference;
        }

        if (theInRec.Complex is not null && theOutRec.Text.StartsWith("?(", StringComparison.Ordinal))
        {
            return NotComplex;
        }

        if (theInRec.Type is { } aStyled && aStyled.EndsWith("STYLED_ITEM", StringComparison.Ordinal) && theOutRec.Text.Contains("NUL REF", StringComparison.Ordinal))
        {
            return StyledItemTarget;
        }

        if (theInRec.Text.Contains('\\') || theInRec.Text.Any(c => c > 126))
        {
            return Text;
        }

        // the complex uncertainty, read by its simple type's tool, and written as such
        if (theInRec.Complex is { } aParts && aParts.Contains("UNCERTAINTY_MEASURE_WITH_UNIT") && theOutRec.Type == "UNCERTAINTY_MEASURE_WITH_UNIT")
        {
            return ComplexUncertainty;
        }

        // NAMED_UNIT(*) written for NAMED_UNIT(#n) in an SI unit
        int aNamed = theOut.IndexOf("NAMED_UNIT");
        if (aNamed >= 0 && theOut.Contains("SI_UNIT") && aNamed + 3 < theOut.Count && theOut[aNamed + 2] == "*")
        {
            var aWithRef = new List<string>(theOut);
            int anInNamed = theIn.IndexOf("NAMED_UNIT");
            if (anInNamed >= 0 && anInNamed + 2 < theIn.Count)
            {
                aWithRef[aNamed + 2] = theIn[anInNamed + 2];
                if (Same2(theIn, aWithRef))
                {
                    return DerivedDimensions;
                }
            }
        }

        // PARAMETER_VALUE(x) or POSITIVE_LENGTH_MEASURE(x) written for an untyped x
        var anUnwrapped = new List<string>();
        string? aWrapper = null;
        for (int i = 0; i < theOut.Count; i++)
        {
            if (theOut[i] is "PARAMETER_VALUE" or "POSITIVE_LENGTH_MEASURE" && i + 3 < theOut.Count && theOut[i + 1] == "(" && theOut[i + 3] == ")" && !theIn.Contains(theOut[i]))
            {
                anUnwrapped.Add(theOut[i + 2]);
                aWrapper = theOut[i];
                i += 3;
                continue;
            }

            anUnwrapped.Add(theOut[i]);
        }

        // CHARACTERIZED_OBJECT(a,b) written CHARACTERIZED_OBJECT(*,*)
        int aChar = theIn.IndexOf("CHARACTERIZED_OBJECT");
        if (aChar >= 0 && aChar + 5 < theIn.Count && theIn[aChar + 3] == "," && theIn[aChar + 5] == ")")
        {
            var aDerived = new List<string>(theIn);
            aDerived[aChar + 2] = "*";
            aDerived[aChar + 4] = "*";
            if (Same2(aDerived, theOut))
            {
                return DerivedCharacterized;
            }
        }

        // POSITIVE_LENGTH_MEASURE(LENGTH_MEASURE(x)) read as POSITIVE_LENGTH_MEASURE(x)
        var anInner = new List<string>();
        for (int i = 0; i < theIn.Count; i++)
        {
            if (theIn[i] == "POSITIVE_LENGTH_MEASURE" && i + 6 < theIn.Count && theIn[i + 1] == "(" && theIn[i + 3] == "(" && theIn[i + 5] == ")" && theIn[i + 6] == ")")
            {
                anInner.AddRange([theIn[i], "(", theIn[i + 4], ")"]);
                i += 6;
                continue;
            }

            anInner.Add(theIn[i]);
        }

        if (anInner.Count != theIn.Count && Same2(anInner, theOut))
        {
            return NestedSize;
        }

        if (anUnwrapped.Count != theOut.Count && Same2(theIn, anUnwrapped))
        {
            return aWrapper == "PARAMETER_VALUE" ? ParameterValue : SizeMember;
        }

        if (theIn.Count == theOut.Count)
        {
            bool isIntAsReal = true, isText = true, isBool = true, isRendering = theInRec.Type?.StartsWith("SURFACE_STYLE_RENDERING", StringComparison.Ordinal) == true,
                 isPurpose = true;
            for (int i = 0; i < theIn.Count; i++)
            {
                string a = theIn[i], b = theOut[i];
                if (a == b || (a[0] == 'R' && b[0] == 'R' && SameReal(a, b)))
                {
                    continue;
                }

                if (!(a[0] == 'I' && b[0] == 'R' && double.Parse(a.AsSpan(1), CultureInfo.InvariantCulture) == double.Parse(b.AsSpan(1), CultureInfo.InvariantCulture)))
                {
                    isIntAsReal = false;
                }

                if (!(a[0] == 'S' && b[0] == 'S' && (a.Contains('\\') || a.Any(c => c > 126))))
                {
                    isText = false;
                }

                if (!((a == ".T." && b == "I1") || (a == ".F." && b == "I0")))
                {
                    isBool = false;
                }

                if (!(a.EndsWith("_SHADING.", StringComparison.Ordinal) && b.EndsWith("_SHADING.", StringComparison.Ordinal)))
                {
                    isRendering = false;
                }

                if (!(a == "APPLICATION_DEFINED_ELEMENT_PURPOSE" && b.StartsWith("ENUMERATED_", StringComparison.Ordinal) && b.EndsWith("_ELEMENT_PURPOSE", StringComparison.Ordinal)))
                {
                    isPurpose = false;
                }
            }

            if (isIntAsReal)
            {
                return IntegerAsReal;
            }

            if (isText)
            {
                return Text;
            }

            if (isBool)
            {
                return BoolAsInt;
            }

            if (isRendering)
            {
                return RenderingMethod;
            }

            if (isPurpose)
            {
                return ElementPurpose;
            }
        }

        return "differs";
    }
}

/// <summary>A STEP record: its type (or a complex record's types) and its tokens.</summary>
internal sealed class StepRecord
{
    public string? Type;
    public List<string>? Complex;
    public string Text = "";
    public List<string> Tokens = [];
    /// <summary>A string of the written record was wrapped onto another line with an indentation.</summary>
    public bool Wrapped;
}

/// <summary>A small reader of a STEP file's DATA records, for the round trips.</summary>
internal static class StepRecords
{
    public static Dictionary<int, StepRecord> Parse(string theText, bool theIsWritten)
    {
        var aResult = new Dictionary<int, StepRecord>();
        int aData = theText.IndexOf("DATA;", StringComparison.OrdinalIgnoreCase);
        int i = aData < 0 ? 0 : aData + 5;
        int aStart = i;
        while (i < theText.Length)
        {
            char c = theText[i];
            if (c == '/' && i + 1 < theText.Length && theText[i + 1] == '*')
            {
                int e = theText.IndexOf("*/", i + 2, StringComparison.Ordinal);
                i = e < 0 ? theText.Length : e + 2;
                continue;
            }

            if (c != '#')
            {
                i++;
                continue;
            }

            int j = i + 1;
            while (j < theText.Length && char.IsDigit(theText[j]))
            {
                j++;
            }

            int k = j;
            while (k < theText.Length && char.IsWhiteSpace(theText[k]))
            {
                k++;
            }

            if (j == i + 1 || k >= theText.Length || theText[k] != '=')
            {
                i = j;
                continue;
            }

            int anId = int.Parse(theText.AsSpan(i + 1, j - i - 1), CultureInfo.InvariantCulture);
            int anEnd = RecordEnd(theText, k + 1);
            var aRec = new StepRecord { Text = theText[(k + 1)..anEnd].Trim() };
            Tokens(aRec, theIsWritten);
            aResult[anId] = aRec;
            i = anEnd + 1;
        }

        _ = aStart;
        return aResult;
    }

    private static int RecordEnd(string s, int i)
    {
        bool inString = false;
        for (; i < s.Length; i++)
        {
            char c = s[i];
            if (inString)
            {
                if (c == '\'')
                {
                    if (i + 1 < s.Length && s[i + 1] == '\'')
                    {
                        i++;
                    }
                    else
                    {
                        inString = false;
                    }
                }
            }
            else if (c == '\'')
            {
                inString = true;
            }
            else if (c == ';')
            {
                return i;
            }
            else if (c == '/' && i + 1 < s.Length && s[i + 1] == '*')
            {
                int e = s.IndexOf("*/", i + 2, StringComparison.Ordinal);
                i = e < 0 ? s.Length : e + 1;
            }
        }

        return s.Length;
    }

    /// <summary>
    /// The record's tokens: S + a string's text (line ends removed, and in written records the indentation after
    /// them), I + an integer, R + a real, keywords upper-cased, punctuation as is; comments dropped.
    /// </summary>
    private static void Tokens(StepRecord theRec, bool theIsWritten)
    {
        string s = theRec.Text;
        var t = theRec.Tokens;
        int i = 0, aDepth = 0;
        var aComplex = new List<string>();
        while (i < s.Length)
        {
            char c = s[i];
            if (char.IsWhiteSpace(c))
            {
                i++;
                continue;
            }

            if (c == '/' && i + 1 < s.Length && s[i + 1] == '*')
            {
                int e = s.IndexOf("*/", i + 2, StringComparison.Ordinal);
                i = e < 0 ? s.Length : e + 2;
                continue;
            }

            if (c == '\'')
            {
                var aText = new StringBuilder();
                int j = i + 1;
                while (j < s.Length)
                {
                    if (s[j] == '\'')
                    {
                        if (j + 1 < s.Length && s[j + 1] == '\'')
                        {
                            aText.Append("''");
                            j += 2;
                            continue;
                        }

                        break;
                    }

                    if (s[j] is '\r' or '\n')
                    {
                        // a line end inside a string is not part of it (ISO 10303-21); the writer indents the next line
                        j++;
                        if (theIsWritten && s[j - 1] == '\n')
                        {
                            while (j < s.Length && s[j] == ' ')
                            {
                                theRec.Wrapped = true;
                                j++;
                            }
                        }

                        continue;
                    }

                    aText.Append(s[j]);
                    j++;
                }

                t.Add("S" + aText);
                i = j + 1;
                continue;
            }

            if (char.IsLetter(c) || c == '_')
            {
                int j = i;
                while (j < s.Length && (char.IsLetterOrDigit(s[j]) || s[j] == '_'))
                {
                    j++;
                }

                string aWord = s[i..j].ToUpperInvariant();
                if (aDepth == 0 && theRec.Type is null && t.Count == 0)
                {
                    theRec.Type = aWord;
                }
                else if (aDepth == 1 && theRec.Type is null)
                {
                    aComplex.Add(aWord);
                }

                t.Add(aWord);
                i = j;
                continue;
            }

            if (c == '#')
            {
                // a reference: one token
                int j = i + 1;
                while (j < s.Length && char.IsDigit(s[j]))
                {
                    j++;
                }

                t.Add(s[i..j]);
                i = j;
                continue;
            }

            if (c == '.' && i + 1 < s.Length && char.IsLetter(s[i + 1]))
            {
                int e = s.IndexOf('.', i + 1);
                t.Add(s[i..(e + 1)].ToUpperInvariant());
                i = e + 1;
                continue;
            }

            if (char.IsDigit(c) || c == '-' || c == '+' || c == '.')
            {
                int j = i + 1;
                while (j < s.Length && (char.IsDigit(s[j]) || s[j] is '.' or 'E' or 'e' || ((s[j] is '+' or '-') && s[j - 1] is 'E' or 'e')))
                {
                    j++;
                }

                string aNum = s[i..j];
                bool isReal = aNum.Contains('.') || aNum.Contains('E') || aNum.Contains('e');
                string aFixed = aNum.EndsWith('.') ? aNum + "0" : aNum.Replace(".E", ".0E").Replace(".e", ".0e");
                if (isReal && double.TryParse(aFixed, NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
                {
                    t.Add("R" + v.ToString("R", CultureInfo.InvariantCulture));
                }
                else if (!isReal && long.TryParse(aNum, NumberStyles.Integer, CultureInfo.InvariantCulture, out long n))
                {
                    t.Add("I" + n.ToString(CultureInfo.InvariantCulture));
                }
                else
                {
                    t.Add(aNum);
                }

                i = j;
                continue;
            }

            if (c == '(')
            {
                aDepth++;
            }
            else if (c == ')')
            {
                aDepth--;
            }

            t.Add(c.ToString());
            i++;
        }

        if (theRec.Type is null && aComplex.Count > 0)
        {
            theRec.Complex = aComplex;
        }
    }
}
