// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Reads OCCT's headers (.hxx, .pxx) and sources (.cxx) into the model: classes with their fields, methods and
// documentation; the RW enum tools' namespaces; function definitions with their bodies as tokens.

namespace StepGen;

internal static class Parser
{
    private static readonly HashSet<string> Builtins =
    [
        "void", "bool", "char", "int", "double", "float", "long", "short", "unsigned", "signed", "size_t",
        "Standard_Integer", "Standard_Real", "Standard_Boolean", "Standard_CString", "Standard_Character",
        "Standard_ExtCharacter", "Standard_Address", "Standard_Size", "uint8_t", "int32_t", "uint32_t",
    ];

    private static readonly HashSet<string> Specifiers =
    [
        "Standard_EXPORT", "Standard_HIDDEN", "virtual", "static", "inline", "explicit", "constexpr", "friend",
        "Standard_NODISCARD", "mutable", "extern",
    ];

    // ------------------------------------------------------------------ token helpers

    public static int SkipComments(List<Token> t, int i)
    {
        while (i < t.Count && t[i].IsComment)
        {
            i++;
        }

        return i;
    }

    /// <summary>The index of the token matching the opening bracket at <paramref name="i"/>.</summary>
    public static int Match(List<Token> t, int i)
    {
        string anOpen = t[i].Text;
        string aClose = anOpen switch { "(" => ")", "[" => "]", "{" => "}", "<" => ">", _ => throw new InvalidOperationException(anOpen) };
        int aDepth = 0;
        for (int j = i; j < t.Count; j++)
        {
            if (t[j].IsComment || t[j].Kind is TokKind.String or TokKind.Char)
            {
                continue;
            }

            if (t[j].Text == anOpen)
            {
                aDepth++;
            }
            else if (t[j].Text == aClose)
            {
                aDepth--;
                if (aDepth == 0)
                {
                    return j;
                }
            }
        }

        throw new FormatException($"Unbalanced '{anOpen}' at line {t[i].Line}");
    }

    /// <summary>Splits a <c>&gt;&gt;</c> token in two, for nested template arguments.</summary>
    private static void SplitShift(List<Token> t, int i)
    {
        if (t[i].Text == ">>")
        {
            t[i].Text = ">";
            t.Insert(i + 1, new Token { Kind = TokKind.Punct, Text = ">", Line = t[i].Line });
        }
    }

    // ------------------------------------------------------------------ types

    public static bool IsBuiltin(string theName) => Builtins.Contains(theName);

    /// <summary>
    /// Parses a type at <paramref name="i"/> (const, a qualified name, template arguments, const, * and &amp;).
    /// Returns null when the tokens don't start a type.
    /// </summary>
    public static CppType? ParseType(List<Token> t, ref int i, Func<string, bool>? theIsTypeName = null)
    {
        int j = SkipComments(t, i);
        var aType = new CppType();
        while (j < t.Count && (t[j].Is("const") || t[j].Is("typename") || t[j].Is("struct") || t[j].Is("class") || t[j].Is("volatile")))
        {
            if (t[j].Is("const"))
            {
                aType.IsConst = true;
            }

            j = SkipComments(t, j + 1);
        }

        if (j >= t.Count || !t[j].IsIdent)
        {
            return null;
        }

        // Handle(T)
        if (t[j].Text == "Handle" && j + 1 < t.Count && t[j + 1].Is("("))
        {
            int k = j + 2;
            var anArg = ParseType(t, ref k);
            if (anArg == null || !t[k].Is(")"))
            {
                return null;
            }

            aType.Name = "occ::handle";
            aType.Args.Add(anArg);
            j = k + 1;
        }
        else
        {
            // a qualified name; multi-word builtins (unsigned int)
            string aName = t[j].Text;
            j++;
            while (j + 1 < t.Count && t[j].Is("::") && t[j + 1].IsIdent)
            {
                aName += "::" + t[j + 1].Text;
                j += 2;
            }

            if (aName is "unsigned" or "signed" or "long" or "short")
            {
                while (j < t.Count && t[j].IsIdent && t[j].Text is "int" or "long" or "char" or "short")
                {
                    aName += " " + t[j].Text;
                    j++;
                }
            }

            if (theIsTypeName != null && !theIsTypeName(aName))
            {
                return null;
            }

            aType.Name = aName;
            if (j < t.Count && t[j].Is("<"))
            {
                j++;
                while (true)
                {
                    j = SkipComments(t, j);
                    if (j >= t.Count)
                    {
                        return null;
                    }

                    if (t[j].Kind == TokKind.Number)
                    {
                        aType.ValueArgs.Add(t[j].Text);
                        j++;
                    }
                    else
                    {
                        var anArg = ParseType(t, ref j);
                        if (anArg == null)
                        {
                            return null;
                        }

                        aType.Args.Add(anArg);
                    }

                    j = SkipComments(t, j);
                    if (j >= t.Count)
                    {
                        return null;
                    }

                    SplitShift(t, j);
                    if (t[j].Is(">"))
                    {
                        j++;
                        break;
                    }

                    if (!t[j].Is(","))
                    {
                        return null;
                    }

                    j++;
                }
            }

            // a nested name after the template (NCollection_Array1<T>::Iterator)
            while (j + 1 < t.Count && t[j].Is("::") && t[j + 1].IsIdent)
            {
                aType.Name += "::" + t[j + 1].Text;
                j += 2;
            }
        }

        while (j < t.Count)
        {
            j = SkipComments(t, j);
            if (j >= t.Count)
            {
                break;
            }

            if (t[j].Is("const"))
            {
                if (aType.Pointers > 0)
                {
                    aType.IsConstPointer = true;
                }
                else
                {
                    aType.IsConst = true;
                }

                j++;
            }
            else if (t[j].Is("*"))
            {
                aType.Pointers++;
                j++;
            }
            else if (t[j].Is("&"))
            {
                aType.IsRef = true;
                j++;
            }
            else if (t[j].Is("&&"))
            {
                aType.IsRef = true; // rvalue references: as references
                j++;
            }
            else
            {
                break;
            }
        }

        i = j;
        return aType;
    }

    // ------------------------------------------------------------------ documentation

    private static List<string> TakeDoc(List<Token> thePending)
    {
        var aDoc = new List<string>();
        foreach (var c in thePending)
        {
            if (c.Text.StartsWith("//!", StringComparison.Ordinal))
            {
                aDoc.Add(c.Text[3..].Trim());
            }
        }

        thePending.Clear();
        return aDoc;
    }

    // ------------------------------------------------------------------ headers

    /// <summary>Reads a header's classes, enums and namespaces.</summary>
    public static (List<CppClass> Classes, List<CppEnum> Enums) ParseHeader(List<Token> t, string thePath)
    {
        var aClasses = new List<CppClass>();
        var anEnums = new List<CppEnum>();
        var aPending = new List<Token>();
        int i = 0;
        while (i < t.Count)
        {
            var k = t[i];
            if (k.IsComment)
            {
                aPending.Add(k);
                i++;
                continue;
            }

            if ((k.Is("class") || k.Is("struct")) && i + 1 < t.Count && t[i + 1].IsIdent)
            {
                // a definition, or a forward declaration
                int j = i + 2;
                while (j < t.Count && !t[j].Is(";") && !t[j].Is("{"))
                {
                    j++;
                }

                if (j < t.Count && t[j].Is("{"))
                {
                    var aCls = ParseClass(t, i, out int anEnd);
                    aCls.Doc = TakeDoc(aPending);
                    aCls.Header = thePath;
                    aClasses.Add(aCls);
                    i = anEnd;
                    continue;
                }

                i = j + 1;
                aPending.Clear();
                continue;
            }

            if (k.Is("enum"))
            {
                var anEnum = ParseEnum(t, ref i);
                anEnum.Doc = TakeDoc(aPending);
                anEnums.Add(anEnum);
                continue;
            }

            if (k.Is("namespace") && i + 2 < t.Count && t[i + 1].IsIdent && t[i + 2].Is("{"))
            {
                var aNs = new CppClass { Name = t[i + 1].Text, IsNamespace = true, Header = thePath, Doc = TakeDoc(aPending) };
                int anEnd = Match(t, i + 2);
                ParseMembers(t, i + 3, anEnd, aNs, "public");
                aClasses.Add(aNs);
                i = anEnd + 1;
                continue;
            }

            aPending.Clear();
            i++;
        }

        return (aClasses, anEnums);
    }

    private static CppEnum ParseEnum(List<Token> t, ref int i)
    {
        var anEnum = new CppEnum();
        i++;
        if (t[i].Is("class") || t[i].Is("struct"))
        {
            anEnum.IsClassEnum = true;
            i++;
        }

        anEnum.Name = t[i].Text;
        i++;
        while (!t[i].Is("{"))
        {
            i++; // ": int" and the like
        }

        int anEnd = Match(t, i);
        var aPending = new List<Token>();
        int j = i + 1;
        while (j < anEnd)
        {
            if (t[j].IsComment)
            {
                // a trailing comment documents the member before it
                if (!t[j].NewlineBefore && anEnum.Members.Count > 0 && t[j].Text.StartsWith("//", StringComparison.Ordinal))
                {
                    anEnum.Members[^1].Doc.Add(t[j].Text.TrimStart('/', '!', '<').Trim());
                }
                else
                {
                    aPending.Add(t[j]);
                }

                j++;
                continue;
            }

            if (t[j].IsIdent)
            {
                string aName = t[j].Text;
                List<Token>? aValue = null;
                j++;
                if (t[j].Is("="))
                {
                    j++;
                    aValue = [];
                    while (!t[j].Is(",") && j < anEnd)
                    {
                        if (!t[j].IsComment)
                        {
                            aValue.Add(t[j]);
                        }

                        j++;
                    }
                }

                anEnum.Members.Add((aName, aValue, TakeDoc(aPending)));
                continue;
            }

            j++;
        }

        i = anEnd + 1;
        if (i < t.Count && t[i].Is(";"))
        {
            i++;
        }

        return anEnum;
    }

    private static CppClass ParseClass(List<Token> t, int i, out int theEnd)
    {
        var aCls = new CppClass { IsStruct = t[i].Is("struct"), Name = t[i + 1].Text };
        int j = i + 2;
        if (t[j].Is("final"))
        {
            j++;
        }

        if (t[j].Is(":"))
        {
            j++;
            while (!t[j].Is("{"))
            {
                if (t[j].Is("public") || t[j].Is("protected") || t[j].Is("private") || t[j].Is("virtual") || t[j].Is(","))
                {
                    j++;
                    continue;
                }

                int k = j;
                var aBase = ParseType(t, ref k) ?? throw new FormatException($"base of {aCls.Name}");
                if (aCls.Base == null)
                {
                    aCls.Base = aBase.Name;
                }
                else
                {
                    aCls.OtherBases.Add(aBase.Name);
                }

                j = k;
            }
        }

        int anEnd = Match(t, j);
        ParseMembers(t, j + 1, anEnd, aCls, aCls.IsStruct ? "public" : "private");
        theEnd = anEnd + 1;
        if (theEnd < t.Count && t[theEnd].Is(";"))
        {
            theEnd++;
        }

        return aCls;
    }

    private static void ParseMembers(List<Token> t, int theStart, int theEnd, CppClass theCls, string theAccess)
    {
        string anAccess = theAccess;
        var aPending = new List<Token>();
        int i = theStart;
        while (i < theEnd)
        {
            var k = t[i];
            if (k.IsComment)
            {
                aPending.Add(k);
                i++;
                continue;
            }

            if ((k.Is("public") || k.Is("protected") || k.Is("private")) && t[i + 1].Is(":"))
            {
                anAccess = k.Text;
                i += 2;
                aPending.Clear();
                continue;
            }

            if (k.IsIdent && k.Text.StartsWith("DEFINE_", StringComparison.Ordinal))
            {
                if (k.Text == "DEFINE_STANDARD_ALLOC")
                {
                    theCls.HasDefineAlloc = true;
                }

                i++;
                if (i < theEnd && t[i].Is("("))
                {
                    i = Match(t, i) + 1;
                }

                if (i < theEnd && t[i].Is(";"))
                {
                    i++;
                }

                aPending.Clear();
                continue;
            }

            if (k.Is("friend"))
            {
                while (!t[i].Is(";"))
                {
                    i++;
                }

                i++;
                continue;
            }

            if (k.Is("enum"))
            {
                var anEnum = ParseEnum(t, ref i);
                anEnum.Doc = TakeDoc(aPending);
                theCls.Enums.Add(anEnum);
                continue;
            }

            if (k.Is(";"))
            {
                i++;
                continue;
            }

            // a member declaration: up to ';' or a body at depth 0
            int j = i;
            int aDepth = 0;
            int aBodyAt = -1;
            while (j < theEnd)
            {
                var x = t[j];
                if (x.IsComment || x.Kind is TokKind.String or TokKind.Char)
                {
                    j++;
                    continue;
                }

                if (x.Is("(") || x.Is("["))
                {
                    aDepth++;
                }
                else if (x.Is(")") || x.Is("]"))
                {
                    aDepth--;
                }
                else if (aDepth == 0 && x.Is(";"))
                {
                    break;
                }
                else if (aDepth == 0 && x.Is("{"))
                {
                    // a body (a function's), or a brace initialiser of a field
                    bool isInit = j > i && (t[j - 1].IsIdent || t[j - 1].Is("=") || t[j - 1].Is("]")) && !LooksLikeFunctionBefore(t, i, j);
                    if (isInit)
                    {
                        j = Match(t, j) + 1;
                        continue;
                    }

                    aBodyAt = j;
                    break;
                }

                j++;
            }

            var aDecl = t.GetRange(i, j - i).Where(x => !x.IsComment).ToList();
            var aDoc = TakeDoc(aPending);
            if (aBodyAt >= 0)
            {
                int aBodyEnd = Match(t, aBodyAt);
                var aMethod = ParseFunctionDecl(aDecl, theCls.Name, theCls.Problems);
                if (aMethod != null)
                {
                    aMethod.Access = anAccess;
                    aMethod.Doc = aDoc;
                    aMethod.IsInline = true;
                    aMethod.Body = t.GetRange(aBodyAt + 1, aBodyEnd - aBodyAt - 1);
                    SplitInitList(aMethod);
                    theCls.Methods.Add(aMethod);
                }

                i = aBodyEnd + 1;
                if (i < theEnd && t[i].Is(";"))
                {
                    i++;
                }

                continue;
            }

            i = j + 1;
            if (aDecl.Count == 0)
            {
                continue;
            }

            if (DirectInitConstant(aDecl) is { } aConst)
            {
                aConst.Access = anAccess;
                aConst.Doc = aDoc;
                theCls.Fields.Add(aConst);
            }
            else if (IsFunctionDecl(aDecl, theCls.Name))
            {
                var aMethod = ParseFunctionDecl(aDecl, theCls.Name, theCls.Problems);
                if (aMethod != null)
                {
                    aMethod.Access = anAccess;
                    aMethod.Doc = aDoc;
                    theCls.Methods.Add(aMethod);
                }
            }
            else
            {
                foreach (var aField in ParseFieldDecl(aDecl, theCls.Problems))
                {
                    aField.Access = anAccess;
                    aField.Doc = aDoc;
                    theCls.Fields.Add(aField);
                }
            }
        }
    }

    /// <summary>
    /// "static [constexpr] Type name(literals...)": a constant initialised directly (not a function declaration,
    /// which C++ would read the same way had the arguments been types).
    /// </summary>
    private static CppField? DirectInitConstant(List<Token> theDecl)
    {
        var d = theDecl.Where(x => !x.IsComment).ToList();
        int i = 0;
        bool isStatic = false, isConstexpr = false;
        while (i < d.Count && (d[i].Is("static") || d[i].Is("constexpr") || d[i].Is("inline") || d[i].Is("Standard_EXPORT")))
        {
            isStatic |= d[i].Is("static");
            isConstexpr |= d[i].Is("constexpr");
            i++;
        }

        int j = i;
        var aType = ParseType(d, ref j);
        if (aType == null || j + 2 >= d.Count || !d[j].IsIdent || !d[j + 1].Is("("))
        {
            return null;
        }

        int aClose = Match(d, j + 1);
        if (aClose != d.Count - 1 || aClose == j + 2)
        {
            return null;
        }

        // every argument a literal (or a negated number)
        for (int k = j + 2; k < aClose; k++)
        {
            if (!(d[k].Kind is TokKind.String or TokKind.Number or TokKind.Char || d[k].Is(",") || d[k].Is("-")))
            {
                return null;
            }
        }

        return new CppField
        {
            Type = aType, Name = d[j].Text, IsStatic = isStatic, IsConstexpr = isConstexpr,
            Init = d.GetRange(j + 1, aClose - j), // "(args)"
        };
    }

    private static bool LooksLikeFunctionBefore(List<Token> t, int theFrom, int theTo)
    {
        // "Name(...) const {" or "Name(...) {" or "Name(...) : init(...) {"
        for (int k = theFrom; k < theTo; k++)
        {
            if (t[k].Is("("))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>Moves a constructor's member initialiser list (": a(x), b(y)") out of the declaration.</summary>
    private static void SplitInitList(CppMethod theMethod)
    {
        // the declaration parser leaves it in the trailing tokens; nothing to do here (ParseFunctionDecl handles it)
        _ = theMethod;
    }

    private static bool IsFunctionDecl(List<Token> theDecl, string theClass)
    {
        // a '(' at depth 0 that follows the declared name (not inside an initialiser)
        for (int k = 0; k < theDecl.Count; k++)
        {
            if (theDecl[k].Is("=") || theDecl[k].Is("{"))
            {
                return false;
            }

            if (theDecl[k].Is("<"))
            {
                // skip template arguments
                int aDepth = 0;
                for (; k < theDecl.Count; k++)
                {
                    if (theDecl[k].Is("<"))
                    {
                        aDepth++;
                    }
                    else if (theDecl[k].Is(">"))
                    {
                        aDepth--;
                    }
                    else if (theDecl[k].Is(">>"))
                    {
                        aDepth -= 2;
                    }

                    if (aDepth <= 0)
                    {
                        break;
                    }
                }

                continue;
            }

            if (theDecl[k].Is("("))
            {
                // Handle(T) is a type, not a call
                if (k > 0 && theDecl[k - 1].Text == "Handle")
                {
                    k = Match(theDecl, k);
                    continue;
                }

                return true;
            }
        }

        _ = theClass;
        return false;
    }

    /// <summary>Parses "specifiers type name(params) trailers [: inits]" (no body).</summary>
    public static CppMethod? ParseFunctionDecl(List<Token> theDecl, string theClass, List<string> theProblems)
    {
        var d = theDecl.Where(x => !x.IsComment).ToList();
        var aMethod = new CppMethod { Origin = string.Join(" ", d.Select(x => x.Text)) };
        int i = 0;
        if (d.Count > 0 && d[0].Is("template"))
        {
            int anEnd = Match(d, 1);
            for (int k = 2; k < anEnd; k++)
            {
                if ((d[k].Is("class") || d[k].Is("typename")) && d[k + 1].IsIdent)
                {
                    aMethod.TemplateParams.Add(d[k + 1].Text);
                }
            }

            i = anEnd + 1;
        }

        while (i < d.Count)
        {
            if (d[i].Is("[["))
            {
                while (!d[i].Is("]]"))
                {
                    i++;
                }

                i++;
                continue;
            }

            if (d[i].IsIdent && Specifiers.Contains(d[i].Text))
            {
                if (d[i].Text == "virtual")
                {
                    aMethod.IsVirtual = true;
                }
                else if (d[i].Text == "static")
                {
                    aMethod.IsStatic = true;
                }
                else if (d[i].Text == "Standard_HIDDEN")
                {
                    aMethod.Hidden = true;
                }
                else if (d[i].Text == "inline")
                {
                    aMethod.IsInline = true;
                }

                i++;
                continue;
            }

            break;
        }

        // constructor, destructor, or "type name"
        string aShort = theClass.Contains("::") ? theClass[(theClass.LastIndexOf(':') + 1)..] : theClass;
        if (d[i].Is("~"))
        {
            aMethod.IsDtor = true;
            aMethod.Name = "~" + d[i + 1].Text;
            i += 2;
        }
        else if (d[i].IsIdent && (d[i].Text == aShort || (d[i].Text == theClass)) && d[i + 1].Is("("))
        {
            aMethod.IsCtor = true;
            aMethod.Name = d[i].Text;
            i++;
        }
        else
        {
            int k = i;
            var aRet = ParseType(d, ref k);
            if (aRet == null)
            {
                theProblems.Add("cannot parse the return type of: " + aMethod.Origin);
                return null;
            }

            // a qualified name (Class::Method), the destructor Class::~Class, or operator
            string aName = "";
            if (d[k].Is("operator"))
            {
                theProblems.Add("operator: " + aMethod.Origin);
                return null;
            }

            if (d[k].IsIdent)
            {
                aName = d[k].Text;
                k++;
                while (d[k].Is("::"))
                {
                    if (d[k + 1].Is("~"))
                    {
                        aMethod.IsDtor = true;
                        aName += "::~" + d[k + 2].Text;
                        k += 3;
                        continue;
                    }

                    aName += "::" + d[k + 1].Text;
                    k += 2;
                }
            }
            else if (aRet.Name.Contains("::") && d[k].Is("("))
            {
                // "Class::Class(" read as a type: a constructor definition
                aName = aRet.Name;
                aRet = null;
                aMethod.IsCtor = true;
            }
            else
            {
                theProblems.Add("cannot parse the name of: " + aMethod.Origin);
                return null;
            }

            if (aRet != null && aName.Contains("::"))
            {
                // Class::Class read with Class as return type? no: here the return type is real
            }

            aMethod.Return = aRet;
            aMethod.Name = aName;
            i = k;
        }

        if (!d[i].Is("("))
        {
            theProblems.Add("expected '(' in: " + aMethod.Origin);
            return null;
        }

        int aClose = Match(d, i);
        aMethod.Params = ParseParams(d, i + 1, aClose, theProblems, aMethod.Origin);
        i = aClose + 1;
        while (i < d.Count)
        {
            if (d[i].Is("const"))
            {
                aMethod.IsConst = true;
                i++;
            }
            else if (d[i].Is("override"))
            {
                aMethod.IsOverride = true;
                i++;
            }
            else if (d[i].Is("final") || d[i].Is("noexcept"))
            {
                i++;
                if (i < d.Count && d[i].Is("("))
                {
                    i = Match(d, i) + 1;
                }
            }
            else if (d[i].Is("=") && i + 1 < d.Count)
            {
                if (d[i + 1].Text == "0")
                {
                    aMethod.IsPure = true;
                }
                else if (d[i + 1].Text == "default")
                {
                    aMethod.IsDefault = true;
                }
                else if (d[i + 1].Text == "delete")
                {
                    aMethod.IsDeleted = true;
                }

                i += 2;
            }
            else if (d[i].Is(":"))
            {
                aMethod.InitList = d.GetRange(i + 1, d.Count - i - 1);
                break;
            }
            else
            {
                theProblems.Add($"unexpected '{d[i].Text}' after the parameters of: " + aMethod.Origin);
                break;
            }
        }

        return aMethod;
    }

    private static List<CppParam> ParseParams(List<Token> d, int theFrom, int theTo, List<string> theProblems, string theOrigin)
    {
        var aParams = new List<CppParam>();
        int i = theFrom;
        if (theTo - theFrom == 1 && d[theFrom].Is("void"))
        {
            return aParams;
        }

        while (i < theTo)
        {
            int k = i;
            var aType = ParseType(d, ref k);
            if (aType == null)
            {
                theProblems.Add("cannot parse a parameter of: " + theOrigin);
                return aParams;
            }

            var aParam = new CppParam { Type = aType };
            if (k < theTo && d[k].IsIdent)
            {
                aParam.Name = d[k].Text;
                k++;
            }

            if (k < theTo && d[k].Is("["))
            {
                aParam.IsArray = true;
                k = Match(d, k) + 1;
            }

            if (k < theTo && d[k].Is("="))
            {
                k++;
                aParam.Default = [];
                int aDepth = 0;
                while (k < theTo)
                {
                    if (d[k].Is("(") || d[k].Is("{") || d[k].Is("<"))
                    {
                        aDepth++;
                    }
                    else if (d[k].Is(")") || d[k].Is("}") || d[k].Is(">"))
                    {
                        aDepth--;
                    }
                    else if (aDepth == 0 && d[k].Is(","))
                    {
                        break;
                    }

                    aParam.Default.Add(d[k]);
                    k++;
                }
            }

            aParams.Add(aParam);
            if (k < theTo && d[k].Is(","))
            {
                k++;
            }
            else if (k < theTo)
            {
                theProblems.Add($"unexpected '{d[k].Text}' in the parameters of: " + theOrigin);
                return aParams;
            }

            i = k;
        }

        return aParams;
    }

    private static List<CppField> ParseFieldDecl(List<Token> theDecl, List<string> theProblems)
    {
        var aFields = new List<CppField>();
        var d = theDecl;
        int i = 0;
        bool isStatic = false, isConstexpr = false;
        while (i < d.Count && d[i].IsIdent && Specifiers.Contains(d[i].Text))
        {
            isStatic |= d[i].Text == "static";
            isConstexpr |= d[i].Text == "constexpr";
            i++;
        }

        if (i < d.Count && (d[i].Is("typedef") || d[i].Is("using")))
        {
            theProblems.Add("typedef or using in a class: " + string.Join(" ", d.Select(x => x.Text)));
            return aFields;
        }

        var aType = ParseType(d, ref i);
        if (aType == null)
        {
            theProblems.Add("cannot parse the field: " + string.Join(" ", d.Select(x => x.Text)));
            return aFields;
        }

        while (i < d.Count)
        {
            var aDeclType = aType.Clone();
            while (d[i].Is("*") || d[i].Is("&"))
            {
                if (d[i].Is("*"))
                {
                    aDeclType.Pointers++;
                }
                else
                {
                    aDeclType.IsRef = true;
                }

                i++;
            }

            var aField = new CppField { Type = aDeclType, Name = d[i].Text, IsStatic = isStatic, IsConstexpr = isConstexpr };
            i++;
            if (i < d.Count && d[i].Is("["))
            {
                int anEnd = Match(d, i);
                aField.ArraySize = string.Join("", d.GetRange(i + 1, anEnd - i - 1).Select(x => x.Text));
                i = anEnd + 1;
            }

            if (i < d.Count && (d[i].Is("=") || d[i].Is("{")))
            {
                int aStart = d[i].Is("=") ? i + 1 : i;
                int k = aStart;
                int aDepth = 0;
                while (k < d.Count && !(aDepth == 0 && d[k].Is(",")))
                {
                    if (d[k].Is("(") || d[k].Is("{"))
                    {
                        aDepth++;
                    }
                    else if (d[k].Is(")") || d[k].Is("}"))
                    {
                        aDepth--;
                    }

                    k++;
                }

                aField.Init = d.GetRange(aStart, k - aStart);
                i = k;
            }

            aFields.Add(aField);
            if (i < d.Count && d[i].Is(","))
            {
                i++;
                continue;
            }

            break;
        }

        return aFields;
    }

    // ------------------------------------------------------------------ sources

    /// <summary>A function defined in a source file.</summary>
    internal sealed class SourceFunction
    {
        public CppMethod Method = null!;
        public string? ClassName; // null for file-level functions
        public bool InAnonymous;
        public bool IsFileStatic;
    }

    /// <summary>Reads a source's function definitions and file-level constants.</summary>
    public static (List<SourceFunction> Functions, List<CppField> Constants, List<string> Problems) ParseSource(List<Token> t)
        => ParseSource(t, null);

    /// <summary>A source's functions and file-level constants; its type aliases (<c>using A = B;</c>) go to <paramref name="theAliases"/>.</summary>
    public static (List<SourceFunction> Functions, List<CppField> Constants, List<string> Problems) ParseSource(List<Token> t, Dictionary<string, CppType>? theAliases)
    {
        var aFunctions = new List<SourceFunction>();
        var aConstants = new List<CppField>();
        var aProblems = new List<string>();
        ParseSourceRange(t, 0, t.Count, false, aFunctions, aConstants, aProblems, theAliases);
        return (aFunctions, aConstants, aProblems);
    }

    private static void ParseSourceRange(List<Token> t, int theFrom, int theTo, bool theAnonymous,
                                         List<SourceFunction> theFunctions, List<CppField> theConstants, List<string> theProblems,
                                         Dictionary<string, CppType>? theAliases = null)
    {
        var aPending = new List<Token>();
        int i = theFrom;
        while (i < theTo)
        {
            var k = t[i];
            if (k.IsComment)
            {
                aPending.Add(k);
                i++;
                continue;
            }

            if (k.Is("namespace"))
            {
                int j = i + 1;
                bool isAnonymous = t[j].Is("{");
                while (!t[j].Is("{"))
                {
                    j++;
                }

                int anEnd = Match(t, j);
                ParseSourceRange(t, j + 1, anEnd, theAnonymous || isAnonymous, theFunctions, theConstants, theProblems, theAliases);
                i = anEnd + 1;
                aPending.Clear();
                continue;
            }

            if (k.IsIdent && (k.Text.StartsWith("IMPLEMENT_", StringComparison.Ordinal) || k.Text == "Standard_DEPRECATED"))
            {
                i++;
                if (t[i].Is("("))
                {
                    i = Match(t, i) + 1;
                }

                if (i < theTo && t[i].Is(";"))
                {
                    i++;
                }

                aPending.Clear();
                continue;
            }

            if (k.Is(";"))
            {
                i++;
                continue;
            }

            // a file-level struct (a hasher for a map): left out, the port's collections hash with .NET's comparers
            if ((k.Is("struct") || k.Is("class")) && i + 2 < theTo && t[i + 1].IsIdent && t[i + 2].Is("{"))
            {
                int anEnd = Match(t, i + 2);
                theProblems.Add("file-level struct " + t[i + 1].Text + " left out");
                i = anEnd + 1;
                if (i < theTo && t[i].Is(";"))
                {
                    i++;
                }

                aPending.Clear();
                continue;
            }

            // "using A = B;": a type alias, resolved where the type is used
            if (k.Is("using") && theAliases != null && t[i + 1].IsIdent && t[i + 2].Is("="))
            {
                int p = i + 3;
                var anAliased = ParseType(t, ref p);
                if (anAliased != null && t[p].Is(";"))
                {
                    theAliases[t[i + 1].Text] = anAliased;
                    i = p + 1;
                    aPending.Clear();
                    continue;
                }
            }

            if (k.Is("using") || k.Is("typedef"))
            {
                while (!t[i].Is(";"))
                {
                    i++;
                }

                theProblems.Add("using/typedef in a source: " + string.Join(" ", t.GetRange(k.Line > 0 ? i : i, 1)));
                i++;
                continue;
            }

            // a definition: up to the body's '{', or a declaration up to ';'
            int jj = i;
            int aDepth = 0;
            int aBodyAt = -1;
            while (jj < theTo)
            {
                var x = t[jj];
                if (x.IsComment || x.Kind is TokKind.String or TokKind.Char)
                {
                    jj++;
                    continue;
                }

                if (x.Is("(") || x.Is("["))
                {
                    aDepth++;
                }
                else if (x.Is(")") || x.Is("]"))
                {
                    aDepth--;
                }
                else if (aDepth == 0 && x.Is(";"))
                {
                    break;
                }
                else if (aDepth == 0 && x.Is("{"))
                {
                    // a brace initialiser of a constant: "= {" or "name{"
                    if (jj > i && (t[jj - 1].Is("=") || (t[jj - 1].IsIdent && !LooksLikeFunctionBefore(t, i, jj)) || t[jj - 1].Is("]")))
                    {
                        jj = Match(t, jj) + 1;
                        continue;
                    }

                    // a constructor's initialiser list "a(x), b{y}": braces after an identifier inside the list
                    aBodyAt = jj;
                    break;
                }

                jj++;
            }

            var aDecl = t.GetRange(i, jj - i).Where(x => !x.IsComment).ToList();
            if (aBodyAt >= 0)
            {
                // constructor initialiser lists may contain "name{...}": find the body as the last top-level brace
                int aBodyEnd = Match(t, aBodyAt);
                bool isStatic = aDecl.Count > 0 && aDecl[0].Is("static");
                string? aClass = null;
                var aMethod = ParseDefinition(aDecl, out aClass, theProblems);
                if (aMethod != null)
                {
                    aMethod.Body = t.GetRange(aBodyAt + 1, aBodyEnd - aBodyAt - 1);
                    aMethod.LeadingComments = aPending.Select(c => c.Text).ToList();
                    theFunctions.Add(new SourceFunction { Method = aMethod, ClassName = aClass, InAnonymous = theAnonymous, IsFileStatic = isStatic });
                }

                aPending.Clear();
                i = aBodyEnd + 1;
                continue;
            }

            // "Class::Class() = default;" and file-level constants
            if (aDecl.Count > 0)
            {
                if (DirectInitConstant(aDecl) is { } aConst)
                {
                    theConstants.Add(aConst);
                }
                else if (IsFunctionDecl(aDecl, "") || aDecl.Any(x => x.Is("default")))
                {
                    var aMethod = ParseDefinition(aDecl, out string? aClass, theProblems);
                    if (aMethod != null && aMethod.IsDefault)
                    {
                        aMethod.Body = [];
                        theFunctions.Add(new SourceFunction { Method = aMethod, ClassName = aClass, InAnonymous = theAnonymous });
                    }
                    else if (aMethod != null)
                    {
                        // a declaration of a file-level function defined later
                    }
                }
                else
                {
                    foreach (var f in ParseFieldDecl(aDecl, theProblems))
                    {
                        f.IsStatic = true;
                        theConstants.Add(f);
                    }
                }
            }

            aPending.Clear();
            i = jj + 1;
        }
    }

    private static CppMethod? ParseDefinition(List<Token> theDecl, out string? theClass, List<string> theProblems)
    {
        theClass = null;
        // find the qualified name "A::B" before the first '(' at depth 0 (skipping template args)
        var d = theDecl;
        int aParenAt = -1;
        int anAngle = 0;
        for (int k = 0; k < d.Count; k++)
        {
            if (d[k].Is("template") && k == 0)
            {
                k = Match(d, 1);
                continue;
            }

            if (d[k].Is("<"))
            {
                anAngle++;
            }
            else if (d[k].Is(">"))
            {
                anAngle--;
            }
            else if (d[k].Is(">>"))
            {
                anAngle -= 2;
            }
            else if (d[k].Is("(") && anAngle == 0)
            {
                if (k > 0 && d[k - 1].Text == "Handle")
                {
                    k = Match(d, k);
                    continue;
                }

                aParenAt = k;
                break;
            }
        }

        if (aParenAt < 1)
        {
            theProblems.Add("cannot find a function in: " + string.Join(" ", d.Select(x => x.Text)));
            return null;
        }

        // the name ends at aParenAt-1; its qualifier is the identifiers joined by '::' before it
        int aNameAt = aParenAt - 1;
        bool isDtor = aNameAt > 0 && d[aNameAt - 1].Is("~");
        int aQualStart = isDtor ? aNameAt - 1 : aNameAt;
        string? aQualifier = null;
        if (aQualStart >= 2 && d[aQualStart - 1].Is("::") && d[aQualStart - 2].IsIdent)
        {
            int q = aQualStart - 2;
            aQualifier = d[q].Text;
            while (q >= 2 && d[q - 1].Is("::") && d[q - 2].IsIdent)
            {
                q -= 2;
                aQualifier = d[q].Text + "::" + aQualifier;
            }

            // rebuild the declaration without the qualifier, so the declaration parser sees "type name(...)"
            var aRest = new List<Token>();
            aRest.AddRange(d.GetRange(0, q));
            aRest.AddRange(d.GetRange(aQualStart, d.Count - aQualStart));
            d = aRest;
        }

        theClass = aQualifier;
        string aClassShort = aQualifier ?? "";
        var aMethod = ParseFunctionDecl(d, aClassShort, theProblems);
        return aMethod;
    }
}
