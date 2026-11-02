// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// C++ expressions: a small Pratt parser over tokens into a tree the translator prints as C#.

namespace StepGen;

internal abstract class Expr;

internal sealed class NameExpr : Expr
{
    public string Name = ""; // possibly qualified: A::B
    public List<CppType>? TemplateArgs;
}

internal sealed class LitExpr : Expr
{
    public string Text = "";
    public TokKind Kind;
}

internal sealed class MemberExpr : Expr
{
    public Expr Obj = null!;
    public string Name = "";
    public bool Arrow;
    public List<CppType>? TemplateArgs;
}

internal sealed class CallExpr : Expr
{
    public Expr Fn = null!;
    public List<Expr> Args = [];
    /// <summary>A braced call: T{...}.</summary>
    public bool Braced;
}

internal sealed class IndexExpr : Expr
{
    public Expr Obj = null!;
    public Expr Index = null!;
}

internal sealed class UnaryExpr : Expr
{
    public string Op = "";
    public Expr X = null!;
    public bool Postfix;
}

internal sealed class BinaryExpr : Expr
{
    public string Op = "";
    public Expr L = null!, R = null!;
}

internal sealed class TernaryExpr : Expr
{
    public Expr C = null!, A = null!, B = null!;
}

internal sealed class CastExpr : Expr
{
    public CppType Type = null!;
    public Expr X = null!;
    public string Kind = "c"; // c, static_cast, reinterpret_cast, const_cast, dynamic_cast
}

internal sealed class NewExpr : Expr
{
    public CppType Type = null!;
    public List<Expr>? Args;
    public Expr? ArraySize;
}

internal sealed class ParenExpr : Expr
{
    public Expr X = null!;
}

/// <summary>A type used as a value: <c>occ::handle&lt;T&gt;()</c>, <c>T{}</c>, <c>TCollection_AsciiString(x)</c>.</summary>
internal sealed class TypeCallExpr : Expr
{
    public CppType Type = null!;
    public List<Expr> Args = [];
}

internal sealed class InitListExpr : Expr
{
    public List<Expr> Items = [];
}

internal sealed class ExprParser
{
    private readonly List<Token> t;
    private int i;
    private readonly int myEnd;
    private readonly Func<string, bool> myIsType;
    public readonly List<string> Comments = [];

    public ExprParser(List<Token> theTokens, int theFrom, int theTo, Func<string, bool> theIsType)
    {
        t = theTokens;
        i = theFrom;
        myEnd = theTo;
        myIsType = theIsType;
    }

    public int Pos => i;

    private Token? Peek(int theAhead = 0)
    {
        int j = i;
        int n = 0;
        while (j < myEnd)
        {
            if (t[j].IsComment)
            {
                j++;
                continue;
            }

            if (n == theAhead)
            {
                return t[j];
            }

            n++;
            j++;
        }

        return null;
    }

    private Token Next()
    {
        while (i < myEnd && t[i].IsComment)
        {
            if (t[i].Kind == TokKind.Comment)
            {
                Comments.Add(t[i].Text);
            }

            i++;
        }

        if (i >= myEnd)
        {
            throw new FormatException("unexpected end of expression");
        }

        return t[i++];
    }

    private bool At(string theText) => Peek() is { } k && k.Is(theText);

    private void Expect(string theText)
    {
        var k = Next();
        if (!k.Is(theText))
        {
            throw new FormatException($"expected '{theText}' at line {k.Line}, got '{k.Text}'");
        }
    }

    public bool AtEnd => Peek() == null;

    public Expr ParseFull()
    {
        var e = ParseComma();
        if (!AtEnd)
        {
            throw new FormatException($"unexpected '{Peek()!.Text}' at line {Peek()!.Line}");
        }

        return e;
    }

    public Expr ParseComma()
    {
        var e = ParseAssign();
        while (At(","))
        {
            Next();
            e = new BinaryExpr { Op = ",", L = e, R = ParseAssign() };
        }

        return e;
    }

    private static readonly HashSet<string> AssignOps = ["=", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "<<=", ">>="];

    public Expr ParseAssign()
    {
        var l = ParseTernary();
        if (Peek() is { } k && k.Kind == TokKind.Punct && AssignOps.Contains(k.Text))
        {
            Next();
            var r = ParseAssign();
            return new BinaryExpr { Op = k.Text, L = l, R = r };
        }

        return l;
    }

    private Expr ParseTernary()
    {
        var c = ParseBinary(0);
        if (At("?"))
        {
            Next();
            var a = ParseAssign();
            Expect(":");
            var b = ParseAssign();
            return new TernaryExpr { C = c, A = a, B = b };
        }

        return c;
    }

    private static int Prec(string theOp) => theOp switch
    {
        "||" => 1,
        "&&" => 2,
        "|" => 3,
        "^" => 4,
        "&" => 5,
        "==" or "!=" => 6,
        "<" or ">" or "<=" or ">=" => 7,
        "<<" or ">>" => 8,
        "+" or "-" => 9,
        "*" or "/" or "%" => 10,
        _ => -1,
    };

    private Expr ParseBinary(int theMin)
    {
        var l = ParseUnary();
        while (Peek() is { } k && k.Kind == TokKind.Punct)
        {
            if (k.Text == ">" && Peek(1) is { } k2 && k2.Text == ">" && k2.Adjacent && !(Peek(2) is { } k3 && k3.Text == "=" && k3.Adjacent))
            {
                // ">" ">" adjacent: a shift
                if (8 < theMin)
                {
                    break;
                }

                Next();
                Next();
                l = new BinaryExpr { Op = ">>", L = l, R = ParseBinary(9) };
                continue;
            }

            int p = Prec(k.Text);
            if (p < 0 || p <= theMin - 1 || p < theMin)
            {
                break;
            }

            Next();
            var r = ParseBinary(p + 1);
            l = new BinaryExpr { Op = k.Text, L = l, R = r };
        }

        return l;
    }

    private Expr ParseUnary()
    {
        var k = Peek() ?? throw new FormatException("unexpected end of expression");
        if (k.Kind == TokKind.Punct && k.Text is "!" or "-" or "+" or "~" or "*" or "&" or "++" or "--")
        {
            Next();
            return new UnaryExpr { Op = k.Text, X = ParseUnary() };
        }

        if (k.Is("sizeof"))
        {
            Next();
            return new UnaryExpr { Op = "sizeof", X = ParseUnary() };
        }

        if (k.Is("new"))
        {
            Next();
            int j = i;
            // skip comments
            while (t[j].IsComment)
            {
                j++;
            }

            var aType = Parser.ParseType(t, ref j) ?? throw new FormatException($"bad new at line {k.Line}");
            i = j;
            var n = new NewExpr { Type = aType };
            if (At("("))
            {
                n.Args = ParseArgs("(", ")");
            }
            else if (At("["))
            {
                Next();
                n.ArraySize = ParseComma();
                Expect("]");
            }
            else if (At("{"))
            {
                n.Args = ParseArgs("{", "}");
            }

            return n;
        }

        if (k.Is("delete"))
        {
            throw new FormatException($"delete at line {k.Line}");
        }

        // C cast: "(type) unary"
        if (k.Is("("))
        {
            int j = i;
            while (t[j].IsComment)
            {
                j++;
            }

            int k2 = j + 1;
            var aType = Parser.ParseType(t, ref k2, myIsType);
            if (aType != null && k2 < myEnd && t[k2].Is(")"))
            {
                // followed by something that can start an operand
                int k3 = k2 + 1;
                while (k3 < myEnd && t[k3].IsComment)
                {
                    k3++;
                }

                if (k3 < myEnd && (t[k3].IsIdent || t[k3].Kind is TokKind.Number or TokKind.String or TokKind.Char || t[k3].Is("(") || t[k3].Is("!") || t[k3].Is("-")))
                {
                    i = k3;
                    return new CastExpr { Type = aType, X = ParseUnary() };
                }
            }
        }

        return ParsePostfix(ParsePrimary());
    }

    private List<Expr> ParseArgs(string theOpen, string theClose)
    {
        Expect(theOpen);
        var aArgs = new List<Expr>();
        if (At(theClose))
        {
            Next();
            return aArgs;
        }

        while (true)
        {
            aArgs.Add(ParseAssign());
            if (At(","))
            {
                Next();
                if (At(theClose))
                {
                    Next();
                    return aArgs; // trailing comma
                }

                continue;
            }

            Expect(theClose);
            return aArgs;
        }
    }

    private Expr ParsePostfix(Expr e)
    {
        while (Peek() is { } k)
        {
            if (k.Is("("))
            {
                e = new CallExpr { Fn = e, Args = ParseArgs("(", ")") };
            }
            else if (k.Is("["))
            {
                Next();
                var anIndex = ParseComma();
                Expect("]");
                e = new IndexExpr { Obj = e, Index = anIndex };
            }
            else if (k.Is(".") || k.Is("->"))
            {
                Next();
                var aName = Next();
                if (aName.Is("template"))
                {
                    aName = Next();
                }

                string aMemberName = aName.Text;
                while (At("::") && Peek(1) is { IsIdent: true })
                {
                    // ent->Base::Method(): the base class's version
                    Next();
                    aMemberName += "::" + Next().Text;
                }

                var m = new MemberExpr { Obj = e, Name = aMemberName, Arrow = k.Is("->") };
                if (At("<") && IsTemplateCall())
                {
                    m.TemplateArgs = ParseTemplateArgs();
                }

                e = m;
            }
            else if (k.Is("++") || k.Is("--"))
            {
                Next();
                e = new UnaryExpr { Op = k.Text, X = e, Postfix = true };
            }
            else
            {
                break;
            }
        }

        return e;
    }

    /// <summary>True when the '&lt;' ahead opens template arguments followed by '(' (a template call).</summary>
    private bool IsTemplateCall()
    {
        int j = i;
        while (t[j].IsComment)
        {
            j++;
        }

        int aDepth = 0;
        for (; j < myEnd; j++)
        {
            if (t[j].Is("<"))
            {
                aDepth++;
            }
            else if (t[j].Is(">"))
            {
                aDepth--;
            }
            else if (t[j].Is(">>"))
            {
                aDepth -= 2;
            }
            else if (!(t[j].IsIdent || t[j].Is("::") || t[j].Is(",") || t[j].Is("*") || t[j].Is("&") || t[j].IsComment || t[j].Kind == TokKind.Number))
            {
                return false;
            }

            if (aDepth <= 0)
            {
                return j + 1 < myEnd && (t[j + 1].Is("(") || t[j + 1].Is("{"));
            }
        }

        return false;
    }

    private List<CppType> ParseTemplateArgs()
    {
        int j = i;
        while (t[j].IsComment)
        {
            j++;
        }

        // reuse the type parser on "X<args>" by parsing a dummy name: parse args one by one
        j++; // '<'
        var aArgs = new List<CppType>();
        while (true)
        {
            var a = Parser.ParseType(t, ref j) ?? throw new FormatException("bad template argument");
            aArgs.Add(a);
            if (t[j].Is(">>"))
            {
                t[j].Text = ">";
                t.Insert(j + 1, new Token { Kind = TokKind.Punct, Text = ">", Line = t[j].Line });
            }

            if (t[j].Is(">"))
            {
                j++;
                break;
            }

            if (t[j].Is(","))
            {
                j++;
                continue;
            }

            throw new FormatException("bad template arguments");
        }

        i = j;
        return aArgs;
    }

    private static readonly HashSet<string> CastKeywords = ["static_cast", "reinterpret_cast", "const_cast", "dynamic_cast"];

    private Expr ParsePrimary()
    {
        var k = Next();
        switch (k.Kind)
        {
            case TokKind.Number:
            case TokKind.String:
            case TokKind.Char:
                return new LitExpr { Text = k.Text, Kind = k.Kind };
        }

        if (k.Is("("))
        {
            var e = ParseComma();
            Expect(")");
            return new ParenExpr { X = e };
        }

        if (k.Is("{"))
        {
            var l = new InitListExpr();
            if (At("}"))
            {
                Next();
                return l;
            }

            while (true)
            {
                l.Items.Add(ParseAssign());
                if (At(","))
                {
                    Next();
                    if (At("}"))
                    {
                        Next();
                        return l;
                    }

                    continue;
                }

                Expect("}");
                return l;
            }
        }

        if (k.Is("::"))
        {
            // global qualifier
            return ParsePrimary();
        }

        if (!k.IsIdent)
        {
            throw new FormatException($"unexpected '{k.Text}' at line {k.Line}");
        }

        if (CastKeywords.Contains(k.Text) && At("<"))
        {
            Next();
            int j = i;
            var aType = Parser.ParseType(t, ref j) ?? throw new FormatException("bad cast");
            i = j;
            if (t[i].Is(">>"))
            {
                t[i].Text = ">";
                t.Insert(i + 1, new Token { Kind = TokKind.Punct, Text = ">", Line = t[i].Line });
            }

            Expect(">");
            Expect("(");
            var x = ParseComma();
            Expect(")");
            return new CastExpr { Type = aType, X = x, Kind = k.Text };
        }

        // Handle(T)::DownCast(x) and Handle(T)() / Handle(T)(x)
        if (k.Text == "Handle" && At("("))
        {
            i--;
            int j = i;
            var aType = Parser.ParseType(t, ref j)!;
            i = j;
            if (At("::"))
            {
                Next();
                var aFn = Next();
                return new NameExpr { Name = "occ::handle::" + aFn.Text, TemplateArgs = [aType.Args[0]] };
            }

            var tc = new TypeCallExpr { Type = aType };
            if (At("("))
            {
                tc.Args = ParseArgs("(", ")");
            }

            return tc;
        }

        // a qualified name, maybe with template arguments
        string aName = k.Text;
        List<CppType>? aTemplateArgs = null;
        while (true)
        {
            if (aTemplateArgs == null && At("<")
                && (myIsType(aName) || aName is "occ::down_cast" or "occ::handle" or "std::array" || IsTemplateCall()))
            {
                aTemplateArgs = ParseTemplateArgs();
            }

            if (At("::") && Peek(1) is { } n && (n.IsIdent || n.Is("~")))
            {
                Next();
                var aPart = Next();
                aName += "::" + aPart.Text;
                continue;
            }

            break;
        }

        // T(...) or T{...} where T is a type: a temporary
        if (myIsType(aName) && !aName.Contains("::", StringComparison.Ordinal) || aName is "occ::handle")
        {
            if (At("(") || At("{"))
            {
                var aType = new CppType { Name = aName, Args = aTemplateArgs ?? [] };
                var tc = new TypeCallExpr { Type = aType };
                tc.Args = At("(") ? ParseArgs("(", ")") : ParseArgs("{", "}");
                return tc;
            }
        }

        return new NameExpr { Name = aName, TemplateArgs = aTemplateArgs };
    }
}
