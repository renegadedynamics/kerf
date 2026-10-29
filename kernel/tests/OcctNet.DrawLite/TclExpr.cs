// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;

namespace OcctNet.DrawLite;

/// <summary>A value in an expression: an integer, a double, or a string that isn't a number.</summary>
internal readonly struct TclValue
{
    public enum Kind : byte
    {
        Int,
        Double,
        String,
    }

    public readonly Kind Type;
    public readonly long Int;
    public readonly double Double;
    private readonly string? myText;

    private TclValue(Kind theType, long theInt, double theDouble, string? theText)
    {
        Type = theType;
        Int = theInt;
        Double = theDouble;
        myText = theText;
    }

    public static TclValue FromInt(long theValue) => new(Kind.Int, theValue, theValue, null);

    public static TclValue FromDouble(double theValue) => new(Kind.Double, 0, theValue, null);

    public static TclValue FromBool(bool theValue) => FromInt(theValue ? 1 : 0);

    /// <summary>A string operand: a number when it reads as one, keeping its text for string comparisons.</summary>
    public static TclValue FromString(string theText)
    {
        if (TclNumber.TryParseInt(theText, out long anInt))
        {
            return new TclValue(Kind.Int, anInt, anInt, theText);
        }

        return TclNumber.TryParseDouble(theText, out double aDouble)
                   ? new TclValue(Kind.Double, 0, aDouble, theText)
                   : new TclValue(Kind.String, 0, 0, theText);
    }

    public bool IsNumber => Type != Kind.String;

    /// <summary>The value as a string: its original text, or the canonical form of a computed number.</summary>
    public string Text => myText ?? Canonical;

    /// <summary>The canonical form: numbers as Tcl prints them (expr's result), strings unchanged.</summary>
    public string Canonical => Type switch
    {
        Kind.Int => TclNumber.FormatInt(Int),
        Kind.Double => TclNumber.FormatDouble(Double),
        _ => myText!,
    };
}

/// <summary>
/// Tcl's expr: the operators of Tcl 8.6 with their precedence, numeric or string comparison, short-circuit
/// logic, the math functions, and $variable, [command] and "string" operands substituted during evaluation.
/// </summary>
internal static class TclExpr
{
    private static readonly ConcurrentDictionary<string, Node> s_cache = new(StringComparer.Ordinal);

    public static string Evaluate(DrawInterpreter theInterpreter, string theExpression)
        => Eval(theInterpreter, theExpression).Canonical;

    public static TclValue Eval(DrawInterpreter theInterpreter, string theExpression)
    {
        if (!s_cache.TryGetValue(theExpression, out Node? aNode))
        {
            aNode = new ExprParser(theExpression).ParseAll();
            if (s_cache.Count > 50000)
            {
                s_cache.Clear();
            }

            s_cache[theExpression] = aNode;
        }

        return aNode.Eval(theInterpreter);
    }

    public static bool EvalBoolean(DrawInterpreter theInterpreter, string theExpression)
    {
        TclValue aValue = Eval(theInterpreter, theExpression);
        return ToBool(aValue);
    }

    internal static bool ToBool(TclValue theValue) => theValue.Type switch
    {
        TclValue.Kind.Int => theValue.Int != 0,
        TclValue.Kind.Double => theValue.Double != 0,
        _ => TclNumber.GetBoolean(theValue.Text),
    };

    // ---------------------------------------------------------------------------------------------------------
    // Syntax tree

    internal abstract class Node
    {
        public abstract TclValue Eval(DrawInterpreter theInterpreter);
    }

    private sealed class Literal(TclValue theValue) : Node
    {
        public override TclValue Eval(DrawInterpreter theInterpreter) => theValue;
    }

    private sealed class Substituted(IReadOnlyList<WordPart> theParts) : Node
    {
        public override TclValue Eval(DrawInterpreter theInterpreter) => TclValue.FromString(theInterpreter.SubstituteParts(theParts));
    }

    private sealed class Unary(string theOp, Node theOperand) : Node
    {
        public override TclValue Eval(DrawInterpreter theInterpreter)
        {
            TclValue v = theOperand.Eval(theInterpreter);
            switch (theOp)
            {
                case "!":
                    return TclValue.FromBool(!ToBool(v));
                case "~":
                    return TclValue.FromInt(~RequireInt(v, theOp));
                case "-":
                    RequireNumber(v, theOp);
                    return v.Type == TclValue.Kind.Int
                               ? (v.Int == long.MinValue ? TclValue.FromDouble(-(double)v.Int) : TclValue.FromInt(-v.Int))
                               : TclValue.FromDouble(-v.Double);
                default:
                    RequireNumber(v, theOp);
                    return v.Type == TclValue.Kind.Int ? TclValue.FromInt(v.Int) : TclValue.FromDouble(v.Double);
            }
        }
    }

    private sealed class Logical(bool theIsAnd, Node theLeft, Node theRight) : Node
    {
        public override TclValue Eval(DrawInterpreter theInterpreter)
        {
            bool aLeft = ToBool(theLeft.Eval(theInterpreter));
            if (theIsAnd ? !aLeft : aLeft)
            {
                return TclValue.FromBool(aLeft);
            }

            return TclValue.FromBool(ToBool(theRight.Eval(theInterpreter)));
        }
    }

    private sealed class Ternary(Node theCondition, Node theThen, Node theElse) : Node
    {
        public override TclValue Eval(DrawInterpreter theInterpreter)
            => ToBool(theCondition.Eval(theInterpreter)) ? theThen.Eval(theInterpreter) : theElse.Eval(theInterpreter);
    }

    private sealed class Binary(string theOp, Node theLeft, Node theRight) : Node
    {
        public override TclValue Eval(DrawInterpreter theInterpreter)
            => Apply(theOp, theLeft.Eval(theInterpreter), theRight.Eval(theInterpreter));
    }

    private sealed class Function(string theName, IReadOnlyList<Node> theArgs) : Node
    {
        public override TclValue Eval(DrawInterpreter theInterpreter)
        {
            var anArgs = new TclValue[theArgs.Count];
            for (int i = 0; i < anArgs.Length; i++)
            {
                anArgs[i] = theArgs[i].Eval(theInterpreter);
            }

            return TclMathFunctions.Call(theInterpreter, theName, anArgs);
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // Operators

    internal static void RequireNumber(TclValue v, string theOp)
    {
        if (!v.IsNumber)
        {
            string aText = v.Text;
            string aWhat = aText.Length == 0 ? "empty string"
                         : LooksLikeBadOctal(aText) ? $"invalid octal number \"{aText}\""
                         : "non-numeric string";
            throw new DrawException($"can't use {aWhat} as operand of \"{theOp}\"");
        }
    }

    private static bool LooksLikeBadOctal(string s)
    {
        string t = s.Trim().TrimStart('+', '-');
        return t.Length > 1 && t[0] == '0' && t.All(char.IsAsciiDigit);
    }

    internal static long RequireInt(TclValue v, string theOp)
    {
        RequireNumber(v, theOp);
        if (v.Type != TclValue.Kind.Int)
        {
            throw new DrawException($"can't use floating-point value \"{v.Text}\" as operand of \"{theOp}\"");
        }

        return v.Int;
    }

    internal static TclValue CheckDouble(double theValue)
        => double.IsNaN(theValue) ? throw new DrawException("domain error: argument not in valid range") : TclValue.FromDouble(theValue);

    private static TclValue Apply(string theOp, TclValue a, TclValue b)
    {
        switch (theOp)
        {
            case "eq":
                return TclValue.FromBool(a.Text == b.Text);
            case "ne":
                return TclValue.FromBool(a.Text != b.Text);
            case "in" or "ni":
            {
                bool aFound = TclList.Parse(b.Text).Contains(a.Text);
                return TclValue.FromBool(theOp == "in" ? aFound : !aFound);
            }

            case "==" or "!=" or "<" or ">" or "<=" or ">=":
            {
                int aCompare = a.IsNumber && b.IsNumber ? CompareNumbers(a, b) : string.CompareOrdinal(a.Text, b.Text);
                if (aCompare == int.MinValue)
                {
                    // NaN compares false except !=
                    return TclValue.FromBool(theOp == "!=");
                }

                return TclValue.FromBool(theOp switch
                {
                    "==" => aCompare == 0,
                    "!=" => aCompare != 0,
                    "<" => aCompare < 0,
                    ">" => aCompare > 0,
                    "<=" => aCompare <= 0,
                    _ => aCompare >= 0,
                });
            }

            case "&" or "|" or "^" or "<<" or ">>":
            {
                long x = RequireInt(a, theOp);
                long y = RequireInt(b, theOp);
                return TclValue.FromInt(theOp switch
                {
                    "&" => x & y,
                    "|" => x | y,
                    "^" => x ^ y,
                    "<<" => y < 0 ? throw new DrawException("negative shift argument") : y >= 64 ? 0 : x << (int)y,
                    _ => y < 0 ? throw new DrawException("negative shift argument") : y >= 64 ? (x < 0 ? -1 : 0) : x >> (int)y,
                });
            }
        }

        RequireNumber(a, theOp);
        RequireNumber(b, theOp);
        if (a.Type == TclValue.Kind.Int && b.Type == TclValue.Kind.Int)
        {
            return IntArithmetic(theOp, a.Int, b.Int);
        }

        double x2 = a.Double;
        double y2 = b.Double;
        return theOp switch
        {
            "+" => CheckDouble(x2 + y2),
            "-" => CheckDouble(x2 - y2),
            "*" => CheckDouble(x2 * y2),
            "/" => CheckDouble(x2 / y2),
            "%" => throw new DrawException($"can't use floating-point value \"{(a.Type == TclValue.Kind.Double ? a.Text : b.Text)}\" as operand of \"%\""),
            "**" => CheckDouble(Math.Pow(x2, y2)),
            _ => throw new DrawException($"unknown operator \"{theOp}\""),
        };
    }

    private static int CompareNumbers(TclValue a, TclValue b)
    {
        if (a.Type == TclValue.Kind.Int && b.Type == TclValue.Kind.Int)
        {
            return a.Int.CompareTo(b.Int);
        }

        if (double.IsNaN(a.Double) || double.IsNaN(b.Double))
        {
            return int.MinValue;
        }

        return a.Double.CompareTo(b.Double);
    }

    private static TclValue IntArithmetic(string theOp, long x, long y)
    {
        try
        {
            switch (theOp)
            {
                case "+":
                    return TclValue.FromInt(checked(x + y));
                case "-":
                    return TclValue.FromInt(checked(x - y));
                case "*":
                    return TclValue.FromInt(checked(x * y));
                case "/":
                {
                    if (y == 0)
                    {
                        throw new DrawException("divide by zero");
                    }

                    // Tcl rounds the quotient down, towards minus infinity
                    long q = x / y;
                    if ((x % y != 0) && ((x < 0) != (y < 0)))
                    {
                        q--;
                    }

                    return TclValue.FromInt(q);
                }

                case "%":
                {
                    if (y == 0)
                    {
                        throw new DrawException("divide by zero");
                    }

                    // The remainder has the sign of the divisor
                    long r = x % y;
                    if (r != 0 && ((r < 0) != (y < 0)))
                    {
                        r += y;
                    }

                    return TclValue.FromInt(r);
                }

                case "**":
                    return TclValue.FromInt(IntPower(x, y));
                default:
                    throw new DrawException($"unknown operator \"{theOp}\"");
            }
        }
        catch (OverflowException)
        {
            // Tcl would continue with a bignum; the port continues in floating point
            return theOp switch
            {
                "+" => TclValue.FromDouble((double)x + y),
                "-" => TclValue.FromDouble((double)x - y),
                "*" => TclValue.FromDouble((double)x * y),
                _ => TclValue.FromDouble(Math.Pow(x, y)),
            };
        }
    }

    private static long IntPower(long x, long y)
    {
        if (y < 0)
        {
            return x switch
            {
                0 => throw new DrawException("exponentiation of zero by negative power"),
                1 => 1,
                -1 => (y & 1) == 0 ? 1 : -1,
                _ => 0,
            };
        }

        long aResult = 1;
        long aBase = x;
        while (y > 0)
        {
            if ((y & 1) != 0)
            {
                aResult = checked(aResult * aBase);
            }

            y >>= 1;
            if (y > 0)
            {
                aBase = checked(aBase * aBase);
            }
        }

        return aResult;
    }

    // ---------------------------------------------------------------------------------------------------------
    // Parser: precedence climbing over Tcl 8.6's operator table

    private sealed class ExprParser(string theText)
    {
        private readonly string s = theText;
        private int myPos;

        public Node ParseAll()
        {
            SkipSpace();
            if (myPos >= s.Length)
            {
                throw new DrawException($"empty expression\nin expression \"{s}\"");
            }

            Node aNode = ParseTernary();
            SkipSpace();
            if (myPos < s.Length)
            {
                throw Syntax($"extra tokens at end of expression");
            }

            return aNode;
        }

        private DrawException Syntax(string theWhat) => new($"syntax error in expression \"{s}\": {theWhat}");

        private void SkipSpace()
        {
            while (myPos < s.Length)
            {
                if (char.IsWhiteSpace(s[myPos]))
                {
                    myPos++;
                }
                else if (s[myPos] == '\\' && myPos + 1 < s.Length && s[myPos + 1] == '\n')
                {
                    myPos += 2;
                }
                else
                {
                    break;
                }
            }
        }

        private Node ParseTernary()
        {
            Node aCondition = ParseBinary(0);
            SkipSpace();
            if (myPos < s.Length && s[myPos] == '?')
            {
                myPos++;
                Node aThen = ParseTernary();
                SkipSpace();
                if (myPos >= s.Length || s[myPos] != ':')
                {
                    throw Syntax("missing \":\" in ternary");
                }

                myPos++;
                Node anElse = ParseTernary();
                return new Ternary(aCondition, aThen, anElse);
            }

            return aCondition;
        }

        // Lowest to highest; ** is handled separately (right-associative, above multiplication)
        private static readonly string[][] s_levels =
        [
            ["||"],
            ["&&"],
            ["|"],
            ["^"],
            ["&"],
            ["in", "ni"],
            ["eq", "ne"],
            ["==", "!="],
            ["<=", ">=", "<", ">"],
            ["<<", ">>"],
            ["+", "-"],
            ["*", "/", "%"],
        ];

        private Node ParseBinary(int theLevel)
        {
            if (theLevel == s_levels.Length)
            {
                return ParsePower();
            }

            Node aLeft = ParseBinary(theLevel + 1);
            while (true)
            {
                SkipSpace();
                string? anOp = MatchOperator(s_levels[theLevel]);
                if (anOp is null)
                {
                    return aLeft;
                }

                Node aRight = ParseBinary(theLevel + 1);
                aLeft = anOp switch
                {
                    "&&" => new Logical(true, aLeft, aRight),
                    "||" => new Logical(false, aLeft, aRight),
                    _ => new Binary(anOp, aLeft, aRight),
                };
            }
        }

        private string? MatchOperator(string[] theOps)
        {
            foreach (string anOp in theOps)
            {
                if (string.CompareOrdinal(s, myPos, anOp, 0, anOp.Length) != 0)
                {
                    continue;
                }

                int anEnd = myPos + anOp.Length;
                char aNext = anEnd < s.Length ? s[anEnd] : '\0';
                bool anOk = anOp switch
                {
                    // Don't split longer operators: & of &&, | of ||, * of **, < of <<, = of ==
                    "&" => aNext != '&',
                    "|" => aNext != '|',
                    "*" => aNext != '*',
                    "<" => aNext is not ('<' or '='),
                    ">" => aNext is not ('>' or '='),
                    "in" or "ni" or "eq" or "ne" => !char.IsLetterOrDigit(aNext) && aNext != '_',
                    _ => true,
                };
                if (anOk)
                {
                    myPos = anEnd;
                    return anOp;
                }
            }

            return null;
        }

        private Node ParsePower()
        {
            Node aBase = ParseUnary();
            SkipSpace();
            if (myPos + 1 < s.Length && s[myPos] == '*' && s[myPos + 1] == '*')
            {
                myPos += 2;
                Node anExponent = ParsePowerRight();
                return new Binary("**", aBase, anExponent);
            }

            return aBase;
        }

        // The right operand of ** may itself have unary operators and a further **
        private Node ParsePowerRight() => ParsePower();

        private Node ParseUnary()
        {
            SkipSpace();
            if (myPos < s.Length && s[myPos] is '-' or '+' or '!' or '~')
            {
                string anOp = s[myPos].ToString();
                myPos++;
                SkipSpace();
                if (anOp == "-" && myPos < s.Length && char.IsAsciiDigit(s[myPos]))
                {
                    // -9223372036854775808 is an integer although 9223372036854775808 alone is not
                    int aStart = myPos;
                    ReadNumber();
                    if (TclNumber.TryParseInt("-" + s[aStart..myPos], out long aNegative))
                    {
                        return new Literal(TclValue.FromInt(aNegative));
                    }

                    myPos = aStart;
                }

                Node anOperand = ParseUnary();
                if (anOperand is Literal aLiteral && anOp is "-" or "+" && aLiteral.Eval(null!) is { IsNumber: true } aNumber)
                {
                    // Fold signed numeric literals, so -9223372036854775808 and -Inf read as numbers
                    return new Literal(new Unary(anOp, new Literal(aNumber)).Eval(null!));
                }

                return new Unary(anOp, anOperand);
            }

            return ParsePrimary();
        }

        private Node ParsePrimary()
        {
            SkipSpace();
            if (myPos >= s.Length)
            {
                throw Syntax("premature end of expression");
            }

            char c = s[myPos];
            int aLine = 1;
            switch (c)
            {
                case '(':
                {
                    myPos++;
                    Node anInner = ParseTernary();
                    SkipSpace();
                    if (myPos >= s.Length || s[myPos] != ')')
                    {
                        throw Syntax("looking for close parenthesis");
                    }

                    myPos++;
                    return anInner;
                }

                case '$':
                {
                    WordPart.Variable? aVariable = TclParser.ReadVariable(s, ref myPos, ref aLine);
                    return aVariable is null ? throw Syntax("invalid character \"$\"") : new Substituted([aVariable]);
                }

                case '[':
                    return new Substituted([new WordPart.Command(TclParser.ReadBracketed(s, ref myPos, ref aLine))]);
                case '"':
                {
                    int aStart = ++myPos;
                    while (myPos < s.Length && s[myPos] != '"')
                    {
                        if (s[myPos] == '\\')
                        {
                            myPos++;
                        }
                        else if (s[myPos] == '[')
                        {
                            TclParser.ReadBracketed(s, ref myPos, ref aLine);
                            continue;
                        }

                        myPos++;
                    }

                    if (myPos >= s.Length)
                    {
                        throw Syntax("missing \"");
                    }

                    string aQuoted = s[aStart..myPos++];
                    IReadOnlyList<WordPart> aParts = TclParser.ParseSubst(aQuoted);
                    return aParts switch
                    {
                        [] => new Literal(TclValue.FromString("")),
                        [WordPart.Text aText] => new Literal(TclValue.FromString(aText.Value)),
                        _ => new Substituted(aParts),
                    };
                }

                case '{':
                    return new Literal(TclValue.FromString(TclParser.ReadBraced(s, ref myPos, ref aLine)));
            }

            if (char.IsAsciiDigit(c) || (c == '.' && myPos + 1 < s.Length && char.IsAsciiDigit(s[myPos + 1])))
            {
                return new Literal(ReadNumber());
            }

            if (char.IsLetter(c) || c == '_' || c == ':')
            {
                int aStart = myPos;
                while (myPos < s.Length && (char.IsLetterOrDigit(s[myPos]) || s[myPos] is '_' or ':'))
                {
                    myPos++;
                }

                string aWord = s[aStart..myPos];
                SkipSpace();
                if (myPos < s.Length && s[myPos] == '(')
                {
                    myPos++;
                    var anArgs = new List<Node>();
                    SkipSpace();
                    if (myPos < s.Length && s[myPos] == ')')
                    {
                        myPos++;
                    }
                    else
                    {
                        while (true)
                        {
                            anArgs.Add(ParseTernary());
                            SkipSpace();
                            if (myPos < s.Length && s[myPos] == ',')
                            {
                                myPos++;
                                continue;
                            }

                            if (myPos < s.Length && s[myPos] == ')')
                            {
                                myPos++;
                                break;
                            }

                            throw Syntax("missing close parenthesis at end of function call");
                        }
                    }

                    return new Function(aWord.TrimStart(':'), anArgs);
                }

                string aLower = aWord.ToLowerInvariant();
                if (aLower is "inf" or "infinity" or "nan")
                {
                    return new Literal(TclValue.FromString(aWord));
                }

                if (TclNumber.TryParseBoolean(aWord, out _))
                {
                    return new Literal(TclValue.FromString(aWord));
                }

                throw new DrawException($"invalid bareword \"{aWord}\"\nin expression \"{s}\"");
            }

            throw Syntax($"character not legal in expressions \"{c}\"");
        }

        private TclValue ReadNumber()
        {
            int aStart = myPos;
            if (s[myPos] == '0' && myPos + 1 < s.Length && s[myPos + 1] is 'x' or 'X' or 'b' or 'B' or 'o' or 'O')
            {
                myPos += 2;
                while (myPos < s.Length && char.IsAsciiHexDigit(s[myPos]))
                {
                    myPos++;
                }
            }
            else
            {
                while (myPos < s.Length && char.IsAsciiDigit(s[myPos]))
                {
                    myPos++;
                }

                if (myPos < s.Length && s[myPos] == '.')
                {
                    myPos++;
                    while (myPos < s.Length && char.IsAsciiDigit(s[myPos]))
                    {
                        myPos++;
                    }
                }

                if (myPos < s.Length && s[myPos] is 'e' or 'E')
                {
                    int aSave = myPos;
                    myPos++;
                    if (myPos < s.Length && s[myPos] is '+' or '-')
                    {
                        myPos++;
                    }

                    if (myPos < s.Length && char.IsAsciiDigit(s[myPos]))
                    {
                        while (myPos < s.Length && char.IsAsciiDigit(s[myPos]))
                        {
                            myPos++;
                        }
                    }
                    else
                    {
                        myPos = aSave;
                    }
                }
            }

            string aText = s[aStart..myPos];
            TclValue aValue = TclValue.FromString(aText);
            if (!aValue.IsNumber)
            {
                throw LooksLikeBadOctal(aText)
                          ? new DrawException($"expected integer but got \"{aText}\" (looks like invalid octal number)")
                          : Syntax($"bad number \"{aText}\"");
            }

            // A literal's value is its canonical number (expr {010} is 8)
            return aValue.Type == TclValue.Kind.Int ? TclValue.FromInt(aValue.Int) : TclValue.FromDouble(aValue.Double);
        }
    }
}
