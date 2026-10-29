// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// Draw variables. In Draw a drawable is held by the Tcl variable of its name, through a trace that drops it when the
/// variable is written or unset; Draw-lite keeps it on the variable itself, so upvar links and procedure frames behave
/// as Draw's do (Draw_VariableCommands.cxx: Draw::Set, Draw::Get, tracevar).
/// </summary>
public sealed partial class DrawInterpreter
{
    /// <summary>The trace on writes: a protected drawable refuses the write, any other is dropped.</summary>
    private static void WriteTrace(TclVar theVar, string? theIndex, string theDisplayName)
    {
        if (theIndex is null)
        {
            if (theVar.Drawable is { } aDrawable)
            {
                if (aDrawable.Protected)
                {
                    throw new DrawException($"can't set \"{theDisplayName}\": variable is protected");
                }

                theVar.Drawable = null;
            }
        }
        else
        {
            theVar.ElementDrawables?.Remove(theIndex);
        }
    }

    /// <summary>
    /// Draw::Set(name, drawable): binds a drawable (or nothing) to the variable of that name in the current frame,
    /// whose value becomes the name. A protected variable is left as it is, with Draw's message.
    /// </summary>
    public void SetDrawable(string theName, DrawDrawable? theDrawable)
    {
        if (theName == ".")
        {
            // the picked drawable: only redisplayed in Draw
            return;
        }

        if (GetDrawable(theName) is { Protected: true })
        {
            Output.WriteLine("variable is protected");
            return;
        }

        UnsetVariable(theName);
        if (theDrawable is null)
        {
            return;
        }

        (string aName, string? anIndex) = SplitName(theName);
        TclVar aVar = FindVar(aName, true)!;
        if (anIndex is null)
        {
            aVar.Array = null;
            aVar.Value = theName;
            aVar.Drawable = theDrawable;
        }
        else
        {
            if (aVar.Value is not null)
            {
                throw new DrawException($"can't set \"{theName}\": variable isn't array");
            }

            aVar.Array ??= new Dictionary<string, string>(StringComparer.Ordinal);
            aVar.Array[anIndex] = theName;
            aVar.ElementDrawables ??= new Dictionary<string, DrawDrawable>(StringComparer.Ordinal);
            aVar.ElementDrawables[anIndex] = theDrawable;
        }

        theDrawable.Name = theName;
    }

    /// <summary>Draw::Get(name): the drawable held by the variable of that name in the current frame, or null.</summary>
    public DrawDrawable? GetDrawable(string theName)
    {
        if (theName == ".")
        {
            // Draw asks to pick an object in a view
            return null;
        }

        (string aName, string? anIndex) = SplitName(theName);
        TclVar? aVar = FindVar(aName, false);
        if (aVar is null)
        {
            return null;
        }

        return anIndex is null ? aVar.Drawable
             : aVar.ElementDrawables is { } anElements && anElements.TryGetValue(anIndex, out DrawDrawable? aDrawable) ? aDrawable
             : null;
    }

    /// <summary>DBRep::Set: binds a shape, frozen first (decision 272), to the variable.</summary>
    public void SetShape(string theName, in TopoDS_Shape theShape)
    {
        if (!theShape.IsNull())
        {
            theShape.TShape()!.FreezeGraph();
        }

        SetDrawable(theName, new DrawShape(theShape));
    }

    /// <summary>DBRep::Get: the shape of the variable, or a null shape when it holds no shape.</summary>
    public TopoDS_Shape GetShape(string theName) => GetDrawable(theName) is DrawShape aShape ? aShape.Shape : new TopoDS_Shape();

    /// <summary>
    /// DBRep::Get(name, type, complain): the shape when it is of the type, or a null shape, with Draw's message
    /// when it isn't.
    /// </summary>
    public TopoDS_Shape GetShape(string theName, TopAbs_ShapeEnum theType, bool theToComplain = true)
    {
        TopoDS_Shape aShape = GetShape(theName);
        if (aShape.IsNull() || theType == TopAbs_ShapeEnum.TopAbs_SHAPE || theType == aShape.ShapeType())
        {
            return aShape;
        }

        if (theToComplain)
        {
            Output.WriteLine($"{theName} is not a {TopAbs.ShapeTypeToString(theType)} but a {TopAbs.ShapeTypeToString(aShape.ShapeType())}");
        }

        return new TopoDS_Shape();
    }

    /// <summary>Draw::Set(name, value): updates the Draw_Number of that name, or binds a new one.</summary>
    public void SetNumber(string theName, double theValue)
    {
        if (GetDrawable(theName) is DrawNumber aNumber)
        {
            aNumber.Value = theValue;
        }
        else
        {
            SetDrawable(theName, new DrawNumber(theValue));
        }
    }

    /// <summary>Draw::Get(name, value): the value of the Draw_Number of that name.</summary>
    public bool TryGetNumber(string theName, out double theValue)
    {
        if (GetDrawable(theName) is DrawNumber aNumber)
        {
            theValue = aNumber.Value;
            return true;
        }

        theValue = 0;
        return false;
    }

    /// <summary>The protected numbers Draw::VariableCommands creates: pi, pinf and minf.</summary>
    internal void SetDrawConstants()
    {
        foreach ((string aName, double aValue) in new[] { ("pi", Math.PI), ("pinf", double.MaxValue), ("minf", -double.MaxValue) })
        {
            SetNumber(aName, aValue);
            GetDrawable(aName)!.Protected = true;
        }
    }

    // -------------------------------------------------------------------------------------------------------------
    // Draw::Atof: the arguments of Draw's commands are expressions of numbers, Draw numbers and function calls

    private bool myParseFailed;

    /// <summary>
    /// Draw::Atof: evaluates <c>+ - * /</c>, parentheses, numbers, the names of Draw numbers and calls
    /// <c>f(a,b)</c> of commands; 0 where it can't.
    /// </summary>
    public double Atof(string theText)
    {
        myParseFailed = false;
        int i = 0;
        double x = Parse(theText, ref i);
        while (i < theText.Length && theText[i] is ' ' or '\t')
        {
            i++;
        }

        if (i < theText.Length)
        {
            myParseFailed = true;
        }

        return x;
    }

    /// <summary>Draw::ParseReal: false when the text isn't an expression.</summary>
    public bool ParseReal(string theText, out double theValue)
    {
        double aValue = Atof(theText);
        if (myParseFailed)
        {
            myParseFailed = false;
            theValue = 0;
            return false;
        }

        theValue = aValue;
        return true;
    }

    /// <summary>Draw::Atoi: Atof cast to an integer.</summary>
    public int Atoi(string theText) => DoubleToInt(Atof(theText));

    /// <summary>C's (int) cast, with the x64 result for values out of range (INT_MIN).</summary>
    internal static int DoubleToInt(double theValue)
        => double.IsNaN(theValue) || theValue >= 2147483648.0 || theValue <= -2147483649.0 ? int.MinValue : (int)theValue;

    private static bool Numeric(char c) => c == '.' || char.IsAsciiDigit(c);

    private static bool Alphabetic(char c) => char.IsAsciiLetter(c) || c == '_';

    private static char At(string s, int i) => i < s.Length ? s[i] : '\0';

    private double ParseValue(string s, ref int i)
    {
        while (At(s, i) is ' ' or '\t')
        {
            i++;
        }

        double x = 0;
        switch (At(s, i))
        {
            case '\0':
                break;
            case '(':
                i++;
                x = Parse(s, ref i);
                if (At(s, i) != ')')
                {
                    Output.WriteLine("Mismatched parenthesis");
                }

                i++;
                break;
            case '+':
                i++;
                x = ParseValue(s, ref i);
                break;
            case '-':
                i++;
                x = -ParseValue(s, ref i);
                break;
            default:
            {
                int p = i;
                while (Numeric(At(s, p)))
                {
                    p++;
                }

                // scientific notation
                if (At(s, p) is 'e' or 'E' && (Numeric(At(s, p + 1)) || At(s, p + 1) is '+' or '-'))
                {
                    p += 2;
                }

                while (Numeric(At(s, p)) || Alphabetic(At(s, p)))
                {
                    p++;
                }

                string aWord = s[i..Math.Min(p, s.Length)];
                if (aWord.Length > 0 && Numeric(aWord[0]))
                {
                    x = Standard_Stream.Strtod(aWord);
                }
                else if (!TryGetNumber(aWord, out x))
                {
                    x = CallFunction(s, ref p, aWord);
                }

                i = p;
                break;
            }
        }

        while (At(s, i) is ' ' or '\t')
        {
            i++;
        }

        return x;
    }

    // name(a, b, ...): the command "name a b ..." and its result read as a real
    private double CallFunction(string s, ref int p, string theName)
    {
        int q = p;
        while (At(s, q) is ' ' or '\t')
        {
            q++;
        }

        if (At(s, q) != '(')
        {
            myParseFailed = true;
            return 0;
        }

        int aDepth = 1;
        var anArgs = new List<string>();
        int aStart = q + 1;
        q++;
        while (aDepth > 0 && q < s.Length)
        {
            char c = s[q];
            if (c == '(')
            {
                aDepth++;
            }
            else if (c == ')')
            {
                aDepth--;
            }

            if ((aDepth == 1 && c == ',') || aDepth == 0)
            {
                anArgs.Add(s[aStart..q]);
                aStart = q + 1;
            }

            q++;
        }

        if (aDepth > 0)
        {
            Output.WriteLine("Unclosed parenthesis");
            p = q;
            return 0;
        }

        p = q;
        string aCall = theName + " " + string.Join(" ", anArgs);
        try
        {
            return Standard_Stream.Strtod(EvalScript(aCall));
        }
        catch (DrawException)
        {
            Output.WriteLine($"Call of function {aCall} failed");
            return 0;
        }
    }

    private double ParseFactor(string s, ref int i)
    {
        double x = ParseValue(s, ref i);
        while (true)
        {
            switch (At(s, i))
            {
                case '*':
                    i++;
                    x *= ParseValue(s, ref i);
                    break;
                case '/':
                    i++;
                    x /= ParseValue(s, ref i);
                    break;
                default:
                    return x;
            }
        }
    }

    private double Parse(string s, ref int i)
    {
        double x = ParseFactor(s, ref i);
        while (true)
        {
            switch (At(s, i))
            {
                case '+':
                    i++;
                    x += ParseFactor(s, ref i);
                    break;
                case '-':
                    i++;
                    x -= ParseFactor(s, ref i);
                    break;
                default:
                    return x;
            }
        }
    }
}
