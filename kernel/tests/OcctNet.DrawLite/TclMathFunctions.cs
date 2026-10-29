// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DrawLite;

/// <summary>expr's math functions (Tcl 8.6 mathfunc).</summary>
internal static class TclMathFunctions
{
    [ThreadStatic]
    private static Random? t_random;

    public static TclValue Call(DrawInterpreter theInterpreter, string theName, TclValue[] theArgs)
    {
        switch (theName)
        {
            case "abs":
            {
                TclValue v = One(theName, theArgs);
                return v.Type == TclValue.Kind.Int
                           ? (v.Int == long.MinValue ? TclValue.FromDouble(Math.Abs((double)v.Int)) : TclValue.FromInt(Math.Abs(v.Int)))
                           : TclValue.FromDouble(Math.Abs(v.Double));
            }

            case "int" or "wide" or "entier":
            {
                TclValue v = One(theName, theArgs);
                if (v.Type == TclValue.Kind.Int)
                {
                    return TclValue.FromInt(v.Int);
                }

                return Truncate(v.Double);
            }

            case "round":
            {
                TclValue v = One(theName, theArgs);
                return v.Type == TclValue.Kind.Int ? TclValue.FromInt(v.Int) : Truncate(Math.Round(v.Double, MidpointRounding.AwayFromZero));
            }

            case "double":
                return TclValue.FromDouble(One(theName, theArgs).Double);
            case "bool":
                return TclValue.FromBool(TclExpr.ToBool(Raw(theName, theArgs, 1)[0]));
            case "isqrt":
            {
                TclValue v = One(theName, theArgs);
                double d = v.Double;
                if (d < 0)
                {
                    throw new DrawException("square root of negative argument");
                }

                return TclValue.FromInt((long)Math.Floor(Math.Sqrt(d)));
            }

            case "min" or "max":
            {
                if (theArgs.Length == 0)
                {
                    throw new DrawException($"too few arguments to math function \"{theName}\"");
                }

                TclValue aBest = theArgs[0];
                TclExpr.RequireNumber(aBest, theName);
                foreach (TclValue v in theArgs.Skip(1))
                {
                    TclExpr.RequireNumber(v, theName);
                    bool aBetter = theName == "min" ? v.Double < aBest.Double : v.Double > aBest.Double;
                    if (v.Type == TclValue.Kind.Int && aBest.Type == TclValue.Kind.Int)
                    {
                        aBetter = theName == "min" ? v.Int < aBest.Int : v.Int > aBest.Int;
                    }

                    if (aBetter)
                    {
                        aBest = v;
                    }
                }

                return aBest;
            }

            case "rand":
                Raw(theName, theArgs, 0);
                return TclValue.FromDouble((t_random ??= new Random()).NextDouble());
            case "srand":
            {
                TclValue v = One(theName, theArgs);
                t_random = new Random((int)TclExpr.RequireInt(v, theName));
                return TclValue.FromDouble(t_random.NextDouble());
            }
        }

        Func<double, double>? aUnary = theName switch
        {
            "sqrt" => Math.Sqrt,
            "exp" => Math.Exp,
            "log" => Math.Log,
            "log10" => Math.Log10,
            "sin" => Math.Sin,
            "cos" => Math.Cos,
            "tan" => Math.Tan,
            "asin" => Math.Asin,
            "acos" => Math.Acos,
            "atan" => Math.Atan,
            "sinh" => Math.Sinh,
            "cosh" => Math.Cosh,
            "tanh" => Math.Tanh,
            "floor" => Math.Floor,
            "ceil" => Math.Ceiling,
            _ => null,
        };
        if (aUnary is not null)
        {
            return TclExpr.CheckDouble(aUnary(One(theName, theArgs).Double));
        }

        Func<double, double, double>? aBinary = theName switch
        {
            "pow" => Math.Pow,
            "atan2" => Math.Atan2,
            "hypot" => double.Hypot,
            "fmod" => static (x, y) => x % y,
            _ => null,
        };
        if (aBinary is not null)
        {
            TclValue[] anArgs = Raw(theName, theArgs, 2);
            TclExpr.RequireNumber(anArgs[0], theName);
            TclExpr.RequireNumber(anArgs[1], theName);
            if (theName == "fmod" && anArgs[1].Double == 0)
            {
                throw new DrawException("domain error: argument not in valid range");
            }

            return TclExpr.CheckDouble(aBinary(anArgs[0].Double, anArgs[1].Double));
        }

        // A proc tcl::mathfunc::name is a user-defined function
        if (theInterpreter.HasCommand($"tcl::mathfunc::{theName}"))
        {
            var aCall = new List<string> { $"tcl::mathfunc::{theName}" };
            aCall.AddRange(theArgs.Select(a => a.Text));
            return TclValue.FromString(theInterpreter.Invoke(aCall));
        }

        throw new DrawException($"invalid command name \"tcl::mathfunc::{theName}\"");
    }

    private static TclValue Truncate(double d)
    {
        if (double.IsNaN(d) || double.IsInfinity(d))
        {
            throw new DrawException("integer value too large to represent");
        }

        double t = Math.Truncate(d);
        return t is >= -9.2233720368547758e18 and < 9.2233720368547758e18 ? TclValue.FromInt((long)t) : TclValue.FromDouble(t);
    }

    private static TclValue One(string theName, TclValue[] theArgs)
    {
        TclValue v = Raw(theName, theArgs, 1)[0];
        TclExpr.RequireNumber(v, theName);
        return v;
    }

    private static TclValue[] Raw(string theName, TclValue[] theArgs, int theCount)
    {
        if (theArgs.Length < theCount)
        {
            throw new DrawException($"too few arguments to math function \"{theName}\"");
        }

        if (theArgs.Length > theCount)
        {
            throw new DrawException($"too many arguments to math function \"{theName}\"");
        }

        return theArgs;
    }
}
