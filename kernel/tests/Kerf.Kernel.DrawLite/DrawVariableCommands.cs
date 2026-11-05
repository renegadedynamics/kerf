// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// Draw's variable commands (Draw_VariableCommands.cxx): bsave, brestore, isdraw, isprot, dtyp, dval, dname, dump,
/// copy, renamevar, dset, protect and unprotect, autodisplay, lastrep, and the numeric functions.
/// </summary>
/// <remarks>
/// <c>lastrep</c> returns the last pick, which only <c>pick</c> in a view sets: Draw-lite has no views, so it always
/// returns OCCT's initial values (view 0, at 0 0, button 0, no name), as a DRAWEXE that has picked nothing. With
/// four names it converts the pick by the view's zoom, which a batch DRAWEXE gives as 0, so the coordinates are NaN.
/// </remarks>
internal static class DrawVariableCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "bsave", Save);
        DrawCxx.Add(theInterpreter, "brestore", Restore);
        DrawCxx.Add(theInterpreter, "isdraw", IsDraw);
        DrawCxx.Add(theInterpreter, "isprot", IsProt);
        DrawCxx.Add(theInterpreter, "dtyp", Whatis);
        DrawCxx.Add(theInterpreter, "dval", Value);
        DrawCxx.Add(theInterpreter, "dname", DName);
        DrawCxx.Add(theInterpreter, "dump", Dump);
        DrawCxx.Add(theInterpreter, "copy", Copy);
        DrawCxx.Add(theInterpreter, "renamevar", Copy);
        DrawCxx.Add(theInterpreter, "dset", Set);
        DrawCxx.Add(theInterpreter, "protect", Protect);
        DrawCxx.Add(theInterpreter, "unprotect", Protect);
        DrawCxx.Add(theInterpreter, "autodisplay", AutoDisplay);
        DrawCxx.Add(theInterpreter, "lastrep", LastRep);
        foreach (string aName in new[] { "cos", "sin", "tan", "acos", "asin", "atan2", "sqrt" })
        {
            DrawCxx.Add(theInterpreter, aName, Trigo);
        }

        theInterpreter.SetDrawConstants();
    }

    private static bool s_autoDisplay = true;

    // save
    private static int Save(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            di.Append("Syntax error: wrong number of arguments!\n");
            return 1;
        }

        DrawDrawable? aDrawable = theDI.GetDrawable(a[1]);
        if (aDrawable is null)
        {
            di.Append("Syntax error: '").Append(a[1]).Append("' is not a drawable");
            return 1;
        }

        var aStream = new StringWriter { NewLine = "\n" };
        StreamUtilities.Precision(aStream, 15);
        aStream.Write(aDrawable.TypeName + "\n");
        aDrawable.Save(aStream);
        aStream.Write("\n");
        aStream.Write("0\n\n");
        theDI.Files.WriteBytes(a[2], Encoding.Latin1.GetBytes(aStream.ToString()));
        di.Append(a[1]);
        return 0;
    }

    // brestore: the drawable whose type the file's first word names, else a shape read from the start
    private static int Restore(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            return 1;
        }

        string aFileName = a[1];
        string aVarName = a[2];
        if (!theDI.Files.TryReadBytes(aFileName, out byte[] aBytes))
        {
            di.Append("Error: cannot open file for reading: '").Append(aFileName).Append("'");
            return 1;
        }

        string aText = Encoding.Latin1.GetString(aBytes);
        TextReader aStream = Reader(aText);
        string? aType = StreamUtilities.ReadWord(aStream);
        if (aType is null || StreamUtilities.Fail(aStream))
        {
            di.Append("Error: cannot read file: '").Append(aFileName).Append("'");
            return 1;
        }

        DrawDrawable? aDrawable = RestoreDrawable(aType, aStream);
        if (aDrawable is null)
        {
            // assume that this file stores a DBRep_DrawableShape variable
            aDrawable = RestoreDrawable("DBRep_DrawableShape", Reader(aText));
        }

        if (aDrawable is null)
        {
            di.Append("Error: cannot restore a ").Append(aType);
            return 1;
        }

        if (aDrawable is DrawShape aShape && !aShape.Shape.IsNull)
        {
            aShape.Shape.TShape()!.FreezeGraph();
        }

        theDI.SetDrawable(aVarName, aDrawable);
        di.Append(aVarName);
        return 0;
    }

    private static StringReader Reader(string theText)
    {
        var aReader = new StringReader(theText);
        StreamUtilities.SetInputLength(aReader, theText.Length);
        return aReader;
    }

    /// <summary>Draw_Drawable3D::Restore: the factory of the type, or null when no factory has that name.</summary>
    internal static DrawDrawable? RestoreDrawable(string theType, TextReader theStream)
    {
        switch (theType)
        {
            case "DBRep_DrawableShape":
            {
                // DBRep_DrawableShape::Restore
                var aShapeSet = new BrepShapeSet();
                aShapeSet.Read(theStream);
                aShapeSet.Read(out Shape aShape, theStream);
                return new DrawShape(aShape);
            }

            case "Draw_Number":
            {
                double aValue = double.MaxValue;
                string? aWord = StreamUtilities.ReadWord(theStream);
                if (aWord is not null)
                {
                    aValue = StreamUtilities.Strtod(aWord);
                }

                return new DrawNumber(aValue);
            }

            default:
                return DrawGeometry.Restore(theType, theStream);
        }
    }

    private static int IsDraw(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }

        di.Append(theDI.GetDrawable(a[1]) is null ? "0" : "1");
        return 0;
    }

    private static int IsProt(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }

        di.Append(theDI.GetDrawable(a[1]) is { Protected: true } ? "1" : "0");
        return 0;
    }

    // dtyp (Draw's "whatis" function)
    private static int Whatis(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count <= 1)
        {
            return 1;
        }

        for (int i = 1; i < a.Count; i++)
        {
            theDI.GetDrawable(a[i])?.Whatis(di);
        }

        return 0;
    }

    private static int Value(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 2)
        {
            return 1;
        }

        di.Append(theDI.Atof(a[1]));
        return 0;
    }

    private static int DName(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count <= 1)
        {
            return 1;
        }

        for (int i = 1; i < a.Count; ++i)
        {
            if (theDI.GetDrawable(a[i]) is { } aDrawable)
            {
                di.Append(aDrawable.Name);
            }
        }

        return 0;
    }

    private static int Dump(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }

        for (int i = 1; i < a.Count; i++)
        {
            if (theDI.GetDrawable(a[i]) is { } aDrawable)
            {
                var aStream = new StringWriter { NewLine = "\n" };
                StreamUtilities.Precision(aStream, 15);
                aStream.Write("\n\n*********** Dump of " + a[i] + " *************\n");
                aDrawable.Dump(aStream);
                di.Append(aStream.ToString());
            }
        }

        return 0;
    }

    // copy and renamevar
    private static int Copy(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        bool aCopy = a[0].Equals("copy", StringComparison.OrdinalIgnoreCase);
        for (int i = 1; i < a.Count; i += 2)
        {
            if (i + 1 >= a.Count)
            {
                return 0;
            }

            DrawDrawable? aDrawable = theDI.GetDrawable(a[i]);
            if (aDrawable is not null)
            {
                if (aCopy)
                {
                    aDrawable = aDrawable.Copy();
                }
                else
                {
                    // clear old name
                    theDI.SetDrawable(a[i], null);
                }

                theDI.SetDrawable(a[i + 1], aDrawable);
            }
        }

        return 0;
    }

    // dset var1 value1 var2 value2 ...
    private static int Set(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }

        double aValue = 0;
        for (int i = 1; i < a.Count; i += 2)
        {
            aValue = 0;
            if (i + 1 < a.Count)
            {
                aValue = theDI.Atof(a[i + 1]);
            }

            theDI.SetNumber(a[i], aValue);
        }

        di.Append(aValue);
        return 0;
    }

    // the last pick (Draw_VariableCommands.cxx's p_id, p_X, p_Y, p_b, p_Name): never set without a view
    private const int p_id = 0;
    private const int p_X = 0;
    private const int p_Y = 0;
    private const int p_b = 0;
    private const string p_Name = "";

    // lastrep id X Y [Z] b, return name
    private static int LastRep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        theDI.SetNumber(a[1], p_id);
        if (n == 5)
        {
            theDI.SetNumber(a[2], p_X);
            theDI.SetNumber(a[3], p_Y);
        }
        else if (n == 6)
        {
            // OCCT: dout.Zoom(p_id) is false (0) in batch mode, so the pick is NaN; dout.GetTrsf leaves the identity
            double z = 0.0;
            var P = new Point3D(p_X / z, p_Y / z, 0);
            var T = new Transform3D();
            T = T.Inverted();
            P = P.Transformed(T);
            theDI.SetNumber(a[2], P.X);
            theDI.SetNumber(a[3], P.Y);
            theDI.SetNumber(a[4], P.Z);
        }
        else
        {
            di.Append("Too many args");
            return 1;
        }
        theDI.SetNumber(a[n - 1], p_b);
        di.Append(p_Name);
        return 0;
    }

    private static int Protect(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count <= 1)
        {
            return 1;
        }

        bool aProtect = a[0][0] != 'u';
        for (int i = 1; i < a.Count; i++)
        {
            if (theDI.GetDrawable(a[i]) is { } aDrawable)
            {
                aDrawable.Protected = aProtect;
                di.Append(a[i]).Append(" ");
            }
        }

        return 0;
    }

    private static int AutoDisplay(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        s_autoDisplay = a.Count <= 1 ? !s_autoDisplay : !a[1].Equals("0", StringComparison.OrdinalIgnoreCase);
        di.Append(s_autoDisplay ? "1" : "0");
        return 0;
    }

    private static int Trigo(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        double x = theDI.Atof(a.Count > 1 ? a[1] : "");
        switch (a[0].ToLowerInvariant())
        {
            case "cos":
                di.Append(Math.Cos(x));
                break;
            case "sin":
                di.Append(Math.Sin(x));
                break;
            case "tan":
                di.Append(Math.Tan(x));
                break;
            case "sqrt":
                di.Append(Math.Sqrt(x));
                break;
            case "acos":
                di.Append(Math.Acos(x));
                break;
            case "asin":
                di.Append(Math.Asin(x));
                break;
            case "atan2":
                di.Append(Math.Atan2(x, theDI.Atof(a.Count > 2 ? a[2] : "")));
                break;
        }

        return 0;
    }
}
