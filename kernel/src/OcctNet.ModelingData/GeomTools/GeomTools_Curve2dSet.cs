// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_Curve2dSet.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_Curve2dSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A set of Geom2d curves, indexed from 1, written and read in OCCT's text format (the "Curve2ds" section of a
/// <c>.brep</c> file).
/// </summary>
/// <remarks>
/// As <see cref="GeomTools_CurveSet"/>: OCCT's exact output, C++'s extraction when reading, curves read are frozen,
/// and curves that can't be read are one null entry. Unlike the 3D set, and as in OCCT, <see cref="Add"/> and
/// <see cref="Index"/> accept null (the null entry).
/// </remarks>
public class GeomTools_Curve2dSet
{
    private const int LINE = 1;
    private const int CIRCLE = 2;
    private const int ELLIPSE = 3;
    private const int PARABOLA = 4;
    private const int HYPERBOLA = 5;
    private const int BEZIER = 6;
    private const int BSPLINE = 7;
    private const int TRIMMED = 8;
    private const int OFFSET = 9;

    // OCCT's NCollection_IndexedMap of handles, which hashes by address and can hold one null handle
    private readonly NCollection_IndexedMap<object> myMap = new(ReferenceEqualityComparer.Instance);

    /// <summary>An empty set of 2D curves.</summary>
    public GeomTools_Curve2dSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>Adds a curve to the set (once) and returns its index. Null is added too, as in OCCT.</summary>
    public int Add(Geom2d_Curve? S)
    {
        return myMap.Add((object?)S ?? GeomTools_CurveSet.theNullKey);
    }

    /// <summary>The curve of index I, or null if there is none.</summary>
    public Geom2d_Curve? Curve2d(int I)
    {
        if (I <= 0 || I > myMap.Extent())
        {
            return null;
        }

        return myMap[I] as Geom2d_Curve;
    }

    /// <summary>The index of the curve, 0 if it isn't in the set (null finds the null entry, as in OCCT).</summary>
    public int Index(Geom2d_Curve? S)
    {
        return myMap.FindIndex((object?)S ?? GeomTools_CurveSet.theNullKey);
    }

    private static void Out(TextWriter OS, double theValue) => Standard_Stream.Write(OS, theValue);

    private static void Out(TextWriter OS, int theValue) => Standard_Stream.Write(OS, theValue);

    private static string Setw(int theWidth, int theValue) => Standard_Stream.Setw(theWidth, theValue.ToString(CultureInfo.InvariantCulture));

    private static void Print(gp_Pnt2d P, TextWriter OS, bool compact)
    {
        Out(OS, P.X());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, P.Y());
        OS.Write(" ");
    }

    private static void Print(gp_Dir2d D, TextWriter OS, bool compact)
    {
        Out(OS, D.X());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, D.Y());
        OS.Write(" ");
    }

    private static void Print(Geom2d_Line L, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, LINE);
            OS.Write(" ");
        }
        else
        {
            OS.Write("Line");
        }

        gp_Lin2d C2d = L.Lin2d();
        if (!compact)
        {
            OS.Write("\n  Origin :");
        }

        Print(C2d.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(C2d.Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom2d_Circle C, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, CIRCLE);
            OS.Write(" ");
        }
        else
        {
            OS.Write("Circle");
        }

        gp_Circ2d C2d = C.Circ2d();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(C2d.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(C2d.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(C2d.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radius :");
        }

        Out(OS, C2d.Radius());
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom2d_Ellipse E, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, ELLIPSE);
            OS.Write(" ");
        }
        else
        {
            OS.Write("Ellipse");
        }

        gp_Elips2d C2d = E.Elips2d();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(C2d.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(C2d.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(C2d.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radii  :");
        }

        Out(OS, C2d.MajorRadius());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, C2d.MinorRadius());
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom2d_Parabola P, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, PARABOLA);
            OS.Write(" ");
        }
        else
        {
            OS.Write("Parabola");
        }

        gp_Parab2d C2d = P.Parab2d();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(C2d.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(C2d.Axis().XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(C2d.Axis().YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Focal  :");
        }

        Out(OS, C2d.Focal());
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom2d_Hyperbola H, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, HYPERBOLA);
            OS.Write(" ");
        }
        else
        {
            OS.Write("Hyperbola");
        }

        gp_Hypr2d C2d = H.Hypr2d();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(C2d.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(C2d.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(C2d.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radii  :");
        }

        Out(OS, C2d.MajorRadius());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, C2d.MinorRadius());
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom2d_BezierCurve B, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, BEZIER);
            OS.Write(" ");
        }
        else
        {
            OS.Write("BezierCurve");
        }

        bool rational = B.IsRational();
        if (compact)
        {
            Out(OS, rational ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (rational)
            {
                OS.Write(" rational");
            }
        }

        // poles and weights
        int i, degree = B.Degree();
        if (!compact)
        {
            OS.Write("\n  Degree :");
        }

        Out(OS, degree);
        OS.Write(" ");

        for (i = 1; i <= degree + 1; i++)
        {
            if (!compact)
            {
                OS.Write("\n  ");
                OS.Write(Setw(2, i));
                OS.Write(" : ");
            }

            Print(B.Pole(i), OS, compact);
            if (rational)
            {
                OS.Write(" ");
                Out(OS, B.Weight(i));
            }

            if (compact)
            {
                OS.Write(" ");
            }
        }

        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom2d_BSplineCurve B, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, BSPLINE);
            OS.Write(" ");
        }
        else
        {
            OS.Write("BSplineCurve");
        }

        bool rational = B.IsRational();
        if (compact)
        {
            Out(OS, rational ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (rational)
            {
                OS.Write(" rational");
            }
        }

        bool periodic = B.IsPeriodic();
        if (compact)
        {
            Out(OS, periodic ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (periodic)
            {
                OS.Write(" periodic");
            }
        }

        // poles and weights
        int i, degree, nbpoles, nbknots;
        degree = B.Degree();
        nbpoles = B.NbPoles();
        nbknots = B.NbKnots();
        if (!compact)
        {
            OS.Write("\n  Degree ");
        }
        else
        {
            OS.Write(" ");
        }

        Out(OS, degree);
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, nbpoles);
        if (!compact)
        {
            OS.Write(" Poles,");
        }

        OS.Write(" ");
        Out(OS, nbknots);
        OS.Write(" ");
        if (!compact)
        {
            OS.Write(" Knots");
        }

        if (!compact)
        {
            OS.Write("Poles :\n");
        }

        for (i = 1; i <= nbpoles; i++)
        {
            if (!compact)
            {
                OS.Write("\n  ");
                OS.Write(Setw(2, i));
                OS.Write(" : ");
            }
            else
            {
                OS.Write(" ");
            }

            Print(B.Pole(i), OS, compact);
            if (rational)
            {
                OS.Write(" ");
                Out(OS, B.Weight(i));
            }
        }

        OS.Write("\n");

        if (!compact)
        {
            OS.Write("Knots :\n");
        }

        for (i = 1; i <= nbknots; i++)
        {
            if (!compact)
            {
                OS.Write("\n  ");
                OS.Write(Setw(2, i));
                OS.Write(" : ");
            }
            else
            {
                OS.Write(" ");
            }

            Out(OS, B.Knot(i));
            OS.Write(" ");
            Out(OS, B.Multiplicity(i));
        }

        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom2d_TrimmedCurve C, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, TRIMMED);
            OS.Write(" ");
        }
        else
        {
            OS.Write("Trimmed curve\n");
        }

        if (!compact)
        {
            OS.Write("Parameters : ");
        }

        Out(OS, C.FirstParameter());
        OS.Write(" ");
        Out(OS, C.LastParameter());
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("Basis curve :\n");
        }

        PrintCurve2d(C.BasisCurve(), OS, compact);
    }

    private static void Print(Geom2d_OffsetCurve C, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, OFFSET);
            OS.Write(" ");
        }
        else
        {
            OS.Write("OffsetCurve");
        }

        if (!compact)
        {
            OS.Write("Offset : ");
        }

        Out(OS, C.Offset());
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("Basis curve :\n");
        }

        PrintCurve2d(C.BasisCurve(), OS, compact);
    }

    /// <summary>
    /// Writes the 2D curve; if <paramref name="compact"/>, in the format <see cref="ReadCurve2d"/> reads, otherwise as
    /// a readable dump, at the writer's precision. An unknown curve type goes to
    /// <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static void PrintCurve2d(Geom2d_Curve C, TextWriter OS, bool compact = false)
    {
        Type TheType = C.DynamicType();
        if (TheType == typeof(Geom2d_Circle))
        {
            Print((Geom2d_Circle)C, OS, compact);
        }
        else if (TheType == typeof(Geom2d_Line))
        {
            Print((Geom2d_Line)C, OS, compact);
        }
        else if (TheType == typeof(Geom2d_Ellipse))
        {
            Print((Geom2d_Ellipse)C, OS, compact);
        }
        else if (TheType == typeof(Geom2d_Parabola))
        {
            Print((Geom2d_Parabola)C, OS, compact);
        }
        else if (TheType == typeof(Geom2d_Hyperbola))
        {
            Print((Geom2d_Hyperbola)C, OS, compact);
        }
        else if (TheType == typeof(Geom2d_BezierCurve))
        {
            Print((Geom2d_BezierCurve)C, OS, compact);
        }
        else if (TheType == typeof(Geom2d_BSplineCurve))
        {
            Print((Geom2d_BSplineCurve)C, OS, compact);
        }
        else if (TheType == typeof(Geom2d_TrimmedCurve))
        {
            Print((Geom2d_TrimmedCurve)C, OS, compact);
        }
        else if (TheType == typeof(Geom2d_OffsetCurve))
        {
            Print((Geom2d_OffsetCurve)C, OS, compact);
        }
        else
        {
            GeomTools.GetUndefinedTypeHandler().PrintCurve2d(C, OS, compact);
        }
    }

    /// <summary>Writes a readable dump of the set.</summary>
    public void Dump(TextWriter OS)
    {
        int i, nbsurf = myMap.Extent();
        OS.Write("\n -------\n");
        OS.Write("Dump of ");
        Out(OS, nbsurf);
        OS.Write(" Curve2ds ");
        OS.Write("\n -------\n\n");

        for (i = 1; i <= nbsurf; i++)
        {
            OS.Write(Setw(4, i));
            OS.Write(" : ");
            // OCCT dereferences a null curve here
            PrintCurve2d(Curve2d(i)!, OS, false);
        }
    }

    /// <summary>
    /// Writes the set in the format <see cref="Read"/> reads: "Curve2ds", the count, then each curve, at 17
    /// significant digits (the writer's precision is restored afterwards).
    /// </summary>
    public void Write(TextWriter OS, Message_ProgressRange theProgress = default)
    {
        int prec = Standard_Stream.Precision(OS, 17);

        int i, nbsurf = myMap.Extent();
        OS.Write("Curve2ds ");
        Out(OS, nbsurf);
        OS.Write("\n");
        using var aPS = new Message_ProgressScope(theProgress, "2D Curves", nbsurf);
        for (i = 1; i <= nbsurf && aPS.More(); i++, aPS.Next())
        {
            // OCCT dereferences a null curve here
            PrintCurve2d(Curve2d(i)!, OS, true);
        }

        Standard_Stream.Precision(OS, prec);
    }

    // operator>> of the points, directions and curves

    private static void ReadPnt(TextReader IS, out gp_Pnt2d P)
    {
        double X, Y;
        GeomTools.GetReal(IS, out X);
        GeomTools.GetReal(IS, out Y);
        P = new gp_Pnt2d(X, Y);
    }

    private static void ReadDir(TextReader IS, ref gp_Dir2d D)
    {
        double X, Y;
        GeomTools.GetReal(IS, out X);
        GeomTools.GetReal(IS, out Y);
        D.SetCoord(X, Y);
    }

    private static Geom2d_Line ReadLine(TextReader IS)
    {
        gp_Pnt2d P;
        gp_Dir2d AX = new(gp_Dir2d.D.X);
        ReadPnt(IS, out P);
        ReadDir(IS, ref AX);
        return new Geom2d_Line(P, AX);
    }

    private static Geom2d_Circle ReadCircle(TextReader IS)
    {
        gp_Pnt2d P;
        gp_Dir2d AX = new(gp_Dir2d.D.X), AY = new(gp_Dir2d.D.X);
        double R;
        ReadPnt(IS, out P);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        GeomTools.GetReal(IS, out R);
        return new Geom2d_Circle(new gp_Ax22d(P, AX, AY), R);
    }

    private static Geom2d_Ellipse ReadEllipse(TextReader IS)
    {
        gp_Pnt2d P;
        gp_Dir2d AX = new(gp_Dir2d.D.X), AY = new(gp_Dir2d.D.X);
        double R1, R2;
        ReadPnt(IS, out P);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        GeomTools.GetReal(IS, out R1);
        GeomTools.GetReal(IS, out R2);
        return new Geom2d_Ellipse(new gp_Ax22d(P, AX, AY), R1, R2);
    }

    private static Geom2d_Parabola ReadParabola(TextReader IS)
    {
        gp_Pnt2d P;
        gp_Dir2d AX = new(gp_Dir2d.D.X), AY = new(gp_Dir2d.D.X);
        double R1;
        ReadPnt(IS, out P);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        GeomTools.GetReal(IS, out R1);
        return new Geom2d_Parabola(new gp_Ax22d(P, AX, AY), R1);
    }

    private static Geom2d_Hyperbola ReadHyperbola(TextReader IS)
    {
        gp_Pnt2d P;
        gp_Dir2d AX = new(gp_Dir2d.D.X), AY = new(gp_Dir2d.D.X);
        double R1, R2;
        ReadPnt(IS, out P);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        GeomTools.GetReal(IS, out R1);
        GeomTools.GetReal(IS, out R2);
        return new Geom2d_Hyperbola(new gp_Ax22d(P, AX, AY), R1, R2);
    }

    private static Geom2d_BezierCurve ReadBezier(TextReader IS)
    {
        bool rational = false;
        Standard_Stream.Read(IS, ref rational);

        // poles and weights
        int i = 0, degree = 0;
        Standard_Stream.Read(IS, ref degree);

        var poles = new NCollection_Array1<gp_Pnt2d>(1, degree + 1);
        var weights = new NCollection_Array1<double>(1, degree + 1);

        for (i = 1; i <= degree + 1; i++)
        {
            ReadPnt(IS, out poles[i]);
            if (rational)
            {
                GeomTools.GetReal(IS, out weights[i]);
            }
        }

        if (rational)
        {
            return new Geom2d_BezierCurve(poles, weights);
        }

        return new Geom2d_BezierCurve(poles);
    }

    private static Geom2d_BSplineCurve ReadBSpline(TextReader IS)
    {
        bool rational = false, periodic = false;
        Standard_Stream.Read(IS, ref rational);
        Standard_Stream.Read(IS, ref periodic);

        // poles and weights
        int i = 0, degree = 0, nbpoles = 0, nbknots = 0;
        Standard_Stream.Read(IS, ref degree);
        Standard_Stream.Read(IS, ref nbpoles);
        Standard_Stream.Read(IS, ref nbknots);

        var poles = new NCollection_Array1<gp_Pnt2d>(1, nbpoles);
        var weights = new NCollection_Array1<double>(1, nbpoles);

        for (i = 1; i <= nbpoles; i++)
        {
            ReadPnt(IS, out poles[i]);
            if (rational)
            {
                GeomTools.GetReal(IS, out weights[i]);
            }
        }

        var knots = new NCollection_Array1<double>(1, nbknots);
        var mults = new NCollection_Array1<int>(1, nbknots);

        for (i = 1; i <= nbknots; i++)
        {
            GeomTools.GetReal(IS, out knots[i]);
            Standard_Stream.Read(IS, ref mults[i]);
        }

        if (rational)
        {
            return new Geom2d_BSplineCurve(poles, weights, knots, mults, degree, periodic);
        }

        return new Geom2d_BSplineCurve(poles, knots, mults, degree, periodic);
    }

    private static Geom2d_TrimmedCurve ReadTrimmed(TextReader IS)
    {
        double p1, p2;
        GeomTools.GetReal(IS, out p1);
        GeomTools.GetReal(IS, out p2);
        Geom2d_Curve? BC = ReadCurve2d(IS);
        return new Geom2d_TrimmedCurve(BC!, p1, p2);
    }

    private static Geom2d_OffsetCurve ReadOffset(TextReader IS)
    {
        double p;
        GeomTools.GetReal(IS, out p);
        Geom2d_Curve? BC = ReadCurve2d(IS);
        return new Geom2d_OffsetCurve(BC!, p);
    }

    /// <summary>
    /// Reads a 2D curve written by <see cref="PrintCurve2d"/> (compact). Returns null if the curve can't be built
    /// from what was read; an unknown type goes to <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static Geom2d_Curve? ReadCurve2d(TextReader IS)
    {
        // OCCT's ctype is uninitialised, and left so when the stream has already failed
        int ctype = 0;

        Geom2d_Curve? C = null;
        try
        {
            Standard_Stream.Read(IS, ref ctype);
            switch (ctype)
            {
                case LINE:
                    C = ReadLine(IS);
                    break;

                case CIRCLE:
                    C = ReadCircle(IS);
                    break;

                case ELLIPSE:
                    C = ReadEllipse(IS);
                    break;

                case PARABOLA:
                    C = ReadParabola(IS);
                    break;

                case HYPERBOLA:
                    C = ReadHyperbola(IS);
                    break;

                case BEZIER:
                    C = ReadBezier(IS);
                    break;

                case BSPLINE:
                    C = ReadBSpline(IS);
                    break;

                case TRIMMED:
                    C = ReadTrimmed(IS);
                    break;

                case OFFSET:
                    C = ReadOffset(IS);
                    break;

                default:
                {
                    Geom2d_Curve? CC = null;
                    GeomTools.GetUndefinedTypeHandler().ReadCurve2d(ctype, IS, ref CC);
                    C = CC;
                }
                    break;
            }
        }
        catch (Standard_Failure)
        {
            // OCCT_DEBUG only: "EXCEPTION in GeomTools_Curve2dSet::ReadCurve2d(..)!!!"
        }
        catch (NullReferenceException)
        {
            // a trimmed or offset curve whose basis couldn't be read: OCCT dereferences the null handle, which
            // OCC_CATCH_SIGNALS turns into a caught Standard_AccessViolation where signals are converted
        }

        return C;
    }

    /// <summary>
    /// Reads a set written by <see cref="Write"/>, adding its curves (frozen) to this one. Writes "Not a Curve2d
    /// table" to the console, as OCCT does, if the text doesn't start with "Curve2ds".
    /// </summary>
    public void Read(TextReader IS, Message_ProgressRange theProgress = default)
    {
        string? buffer = Standard_Stream.ReadWord(IS);
        if (buffer != "Curve2ds")
        {
            Console.Out.Write("Not a Curve2d table" + "\n");
            return;
        }

        // OCCT's nbcurve is uninitialised if the count can't be read
        int i, nbcurve = 0;
        Standard_Stream.Read(IS, ref nbcurve);
        using var aPS = new Message_ProgressScope(theProgress, "2D Curves", nbcurve);
        for (i = 1; i <= nbcurve && aPS.More(); i++, aPS.Next())
        {
            Geom2d_Curve? C = ReadCurve2d(IS);
            C?.Freeze();
            myMap.Add((object?)C ?? GeomTools_CurveSet.theNullKey);
        }
    }
}
