// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_CurveSet.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_CurveSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A set of Geom curves, indexed from 1, written and read in OCCT's text format (the "Curves" section of a
/// <c>.brep</c> file).
/// </summary>
/// <remarks>
/// <see cref="Write"/> writes exactly what OCCT writes (17 significant digits, <c>'\n'</c> line ends);
/// <see cref="Read"/> reads OCCT's files, following C++'s stream extraction (<see cref="Standard_Stream"/>). The curves
/// that <see cref="Read"/> creates are frozen: the set shares them with whoever asks for them. A curve that can't be
/// read (its constructor throws) is stored as null, as in OCCT, which stores one null entry however many fail.
/// </remarks>
public class GeomTools_CurveSet
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

    // the null handle as a key
    internal static readonly object theNullKey = new();

    /// <summary>An empty set of curves.</summary>
    public GeomTools_CurveSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>Adds a curve to the set (once) and returns its index; 0 for null.</summary>
    public int Add(Geom_Curve? C)
    {
        return (C is null) ? 0 : myMap.Add(C);
    }

    /// <summary>The curve of index I, or null if there is none.</summary>
    public Geom_Curve? Curve(int I)
    {
        if (I <= 0 || I > myMap.Extent())
        {
            return null;
        }

        return myMap[I] as Geom_Curve;
    }

    /// <summary>The index of the curve, 0 if it isn't in the set.</summary>
    public int Index(Geom_Curve? S)
    {
        return S is null ? 0 : myMap.FindIndex(S);
    }

    private static void Out(TextWriter OS, double theValue) => Standard_Stream.Write(OS, theValue);

    private static void Out(TextWriter OS, int theValue) => Standard_Stream.Write(OS, theValue);

    private static void Print(in gp_Pnt P, TextWriter OS, bool compact)
    {
        Out(OS, P.X());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, P.Y());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, P.Z());
        OS.Write(" ");
    }

    private static void Print(in gp_Dir D, TextWriter OS, bool compact)
    {
        Out(OS, D.X());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, D.Y());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, D.Z());
        OS.Write(" ");
    }

    private static void Print(Geom_Line L, TextWriter OS, bool compact)
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

        gp_Lin C = L.Lin();
        if (!compact)
        {
            OS.Write("\n  Origin :");
        }

        Print(C.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(C.Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom_Circle CC, TextWriter OS, bool compact)
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

        gp_Circ C = CC.Circ();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(C.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(C.Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(C.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(C.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radius :");
        }

        Out(OS, C.Radius());
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom_Ellipse E, TextWriter OS, bool compact)
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

        gp_Elips C = E.Elips();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(C.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(C.Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(C.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(C.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radii  :");
        }

        Out(OS, C.MajorRadius());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, C.MinorRadius());
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom_Parabola P, TextWriter OS, bool compact)
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

        gp_Parab C = P.Parab();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(C.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(C.Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(C.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(C.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Focal  :");
        }

        Out(OS, C.Focal());
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom_Hyperbola H, TextWriter OS, bool compact)
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

        gp_Hypr C = H.Hypr();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(C.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(C.Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(C.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(C.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radii  :");
        }

        Out(OS, C.MajorRadius());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, C.MinorRadius());
        if (!compact)
        {
            OS.Write("\n");
        }

        OS.Write("\n");
    }

    private static void Print(Geom_BezierCurve B, TextWriter OS, bool compact)
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
                OS.Write(Standard_Stream.Setw(2, i.ToString(System.Globalization.CultureInfo.InvariantCulture)));
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

        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_BSplineCurve B, TextWriter OS, bool compact)
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
            OS.Write(" Knots\n");
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
                OS.Write(Standard_Stream.Setw(2, i.ToString(System.Globalization.CultureInfo.InvariantCulture)));
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
                OS.Write(Standard_Stream.Setw(2, i.ToString(System.Globalization.CultureInfo.InvariantCulture)));
                OS.Write(" : ");
            }

            OS.Write(" ");
            Out(OS, B.Knot(i));
            OS.Write(" ");
            Out(OS, B.Multiplicity(i));
        }

        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_TrimmedCurve C, TextWriter OS, bool compact)
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

        PrintCurve(C.BasisCurve(), OS, compact);
    }

    private static void Print(Geom_OffsetCurve C, TextWriter OS, bool compact)
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
            OS.Write("Direction : ");
        }

        Print(C.Direction(), OS, compact);
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("Basis curve :\n");
        }

        PrintCurve(C.BasisCurve(), OS, compact);
    }

    /// <summary>
    /// Writes the curve; if <paramref name="compact"/>, in the format <see cref="ReadCurve"/> reads, otherwise as a
    /// readable dump. Reals are written at the writer's precision (<see cref="Standard_Stream.Precision(TextWriter)"/>,
    /// 6 unless set). An unknown curve type goes to <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static void PrintCurve(Geom_Curve C, TextWriter OS, bool compact = false)
    {
        Type TheType = C.DynamicType();

        if (TheType == typeof(Geom_Line))
        {
            Print((Geom_Line)C, OS, compact);
        }
        else if (TheType == typeof(Geom_Circle))
        {
            Print((Geom_Circle)C, OS, compact);
        }
        else if (TheType == typeof(Geom_Ellipse))
        {
            Print((Geom_Ellipse)C, OS, compact);
        }
        else if (TheType == typeof(Geom_Parabola))
        {
            Print((Geom_Parabola)C, OS, compact);
        }
        else if (TheType == typeof(Geom_Hyperbola))
        {
            Print((Geom_Hyperbola)C, OS, compact);
        }
        else if (TheType == typeof(Geom_BezierCurve))
        {
            Print((Geom_BezierCurve)C, OS, compact);
        }
        else if (TheType == typeof(Geom_BSplineCurve))
        {
            Print((Geom_BSplineCurve)C, OS, compact);
        }
        else if (TheType == typeof(Geom_TrimmedCurve))
        {
            Print((Geom_TrimmedCurve)C, OS, compact);
        }
        else if (TheType == typeof(Geom_OffsetCurve))
        {
            Print((Geom_OffsetCurve)C, OS, compact);
        }
        else
        {
            GeomTools.GetUndefinedTypeHandler().PrintCurve(C, OS, compact);
        }
    }

    /// <summary>Writes a readable dump of the set.</summary>
    public void Dump(TextWriter OS)
    {
        int i, nbsurf = myMap.Extent();
        OS.Write("\n -------\n");
        OS.Write("Dump of ");
        Out(OS, nbsurf);
        OS.Write(" Curves ");
        OS.Write("\n -------\n\n");

        for (i = 1; i <= nbsurf; i++)
        {
            OS.Write(Standard_Stream.Setw(4, i.ToString(System.Globalization.CultureInfo.InvariantCulture)));
            OS.Write(" : ");
            // OCCT dereferences a null curve here (one that couldn't be read)
            PrintCurve(Curve(i)!, OS, false);
        }
    }

    /// <summary>
    /// Writes the set in the format <see cref="Read"/> reads: "Curves", the count, then each curve, at 17
    /// significant digits (the writer's precision is restored afterwards).
    /// </summary>
    public void Write(TextWriter OS, Message_ProgressRange theProgress = default)
    {
        int prec = Standard_Stream.Precision(OS, 17);

        int i, nbcurve = myMap.Extent();
        OS.Write("Curves ");
        Out(OS, nbcurve);
        OS.Write("\n");
        using var aPS = new Message_ProgressScope(theProgress, "3D Curves", nbcurve);
        for (i = 1; i <= nbcurve && aPS.More(); i++, aPS.Next())
        {
            // OCCT dereferences a null curve here (one that couldn't be read)
            PrintCurve(Curve(i)!, OS, true);
        }

        Standard_Stream.Precision(OS, prec);
    }

    // operator>> of the points, directions and curves

    private static void ReadPnt(TextReader IS, out gp_Pnt P)
    {
        double X, Y, Z;
        GeomTools.GetReal(IS, out X);
        GeomTools.GetReal(IS, out Y);
        GeomTools.GetReal(IS, out Z);
        P = new gp_Pnt(X, Y, Z);
    }

    private static void ReadDir(TextReader IS, ref gp_Dir D)
    {
        double X, Y, Z;
        GeomTools.GetReal(IS, out X);
        GeomTools.GetReal(IS, out Y);
        GeomTools.GetReal(IS, out Z);
        D.SetCoord(X, Y, Z);
    }

    private static Geom_Line ReadLine(TextReader IS)
    {
        gp_Pnt P;
        gp_Dir AX = new(gp_Dir.D.X);
        ReadPnt(IS, out P);
        ReadDir(IS, ref AX);
        return new Geom_Line(P, AX);
    }

    private static Geom_Circle ReadCircle(TextReader IS)
    {
        gp_Pnt P;
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        double R;
        ReadPnt(IS, out P);
        ReadDir(IS, ref A);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        GeomTools.GetReal(IS, out R);
        return new Geom_Circle(new gp_Ax2(P, A, AX), R);
    }

    private static Geom_Ellipse ReadEllipse(TextReader IS)
    {
        gp_Pnt P;
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        double R1, R2;
        ReadPnt(IS, out P);
        ReadDir(IS, ref A);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        GeomTools.GetReal(IS, out R1);
        GeomTools.GetReal(IS, out R2);
        return new Geom_Ellipse(new gp_Ax2(P, A, AX), R1, R2);
    }

    private static Geom_Parabola ReadParabola(TextReader IS)
    {
        gp_Pnt P;
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        double R1;
        ReadPnt(IS, out P);
        ReadDir(IS, ref A);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        GeomTools.GetReal(IS, out R1);
        return new Geom_Parabola(new gp_Ax2(P, A, AX), R1);
    }

    private static Geom_Hyperbola ReadHyperbola(TextReader IS)
    {
        gp_Pnt P;
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        double R1, R2;
        ReadPnt(IS, out P);
        ReadDir(IS, ref A);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        GeomTools.GetReal(IS, out R1);
        GeomTools.GetReal(IS, out R2);
        return new Geom_Hyperbola(new gp_Ax2(P, A, AX), R1, R2);
    }

    private static Geom_BezierCurve ReadBezier(TextReader IS)
    {
        bool rational = false;
        Standard_Stream.Read(IS, ref rational);

        // poles and weights
        int i = 0, degree = 0;
        Standard_Stream.Read(IS, ref degree);

        var poles = new NCollection_Array1<gp_Pnt>(1, degree + 1);
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
            return new Geom_BezierCurve(poles, weights);
        }

        return new Geom_BezierCurve(poles);
    }

    private static Geom_BSplineCurve ReadBSpline(TextReader IS)
    {
        bool rational = false, periodic = false;
        Standard_Stream.Read(IS, ref rational);
        Standard_Stream.Read(IS, ref periodic);

        // poles and weights
        int i = 0, degree = 0, nbpoles = 0, nbknots = 0;
        Standard_Stream.Read(IS, ref degree);
        Standard_Stream.Read(IS, ref nbpoles);
        Standard_Stream.Read(IS, ref nbknots);

        var poles = new NCollection_Array1<gp_Pnt>(1, nbpoles);
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
            return new Geom_BSplineCurve(poles, weights, knots, mults, degree, periodic);
        }

        return new Geom_BSplineCurve(poles, knots, mults, degree, periodic);
    }

    private static Geom_TrimmedCurve ReadTrimmed(TextReader IS)
    {
        double p1, p2;
        GeomTools.GetReal(IS, out p1);
        GeomTools.GetReal(IS, out p2);
        Geom_Curve? BC = ReadCurve(IS);
        // a null basis curve throws in the constructor, which ReadCurve catches, as OCCT's does
        return new Geom_TrimmedCurve(BC!, p1, p2);
    }

    private static Geom_OffsetCurve ReadOffset(TextReader IS)
    {
        double p;
        GeomTools.GetReal(IS, out p);
        gp_Dir D = new(gp_Dir.D.X);
        ReadDir(IS, ref D);
        Geom_Curve? BC = ReadCurve(IS);
        return new Geom_OffsetCurve(BC!, p, D);
    }

    /// <summary>
    /// Reads a curve written by <see cref="PrintCurve"/> (compact). Returns null if the curve can't be built from what
    /// was read (OCCT catches the constructor's exception); an unknown type goes to
    /// <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static Geom_Curve? ReadCurve(TextReader IS)
    {
        // OCCT's ctype is uninitialised, and left so when the stream has already failed
        int ctype = 0;

        Geom_Curve? C = null;
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
                    Geom_Curve? CC = null;
                    GeomTools.GetUndefinedTypeHandler().ReadCurve(ctype, IS, ref CC);
                    C = CC;
                }
                    break;
            }
        }
        catch (Standard_Failure)
        {
            // OCCT_DEBUG only: "EXCEPTION in GeomTools_CurveSet::ReadCurve(..)!!!"
        }
        catch (NullReferenceException)
        {
            // a trimmed or offset curve whose basis couldn't be read: OCCT dereferences the null handle, which
            // OCC_CATCH_SIGNALS turns into a caught Standard_AccessViolation where signals are converted
        }

        return C;
    }

    /// <summary>
    /// Clears nothing and reads a set written by <see cref="Write"/>, adding its curves (frozen) to this one. Writes
    /// "Not a Curve table" to the console, as OCCT does, if the text doesn't start with "Curves".
    /// </summary>
    public void Read(TextReader IS, Message_ProgressRange theProgress = default)
    {
        string? buffer = Standard_Stream.ReadWord(IS);
        if (buffer != "Curves")
        {
            Console.Out.Write("Not a Curve table" + "\n");
            return;
        }

        // OCCT's nbcurve is uninitialised if the count can't be read
        int i, nbcurve = 0;
        Standard_Stream.Read(IS, ref nbcurve);
        using var aPS = new Message_ProgressScope(theProgress, "3D Curves", nbcurve);
        for (i = 1; i <= nbcurve && aPS.More(); i++, aPS.Next())
        {
            Geom_Curve? C = ReadCurve(IS);
            C?.Freeze();
            myMap.Add((object?)C ?? theNullKey);
        }
    }
}
