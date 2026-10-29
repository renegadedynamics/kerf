// Port of: src/ModelingData/TKBRep/BinTools/BinTools_CurveSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_CurveSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A set of Geom curves, indexed from 1, written and read in OCCT's binary format (the "Curves" section of a binary
/// BRep).
/// </summary>
/// <remarks>
/// The curves that <see cref="Read(Stream, Message_ProgressRange)"/> creates are frozen: the set shares them with whoever asks. A curve that can't be
/// read makes <see cref="Read(Stream, Message_ProgressRange)"/> throw (OCCT's binary sets don't keep null entries, unlike GeomTools).
/// </remarks>
public class BinTools_CurveSet
{
    private const byte LINE = 1;
    private const byte CIRCLE = 2;
    private const byte ELLIPSE = 3;
    private const byte PARABOLA = 4;
    private const byte HYPERBOLA = 5;
    private const byte BEZIER = 6;
    private const byte BSPLINE = 7;
    private const byte TRIMMED = 8;
    private const byte OFFSET = 9;

    // OCCT's NCollection_IndexedMap of handles, which hashes by address
    private readonly NCollection_IndexedMap<object> myMap = new(ReferenceEqualityComparer.Instance);

    /// <summary>An empty set of curves.</summary>
    public BinTools_CurveSet()
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

    /// <summary>The curve of index I; null for 0.</summary>
    public Geom_Curve? Curve(int I)
    {
        if (I == 0)
        {
            return null;
        }
        else
        {
            return myMap[I] as Geom_Curve;
        }
    }

    /// <summary>The index of the curve, 0 if it is null or isn't in the set.</summary>
    public int Index(Geom_Curve? S)
    {
        return S is null ? 0 : myMap.FindIndex(S);
    }

    // the number of curves, for the readers' checks
    internal int NbCurves() => myMap.Extent();

    private static void Write(BinTools_OStream OS, Geom_Line L)
    {
        OS = OS << LINE;
        gp_Lin C = L.Lin();
        OS = OS << C.Location();  // Pnt
        OS = OS << C.Direction(); // Dir
    }

    private static void Write(BinTools_OStream OS, Geom_Circle CC)
    {
        OS = OS << CIRCLE;
        gp_Circ C = CC.Circ();
        OS = OS << C.Location();
        OS = OS << C.Axis().Direction();
        OS = OS << C.XAxis().Direction();
        OS = OS << C.YAxis().Direction();
        OS = OS << C.Radius();
    }

    private static void Write(BinTools_OStream OS, Geom_Ellipse E)
    {
        OS = OS << ELLIPSE;
        gp_Elips C = E.Elips();
        OS = OS << C.Location();
        OS = OS << C.Axis().Direction();
        OS = OS << C.XAxis().Direction();
        OS = OS << C.YAxis().Direction();
        OS = OS << C.MajorRadius();
        OS = OS << C.MinorRadius();
    }

    private static void Write(BinTools_OStream OS, Geom_Parabola P)
    {
        OS = OS << PARABOLA;
        gp_Parab C = P.Parab();
        OS = OS << C.Location();
        OS = OS << C.Axis().Direction();
        OS = OS << C.XAxis().Direction();
        OS = OS << C.YAxis().Direction();
        OS = OS << C.Focal();
    }

    private static void Write(BinTools_OStream OS, Geom_Hyperbola H)
    {
        OS = OS << HYPERBOLA;
        gp_Hypr C = H.Hypr();
        OS = OS << C.Location();
        OS = OS << C.Axis().Direction();
        OS = OS << C.XAxis().Direction();
        OS = OS << C.YAxis().Direction();
        OS = OS << C.MajorRadius();
        OS = OS << C.MinorRadius();
    }

    private static void Write(BinTools_OStream OS, Geom_BezierCurve B)
    {
        OS = OS << BEZIER;
        bool aRational = B.IsRational();
        OS = OS << aRational; // rational
        // poles and weights
        int i, aDegree = B.Degree();
        OS = OS << (char)aDegree; //<< Degree
        for (i = 1; i <= aDegree + 1; i++)
        {
            OS = OS << B.Pole(i); // Pnt
            if (aRational)
            {
                OS = OS << B.Weight(i); // Real
            }
        }
    }

    private static void Write(BinTools_OStream OS, Geom_BSplineCurve B)
    {
        OS = OS << BSPLINE;
        bool aRational = B.IsRational();
        OS = OS << aRational; // rational
        bool aPeriodic = B.IsPeriodic();
        OS = OS << aPeriodic; // periodic
        // poles and weights
        int i, aDegree, aNbPoles, aNbKnots;
        aDegree = B.Degree();
        aNbPoles = B.NbPoles();
        aNbKnots = B.NbKnots();
        OS = OS << (char)aDegree;
        OS = OS << aNbPoles;
        OS = OS << aNbKnots;
        for (i = 1; i <= aNbPoles; i++)
        {
            OS = OS << B.Pole(i); // Pnt
            if (aRational)
            {
                OS = OS << B.Weight(i);
            }
        }

        for (i = 1; i <= aNbKnots; i++)
        {
            OS = OS << B.Knot(i);
            OS = OS << B.Multiplicity(i);
        }
    }

    private static void Write(BinTools_OStream OS, Geom_TrimmedCurve C)
    {
        OS = OS << TRIMMED;
        OS = OS << C.FirstParameter();
        OS = OS << C.LastParameter();
        WriteCurve(C.BasisCurve(), OS);
    }

    private static void Write(BinTools_OStream OS, Geom_OffsetCurve C)
    {
        OS = OS << OFFSET;
        OS = OS << C.Offset(); // Offset
        OS = OS << C.Direction();
        WriteCurve(C.BasisCurve(), OS);
    }

    /// <summary>Writes a curve (its type byte, then its data).</summary>
    /// <exception cref="Standard_Failure">The curve is of a type the format doesn't know.</exception>
    public static void WriteCurve(Geom_Curve C, BinTools_OStream OS)
    {
        Type TheType = C.GetType();
        try
        {
            if (TheType == typeof(Geom_Line))
            {
                Write(OS, (Geom_Line)C);
            }
            else if (TheType == typeof(Geom_Circle))
            {
                Write(OS, (Geom_Circle)C);
            }
            else if (TheType == typeof(Geom_Ellipse))
            {
                Write(OS, (Geom_Ellipse)C);
            }
            else if (TheType == typeof(Geom_Parabola))
            {
                Write(OS, (Geom_Parabola)C);
            }
            else if (TheType == typeof(Geom_Hyperbola))
            {
                Write(OS, (Geom_Hyperbola)C);
            }
            else if (TheType == typeof(Geom_BezierCurve))
            {
                Write(OS, (Geom_BezierCurve)C);
            }
            else if (TheType == typeof(Geom_BSplineCurve))
            {
                Write(OS, (Geom_BSplineCurve)C);
            }
            else if (TheType == typeof(Geom_TrimmedCurve))
            {
                Write(OS, (Geom_TrimmedCurve)C);
            }
            else if (TheType == typeof(Geom_OffsetCurve))
            {
                Write(OS, (Geom_OffsetCurve)C);
            }
            else
            {
                throw new Standard_Failure("UNKNOWN CURVE TYPE");
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_CurveSet::WriteCurve(..)", anException);
        }
    }

    /// <summary>Writes the set ("Curves", the count, then each curve).</summary>
    public void Write(Stream OS, Message_ProgressRange theRange = default)
    {
        int i, nbcurv = myMap.Extent();
        using var aPS = new Message_ProgressScope(theRange, "Writing curves", nbcurv);
        BinTools.PutText(OS, "Curves ");
        BinTools.PutText(OS, nbcurv);
        BinTools.PutText(OS, "\n");
        var aStream = new BinTools_OStream(OS);
        for (i = 1; i <= nbcurv && aPS.More(); i++, aPS.Next())
        {
            WriteCurve((Geom_Curve)myMap[i], aStream);
        }
    }

    internal static void Read(Stream IS, out gp_Pnt P)
    {
        BinTools.GetReal(IS, out double X);
        BinTools.GetReal(IS, out double Y);
        BinTools.GetReal(IS, out double Z);
        P = new gp_Pnt(X, Y, Z);
    }

    internal static void Read(Stream IS, ref gp_Dir D)
    {
        BinTools.GetReal(IS, out double X);
        BinTools.GetReal(IS, out double Y);
        BinTools.GetReal(IS, out double Z);
        D.SetCoord(X, Y, Z);
    }

    private static Geom_Line ReadLine(Stream IS)
    {
        gp_Dir AX = new(gp_Dir.D.X);
        Read(IS, out gp_Pnt P);
        Read(IS, ref AX);
        return new Geom_Line(P, AX);
    }

    private static Geom_Circle ReadCircle(Stream IS)
    {
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        Read(IS, out gp_Pnt P);
        Read(IS, ref A);
        Read(IS, ref AX);
        Read(IS, ref AY);
        BinTools.GetReal(IS, out double R);
        return new Geom_Circle(new gp_Ax2(P, A, AX), R);
    }

    private static Geom_Ellipse ReadEllipse(Stream IS)
    {
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        Read(IS, out gp_Pnt P);
        Read(IS, ref A);
        Read(IS, ref AX);
        Read(IS, ref AY);
        BinTools.GetReal(IS, out double R1);
        BinTools.GetReal(IS, out double R2);
        return new Geom_Ellipse(new gp_Ax2(P, A, AX), R1, R2);
    }

    private static Geom_Parabola ReadParabola(Stream IS)
    {
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        Read(IS, out gp_Pnt P);
        Read(IS, ref A);
        Read(IS, ref AX);
        Read(IS, ref AY);
        BinTools.GetReal(IS, out double R1);
        return new Geom_Parabola(new gp_Ax2(P, A, AX), R1);
    }

    private static Geom_Hyperbola ReadHyperbola(Stream IS)
    {
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        Read(IS, out gp_Pnt P);
        Read(IS, ref A);
        Read(IS, ref AX);
        Read(IS, ref AY);
        BinTools.GetReal(IS, out double R1);
        BinTools.GetReal(IS, out double R2);
        return new Geom_Hyperbola(new gp_Ax2(P, A, AX), R1, R2);
    }

    private static Geom_BezierCurve ReadBezier(Stream IS)
    {
        BinTools.GetBool(IS, out bool rational);

        // poles and weights
        int i, degree;
        // degree;
        BinTools.GetExtChar(IS, out char aVal);
        degree = aVal;
        // OCCT.NET: untrusted input
        BinTools.CheckCount(IS, degree + 1L, rational ? 32 : 24, "poles");

        var poles = new NCollection_Array1<gp_Pnt>(1, degree + 1);
        var weights = new NCollection_Array1<double>(1, degree + 1);
        for (i = 1; i <= degree + 1; i++)
        {
            Read(IS, out poles[i]); // Pnt
            if (rational)
            {
                BinTools.GetReal(IS, out weights[i]);
            }
        }

        if (rational)
        {
            return new Geom_BezierCurve(poles, weights);
        }

        return new Geom_BezierCurve(poles);
    }

    private static Geom_BSplineCurve ReadBSpline(Stream IS)
    {
        BinTools.GetBool(IS, out bool rational);
        BinTools.GetBool(IS, out bool periodic);

        // poles and weights
        int i, degree, nbpoles, nbknots;
        BinTools.GetExtChar(IS, out char aVal);
        degree = aVal;
        BinTools.GetInteger(IS, out nbpoles);
        BinTools.GetInteger(IS, out nbknots);
        // OCCT.NET: untrusted input
        BinTools.CheckCount(IS, nbpoles, rational ? 32 : 24, "poles");
        BinTools.CheckCount(IS, nbknots, 12, "knots");

        var poles = new NCollection_Array1<gp_Pnt>(1, nbpoles);
        var weights = new NCollection_Array1<double>(1, nbpoles);
        for (i = 1; i <= nbpoles; i++)
        {
            Read(IS, out poles[i]); // Pnt
            if (rational)
            {
                BinTools.GetReal(IS, out weights[i]);
            }
        }

        var knots = new NCollection_Array1<double>(1, nbknots);
        var mults = new NCollection_Array1<int>(1, nbknots);
        for (i = 1; i <= nbknots; i++)
        {
            BinTools.GetReal(IS, out knots[i]);
            BinTools.GetInteger(IS, out mults[i]);
        }

        if (rational)
        {
            return new Geom_BSplineCurve(poles, weights, knots, mults, degree, periodic);
        }

        return new Geom_BSplineCurve(poles, knots, mults, degree, periodic);
    }

    private static Geom_TrimmedCurve ReadTrimmed(Stream IS, int theDepth)
    {
        BinTools.GetReal(IS, out double p1); // FirstParameter
        BinTools.GetReal(IS, out double p2); // LastParameter
        ReadCurve(IS, out Geom_Curve? BC, theDepth + 1);
        return new Geom_TrimmedCurve(BC!, p1, p2);
    }

    private static Geom_OffsetCurve ReadOffset(Stream IS, int theDepth)
    {
        BinTools.GetReal(IS, out double p); // Offset
        gp_Dir D = new(gp_Dir.D.X);
        Read(IS, ref D);
        ReadCurve(IS, out Geom_Curve? BC, theDepth + 1);
        return new Geom_OffsetCurve(BC!, p, D);
    }

    /// <summary>Reads a curve written by <see cref="WriteCurve"/>. The curve is new and not frozen.</summary>
    /// <exception cref="Standard_Failure">The curve can't be read.</exception>
    public static Stream ReadCurve(Stream IS, out Geom_Curve? C) => ReadCurve(IS, out C, 0);

    internal static Stream ReadCurve(Stream IS, out Geom_Curve? C, int theDepth)
    {
        C = null;
        try
        {
            if (theDepth > BinTools.MaxGeometryNesting)
            {
                // OCCT.NET: untrusted input: OCCT recurses without limit
                throw new Storage_StreamFormatError("BinTools_CurveSet: curves nested too deeply");
            }

            byte ctype = (byte)BinTools.Get(IS);
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
                    C = ReadTrimmed(IS, theDepth);
                    break;

                case OFFSET:
                    C = ReadOffset(IS, theDepth);
                    break;

                default:
                {
                    C = null;
                    throw new Standard_Failure("UNKNOWN CURVE TYPE");
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            C = null;
            throw BinTools.Rethrow("BinTools_CurveSet::ReadCurve(..)", anException);
        }

        return IS;
    }

    /// <summary>
    /// Reads a set written by <see cref="Write(Stream, Message_ProgressRange)"/>, adding its curves (frozen) to this one.
    /// </summary>
    /// <exception cref="Standard_Failure">The stream doesn't hold a curve table, or a curve can't be read.</exception>
    public void Read(Stream IS, Message_ProgressRange theRange = default)
    {
        string? buffer = BinTools.ReadWord(IS);
        if (buffer is null || buffer != "Curves")
        {
            throw new Standard_Failure("BinTools_CurveSet::Read:  Not a Curve table" + "\n");
        }

        int i, nbcurve;
        nbcurve = BinTools.ReadFormattedInteger(IS);
        using var aPS = new Message_ProgressScope(theRange, "Reading curves", nbcurve);

        BinTools.Get(IS); // remove <lf>
        // OCCT.NET: untrusted input: a curve takes at least 28 bytes (a Bezier curve of degree 0)
        BinTools.CheckCount(IS, nbcurve, 28, "curves");
        for (i = 1; i <= nbcurve && aPS.More(); i++, aPS.Next())
        {
            ReadCurve(IS, out Geom_Curve? C);
            C!.Freeze();
            myMap.Add(C);
        }
    }
}
