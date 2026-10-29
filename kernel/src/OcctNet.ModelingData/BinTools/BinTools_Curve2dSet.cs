// Port of: src/ModelingData/TKBRep/BinTools/BinTools_Curve2dSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_Curve2dSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A set of Geom2d curves, indexed from 1, written and read in OCCT's binary format (the "Curve2ds" section of a binary
/// BRep).
/// </summary>
/// <remarks>
/// As in OCCT, the set can hold the null curve once (<see cref="Add"/> adds it). The curves that <see cref="Read(Stream, Message_ProgressRange)"/>
/// creates are frozen. OCCT declares a <c>Dump</c> it doesn't define; it isn't ported.
/// </remarks>
public class BinTools_Curve2dSet
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

    // OCCT's NCollection_IndexedMap of handles, which hashes by address and can hold one null handle
    private readonly NCollection_IndexedMap<object> myMap = new(ReferenceEqualityComparer.Instance);

    // the null handle as a key
    private static readonly object theNullKey = new();

    /// <summary>An empty set of 2D curves.</summary>
    public BinTools_Curve2dSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>Adds a curve to the set (once; null too) and returns its index.</summary>
    public int Add(Geom2d_Curve? S)
    {
        return myMap.Add((object?)S ?? theNullKey);
    }

    /// <summary>The curve of index I (null for the null entry).</summary>
    /// <exception cref="Standard_OutOfRange">There is no curve of index I.</exception>
    public Geom2d_Curve? Curve2d(int I)
    {
        return myMap[I] as Geom2d_Curve;
    }

    /// <summary>The index of the curve, 0 if it isn't in the set.</summary>
    public int Index(Geom2d_Curve? S)
    {
        return myMap.FindIndex((object?)S ?? theNullKey);
    }

    // the number of curves, for the readers' checks
    internal int NbCurves() => myMap.Extent();

    private static void Write(BinTools_OStream OS, Geom2d_Line L)
    {
        OS = OS << LINE;
        gp_Lin2d C2d = L.Lin2d();
        OS = OS << C2d.Location();  // Pnt2d
        OS = OS << C2d.Direction(); // Dir2d
    }

    private static void Write(BinTools_OStream OS, Geom2d_Circle C)
    {
        OS = OS << CIRCLE;
        gp_Circ2d C2d = C.Circ2d();
        OS = OS << C2d.Location();
        OS = OS << C2d.XAxis().Direction();
        OS = OS << C2d.YAxis().Direction();
        OS = OS << C2d.Radius();
    }

    private static void Write(BinTools_OStream OS, Geom2d_Ellipse E)
    {
        OS = OS << ELLIPSE;
        gp_Elips2d C2d = E.Elips2d();
        OS = OS << C2d.Location();
        OS = OS << C2d.XAxis().Direction();
        OS = OS << C2d.YAxis().Direction();
        OS = OS << C2d.MajorRadius();
        OS = OS << C2d.MinorRadius();
    }

    private static void Write(BinTools_OStream OS, Geom2d_Parabola P)
    {
        OS = OS << PARABOLA;
        gp_Parab2d C2d = P.Parab2d();
        OS = OS << C2d.Location();                 // Loc
        OS = OS << C2d.Axis().XAxis().Direction(); // XDir
        OS = OS << C2d.Axis().YAxis().Direction(); // YDir
        OS = OS << C2d.Focal();                    // Focal
    }

    private static void Write(BinTools_OStream OS, Geom2d_Hyperbola H)
    {
        OS = OS << HYPERBOLA;
        gp_Hypr2d C2d = H.Hypr2d();
        OS = OS << C2d.Location();          // Loc
        OS = OS << C2d.XAxis().Direction(); // XDir
        OS = OS << C2d.YAxis().Direction(); // YDir
        OS = OS << C2d.MajorRadius();       // MajR
        OS = OS << C2d.MinorRadius();
    }

    private static void Write(BinTools_OStream OS, Geom2d_BezierCurve B)
    {
        OS = OS << BEZIER;
        bool aRational = B.IsRational();
        OS = OS << aRational; // rational
        // poles and weights
        int i, aDegree = B.Degree();
        OS = OS << (char)aDegree; // Degree
        for (i = 1; i <= aDegree + 1; i++)
        {
            OS = OS << B.Pole(i); // Pnt2d
            if (aRational)
            {
                OS = OS << B.Weight(i); // Real
            }
        }
    }

    private static void Write(BinTools_OStream OS, Geom2d_BSplineCurve B)
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
            OS = OS << B.Pole(i); // Pnt2d
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

    private static void Write(BinTools_OStream OS, Geom2d_TrimmedCurve C)
    {
        OS = OS << TRIMMED;
        OS = OS << C.FirstParameter();
        OS = OS << C.LastParameter();
        WriteCurve2d(C.BasisCurve(), OS);
    }

    private static void Write(BinTools_OStream OS, Geom2d_OffsetCurve C)
    {
        OS = OS << OFFSET;
        OS = OS << C.Offset(); // Offset
        WriteCurve2d(C.BasisCurve(), OS);
    }

    /// <summary>Writes a 2D curve (its type byte, then its data).</summary>
    /// <exception cref="Standard_Failure">The curve is of a type the format doesn't know.</exception>
    public static void WriteCurve2d(Geom2d_Curve C, BinTools_OStream OS)
    {
        Type TheType = C.GetType();
        try
        {
            if (TheType == typeof(Geom2d_Circle))
            {
                Write(OS, (Geom2d_Circle)C);
            }
            else if (TheType == typeof(Geom2d_Line))
            {
                Write(OS, (Geom2d_Line)C);
            }
            else if (TheType == typeof(Geom2d_Ellipse))
            {
                Write(OS, (Geom2d_Ellipse)C);
            }
            else if (TheType == typeof(Geom2d_Parabola))
            {
                Write(OS, (Geom2d_Parabola)C);
            }
            else if (TheType == typeof(Geom2d_Hyperbola))
            {
                Write(OS, (Geom2d_Hyperbola)C);
            }
            else if (TheType == typeof(Geom2d_BezierCurve))
            {
                Write(OS, (Geom2d_BezierCurve)C);
            }
            else if (TheType == typeof(Geom2d_BSplineCurve))
            {
                Write(OS, (Geom2d_BSplineCurve)C);
            }
            else if (TheType == typeof(Geom2d_TrimmedCurve))
            {
                Write(OS, (Geom2d_TrimmedCurve)C);
            }
            else if (TheType == typeof(Geom2d_OffsetCurve))
            {
                Write(OS, (Geom2d_OffsetCurve)C);
            }
            else
            {
                throw new Standard_Failure("UNKNOWN CURVE2d TYPE");
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_Curve2dSet::WriteCurve2d(..)", anException);
        }
    }

    /// <summary>Writes the set ("Curve2ds", the count, then each curve).</summary>
    public void Write(Stream OS, Message_ProgressRange theRange = default)
    {
        int i, aNbCurves = myMap.Extent();
        using var aPS = new Message_ProgressScope(theRange, "Writing 2D curves", aNbCurves);
        BinTools.PutText(OS, "Curve2ds ");
        BinTools.PutText(OS, aNbCurves);
        BinTools.PutText(OS, "\n");
        var aStream = new BinTools_OStream(OS);
        for (i = 1; i <= aNbCurves && aPS.More(); i++, aPS.Next())
        {
            // OCCT dereferences the null entry outside its try block, and crashes; the port throws
            Geom2d_Curve aCurve = myMap[i] as Geom2d_Curve
                                  ?? throw new Standard_Failure("BinTools_Curve2dSet::Write: a null curve");
            WriteCurve2d(aCurve, aStream);
        }
    }

    private static void Read(Stream IS, out gp_Pnt2d P)
    {
        BinTools.GetReal(IS, out double X);
        BinTools.GetReal(IS, out double Y);
        P = new gp_Pnt2d(X, Y);
    }

    private static void Read(Stream IS, ref gp_Dir2d D)
    {
        BinTools.GetReal(IS, out double X);
        BinTools.GetReal(IS, out double Y);
        D.SetCoord(X, Y);
    }

    private static Geom2d_Line ReadLine(Stream IS)
    {
        gp_Dir2d AX = new(gp_Dir2d.D.X);
        Read(IS, out gp_Pnt2d P);
        Read(IS, ref AX);
        return new Geom2d_Line(P, AX);
    }

    private static Geom2d_Circle ReadCircle(Stream IS)
    {
        gp_Dir2d AX = new(gp_Dir2d.D.X), AY = new(gp_Dir2d.D.X);
        Read(IS, out gp_Pnt2d P);
        Read(IS, ref AX);
        Read(IS, ref AY);
        BinTools.GetReal(IS, out double R);
        return new Geom2d_Circle(new gp_Ax22d(P, AX, AY), R);
    }

    private static Geom2d_Ellipse ReadEllipse(Stream IS)
    {
        gp_Dir2d AX = new(gp_Dir2d.D.X), AY = new(gp_Dir2d.D.X);
        Read(IS, out gp_Pnt2d P);
        Read(IS, ref AX);
        Read(IS, ref AY);
        BinTools.GetReal(IS, out double R1);
        BinTools.GetReal(IS, out double R2);
        return new Geom2d_Ellipse(new gp_Ax22d(P, AX, AY), R1, R2);
    }

    private static Geom2d_Parabola ReadParabola(Stream IS)
    {
        gp_Dir2d AX = new(gp_Dir2d.D.X), AY = new(gp_Dir2d.D.X);
        Read(IS, out gp_Pnt2d P);
        Read(IS, ref AX);
        Read(IS, ref AY);
        BinTools.GetReal(IS, out double R1);
        return new Geom2d_Parabola(new gp_Ax22d(P, AX, AY), R1);
    }

    private static Geom2d_Hyperbola ReadHyperbola(Stream IS)
    {
        gp_Dir2d AX = new(gp_Dir2d.D.X), AY = new(gp_Dir2d.D.X);
        Read(IS, out gp_Pnt2d P);
        Read(IS, ref AX);
        Read(IS, ref AY);
        BinTools.GetReal(IS, out double R1);
        BinTools.GetReal(IS, out double R2);
        return new Geom2d_Hyperbola(new gp_Ax22d(P, AX, AY), R1, R2);
    }

    private static Geom2d_BezierCurve ReadBezier(Stream IS)
    {
        BinTools.GetBool(IS, out bool rational);

        // poles and weights
        int i, degree;
        // degree;
        BinTools.GetExtChar(IS, out char aVal);
        degree = aVal;
        // OCCT.NET: untrusted input
        BinTools.CheckCount(IS, degree + 1L, rational ? 24 : 16, "poles");

        var poles = new NCollection_Array1<gp_Pnt2d>(1, degree + 1);
        var weights = new NCollection_Array1<double>(1, degree + 1);
        for (i = 1; i <= degree + 1; i++)
        {
            Read(IS, out poles[i]); // Pnt2d
            if (rational)
            {
                BinTools.GetReal(IS, out weights[i]);
            }
        }

        if (rational)
        {
            return new Geom2d_BezierCurve(poles, weights);
        }

        return new Geom2d_BezierCurve(poles);
    }

    private static Geom2d_BSplineCurve ReadBSpline(Stream IS)
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
        BinTools.CheckCount(IS, nbpoles, rational ? 24 : 16, "poles");
        BinTools.CheckCount(IS, nbknots, 12, "knots");

        var poles = new NCollection_Array1<gp_Pnt2d>(1, nbpoles);
        var weights = new NCollection_Array1<double>(1, nbpoles);
        for (i = 1; i <= nbpoles; i++)
        {
            Read(IS, out poles[i]); // Pnt2d
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
            return new Geom2d_BSplineCurve(poles, weights, knots, mults, degree, periodic);
        }

        return new Geom2d_BSplineCurve(poles, knots, mults, degree, periodic);
    }

    private static Geom2d_TrimmedCurve ReadTrimmed(Stream IS, int theDepth)
    {
        BinTools.GetReal(IS, out double p1); // FirstParameter
        BinTools.GetReal(IS, out double p2); // LastParameter
        ReadCurve2d(IS, out Geom2d_Curve? BC, theDepth + 1);
        return new Geom2d_TrimmedCurve(BC!, p1, p2);
    }

    private static Geom2d_OffsetCurve ReadOffset(Stream IS, int theDepth)
    {
        BinTools.GetReal(IS, out double p); // Offset
        ReadCurve2d(IS, out Geom2d_Curve? BC, theDepth + 1);
        return new Geom2d_OffsetCurve(BC!, p);
    }

    /// <summary>Reads a 2D curve written by <see cref="WriteCurve2d"/>. The curve is new and not frozen.</summary>
    /// <exception cref="Standard_Failure">The curve can't be read.</exception>
    public static Stream ReadCurve2d(Stream IS, out Geom2d_Curve? C) => ReadCurve2d(IS, out C, 0);

    internal static Stream ReadCurve2d(Stream IS, out Geom2d_Curve? C, int theDepth)
    {
        C = null;
        try
        {
            if (theDepth > BinTools.MaxGeometryNesting)
            {
                // OCCT.NET: untrusted input: OCCT recurses without limit
                throw new Storage_StreamFormatError("BinTools_Curve2dSet: curves nested too deeply");
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
                    throw new Standard_Failure("UNKNOWN CURVE2d TYPE");
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            C = null;
            throw BinTools.Rethrow("BinTools_Curve2dSet::ReadCurve2d(...)", anException);
        }

        return IS;
    }

    /// <summary>Reads a set written by <see cref="Write(Stream, Message_ProgressRange)"/>, adding its curves (frozen) to this one.</summary>
    /// <exception cref="Standard_Failure">The stream doesn't hold a 2D curve table, or a curve can't be read.</exception>
    public void Read(Stream IS, Message_ProgressRange theRange = default)
    {
        string? buffer = BinTools.ReadWord(IS);
        if (buffer is null || buffer != "Curve2ds")
        {
            throw new Standard_Failure("BinTools_Curve2dSet::Read:  Not a Curve2d table" + "\n");
        }

        int i, aNbCurves;
        aNbCurves = BinTools.ReadFormattedInteger(IS);
        using var aPS = new Message_ProgressScope(theRange, "Reading curves 2d", aNbCurves);
        BinTools.Get(IS); // remove <lf>
        // OCCT.NET: untrusted input: a 2D curve takes at least 20 bytes (a Bezier curve of degree 0)
        BinTools.CheckCount(IS, aNbCurves, 20, "2D curves");
        for (i = 1; i <= aNbCurves && aPS.More(); i++, aPS.Next())
        {
            ReadCurve2d(IS, out Geom2d_Curve? C);
            C!.Freeze();
            myMap.Add(C);
        }
    }
}
