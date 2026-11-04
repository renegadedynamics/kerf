// Port of: src/ModelingData/TKBRep/BinTools/BinTools_Curve2dSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_Curve2dSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// A set of Geom2d curves, indexed from 1, written and read in OCCT's binary format (the "Curve2ds" section of a binary
/// BRep).
/// </summary>
/// <remarks>
/// As in OCCT, the set can hold the null curve once (<see cref="Add"/> adds it). The curves that <see cref="Read(Stream, ProgressRange)"/>
/// creates are frozen. OCCT declares a <c>Dump</c> it doesn't define; it isn't ported.
/// </remarks>
internal class BinaryCurve2dSet
{
    private const byte Line = 1;
    private const byte Circle = 2;
    private const byte Ellipse = 3;
    private const byte Parabola = 4;
    private const byte Hyperbola = 5;
    private const byte Bezier = 6;
    private const byte Bspline = 7;
    private const byte Trimmed = 8;
    private const byte Offset = 9;

    // OCCT's NCollection_IndexedMap of handles, which hashes by address and can hold one null handle
    private readonly IndexedSet<object> _map = new(ReferenceEqualityComparer.Instance);

    // the null handle as a key
    private static readonly object NullKey = new();

    /// <summary>An empty set of 2D curves.</summary>
    public BinaryCurve2dSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>Adds a curve to the set (once; null too) and returns its index.</summary>
    public int Add(Curve2D? s)
    {
        return _map.Add((object?)s ?? NullKey) + 1;
    }

    /// <summary>The curve of index I (null for the null entry).</summary>
    /// <exception cref="OutOfRangeException">There is no curve of index I.</exception>
    public Curve2D? Curve2d(int i)
    {
        return _map[i - 1] as Curve2D;
    }

    /// <summary>The index of the curve, 0 if it isn't in the set.</summary>
    public int Index(Curve2D? s)
    {
        return _map.IndexOf((object?)s ?? NullKey) + 1;
    }

    // the number of curves, for the readers' checks
    internal int CurveCount => _map.Count;

    private static void Write(OStream os, LineCurve2D l)
    {
        os = os << Line;
        Line2D c2d = l.Line;
        os = os << c2d.Location;  // Pnt2d
        os = os << c2d.Direction; // Dir2d
    }

    private static void Write(OStream os, CircleCurve2D c)
    {
        os = os << Circle;
        Circle2D c2d = c.Circle;
        os = os << c2d.Location;
        os = os << c2d.XAxis.Direction;
        os = os << c2d.YAxis.Direction;
        os = os << c2d.Radius;
    }

    private static void Write(OStream os, EllipseCurve2D e)
    {
        os = os << Ellipse;
        Ellipse2D c2d = e.Ellipse;
        os = os << c2d.Location;
        os = os << c2d.XAxis.Direction;
        os = os << c2d.YAxis.Direction;
        os = os << c2d.MajorRadius;
        os = os << c2d.MinorRadius;
    }

    private static void Write(OStream os, ParabolaCurve2D p)
    {
        os = os << Parabola;
        Parabola2D c2d = p.Parabola;
        os = os << c2d.Location;                 // Loc
        os = os << c2d.Axis.XAxis.Direction; // XDir
        os = os << c2d.Axis.YAxis.Direction; // YDir
        os = os << c2d.Focal;                    // Focal
    }

    private static void Write(OStream os, HyperbolaCurve2D h)
    {
        os = os << Hyperbola;
        Hyperbola2D c2d = h.Hyperbola;
        os = os << c2d.Location;          // Loc
        os = os << c2d.XAxis.Direction; // XDir
        os = os << c2d.YAxis.Direction; // YDir
        os = os << c2d.MajorRadius;       // MajR
        os = os << c2d.MinorRadius;
    }

    private static void Write(OStream os, BezierCurve2D b)
    {
        os = os << Bezier;
        bool rational = b.IsRational;
        os = os << rational; // rational
        // poles and weights
        int i, degree = b.Degree;
        os = os << (char)degree; // Degree
        for (i = 1; i <= degree + 1; i++)
        {
            os = os << b.Pole(i); // Pnt2d
            if (rational)
            {
                os = os << b.Weight(i); // Real
            }
        }
    }

    private static void Write(OStream os, BSplineCurve2D b)
    {
        os = os << Bspline;
        bool rational = b.IsRational;
        os = os << rational; // rational
        bool periodic = b.IsPeriodic;
        os = os << periodic; // periodic
        // poles and weights
        int i, degree, nbPoles, nbKnots;
        degree = b.Degree;
        nbPoles = b.PoleCount;
        nbKnots = b.KnotCount;
        os = os << (char)degree;
        os = os << nbPoles;
        os = os << nbKnots;
        for (i = 1; i <= nbPoles; i++)
        {
            os = os << b.Pole(i); // Pnt2d
            if (rational)
            {
                os = os << b.Weight(i);
            }
        }

        for (i = 1; i <= nbKnots; i++)
        {
            os = os << b.Knot(i);
            os = os << b.Multiplicity(i);
        }
    }

    private static void Write(OStream os, TrimmedCurve2D c)
    {
        os = os << Trimmed;
        os = os << c.FirstParameter;
        os = os << c.LastParameter;
        WriteCurve2d(c.BasisCurve, os);
    }

    private static void Write(OStream os, OffsetCurve2D c)
    {
        os = os << Offset;
        os = os << c.Offset; // Offset
        WriteCurve2d(c.BasisCurve, os);
    }

    /// <summary>Writes a 2D curve (its type byte, then its data).</summary>
    /// <exception cref="FailureException">The curve is of a type the format doesn't know.</exception>
    public static void WriteCurve2d(Curve2D c, OStream os)
    {
        Type theType = c.GetType();
        try
        {
            if (theType == typeof(CircleCurve2D))
            {
                Write(os, (CircleCurve2D)c);
            }
            else if (theType == typeof(LineCurve2D))
            {
                Write(os, (LineCurve2D)c);
            }
            else if (theType == typeof(EllipseCurve2D))
            {
                Write(os, (EllipseCurve2D)c);
            }
            else if (theType == typeof(ParabolaCurve2D))
            {
                Write(os, (ParabolaCurve2D)c);
            }
            else if (theType == typeof(HyperbolaCurve2D))
            {
                Write(os, (HyperbolaCurve2D)c);
            }
            else if (theType == typeof(BezierCurve2D))
            {
                Write(os, (BezierCurve2D)c);
            }
            else if (theType == typeof(BSplineCurve2D))
            {
                Write(os, (BSplineCurve2D)c);
            }
            else if (theType == typeof(TrimmedCurve2D))
            {
                Write(os, (TrimmedCurve2D)c);
            }
            else if (theType == typeof(OffsetCurve2D))
            {
                Write(os, (OffsetCurve2D)c);
            }
            else
            {
                throw new FailureException("UNKNOWN CURVE2d TYPE");
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_Curve2dSet::WriteCurve2d(..)", exception);
        }
    }

    /// <summary>Writes the set ("Curve2ds", the count, then each curve).</summary>
    public void Write(Stream os, ProgressRange range = default)
    {
        int i, nbCurves = _map.Count;
        using var ps = new ProgressScope(range, "Writing 2D curves", nbCurves);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "Curve2ds ");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, nbCurves);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");
        var stream = new OStream(os);
        for (i = 1; i <= nbCurves && ps.More(); i++, ps.Next())
        {
            // OCCT dereferences the null entry outside its try block, and crashes; the port throws
            Curve2D curve = _map[i - 1] as Curve2D
                                  ?? throw new FailureException("BinTools_Curve2dSet::Write: a null curve");
            WriteCurve2d(curve, stream);
        }
    }

    private static void Read(Stream IS, out Point2D p)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double x);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double y);
        p = new Point2D(x, y);
    }

    private static void Read(Stream IS, ref Direction2D d)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double x);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double y);
        d = d.WithCoord(x, y);
    }

    private static LineCurve2D ReadLine(Stream IS)
    {
        Direction2D ax = new(Direction2D.D.X);
        Read(IS, out Point2D p);
        Read(IS, ref ax);
        return new LineCurve2D(p, ax);
    }

    private static CircleCurve2D ReadCircle(Stream IS)
    {
        Direction2D ax = new(Direction2D.D.X), ay = new(Direction2D.D.X);
        Read(IS, out Point2D p);
        Read(IS, ref ax);
        Read(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r);
        return new CircleCurve2D(new Frame2D(p, ax, ay), r);
    }

    private static EllipseCurve2D ReadEllipse(Stream IS)
    {
        Direction2D ax = new(Direction2D.D.X), ay = new(Direction2D.D.X);
        Read(IS, out Point2D p);
        Read(IS, ref ax);
        Read(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r1);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r2);
        return new EllipseCurve2D(new Frame2D(p, ax, ay), r1, r2);
    }

    private static ParabolaCurve2D ReadParabola(Stream IS)
    {
        Direction2D ax = new(Direction2D.D.X), ay = new(Direction2D.D.X);
        Read(IS, out Point2D p);
        Read(IS, ref ax);
        Read(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r1);
        return new ParabolaCurve2D(new Frame2D(p, ax, ay), r1);
    }

    private static HyperbolaCurve2D ReadHyperbola(Stream IS)
    {
        Direction2D ax = new(Direction2D.D.X), ay = new(Direction2D.D.X);
        Read(IS, out Point2D p);
        Read(IS, ref ax);
        Read(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r1);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r2);
        return new HyperbolaCurve2D(new Frame2D(p, ax, ay), r1, r2);
    }

    private static BezierCurve2D ReadBezier(Stream IS)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool rational);

        // poles and weights
        int i, degree;
        // degree;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetExtChar(IS, out char val);
        degree = val;
        // OCCT.NET: untrusted input
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, degree + 1L, rational ? 24 : 16, "poles");

        var poles = new NCollection_Array1<Point2D>(1, degree + 1);
        var weights = new NCollection_Array1<double>(1, degree + 1);
        for (i = 1; i <= degree + 1; i++)
        {
            Read(IS, out poles[i]); // Pnt2d
            if (rational)
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out weights[i]);
            }
        }

        if (rational)
        {
            return new BezierCurve2D(poles, weights);
        }

        return new BezierCurve2D(poles);
    }

    private static BSplineCurve2D ReadBSpline(Stream IS)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool rational);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool periodic);

        // poles and weights
        int i, degree, nbpoles, nbknots;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetExtChar(IS, out char val);
        degree = val;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out nbpoles);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out nbknots);
        // OCCT.NET: untrusted input
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbpoles, rational ? 24 : 16, "poles");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbknots, 12, "knots");

        var poles = new NCollection_Array1<Point2D>(1, nbpoles);
        var weights = new NCollection_Array1<double>(1, nbpoles);
        for (i = 1; i <= nbpoles; i++)
        {
            Read(IS, out poles[i]); // Pnt2d
            if (rational)
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out weights[i]);
            }
        }

        var knots = new NCollection_Array1<double>(1, nbknots);
        var mults = new NCollection_Array1<int>(1, nbknots);
        for (i = 1; i <= nbknots; i++)
        {
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out knots[i]);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out mults[i]);
        }

        if (rational)
        {
            return new BSplineCurve2D(poles, weights, knots, mults, degree, periodic);
        }

        return new BSplineCurve2D(poles, knots, mults, degree, periodic);
    }

    private static TrimmedCurve2D ReadTrimmed(Stream IS, int depth)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double p1); // FirstParameter
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double p2); // LastParameter
        ReadCurve2d(IS, out Curve2D? bc, depth + 1);
        return new TrimmedCurve2D(bc!, p1, p2);
    }

    private static OffsetCurve2D ReadOffset(Stream IS, int depth)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double p); // Offset
        ReadCurve2d(IS, out Curve2D? bc, depth + 1);
        return new OffsetCurve2D(bc!, p);
    }

    /// <summary>Reads a 2D curve written by <see cref="WriteCurve2d"/>. The curve is new and not frozen.</summary>
    /// <exception cref="FailureException">The curve can't be read.</exception>
    public static Stream ReadCurve2d(Stream IS, out Curve2D? c) => ReadCurve2d(IS, out c, 0);

    internal static Stream ReadCurve2d(Stream IS, out Curve2D? c, int depth)
    {
        c = null;
        try
        {
            if (depth > global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxGeometryNesting)
            {
                // OCCT.NET: untrusted input: OCCT recurses without limit
                throw new StreamFormatErrorException("BinTools_Curve2dSet: curves nested too deeply");
            }

            byte ctype = (byte)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS);
            switch (ctype)
            {
                case Line:
                    c = ReadLine(IS);
                    break;

                case Circle:
                    c = ReadCircle(IS);
                    break;

                case Ellipse:
                    c = ReadEllipse(IS);
                    break;

                case Parabola:
                    c = ReadParabola(IS);
                    break;

                case Hyperbola:
                    c = ReadHyperbola(IS);
                    break;

                case Bezier:
                    c = ReadBezier(IS);
                    break;

                case Bspline:
                    c = ReadBSpline(IS);
                    break;

                case Trimmed:
                    c = ReadTrimmed(IS, depth);
                    break;

                case Offset:
                    c = ReadOffset(IS, depth);
                    break;

                default:
                {
                    c = null;
                    throw new FailureException("UNKNOWN CURVE2d TYPE");
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            c = null;
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_Curve2dSet::ReadCurve2d(...)", exception);
        }

        return IS;
    }

    /// <summary>Reads a set written by <see cref="Write(Stream, ProgressRange)"/>, adding its curves (frozen) to this one.</summary>
    /// <exception cref="FailureException">The stream doesn't hold a 2D curve table, or a curve can't be read.</exception>
    public void Read(Stream IS, ProgressRange range = default)
    {
        string? buffer = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadWord(IS);
        if (buffer is null || buffer != "Curve2ds")
        {
            throw new FailureException("BinTools_Curve2dSet::Read:  Not a Curve2d table" + "\n");
        }

        int i, nbCurves;
        nbCurves = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedInteger(IS);
        using var ps = new ProgressScope(range, "Reading curves 2d", nbCurves);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // remove <lf>
        // OCCT.NET: untrusted input: a 2D curve takes at least 20 bytes (a Bezier curve of degree 0)
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbCurves, 20, "2D curves");
        for (i = 1; i <= nbCurves && ps.More(); i++, ps.Next())
        {
            ReadCurve2d(IS, out Curve2D? c);
            c!.Freeze();
            _map.Add(c);
        }
    }
}
