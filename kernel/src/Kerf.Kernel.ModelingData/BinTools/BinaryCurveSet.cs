// Port of: src/ModelingData/TKBRep/BinTools/BinTools_CurveSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_CurveSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// A set of Geom curves, indexed from 1, written and read in OCCT's binary format (the "Curves" section of a binary
/// BRep).
/// </summary>
/// <remarks>
/// The curves that <see cref="Read(Stream, ProgressRange)"/> creates are frozen: the set shares them with whoever asks. A curve that can't be
/// read makes <see cref="Read(Stream, ProgressRange)"/> throw (OCCT's binary sets don't keep null entries, unlike GeomTools).
/// </remarks>
internal class BinaryCurveSet
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

    // OCCT's NCollection_IndexedMap of handles, which hashes by address
    private readonly IndexedSet<object> _map = new(ReferenceEqualityComparer.Instance);

    /// <summary>An empty set of curves.</summary>
    public BinaryCurveSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>Adds a curve to the set (once) and returns its index; 0 for null.</summary>
    public int Add(Curve3D? c)
    {
        return (c is null) ? 0 : (_map.Add(c) + 1);
    }

    /// <summary>The curve of index I; null for 0.</summary>
    public Curve3D? Curve(int i)
    {
        if (i == 0)
        {
            return null;
        }
        else
        {
            return _map[i - 1] as Curve3D;
        }
    }

    /// <summary>The index of the curve, 0 if it is null or isn't in the set.</summary>
    public int Index(Curve3D? s)
    {
        return s is null ? 0 : (_map.IndexOf(s) + 1);
    }

    // the number of curves, for the readers' checks
    internal int CurveCount => _map.Count;

    private static void Write(OStream os, LineCurve3D l)
    {
        os = os << Line;
        Line3D c = l.Line;
        os = os << c.Location;  // Pnt
        os = os << c.Direction; // Dir
    }

    private static void Write(OStream os, CircleCurve3D cc)
    {
        os = os << Circle;
        Circle3D c = cc.Circle;
        os = os << c.Location;
        os = os << c.Axis.Direction;
        os = os << c.XAxis.Direction;
        os = os << c.YAxis.Direction;
        os = os << c.Radius;
    }

    private static void Write(OStream os, EllipseCurve3D e)
    {
        os = os << Ellipse;
        Ellipse3D c = e.Ellipse;
        os = os << c.Location;
        os = os << c.Axis.Direction;
        os = os << c.XAxis.Direction;
        os = os << c.YAxis.Direction;
        os = os << c.MajorRadius;
        os = os << c.MinorRadius;
    }

    private static void Write(OStream os, ParabolaCurve3D p)
    {
        os = os << Parabola;
        Parabola3D c = p.Parabola;
        os = os << c.Location;
        os = os << c.Axis.Direction;
        os = os << c.XAxis.Direction;
        os = os << c.YAxis.Direction;
        os = os << c.Focal;
    }

    private static void Write(OStream os, HyperbolaCurve3D h)
    {
        os = os << Hyperbola;
        Hyperbola3D c = h.Hyperbola;
        os = os << c.Location;
        os = os << c.Axis.Direction;
        os = os << c.XAxis.Direction;
        os = os << c.YAxis.Direction;
        os = os << c.MajorRadius;
        os = os << c.MinorRadius;
    }

    private static void Write(OStream os, BezierCurve3D b)
    {
        os = os << Bezier;
        bool rational = b.IsRational;
        os = os << rational; // rational
        // poles and weights
        int i, degree = b.Degree;
        os = os << (char)degree; //<< Degree
        for (i = 1; i <= degree + 1; i++)
        {
            os = os << b.Pole(i); // Pnt
            if (rational)
            {
                os = os << b.Weight(i); // Real
            }
        }
    }

    private static void Write(OStream os, BSplineCurve3D b)
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
            os = os << b.Pole(i); // Pnt
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

    private static void Write(OStream os, TrimmedCurve3D c)
    {
        os = os << Trimmed;
        os = os << c.FirstParameter;
        os = os << c.LastParameter;
        WriteCurve(c.BasisCurve, os);
    }

    private static void Write(OStream os, OffsetCurve3D c)
    {
        os = os << Offset;
        os = os << c.Offset; // Offset
        os = os << c.Direction;
        WriteCurve(c.BasisCurve, os);
    }

    /// <summary>Writes a curve (its type byte, then its data).</summary>
    /// <exception cref="FailureException">The curve is of a type the format doesn't know.</exception>
    public static void WriteCurve(Curve3D c, OStream os)
    {
        Type theType = c.GetType();
        try
        {
            if (theType == typeof(LineCurve3D))
            {
                Write(os, (LineCurve3D)c);
            }
            else if (theType == typeof(CircleCurve3D))
            {
                Write(os, (CircleCurve3D)c);
            }
            else if (theType == typeof(EllipseCurve3D))
            {
                Write(os, (EllipseCurve3D)c);
            }
            else if (theType == typeof(ParabolaCurve3D))
            {
                Write(os, (ParabolaCurve3D)c);
            }
            else if (theType == typeof(HyperbolaCurve3D))
            {
                Write(os, (HyperbolaCurve3D)c);
            }
            else if (theType == typeof(BezierCurve3D))
            {
                Write(os, (BezierCurve3D)c);
            }
            else if (theType == typeof(BSplineCurve3D))
            {
                Write(os, (BSplineCurve3D)c);
            }
            else if (theType == typeof(TrimmedCurve3D))
            {
                Write(os, (TrimmedCurve3D)c);
            }
            else if (theType == typeof(OffsetCurve3D))
            {
                Write(os, (OffsetCurve3D)c);
            }
            else
            {
                throw new FailureException("UNKNOWN CURVE TYPE");
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_CurveSet::WriteCurve(..)", exception);
        }
    }

    /// <summary>Writes the set ("Curves", the count, then each curve).</summary>
    public void Write(Stream os, ProgressRange range = default)
    {
        int i, nbcurv = _map.Count;
        using var ps = new ProgressScope(range, "Writing curves", nbcurv);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "Curves ");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, nbcurv);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");
        var stream = new OStream(os);
        for (i = 1; i <= nbcurv && ps.More(); i++, ps.Next())
        {
            WriteCurve((Curve3D)_map[i - 1], stream);
        }
    }

    internal static void Read(Stream IS, out Point3D p)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double x);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double y);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double z);
        p = new Point3D(x, y, z);
    }

    internal static void Read(Stream IS, ref Direction3D d)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double x);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double y);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double z);
        d = d.WithCoord(x, y, z);
    }

    private static LineCurve3D ReadLine(Stream IS)
    {
        Direction3D ax = new(Direction3D.D.X);
        Read(IS, out Point3D p);
        Read(IS, ref ax);
        return new LineCurve3D(p, ax);
    }

    private static CircleCurve3D ReadCircle(Stream IS)
    {
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        Read(IS, out Point3D p);
        Read(IS, ref a);
        Read(IS, ref ax);
        Read(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r);
        return new CircleCurve3D(new Frame3D(p, a, ax), r);
    }

    private static EllipseCurve3D ReadEllipse(Stream IS)
    {
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        Read(IS, out Point3D p);
        Read(IS, ref a);
        Read(IS, ref ax);
        Read(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r1);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r2);
        return new EllipseCurve3D(new Frame3D(p, a, ax), r1, r2);
    }

    private static ParabolaCurve3D ReadParabola(Stream IS)
    {
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        Read(IS, out Point3D p);
        Read(IS, ref a);
        Read(IS, ref ax);
        Read(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r1);
        return new ParabolaCurve3D(new Frame3D(p, a, ax), r1);
    }

    private static HyperbolaCurve3D ReadHyperbola(Stream IS)
    {
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        Read(IS, out Point3D p);
        Read(IS, ref a);
        Read(IS, ref ax);
        Read(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r1);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r2);
        return new HyperbolaCurve3D(new Frame3D(p, a, ax), r1, r2);
    }

    private static BezierCurve3D ReadBezier(Stream IS)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool rational);

        // poles and weights
        int i, degree;
        // degree;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetExtChar(IS, out char val);
        degree = val;
        // Kerf: untrusted input
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, degree + 1L, rational ? 32 : 24, "poles");

        var poles = new NCollection_Array1<Point3D>(1, degree + 1);
        var weights = new NCollection_Array1<double>(1, degree + 1);
        for (i = 1; i <= degree + 1; i++)
        {
            Read(IS, out poles[i]); // Pnt
            if (rational)
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out weights[i]);
            }
        }

        if (rational)
        {
            return new BezierCurve3D(poles, weights);
        }

        return new BezierCurve3D(poles);
    }

    private static BSplineCurve3D ReadBSpline(Stream IS)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool rational);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool periodic);

        // poles and weights
        int i, degree, nbpoles, nbknots;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetExtChar(IS, out char val);
        degree = val;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out nbpoles);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out nbknots);
        // Kerf: untrusted input
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbpoles, rational ? 32 : 24, "poles");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbknots, 12, "knots");

        var poles = new NCollection_Array1<Point3D>(1, nbpoles);
        var weights = new NCollection_Array1<double>(1, nbpoles);
        for (i = 1; i <= nbpoles; i++)
        {
            Read(IS, out poles[i]); // Pnt
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
            return new BSplineCurve3D(poles, weights, knots, mults, degree, periodic);
        }

        return new BSplineCurve3D(poles, knots, mults, degree, periodic);
    }

    private static TrimmedCurve3D ReadTrimmed(Stream IS, int depth)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double p1); // FirstParameter
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double p2); // LastParameter
        ReadCurve(IS, out Curve3D? bc, depth + 1);
        return new TrimmedCurve3D(bc!, p1, p2);
    }

    private static OffsetCurve3D ReadOffset(Stream IS, int depth)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double p); // Offset
        Direction3D d = new(Direction3D.D.X);
        Read(IS, ref d);
        ReadCurve(IS, out Curve3D? bc, depth + 1);
        return new OffsetCurve3D(bc!, p, d);
    }

    /// <summary>Reads a curve written by <see cref="WriteCurve"/>. The curve is new and not frozen.</summary>
    /// <exception cref="FailureException">The curve can't be read.</exception>
    public static Stream ReadCurve(Stream IS, out Curve3D? c) => ReadCurve(IS, out c, 0);

    internal static Stream ReadCurve(Stream IS, out Curve3D? c, int depth)
    {
        c = null;
        try
        {
            if (depth > global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxGeometryNesting)
            {
                // Kerf: untrusted input: OCCT recurses without limit
                throw new StreamFormatErrorException("BinTools_CurveSet: curves nested too deeply");
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
                    throw new FailureException("UNKNOWN CURVE TYPE");
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            c = null;
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_CurveSet::ReadCurve(..)", exception);
        }

        return IS;
    }

    /// <summary>
    /// Reads a set written by <see cref="Write(Stream, ProgressRange)"/>, adding its curves (frozen) to this one.
    /// </summary>
    /// <exception cref="FailureException">The stream doesn't hold a curve table, or a curve can't be read.</exception>
    public void Read(Stream IS, ProgressRange range = default)
    {
        string? buffer = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadWord(IS);
        if (buffer is null || buffer != "Curves")
        {
            throw new FailureException("BinTools_CurveSet::Read:  Not a Curve table" + "\n");
        }

        int i, nbcurve;
        nbcurve = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedInteger(IS);
        using var ps = new ProgressScope(range, "Reading curves", nbcurve);

        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // remove <lf>
        // Kerf: untrusted input: a curve takes at least 28 bytes (a Bezier curve of degree 0)
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbcurve, 28, "curves");
        for (i = 1; i <= nbcurve && ps.More(); i++, ps.Next())
        {
            ReadCurve(IS, out Curve3D? c);
            c!.Freeze();
            _map.Add(c);
        }
    }
}
