// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_Curve2dSet.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_Curve2dSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// A set of Geom2d curves, indexed from 1, written and read in OCCT's text format (the "Curve2ds" section of a
/// <c>.brep</c> file).
/// </summary>
/// <remarks>
/// As <see cref="CurveSet"/>: OCCT's exact output, C++'s extraction when reading, curves read are frozen,
/// and curves that can't be read are one null entry. Unlike the 3D set, and as in OCCT, <see cref="Add"/> and
/// <see cref="Index"/> accept null (the null entry).
/// </remarks>
internal class Curve2dSet
{
    private const int Line = 1;
    private const int Circle = 2;
    private const int Ellipse = 3;
    private const int Parabola = 4;
    private const int Hyperbola = 5;
    private const int Bezier = 6;
    private const int Bspline = 7;
    private const int Trimmed = 8;
    private const int Offset = 9;

    // OCCT's NCollection_IndexedMap of handles, which hashes by address and can hold one null handle
    private readonly IndexedSet<object> _map = new(ReferenceEqualityComparer.Instance);

    /// <summary>An empty set of 2D curves.</summary>
    public Curve2dSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>Adds a curve to the set (once) and returns its index. Null is added too, as in OCCT.</summary>
    public int Add(Curve2D? s)
    {
        // OCCT: adds the null handle too, as one entry (occt-issues.md)
        return _map.Add((object?)s ?? CurveSet.NullKey) + 1;
    }

    /// <summary>The curve of index I, or null if there is none.</summary>
    public Curve2D? Curve2d(int i)
    {
        if (i <= 0 || i > _map.Count)
        {
            return null;
        }

        return _map[i - 1] as Curve2D;
    }

    /// <summary>The index of the curve, 0 if it isn't in the set (null finds the null entry, as in OCCT).</summary>
    public int Index(Curve2D? s)
    {
        return _map.IndexOf((object?)s ?? CurveSet.NullKey) + 1;
    }

    private static void Out(TextWriter os, double value) => StreamUtilities.Write(os, value);

    private static void Out(TextWriter os, int value) => StreamUtilities.Write(os, value);

    private static string Setw(int width, int value) => StreamUtilities.Setw(width, value.ToString(CultureInfo.InvariantCulture));

    private static void Print(Point2D p, TextWriter os, bool compact)
    {
        Out(os, p.X);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, p.Y);
        os.Write(" ");
    }

    private static void Print(Direction2D d, TextWriter os, bool compact)
    {
        Out(os, d.X);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, d.Y);
        os.Write(" ");
    }

    private static void Print(LineCurve2D l, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Line);
            os.Write(" ");
        }
        else
        {
            os.Write("Line");
        }

        Line2D c2d = l.Line;
        if (!compact)
        {
            os.Write("\n  Origin :");
        }

        Print(c2d.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(c2d.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(CircleCurve2D c, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Circle);
            os.Write(" ");
        }
        else
        {
            os.Write("Circle");
        }

        Circle2D c2d = c.Circle;
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(c2d.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(c2d.XAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(c2d.YAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  Radius :");
        }

        Out(os, c2d.Radius);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(EllipseCurve2D e, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Ellipse);
            os.Write(" ");
        }
        else
        {
            os.Write("Ellipse");
        }

        Ellipse2D c2d = e.Ellipse;
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(c2d.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(c2d.XAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(c2d.YAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  Radii  :");
        }

        Out(os, c2d.MajorRadius);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, c2d.MinorRadius);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(ParabolaCurve2D p, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Parabola);
            os.Write(" ");
        }
        else
        {
            os.Write("Parabola");
        }

        Parabola2D c2d = p.Parabola;
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(c2d.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(c2d.Axis.XAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(c2d.Axis.YAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  Focal  :");
        }

        Out(os, c2d.Focal);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(HyperbolaCurve2D h, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Hyperbola);
            os.Write(" ");
        }
        else
        {
            os.Write("Hyperbola");
        }

        Hyperbola2D c2d = h.Hyperbola;
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(c2d.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(c2d.XAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(c2d.YAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  Radii  :");
        }

        Out(os, c2d.MajorRadius);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, c2d.MinorRadius);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(BezierCurve2D b, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Bezier);
            os.Write(" ");
        }
        else
        {
            os.Write("BezierCurve");
        }

        bool rational = b.IsRational;
        if (compact)
        {
            Out(os, rational ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (rational)
            {
                os.Write(" rational");
            }
        }

        // poles and weights
        int i, degree = b.Degree;
        if (!compact)
        {
            os.Write("\n  Degree :");
        }

        Out(os, degree);
        os.Write(" ");

        for (i = 1; i <= degree + 1; i++)
        {
            if (!compact)
            {
                os.Write("\n  ");
                os.Write(Setw(2, i));
                os.Write(" : ");
            }

            Print(b.Pole(i), os, compact);
            if (rational)
            {
                os.Write(" ");
                Out(os, b.Weight(i));
            }

            if (compact)
            {
                os.Write(" ");
            }
        }

        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(BSplineCurve2D b, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Bspline);
            os.Write(" ");
        }
        else
        {
            os.Write("BSplineCurve");
        }

        bool rational = b.IsRational;
        if (compact)
        {
            Out(os, rational ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (rational)
            {
                os.Write(" rational");
            }
        }

        bool periodic = b.IsPeriodic;
        if (compact)
        {
            Out(os, periodic ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (periodic)
            {
                os.Write(" periodic");
            }
        }

        // poles and weights
        int i, degree, nbpoles, nbknots;
        degree = b.Degree;
        nbpoles = b.PoleCount;
        nbknots = b.KnotCount;
        if (!compact)
        {
            os.Write("\n  Degree ");
        }
        else
        {
            os.Write(" ");
        }

        Out(os, degree);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, nbpoles);
        if (!compact)
        {
            os.Write(" Poles,");
        }

        os.Write(" ");
        Out(os, nbknots);
        os.Write(" ");
        if (!compact)
        {
            os.Write(" Knots");
        }

        if (!compact)
        {
            os.Write("Poles :\n");
        }

        for (i = 1; i <= nbpoles; i++)
        {
            if (!compact)
            {
                os.Write("\n  ");
                os.Write(Setw(2, i));
                os.Write(" : ");
            }
            else
            {
                os.Write(" ");
            }

            Print(b.Pole(i), os, compact);
            if (rational)
            {
                os.Write(" ");
                Out(os, b.Weight(i));
            }
        }

        os.Write("\n");

        if (!compact)
        {
            os.Write("Knots :\n");
        }

        for (i = 1; i <= nbknots; i++)
        {
            if (!compact)
            {
                os.Write("\n  ");
                os.Write(Setw(2, i));
                os.Write(" : ");
            }
            else
            {
                os.Write(" ");
            }

            Out(os, b.Knot(i));
            os.Write(" ");
            Out(os, b.Multiplicity(i));
        }

        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(TrimmedCurve2D c, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Trimmed);
            os.Write(" ");
        }
        else
        {
            os.Write("Trimmed curve\n");
        }

        if (!compact)
        {
            os.Write("Parameters : ");
        }

        Out(os, c.FirstParameter);
        os.Write(" ");
        Out(os, c.LastParameter);
        os.Write("\n");
        if (!compact)
        {
            os.Write("Basis curve :\n");
        }

        PrintCurve2d(c.BasisCurve, os, compact);
    }

    private static void Print(OffsetCurve2D c, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Offset);
            os.Write(" ");
        }
        else
        {
            os.Write("OffsetCurve");
        }

        if (!compact)
        {
            os.Write("Offset : ");
        }

        Out(os, c.Offset);
        os.Write("\n");
        if (!compact)
        {
            os.Write("Basis curve :\n");
        }

        PrintCurve2d(c.BasisCurve, os, compact);
    }

    /// <summary>
    /// Writes the 2D curve; if <paramref name="compact"/>, in the format <see cref="ReadCurve2d"/> reads, otherwise as
    /// a readable dump, at the writer's precision. An unknown curve type goes to
    /// <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static void PrintCurve2d(Curve2D c, TextWriter os, bool compact = false)
    {
        Type theType = c.DynamicType();
        if (theType == typeof(CircleCurve2D))
        {
            Print((CircleCurve2D)c, os, compact);
        }
        else if (theType == typeof(LineCurve2D))
        {
            Print((LineCurve2D)c, os, compact);
        }
        else if (theType == typeof(EllipseCurve2D))
        {
            Print((EllipseCurve2D)c, os, compact);
        }
        else if (theType == typeof(ParabolaCurve2D))
        {
            Print((ParabolaCurve2D)c, os, compact);
        }
        else if (theType == typeof(HyperbolaCurve2D))
        {
            Print((HyperbolaCurve2D)c, os, compact);
        }
        else if (theType == typeof(BezierCurve2D))
        {
            Print((BezierCurve2D)c, os, compact);
        }
        else if (theType == typeof(BSplineCurve2D))
        {
            Print((BSplineCurve2D)c, os, compact);
        }
        else if (theType == typeof(TrimmedCurve2D))
        {
            Print((TrimmedCurve2D)c, os, compact);
        }
        else if (theType == typeof(OffsetCurve2D))
        {
            Print((OffsetCurve2D)c, os, compact);
        }
        else
        {
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetUndefinedTypeHandler().PrintCurve2d(c, os, compact);
        }
    }

    /// <summary>Writes a readable dump of the set.</summary>
    public void Dump(TextWriter os)
    {
        int i, nbsurf = _map.Count;
        os.Write("\n -------\n");
        os.Write("Dump of ");
        Out(os, nbsurf);
        os.Write(" Curve2ds ");
        os.Write("\n -------\n\n");

        for (i = 1; i <= nbsurf; i++)
        {
            os.Write(Setw(4, i));
            os.Write(" : ");
            // OCCT dereferences a null curve here
            PrintCurve2d(Curve2d(i)!, os, false);
        }
    }

    /// <summary>
    /// Writes the set in the format <see cref="Read"/> reads: "Curve2ds", the count, then each curve, at 17
    /// significant digits (the writer's precision is restored afterwards).
    /// </summary>
    public void Write(TextWriter os, ProgressRange progress = default)
    {
        int prec = StreamUtilities.Precision(os, 17);

        int i, nbsurf = _map.Count;
        os.Write("Curve2ds ");
        Out(os, nbsurf);
        os.Write("\n");
        using var ps = new ProgressScope(progress, "2D Curves", nbsurf);
        for (i = 1; i <= nbsurf && ps.More(); i++, ps.Next())
        {
            // OCCT dereferences a null curve here
            PrintCurve2d(Curve2d(i)!, os, true);
        }

        StreamUtilities.Precision(os, prec);
    }

    // operator>> of the points, directions and curves

    private static void ReadPnt(TextReader IS, out Point2D p)
    {
        double x, y;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
        p = new Point2D(x, y);
    }

    private static void ReadDir(TextReader IS, ref Direction2D d)
    {
        double x, y;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
        d = d.WithCoord(x, y);
    }

    private static LineCurve2D ReadLine(TextReader IS)
    {
        Point2D p;
        Direction2D ax = new(Direction2D.D.X);
        ReadPnt(IS, out p);
        ReadDir(IS, ref ax);
        return new LineCurve2D(p, ax);
    }

    private static CircleCurve2D ReadCircle(TextReader IS)
    {
        Point2D p;
        Direction2D ax = new(Direction2D.D.X), ay = new(Direction2D.D.X);
        double r;
        ReadPnt(IS, out p);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r);
        return new CircleCurve2D(new Frame2D(p, ax, ay), r);
    }

    private static EllipseCurve2D ReadEllipse(TextReader IS)
    {
        Point2D p;
        Direction2D ax = new(Direction2D.D.X), ay = new(Direction2D.D.X);
        double r1, r2;
        ReadPnt(IS, out p);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r2);
        return new EllipseCurve2D(new Frame2D(p, ax, ay), r1, r2);
    }

    private static ParabolaCurve2D ReadParabola(TextReader IS)
    {
        Point2D p;
        Direction2D ax = new(Direction2D.D.X), ay = new(Direction2D.D.X);
        double r1;
        ReadPnt(IS, out p);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r1);
        return new ParabolaCurve2D(new Frame2D(p, ax, ay), r1);
    }

    private static HyperbolaCurve2D ReadHyperbola(TextReader IS)
    {
        Point2D p;
        Direction2D ax = new(Direction2D.D.X), ay = new(Direction2D.D.X);
        double r1, r2;
        ReadPnt(IS, out p);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r2);
        return new HyperbolaCurve2D(new Frame2D(p, ax, ay), r1, r2);
    }

    private static BezierCurve2D ReadBezier(TextReader IS)
    {
        bool rational = false;
        StreamUtilities.Read(IS, ref rational);

        // poles and weights
        int i = 0, degree = 0;
        StreamUtilities.Read(IS, ref degree);

        // Kerf: untrusted input: counts checked against the input before allocating (a pole takes at least 4
        // characters, a knot 4)
        StreamUtilities.CheckCount(IS, Math.Max(degree + 1L, 0), 4, "Bezier poles");
        var poles = new NCollection_Array1<Point2D>(1, degree + 1);
        var weights = new NCollection_Array1<double>(1, degree + 1);

        for (i = 1; i <= degree + 1; i++)
        {
            ReadPnt(IS, out poles[i]);
            if (rational)
            {
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out weights[i]);
            }
        }

        if (rational)
        {
            return new BezierCurve2D(poles, weights);
        }

        return new BezierCurve2D(poles);
    }

    private static BSplineCurve2D ReadBSpline(TextReader IS)
    {
        bool rational = false, periodic = false;
        StreamUtilities.Read(IS, ref rational);
        StreamUtilities.Read(IS, ref periodic);

        // poles and weights
        int i = 0, degree = 0, nbpoles = 0, nbknots = 0;
        StreamUtilities.Read(IS, ref degree);
        StreamUtilities.Read(IS, ref nbpoles);
        StreamUtilities.Read(IS, ref nbknots);

        // Kerf: untrusted input: counts checked against the input before allocating (a pole takes at least 4
        // characters, a knot 4)
        StreamUtilities.CheckCount(IS, Math.Max(nbpoles, 0), 4, "B-spline poles");
        StreamUtilities.CheckCount(IS, Math.Max(nbknots, 0), 4, "B-spline knots");
        var poles = new NCollection_Array1<Point2D>(1, nbpoles);
        var weights = new NCollection_Array1<double>(1, nbpoles);

        for (i = 1; i <= nbpoles; i++)
        {
            ReadPnt(IS, out poles[i]);
            if (rational)
            {
                global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out weights[i]);
            }
        }

        var knots = new NCollection_Array1<double>(1, nbknots);
        var mults = new NCollection_Array1<int>(1, nbknots);

        for (i = 1; i <= nbknots; i++)
        {
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out knots[i]);
            StreamUtilities.Read(IS, ref mults[i]);
        }

        if (rational)
        {
            return new BSplineCurve2D(poles, weights, knots, mults, degree, periodic);
        }

        return new BSplineCurve2D(poles, knots, mults, degree, periodic);
    }

    private static TrimmedCurve2D ReadTrimmed(TextReader IS)
    {
        double p1, p2;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out p1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out p2);
        Curve2D? bc = ReadCurve2d(IS);
        return new TrimmedCurve2D(bc!, p1, p2);
    }

    private static OffsetCurve2D ReadOffset(TextReader IS)
    {
        double p;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out p);
        Curve2D? bc = ReadCurve2d(IS);
        return new OffsetCurve2D(bc!, p);
    }

    /// <summary>
    /// Reads a 2D curve written by <see cref="PrintCurve2d"/> (compact). Returns null if the curve can't be built
    /// from what was read; an unknown type goes to <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static Curve2D? ReadCurve2d(TextReader IS)
    {
        // OCCT's ctype is uninitialised, and left so when the stream has already failed
        int ctype = 0;

        Curve2D? c = null;
        // Kerf: untrusted input: trimmed, offset and swept geometry recurse; the nesting is capped
        using var nesting = global::Kerf.Engine.Exchange.Brep.GeomTools.Nesting();
        try
        {
            StreamUtilities.Read(IS, ref ctype);
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
                    c = ReadTrimmed(IS);
                    break;

                case Offset:
                    c = ReadOffset(IS);
                    break;

                default:
                {
                    Curve2D? cc = null;
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetUndefinedTypeHandler().ReadCurve2d(ctype, IS, ref cc);
                    c = cc;
                }
                    break;
            }
        }
        catch (Exception exception) when (Refusal.IsFailure(exception) && (exception is not StreamFormatErrorException))
        {
            // OCCT_DEBUG only: "EXCEPTION in GeomTools_Curve2dSet::ReadCurve2d(..)!!!"
        }
        catch (NullReferenceException)
        {
            // OCCT: a trimmed or offset curve whose basis couldn't be read: OCCT dereferences the null handle, which
            // OCC_CATCH_SIGNALS turns into a caught Standard_AccessViolation where signals are converted
        }

        return c;
    }

    /// <summary>
    /// Reads a set written by <see cref="Write"/>, adding its curves (frozen) to this one. Writes "Not a Curve2d
    /// table" to the console, as OCCT does, if the text doesn't start with "Curve2ds".
    /// </summary>
    public void Read(TextReader IS, ProgressRange progress = default)
    {
        // OCCT reads into char[255]: a longer word is an error (Kerf: untrusted input)
        char[] buffer = new char[255];
        bool isRead = StreamUtilities.ReadWord(IS, buffer);
        if (!isRead || StreamUtilities.CString(buffer) != "Curve2ds")
        {
            Console.Out.Write("Not a Curve2d table" + "\n");
            return;
        }

        // OCCT's nbcurve is uninitialised if the count can't be read
        int i, nbcurve = 0;
        StreamUtilities.Read(IS, ref nbcurve);
        // Kerf: untrusted input: an object takes at least 2 characters
        StreamUtilities.CheckCount(IS, Math.Max(nbcurve, 0), 2, "2D curves");
        using var ps = new ProgressScope(progress, "2D Curves", nbcurve);
        for (i = 1; i <= nbcurve && ps.More(); i++, ps.Next())
        {
            Curve2D? c = ReadCurve2d(IS);
            c?.Freeze();
            // OCCT: every object that can't be read is the one null entry, so the later indices shift
            _map.Add((object?)c ?? CurveSet.NullKey);
        }
    }
}
