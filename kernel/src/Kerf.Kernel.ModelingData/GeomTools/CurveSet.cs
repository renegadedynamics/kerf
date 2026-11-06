// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_CurveSet.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_CurveSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// A set of Geom curves, indexed from 1, written and read in OCCT's text format (the "Curves" section of a
/// <c>.brep</c> file).
/// </summary>
/// <remarks>
/// <see cref="Write"/> writes exactly what OCCT writes (17 significant digits, <c>'\n'</c> line ends);
/// <see cref="Read"/> reads OCCT's files, following C++'s stream extraction (<see cref="StreamUtilities"/>). The curves
/// that <see cref="Read"/> creates are frozen: the set shares them with whoever asks for them. A curve that can't be
/// read (its constructor throws) is stored as null, as in OCCT, which stores one null entry however many fail.
/// </remarks>
internal class CurveSet
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

    // the null handle as a key
    internal static readonly object NullKey = new();

    /// <summary>An empty set of curves.</summary>
    public CurveSet()
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

    /// <summary>The curve of index I, or null if there is none.</summary>
    public Curve3D? Curve(int i)
    {
        if (i <= 0 || i > _map.Count)
        {
            return null;
        }

        return _map[i - 1] as Curve3D;
    }

    /// <summary>The index of the curve, 0 if it isn't in the set.</summary>
    public int Index(Curve3D? s)
    {
        return s is null ? 0 : (_map.IndexOf(s) + 1);
    }

    private static void Out(TextWriter os, double value) => StreamUtilities.Write(os, value);

    private static void Out(TextWriter os, int value) => StreamUtilities.Write(os, value);

    private static void Print(in Point3D p, TextWriter os, bool compact)
    {
        Out(os, p.X);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, p.Y);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, p.Z);
        os.Write(" ");
    }

    private static void Print(in Direction3D d, TextWriter os, bool compact)
    {
        Out(os, d.X);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, d.Y);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, d.Z);
        os.Write(" ");
    }

    private static void Print(LineCurve3D l, TextWriter os, bool compact)
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

        Line3D c = l.Line;
        if (!compact)
        {
            os.Write("\n  Origin :");
        }

        Print(c.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(c.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(CircleCurve3D cc, TextWriter os, bool compact)
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

        Circle3D c = cc.Circle;
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(c.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(c.Axis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(c.XAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(c.YAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  Radius :");
        }

        Out(os, c.Radius);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(EllipseCurve3D e, TextWriter os, bool compact)
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

        Ellipse3D c = e.Ellipse;
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(c.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(c.Axis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(c.XAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(c.YAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  Radii  :");
        }

        Out(os, c.MajorRadius);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, c.MinorRadius);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(ParabolaCurve3D p, TextWriter os, bool compact)
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

        Parabola3D c = p.Parabola;
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(c.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(c.Axis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(c.XAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(c.YAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  Focal  :");
        }

        Out(os, c.Focal);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(HyperbolaCurve3D h, TextWriter os, bool compact)
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

        Hyperbola3D c = h.Hyperbola;
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(c.Location, os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(c.Axis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(c.XAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(c.YAxis.Direction, os, compact);
        if (!compact)
        {
            os.Write("\n  Radii  :");
        }

        Out(os, c.MajorRadius);
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, c.MinorRadius);
        if (!compact)
        {
            os.Write("\n");
        }

        os.Write("\n");
    }

    private static void Print(BezierCurve3D b, TextWriter os, bool compact)
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
                os.Write(StreamUtilities.Setw(2, i.ToString(System.Globalization.CultureInfo.InvariantCulture)));
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

        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(BSplineCurve3D b, TextWriter os, bool compact)
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
            os.Write(" Knots\n");
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
                os.Write(StreamUtilities.Setw(2, i.ToString(System.Globalization.CultureInfo.InvariantCulture)));
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
                os.Write(StreamUtilities.Setw(2, i.ToString(System.Globalization.CultureInfo.InvariantCulture)));
                os.Write(" : ");
            }

            os.Write(" ");
            Out(os, b.Knot(i));
            os.Write(" ");
            Out(os, b.Multiplicity(i));
        }

        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(TrimmedCurve3D c, TextWriter os, bool compact)
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

        PrintCurve(c.BasisCurve, os, compact);
    }

    private static void Print(OffsetCurve3D c, TextWriter os, bool compact)
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
            os.Write("Direction : ");
        }

        Print(c.Direction, os, compact);
        os.Write("\n");
        if (!compact)
        {
            os.Write("Basis curve :\n");
        }

        PrintCurve(c.BasisCurve, os, compact);
    }

    /// <summary>
    /// Writes the curve; if <paramref name="compact"/>, in the format <see cref="ReadCurve"/> reads, otherwise as a
    /// readable dump. Reals are written at the writer's precision (<see cref="StreamUtilities.Precision(TextWriter)"/>,
    /// 6 unless set). An unknown curve type goes to <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static void PrintCurve(Curve3D c, TextWriter os, bool compact = false)
    {
        Type theType = c.DynamicType();

        if (theType == typeof(LineCurve3D))
        {
            Print((LineCurve3D)c, os, compact);
        }
        else if (theType == typeof(CircleCurve3D))
        {
            Print((CircleCurve3D)c, os, compact);
        }
        else if (theType == typeof(EllipseCurve3D))
        {
            Print((EllipseCurve3D)c, os, compact);
        }
        else if (theType == typeof(ParabolaCurve3D))
        {
            Print((ParabolaCurve3D)c, os, compact);
        }
        else if (theType == typeof(HyperbolaCurve3D))
        {
            Print((HyperbolaCurve3D)c, os, compact);
        }
        else if (theType == typeof(BezierCurve3D))
        {
            Print((BezierCurve3D)c, os, compact);
        }
        else if (theType == typeof(BSplineCurve3D))
        {
            Print((BSplineCurve3D)c, os, compact);
        }
        else if (theType == typeof(TrimmedCurve3D))
        {
            Print((TrimmedCurve3D)c, os, compact);
        }
        else if (theType == typeof(OffsetCurve3D))
        {
            Print((OffsetCurve3D)c, os, compact);
        }
        else
        {
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetUndefinedTypeHandler().PrintCurve(c, os, compact);
        }
    }

    /// <summary>Writes a readable dump of the set.</summary>
    public void Dump(TextWriter os)
    {
        int i, nbsurf = _map.Count;
        os.Write("\n -------\n");
        os.Write("Dump of ");
        Out(os, nbsurf);
        os.Write(" Curves ");
        os.Write("\n -------\n\n");

        for (i = 1; i <= nbsurf; i++)
        {
            os.Write(StreamUtilities.Setw(4, i.ToString(System.Globalization.CultureInfo.InvariantCulture)));
            os.Write(" : ");
            // OCCT dereferences a null curve here (one that couldn't be read)
            PrintCurve(Curve(i)!, os, false);
        }
    }

    /// <summary>
    /// Writes the set in the format <see cref="Read"/> reads: "Curves", the count, then each curve, at 17
    /// significant digits (the writer's precision is restored afterwards).
    /// </summary>
    public void Write(TextWriter os, ProgressRange progress = default)
    {
        int prec = StreamUtilities.Precision(os, 17);

        int i, nbcurve = _map.Count;
        os.Write("Curves ");
        Out(os, nbcurve);
        os.Write("\n");
        using var ps = new ProgressScope(progress, "3D Curves", nbcurve);
        for (i = 1; i <= nbcurve && ps.More(); i++, ps.Next())
        {
            // OCCT dereferences a null curve here (one that couldn't be read)
            PrintCurve(Curve(i)!, os, true);
        }

        StreamUtilities.Precision(os, prec);
    }

    // operator>> of the points, directions and curves

    private static void ReadPnt(TextReader IS, out Point3D p)
    {
        double x, y, z;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out z);
        p = new Point3D(x, y, z);
    }

    private static void ReadDir(TextReader IS, ref Direction3D d)
    {
        double x, y, z;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out z);
        d = d.WithCoord(x, y, z);
    }

    private static LineCurve3D ReadLine(TextReader IS)
    {
        Point3D p;
        Direction3D ax = new(Direction3D.D.X);
        ReadPnt(IS, out p);
        ReadDir(IS, ref ax);
        return new LineCurve3D(p, ax);
    }

    private static CircleCurve3D ReadCircle(TextReader IS)
    {
        Point3D p;
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        double r;
        ReadPnt(IS, out p);
        ReadDir(IS, ref a);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r);
        return new CircleCurve3D(new Frame3D(p, a, ax), r);
    }

    private static EllipseCurve3D ReadEllipse(TextReader IS)
    {
        Point3D p;
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        double r1, r2;
        ReadPnt(IS, out p);
        ReadDir(IS, ref a);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r2);
        return new EllipseCurve3D(new Frame3D(p, a, ax), r1, r2);
    }

    private static ParabolaCurve3D ReadParabola(TextReader IS)
    {
        Point3D p;
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        double r1;
        ReadPnt(IS, out p);
        ReadDir(IS, ref a);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r1);
        return new ParabolaCurve3D(new Frame3D(p, a, ax), r1);
    }

    private static HyperbolaCurve3D ReadHyperbola(TextReader IS)
    {
        Point3D p;
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        double r1, r2;
        ReadPnt(IS, out p);
        ReadDir(IS, ref a);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r2);
        return new HyperbolaCurve3D(new Frame3D(p, a, ax), r1, r2);
    }

    private static BezierCurve3D ReadBezier(TextReader IS)
    {
        bool rational = false;
        StreamUtilities.Read(IS, ref rational);

        // poles and weights
        int i = 0, degree = 0;
        StreamUtilities.Read(IS, ref degree);

        // Kerf: untrusted input: counts checked against the input before allocating (a pole takes at least 6
        // characters, a knot 4)
        StreamUtilities.CheckCount(IS, Math.Max(degree + 1L, 0), 6, "Bezier poles");
        var poles = new NCollection_Array1<Point3D>(1, degree + 1);
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
            return new BezierCurve3D(poles, weights);
        }

        return new BezierCurve3D(poles);
    }

    private static BSplineCurve3D ReadBSpline(TextReader IS)
    {
        bool rational = false, periodic = false;
        StreamUtilities.Read(IS, ref rational);
        StreamUtilities.Read(IS, ref periodic);

        // poles and weights
        int i = 0, degree = 0, nbpoles = 0, nbknots = 0;
        StreamUtilities.Read(IS, ref degree);
        StreamUtilities.Read(IS, ref nbpoles);
        StreamUtilities.Read(IS, ref nbknots);

        // Kerf: untrusted input: counts checked against the input before allocating (a pole takes at least 6
        // characters, a knot 4)
        StreamUtilities.CheckCount(IS, Math.Max(nbpoles, 0), 6, "B-spline poles");
        StreamUtilities.CheckCount(IS, Math.Max(nbknots, 0), 4, "B-spline knots");
        var poles = new NCollection_Array1<Point3D>(1, nbpoles);
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
            return new BSplineCurve3D(poles, weights, knots, mults, degree, periodic);
        }

        return new BSplineCurve3D(poles, knots, mults, degree, periodic);
    }

    private static TrimmedCurve3D ReadTrimmed(TextReader IS)
    {
        double p1, p2;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out p1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out p2);
        Curve3D? bc = ReadCurve(IS);
        // a null basis curve throws in the constructor, which ReadCurve catches, as OCCT's does
        return new TrimmedCurve3D(bc!, p1, p2);
    }

    private static OffsetCurve3D ReadOffset(TextReader IS)
    {
        double p;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out p);
        Direction3D d = new(Direction3D.D.X);
        ReadDir(IS, ref d);
        Curve3D? bc = ReadCurve(IS);
        return new OffsetCurve3D(bc!, p, d);
    }

    /// <summary>
    /// Reads a curve written by <see cref="PrintCurve"/> (compact). Returns null if the curve can't be built from what
    /// was read (OCCT catches the constructor's exception); an unknown type goes to
    /// <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static Curve3D? ReadCurve(TextReader IS)
    {
        // OCCT's ctype is uninitialised, and left so when the stream has already failed
        int ctype = 0;

        Curve3D? c = null;
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
                    Curve3D? cc = null;
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetUndefinedTypeHandler().ReadCurve(ctype, IS, ref cc);
                    c = cc;
                }
                    break;
            }
        }
        catch (Exception exception) when (Refusal.IsFailure(exception) && (exception is not StreamFormatErrorException))
        {
            // OCCT_DEBUG only: "EXCEPTION in GeomTools_CurveSet::ReadCurve(..)!!!"
        }
        catch (NullReferenceException)
        {
            // OCCT: a trimmed or offset curve whose basis couldn't be read: OCCT dereferences the null handle, which
            // OCC_CATCH_SIGNALS turns into a caught Standard_AccessViolation where signals are converted
        }

        return c;
    }

    /// <summary>
    /// Clears nothing and reads a set written by <see cref="Write"/>, adding its curves (frozen) to this one. Writes
    /// "Not a Curve table" to the console, as OCCT does, if the text doesn't start with "Curves".
    /// </summary>
    public void Read(TextReader IS, ProgressRange progress = default)
    {
        // OCCT reads into char[255]: a longer word is an error (Kerf: untrusted input)
        char[] buffer = new char[255];
        bool isRead = StreamUtilities.ReadWord(IS, buffer);
        if (!isRead || StreamUtilities.CString(buffer) != "Curves")
        {
            Console.Out.Write("Not a Curve table" + "\n");
            return;
        }

        // OCCT's nbcurve is uninitialised if the count can't be read
        int i, nbcurve = 0;
        StreamUtilities.Read(IS, ref nbcurve);
        // Kerf: untrusted input: an object takes at least 2 characters
        StreamUtilities.CheckCount(IS, Math.Max(nbcurve, 0), 2, "curves");
        using var ps = new ProgressScope(progress, "3D Curves", nbcurve);
        for (i = 1; i <= nbcurve && ps.More(); i++, ps.Next())
        {
            Curve3D? c = ReadCurve(IS);
            c?.Freeze();
            // OCCT: every curve that can't be read is the one null entry, so the later indices shift
            _map.Add((object?)c ?? NullKey);
        }
    }
}
