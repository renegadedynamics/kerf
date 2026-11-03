// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Geometry;
using System.Globalization;
using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Native OCCT's GccAna solvers recorded by tools/gccrec (unit 5K1): every constructor of every class on random
/// configurations, each case's inputs as round-trip decimals and native's results as named tokens, every double as
/// its 64 bits in hex. <see cref="Run"/> replays a case on the port and writes its results in the same format, so that
/// the two compare token by token.
/// </summary>
internal static class GccAnaRecordings
{
    public sealed record Case(string Ctor, double[] Inputs, string[] Native);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    public static List<Case> Cases => theCases.Value;

    public static IEnumerable<object[]> CtorNames() => Cases.Select(c => c.Ctor).Distinct().Select(n => new object[] { n });

    private static List<Case> Load()
    {
        using Stream aRes = typeof(GccAnaRecordings).Assembly.GetManifestResourceStream("GccAna.gccrec.txt.gz")
                            ?? throw new InvalidOperationException("GccAna.gccrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        List<Case> aCases = [];
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            int aSemi = aLine.IndexOf(';');
            string[] aIn = aLine[..aSemi].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            double[] aValues = aIn.Skip(1).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            string[] aOut = aLine[(aSemi + 1)..].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            aCases.Add(new Case(aIn[0], aValues, aOut));
        }
        return aCases;
    }

    //=============================================================================================
    // The port's side: the inputs are read in the order gccrec generated them.

    private sealed class Reader(double[] theIn)
    {
        private int myPos;

        public double D() => theIn[myPos++];

        public Point2D Pnt()
        {
            double x = D();
            double y = D();
            return new Point2D(x, y);
        }

        public Direction2D Dir()
        {
            double x = D();
            double y = D();
            return new Direction2D(x, y);
        }

        public Line2D Lin()
        {
            Point2D p = Pnt();
            Direction2D d = Dir();
            return new Line2D(p, d);
        }

        public Circle2D Circ()
        {
            Point2D p = Pnt();
            Direction2D d = Dir();
            double r = D();
            bool s = D() != 0.0;
            return new Circle2D(new Axis2D(p, d), r, s);
        }

        public QualifiedCirc QC()
        {
            Circle2D c = Circ();
            return new QualifiedCirc(c, (Qualifier)(int)D());
        }

        public QualifiedLin QL()
        {
            Line2D l = Lin();
            return new QualifiedLin(l, (Qualifier)(int)D());
        }
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x", CultureInfo.InvariantCulture);

    private static string Hex(Point2D theP) => Hex(theP.X) + "," + Hex(theP.Y);

    private static string Hex(Direction2D theD) => Hex(theD.X) + "," + Hex(theD.Y);

    private static string Hex(Frame2D theA) => Hex(theA.Location) + "," + Hex(theA.XDirection) + "," + Hex(theA.YDirection);

    private static string Hex(Circle2D theC) => Hex(theC.Position) + "," + Hex(theC.Radius);

    private static string Hex(Line2D theL) => Hex(theL.Location) + "," + Hex(theL.Direction);

    private static string Q(Qualifier p) => ((int)p).ToString(CultureInfo.InvariantCulture);

    private delegate void TanFn(out double a, out double b, out Point2D p);

    private static string Tan(TanFn theF)
    {
        theF(out double a, out double b, out Point2D p);
        return Hex(a) + "," + Hex(b) + "," + Hex(p);
    }

    private static string Bisec(ConstraintBisec? theB)
    {
        if (theB is null)
        {
            return "null";
        }
        switch (theB.ArcType())
        {
            case ConstraintCurveType.Lin:
                return "Lin," + Hex(theB.Line());
            case ConstraintCurveType.Cir:
                return "Cir," + Hex(theB.Circle());
            case ConstraintCurveType.Ell:
                {
                    Ellipse2D anE = theB.Ellipse();
                    return "Ell," + Hex(anE.Axis) + "," + Hex(anE.MajorRadius) + "," + Hex(anE.MinorRadius);
                }
            case ConstraintCurveType.Hpr:
                {
                    Hyperbola2D aH = theB.Hyperbola();
                    return "Hpr," + Hex(aH.Axis) + "," + Hex(aH.MajorRadius) + "," + Hex(aH.MinorRadius);
                }
            case ConstraintCurveType.Par:
                {
                    Parabola2D aP = theB.Parabola();
                    return "Par," + Hex(aP.Axis) + "," + Hex(aP.Focal);
                }
            case ConstraintCurveType.Pnt:
                return "Pnt," + Hex(theB.Point());
        }
        return "?";
    }

    private sealed class Writer
    {
        public readonly List<string> Tokens = [];

        public void Put(string theTok) => Tokens.Add(theTok);

        public void Tok(string theName, Func<string> theF)
        {
            string aVal;
            try
            {
                aVal = theF();
            }
            catch (Exception e)
            {
                aVal = "!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType());
            }
            Put(theName + ":" + aVal);
        }

        // The solvers' accessors, through delegates: OCCT's classes share no base
        public void Solutions(bool theDone, Func<int> theNb, Action<int> theSolution)
        {
            Put("done:" + (theDone ? "1" : "0"));
            if (!theDone)
            {
                return;
            }
            int aNb;
            try
            {
                aNb = theNb();
            }
            catch (Exception e)
            {
                Put("nb:!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
                return;
            }
            Put("nb:" + aNb.ToString(CultureInfo.InvariantCulture));
            for (int i = 1; i <= aNb; i++)
            {
                theSolution(i);
            }
        }
    }

    private static string B(bool b) => b ? "1" : "0";

    private static void Dump(Writer w, AnalyticCirc2d3Tan S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a, out Qualifier b, out Qualifier c);
                return Q(a) + "," + Q(b) + "," + Q(c);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
            w.Tok("t2", () => Tan((out double a, out double b, out Point2D p) => S.Tangency2(i, out a, out b, out p)));
            w.Tok("s2", () => B(S.IsTheSame2(i)));
            w.Tok("t3", () => Tan((out double a, out double b, out Point2D p) => S.Tangency3(i, out a, out b, out p)));
            w.Tok("s3", () => B(S.IsTheSame3(i)));
        });

    private static void Dump(Writer w, AnalyticCirc2d2TanOn S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a, out Qualifier b);
                return Q(a) + "," + Q(b);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
            w.Tok("t2", () => Tan((out double a, out double b, out Point2D p) => S.Tangency2(i, out a, out b, out p)));
            w.Tok("s2", () => B(S.IsTheSame2(i)));
            w.Tok("c3", () =>
            {
                S.CenterOn3(i, out double a, out Point2D p);
                return Hex(a) + "," + Hex(p);
            });
        });

    private static void Dump(Writer w, AnalyticCirc2d2TanRad S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a, out Qualifier b);
                return Q(a) + "," + Q(b);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
            w.Tok("t2", () => Tan((out double a, out double b, out Point2D p) => S.Tangency2(i, out a, out b, out p)));
            w.Tok("s2", () => B(S.IsTheSame2(i)));
        });

    private static void Dump(Writer w, AnalyticCirc2dTanOnRad S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
            w.Tok("c3", () =>
            {
                S.CenterOn3(i, out double a, out Point2D p);
                return Hex(a) + "," + Hex(p);
            });
        });

    private static void Dump(Writer w, AnalyticCirc2dTanCen S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
        });

    private static void Dump(Writer w, AnalyticLin2d2Tan S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a, out Qualifier b);
                return Q(a) + "," + Q(b);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("t2", () => Tan((out double a, out double b, out Point2D p) => S.Tangency2(i, out a, out b, out p)));
        });

    private static void Dump(Writer w, AnalyticLin2dTanObl S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("i2", () => Tan((out double a, out double b, out Point2D p) => S.Intersection2(i, out a, out b, out p)));
        });

    private static void Dump(Writer w, Lin2dTanPar S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
        });

    private static void Dump(Writer w, Lin2dTanPer S)
        => w.Solutions(S.IsDone, () => S.SolutionCount, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out Qualifier a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out Point2D p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("i2", () => Tan((out double a, out double b, out Point2D p) => S.Intersection2(i, out a, out b, out p)));
        });

    private static void DumpBisec(Writer w, bool theDone, Func<int> theNb, Func<int, ConstraintBisec?> theSol)
        => w.Solutions(theDone, theNb, i => w.Tok("bis", () => Bisec(theSol(i))));

    /// <summary>Runs the case on the port and writes its results as gccrec does.</summary>
    public static string[] Run(Case theCase)
    {
        Reader r = new(theCase.Inputs);
        Writer w = new();
        try
        {
            switch (theCase.Ctor)
            {
                case "3Tan.CCC": { var a = r.QC(); var b = r.QC(); var c = r.QC(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.CCL": { var a = r.QC(); var b = r.QC(); var c = r.QL(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.CLL": { var a = r.QC(); var b = r.QL(); var c = r.QL(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.LLL": { var a = r.QL(); var b = r.QL(); var c = r.QL(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.CCP": { var a = r.QC(); var b = r.QC(); var c = r.Pnt(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.CLP": { var a = r.QC(); var b = r.QL(); var c = r.Pnt(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.LLP": { var a = r.QL(); var b = r.QL(); var c = r.Pnt(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.CPP": { var a = r.QC(); var b = r.Pnt(); var c = r.Pnt(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.LPP": { var a = r.QL(); var b = r.Pnt(); var c = r.Pnt(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "3Tan.PPP": { var a = r.Pnt(); var b = r.Pnt(); var c = r.Pnt(); double t = r.D(); Dump(w, new AnalyticCirc2d3Tan(a, b, c, t)); break; }
                case "2TanOn.CCl": { var a = r.QC(); var b = r.QC(); var c = r.Lin(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CLl": { var a = r.QC(); var b = r.QL(); var c = r.Lin(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.LLl": { var a = r.QL(); var b = r.QL(); var c = r.Lin(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CPl": { var a = r.QC(); var b = r.Pnt(); var c = r.Lin(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.LPl": { var a = r.QL(); var b = r.Pnt(); var c = r.Lin(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.PPl": { var a = r.Pnt(); var b = r.Pnt(); var c = r.Lin(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CCc": { var a = r.QC(); var b = r.QC(); var c = r.Circ(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CLc": { var a = r.QC(); var b = r.QL(); var c = r.Circ(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CPc": { var a = r.QC(); var b = r.Pnt(); var c = r.Circ(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.LLc": { var a = r.QL(); var b = r.QL(); var c = r.Circ(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.LPc": { var a = r.QL(); var b = r.Pnt(); var c = r.Circ(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.PPc": { var a = r.Pnt(); var b = r.Pnt(); var c = r.Circ(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanOn(a, b, c, t)); break; }
                case "2TanRad.CC": { var a = r.QC(); var b = r.QC(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.CL": { var a = r.QC(); var b = r.QL(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.CP": { var a = r.QC(); var b = r.Pnt(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.LP": { var a = r.QL(); var b = r.Pnt(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.LL": { var a = r.QL(); var b = r.QL(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.PP": { var a = r.Pnt(); var b = r.Pnt(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2d2TanRad(a, b, R, t)); break; }
                case "TanOnRad.Cl": { var a = r.QC(); var c = r.Lin(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Ll": { var a = r.QL(); var c = r.Lin(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Pl": { var a = r.Pnt(); var c = r.Lin(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Cc": { var a = r.QC(); var c = r.Circ(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Lc": { var a = r.QL(); var c = r.Circ(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Pc": { var a = r.Pnt(); var c = r.Circ(); double R = r.D(); double t = r.D(); Dump(w, new AnalyticCirc2dTanOnRad(a, c, R, t)); break; }
                case "TanCen.C": { var a = r.QC(); var p = r.Pnt(); double t = r.D(); Dump(w, new AnalyticCirc2dTanCen(a, p, t)); break; }
                case "TanCen.L": { var a = r.Lin(); var p = r.Pnt(); Dump(w, new AnalyticCirc2dTanCen(a, p)); break; }
                case "TanCen.P": { var a = r.Pnt(); var p = r.Pnt(); Dump(w, new AnalyticCirc2dTanCen(a, p)); break; }
                case "Lin2d2Tan.PP": { var a = r.Pnt(); var b = r.Pnt(); double t = r.D(); Dump(w, new AnalyticLin2d2Tan(a, b, t)); break; }
                case "Lin2d2Tan.CP": { var a = r.QC(); var b = r.Pnt(); double t = r.D(); Dump(w, new AnalyticLin2d2Tan(a, b, t)); break; }
                case "Lin2d2Tan.CC": { var a = r.QC(); var b = r.QC(); double t = r.D(); Dump(w, new AnalyticLin2d2Tan(a, b, t)); break; }
                case "TanObl.P": { var a = r.Pnt(); var l = r.Lin(); double g = r.D(); Dump(w, new AnalyticLin2dTanObl(a, l, g)); break; }
                case "TanObl.C": { var a = r.QC(); var l = r.Lin(); double g = r.D(); Dump(w, new AnalyticLin2dTanObl(a, l, g)); break; }
                case "TanPar.P": { var a = r.Pnt(); var l = r.Lin(); Dump(w, new Lin2dTanPar(a, l)); break; }
                case "TanPar.C": { var a = r.QC(); var l = r.Lin(); Dump(w, new Lin2dTanPar(a, l)); break; }
                case "TanPer.Pl": { var a = r.Pnt(); var l = r.Lin(); Dump(w, new Lin2dTanPer(a, l)); break; }
                case "TanPer.Pc": { var a = r.Pnt(); var c = r.Circ(); Dump(w, new Lin2dTanPer(a, c)); break; }
                case "TanPer.Cl": { var a = r.QC(); var l = r.Lin(); Dump(w, new Lin2dTanPer(a, l)); break; }
                case "TanPer.Cc": { var a = r.QC(); var c = r.Circ(); Dump(w, new Lin2dTanPer(a, c)); break; }
                case "Bisec.PP":
                    {
                        var a = r.Pnt();
                        var b = r.Pnt();
                        Pnt2dBisec S = new(a, b);
                        w.Put("done:" + B(S.IsDone));
                        w.Put("has:" + B(S.HasSolution));
                        w.Tok("lin", () => Hex(S.ThisSolution()));
                        break;
                    }
                case "Bisec.LP":
                    {
                        var a = r.Lin();
                        var b = r.Pnt();
                        LinPnt2dBisec S = new(a, b);
                        w.Put("done:" + B(S.IsDone));
                        w.Tok("bis", () => Bisec(S.ThisSolution()));
                        break;
                    }
                case "Bisec.LL":
                    {
                        var a = r.Lin();
                        var b = r.Lin();
                        Lin2dBisec S = new(a, b);
                        w.Solutions(S.IsDone, () => S.SolutionCount, i =>
                        {
                            w.Tok("sol", () => Hex(S.ThisSolution(i)));
                            w.Tok("i1", () => Tan((out double x, out double y, out Point2D p) => S.Intersection1(i, out x, out y, out p)));
                            w.Tok("i2", () => Tan((out double x, out double y, out Point2D p) => S.Intersection2(i, out x, out y, out p)));
                        });
                        break;
                    }
                case "Bisec.CL": { var a = r.Circ(); var b = r.Lin(); CircLin2dBisec S = new(a, b); DumpBisec(w, S.IsDone, () => S.SolutionCount, S.ThisSolution); break; }
                case "Bisec.CP": { var a = r.Circ(); var b = r.Pnt(); CircPnt2dBisec S = new(a, b); DumpBisec(w, S.IsDone, () => S.SolutionCount, S.ThisSolution); break; }
                case "Bisec.CPt": { var a = r.Circ(); var b = r.Pnt(); double t = r.D(); CircPnt2dBisec S = new(a, b, t); DumpBisec(w, S.IsDone, () => S.SolutionCount, S.ThisSolution); break; }
                case "Bisec.CC": { var a = r.Circ(); var b = r.Circ(); Circ2dBisec S = new(a, b); DumpBisec(w, S.IsDone, () => S.SolutionCount, S.ThisSolution); break; }
                default:
                    throw new InvalidOperationException(theCase.Ctor);
            }
        }
        catch (Exception e) when (e is not InvalidOperationException)
        {
            w.Put("ctor:!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
        }
        return w.Tokens.ToArray();
    }
}
