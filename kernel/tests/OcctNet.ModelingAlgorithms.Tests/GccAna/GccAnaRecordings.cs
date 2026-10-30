// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Text;

namespace OcctNet.ModelingAlgorithms.Tests;

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

        public gp_Pnt2d Pnt()
        {
            double x = D();
            double y = D();
            return new gp_Pnt2d(x, y);
        }

        public gp_Dir2d Dir()
        {
            double x = D();
            double y = D();
            return new gp_Dir2d(x, y);
        }

        public gp_Lin2d Lin()
        {
            gp_Pnt2d p = Pnt();
            gp_Dir2d d = Dir();
            return new gp_Lin2d(p, d);
        }

        public gp_Circ2d Circ()
        {
            gp_Pnt2d p = Pnt();
            gp_Dir2d d = Dir();
            double r = D();
            bool s = D() != 0.0;
            return new gp_Circ2d(new gp_Ax2d(p, d), r, s);
        }

        public GccEnt_QualifiedCirc QC()
        {
            gp_Circ2d c = Circ();
            return new GccEnt_QualifiedCirc(c, (GccEnt_Position)(int)D());
        }

        public GccEnt_QualifiedLin QL()
        {
            gp_Lin2d l = Lin();
            return new GccEnt_QualifiedLin(l, (GccEnt_Position)(int)D());
        }
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x", CultureInfo.InvariantCulture);

    private static string Hex(gp_Pnt2d theP) => Hex(theP.X()) + "," + Hex(theP.Y());

    private static string Hex(gp_Dir2d theD) => Hex(theD.X()) + "," + Hex(theD.Y());

    private static string Hex(gp_Ax22d theA) => Hex(theA.Location()) + "," + Hex(theA.XDirection()) + "," + Hex(theA.YDirection());

    private static string Hex(gp_Circ2d theC) => Hex(theC.Position()) + "," + Hex(theC.Radius());

    private static string Hex(gp_Lin2d theL) => Hex(theL.Location()) + "," + Hex(theL.Direction());

    private static string Q(GccEnt_Position p) => ((int)p).ToString(CultureInfo.InvariantCulture);

    private delegate void TanFn(out double a, out double b, out gp_Pnt2d p);

    private static string Tan(TanFn theF)
    {
        theF(out double a, out double b, out gp_Pnt2d p);
        return Hex(a) + "," + Hex(b) + "," + Hex(p);
    }

    private static string Bisec(GccInt_Bisec? theB)
    {
        if (theB is null)
        {
            return "null";
        }
        switch (theB.ArcType())
        {
            case GccInt_IType.GccInt_Lin:
                return "Lin," + Hex(theB.Line());
            case GccInt_IType.GccInt_Cir:
                return "Cir," + Hex(theB.Circle());
            case GccInt_IType.GccInt_Ell:
                {
                    gp_Elips2d anE = theB.Ellipse();
                    return "Ell," + Hex(anE.Axis()) + "," + Hex(anE.MajorRadius()) + "," + Hex(anE.MinorRadius());
                }
            case GccInt_IType.GccInt_Hpr:
                {
                    gp_Hypr2d aH = theB.Hyperbola();
                    return "Hpr," + Hex(aH.Axis()) + "," + Hex(aH.MajorRadius()) + "," + Hex(aH.MinorRadius());
                }
            case GccInt_IType.GccInt_Par:
                {
                    gp_Parab2d aP = theB.Parabola();
                    return "Par," + Hex(aP.Axis()) + "," + Hex(aP.Focal());
                }
            case GccInt_IType.GccInt_Pnt:
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
                aVal = "!" + e.GetType().Name;
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
                Put("nb:!" + e.GetType().Name);
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

    private static void Dump(Writer w, GccAna_Circ2d3Tan S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a, out GccEnt_Position b, out GccEnt_Position c);
                return Q(a) + "," + Q(b) + "," + Q(c);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
            w.Tok("t2", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency2(i, out a, out b, out p)));
            w.Tok("s2", () => B(S.IsTheSame2(i)));
            w.Tok("t3", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency3(i, out a, out b, out p)));
            w.Tok("s3", () => B(S.IsTheSame3(i)));
        });

    private static void Dump(Writer w, GccAna_Circ2d2TanOn S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a, out GccEnt_Position b);
                return Q(a) + "," + Q(b);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
            w.Tok("t2", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency2(i, out a, out b, out p)));
            w.Tok("s2", () => B(S.IsTheSame2(i)));
            w.Tok("c3", () =>
            {
                S.CenterOn3(i, out double a, out gp_Pnt2d p);
                return Hex(a) + "," + Hex(p);
            });
        });

    private static void Dump(Writer w, GccAna_Circ2d2TanRad S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a, out GccEnt_Position b);
                return Q(a) + "," + Q(b);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
            w.Tok("t2", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency2(i, out a, out b, out p)));
            w.Tok("s2", () => B(S.IsTheSame2(i)));
        });

    private static void Dump(Writer w, GccAna_Circ2dTanOnRad S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
            w.Tok("c3", () =>
            {
                S.CenterOn3(i, out double a, out gp_Pnt2d p);
                return Hex(a) + "," + Hex(p);
            });
        });

    private static void Dump(Writer w, GccAna_Circ2dTanCen S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("s1", () => B(S.IsTheSame1(i)));
        });

    private static void Dump(Writer w, GccAna_Lin2d2Tan S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a, out GccEnt_Position b);
                return Q(a) + "," + Q(b);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("t2", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency2(i, out a, out b, out p)));
        });

    private static void Dump(Writer w, GccAna_Lin2dTanObl S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("i2", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Intersection2(i, out a, out b, out p)));
        });

    private static void Dump(Writer w, GccAna_Lin2dTanPar S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
        });

    private static void Dump(Writer w, GccAna_Lin2dTanPer S)
        => w.Solutions(S.IsDone(), S.NbSolutions, i =>
        {
            w.Tok("sol", () => Hex(S.ThisSolution(i)));
            w.Tok("q", () =>
            {
                S.WhichQualifier(i, out GccEnt_Position a);
                return Q(a);
            });
            w.Tok("t1", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Tangency1(i, out a, out b, out p)));
            w.Tok("i2", () => Tan((out double a, out double b, out gp_Pnt2d p) => S.Intersection2(i, out a, out b, out p)));
        });

    private static void DumpBisec(Writer w, bool theDone, Func<int> theNb, Func<int, GccInt_Bisec?> theSol)
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
                case "3Tan.CCC": { var a = r.QC(); var b = r.QC(); var c = r.QC(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.CCL": { var a = r.QC(); var b = r.QC(); var c = r.QL(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.CLL": { var a = r.QC(); var b = r.QL(); var c = r.QL(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.LLL": { var a = r.QL(); var b = r.QL(); var c = r.QL(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.CCP": { var a = r.QC(); var b = r.QC(); var c = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.CLP": { var a = r.QC(); var b = r.QL(); var c = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.LLP": { var a = r.QL(); var b = r.QL(); var c = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.CPP": { var a = r.QC(); var b = r.Pnt(); var c = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.LPP": { var a = r.QL(); var b = r.Pnt(); var c = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "3Tan.PPP": { var a = r.Pnt(); var b = r.Pnt(); var c = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Circ2d3Tan(a, b, c, t)); break; }
                case "2TanOn.CCl": { var a = r.QC(); var b = r.QC(); var c = r.Lin(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CLl": { var a = r.QC(); var b = r.QL(); var c = r.Lin(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.LLl": { var a = r.QL(); var b = r.QL(); var c = r.Lin(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CPl": { var a = r.QC(); var b = r.Pnt(); var c = r.Lin(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.LPl": { var a = r.QL(); var b = r.Pnt(); var c = r.Lin(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.PPl": { var a = r.Pnt(); var b = r.Pnt(); var c = r.Lin(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CCc": { var a = r.QC(); var b = r.QC(); var c = r.Circ(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CLc": { var a = r.QC(); var b = r.QL(); var c = r.Circ(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.CPc": { var a = r.QC(); var b = r.Pnt(); var c = r.Circ(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.LLc": { var a = r.QL(); var b = r.QL(); var c = r.Circ(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.LPc": { var a = r.QL(); var b = r.Pnt(); var c = r.Circ(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanOn.PPc": { var a = r.Pnt(); var b = r.Pnt(); var c = r.Circ(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanOn(a, b, c, t)); break; }
                case "2TanRad.CC": { var a = r.QC(); var b = r.QC(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.CL": { var a = r.QC(); var b = r.QL(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.CP": { var a = r.QC(); var b = r.Pnt(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.LP": { var a = r.QL(); var b = r.Pnt(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.LL": { var a = r.QL(); var b = r.QL(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanRad(a, b, R, t)); break; }
                case "2TanRad.PP": { var a = r.Pnt(); var b = r.Pnt(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2d2TanRad(a, b, R, t)); break; }
                case "TanOnRad.Cl": { var a = r.QC(); var c = r.Lin(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Ll": { var a = r.QL(); var c = r.Lin(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Pl": { var a = r.Pnt(); var c = r.Lin(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Cc": { var a = r.QC(); var c = r.Circ(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Lc": { var a = r.QL(); var c = r.Circ(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2dTanOnRad(a, c, R, t)); break; }
                case "TanOnRad.Pc": { var a = r.Pnt(); var c = r.Circ(); double R = r.D(); double t = r.D(); Dump(w, new GccAna_Circ2dTanOnRad(a, c, R, t)); break; }
                case "TanCen.C": { var a = r.QC(); var p = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Circ2dTanCen(a, p, t)); break; }
                case "TanCen.L": { var a = r.Lin(); var p = r.Pnt(); Dump(w, new GccAna_Circ2dTanCen(a, p)); break; }
                case "TanCen.P": { var a = r.Pnt(); var p = r.Pnt(); Dump(w, new GccAna_Circ2dTanCen(a, p)); break; }
                case "Lin2d2Tan.PP": { var a = r.Pnt(); var b = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Lin2d2Tan(a, b, t)); break; }
                case "Lin2d2Tan.CP": { var a = r.QC(); var b = r.Pnt(); double t = r.D(); Dump(w, new GccAna_Lin2d2Tan(a, b, t)); break; }
                case "Lin2d2Tan.CC": { var a = r.QC(); var b = r.QC(); double t = r.D(); Dump(w, new GccAna_Lin2d2Tan(a, b, t)); break; }
                case "TanObl.P": { var a = r.Pnt(); var l = r.Lin(); double g = r.D(); Dump(w, new GccAna_Lin2dTanObl(a, l, g)); break; }
                case "TanObl.C": { var a = r.QC(); var l = r.Lin(); double g = r.D(); Dump(w, new GccAna_Lin2dTanObl(a, l, g)); break; }
                case "TanPar.P": { var a = r.Pnt(); var l = r.Lin(); Dump(w, new GccAna_Lin2dTanPar(a, l)); break; }
                case "TanPar.C": { var a = r.QC(); var l = r.Lin(); Dump(w, new GccAna_Lin2dTanPar(a, l)); break; }
                case "TanPer.Pl": { var a = r.Pnt(); var l = r.Lin(); Dump(w, new GccAna_Lin2dTanPer(a, l)); break; }
                case "TanPer.Pc": { var a = r.Pnt(); var c = r.Circ(); Dump(w, new GccAna_Lin2dTanPer(a, c)); break; }
                case "TanPer.Cl": { var a = r.QC(); var l = r.Lin(); Dump(w, new GccAna_Lin2dTanPer(a, l)); break; }
                case "TanPer.Cc": { var a = r.QC(); var c = r.Circ(); Dump(w, new GccAna_Lin2dTanPer(a, c)); break; }
                case "Bisec.PP":
                    {
                        var a = r.Pnt();
                        var b = r.Pnt();
                        GccAna_Pnt2dBisec S = new(a, b);
                        w.Put("done:" + B(S.IsDone()));
                        w.Put("has:" + B(S.HasSolution()));
                        w.Tok("lin", () => Hex(S.ThisSolution()));
                        break;
                    }
                case "Bisec.LP":
                    {
                        var a = r.Lin();
                        var b = r.Pnt();
                        GccAna_LinPnt2dBisec S = new(a, b);
                        w.Put("done:" + B(S.IsDone()));
                        w.Tok("bis", () => Bisec(S.ThisSolution()));
                        break;
                    }
                case "Bisec.LL":
                    {
                        var a = r.Lin();
                        var b = r.Lin();
                        GccAna_Lin2dBisec S = new(a, b);
                        w.Solutions(S.IsDone(), S.NbSolutions, i =>
                        {
                            w.Tok("sol", () => Hex(S.ThisSolution(i)));
                            w.Tok("i1", () => Tan((out double x, out double y, out gp_Pnt2d p) => S.Intersection1(i, out x, out y, out p)));
                            w.Tok("i2", () => Tan((out double x, out double y, out gp_Pnt2d p) => S.Intersection2(i, out x, out y, out p)));
                        });
                        break;
                    }
                case "Bisec.CL": { var a = r.Circ(); var b = r.Lin(); GccAna_CircLin2dBisec S = new(a, b); DumpBisec(w, S.IsDone(), S.NbSolutions, S.ThisSolution); break; }
                case "Bisec.CP": { var a = r.Circ(); var b = r.Pnt(); GccAna_CircPnt2dBisec S = new(a, b); DumpBisec(w, S.IsDone(), S.NbSolutions, S.ThisSolution); break; }
                case "Bisec.CPt": { var a = r.Circ(); var b = r.Pnt(); double t = r.D(); GccAna_CircPnt2dBisec S = new(a, b, t); DumpBisec(w, S.IsDone(), S.NbSolutions, S.ThisSolution); break; }
                case "Bisec.CC": { var a = r.Circ(); var b = r.Circ(); GccAna_Circ2dBisec S = new(a, b); DumpBisec(w, S.IsDone(), S.NbSolutions, S.ThisSolution); break; }
                default:
                    throw new InvalidOperationException(theCase.Ctor);
            }
        }
        catch (Exception e) when (e is not InvalidOperationException)
        {
            w.Put("ctor:!" + e.GetType().Name);
        }
        return w.Tokens.ToArray();
    }
}
