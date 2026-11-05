// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Fairing;
using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using System.Globalization;
using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Native OCCT's fair curves recorded by tools/fairrec (unit 5Q): FairCurve_Batten and FairCurve_MinimalVariation
/// through their setters and successive Computes, Draw's battencurve/minvarcurve sequences with its editing commands,
/// and the energies FairCurve_EnergyOfBatten and FairCurve_EnergyOfMVC on random poles. Each case's inputs are
/// round-trip decimals and native's results named tokens, every double as its 64 bits in hex. <see cref="Run"/>
/// replays a case on the port (the same code as fairrec's, reading the inputs in the order fairrec generated them) and
/// writes its results in the same format.
/// </summary>
internal static class FairCurveRecordings
{
    public sealed record Case(string Kind, double[] Inputs, string[] Native);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    public static List<Case> Cases => theCases.Value;

    public static IEnumerable<object[]> Kinds() => Cases.Select(c => c.Kind).Distinct().Select(n => new object[] { n });

    private static List<Case> Load()
    {
        using Stream aRes = typeof(FairCurveRecordings).Assembly.GetManifestResourceStream("FairCurve.fairrec.txt.gz")
                            ?? throw new InvalidOperationException("FairCurve.fairrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        List<Case> aCases = [];
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            if (aLine.Length == 0)
            {
                continue;
            }
            int aSemi = aLine.IndexOf(';');
            string[] aIn = aLine[..aSemi].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            double[] aValues = aIn.Skip(1).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            string[] aOut = aLine[(aSemi + 1)..].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            aCases.Add(new Case(aIn[0], aValues, aOut));
        }
        return aCases;
    }

    //=============================================================================================
    // The port's side: fairrec's code, reading its inputs in the order it generated them.

    private sealed class Reader(double[] theIn)
    {
        private int myPos;

        public double D() => theIn[myPos++];

        public int I() => (int)D();

        public bool B() => D() != 0.0;

        public bool AtEnd => myPos == theIn.Length;

        public Point2D Pnt()
        {
            double x = D();
            double y = D();
            return new Point2D(x, y);
        }

        public double Angle() => D() * M_PI / 180;
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x", CultureInfo.InvariantCulture);

    private static string Hex(Point2D theP) => Hex(theP.X) + "," + Hex(theP.Y);

    private static string Str(int theValue) => theValue.ToString(CultureInfo.InvariantCulture);

    private static string Curve(BSplineCurve2D theC)
    {
        StringBuilder aS = new(Str(theC.Degree) + "," + Str(theC.PoleCount));
        for (int i = 1; i <= theC.PoleCount; i++)
        {
            aS.Append(',').Append(Hex(theC.Pole(i)));
        }
        aS.Append(',').Append(Str(theC.KnotCount));
        for (int i = 1; i <= theC.KnotCount; i++)
        {
            aS.Append(',').Append(Hex(theC.Knot(i))).Append(',').Append(Str(theC.Multiplicity(i)));
        }
        return aS.ToString();
    }

    private static string Vec(math_Vector theV)
    {
        StringBuilder aS = new(Str(theV.Lower()) + "," + Str(theV.Upper()));
        for (int i = theV.Lower(); i <= theV.Upper(); i++)
        {
            aS.Append(',').Append(Hex(theV[i]));
        }
        return aS.ToString();
    }

    private static string Mat(Matrix theM)
    {
        StringBuilder aS = new(Str(theM.RowNumber()));
        for (int i = theM.LowerRow(); i <= theM.UpperRow(); i++)
        {
            for (int j = theM.LowerCol(); j <= theM.UpperCol(); j++)
            {
                aS.Append(',').Append(Hex(theM[i, j]));
            }
        }
        return aS.ToString();
    }

    private sealed class Writer
    {
        public readonly List<string> Tokens = [];

        public void Put(string theTok) => Tokens.Add(theTok);

        /// <summary>name:value, or name:!Exception for a Standard_Failure (fairrec catches only those).</summary>
        public void Tok(string theName, Func<string> theF)
        {
            string aVal;
            try
            {
                aVal = theF();
            }
            catch (Exception anExc) when (Refusal.IsFailure(anExc))
            {
                aVal = "!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anExc).GetType());
            }
            Put(theName + ":" + aVal);
        }
    }

    private static void State(Writer w, Batten theB, string p, MinimalVariation? theMV)
    {
        w.Tok(p + "a1", () => Hex(theB.GetAngle1()));
        w.Tok(p + "a2", () => Hex(theB.GetAngle2()));
        w.Tok(p + "sf", () => Hex(theB.GetSlidingFactor()));
        w.Tok(p + "p1", () => Hex(theB.GetP1()));
        w.Tok(p + "p2", () => Hex(theB.GetP2()));
        w.Tok(p + "h", () => Hex(theB.GetHeight()));
        w.Tok(p + "sl", () => Hex(theB.GetSlope()));
        w.Tok(p + "o", () => Str(theB.GetConstraintOrder1()) + "," + Str(theB.GetConstraintOrder2()) + ","
                             + Str(theB.GetFreeSliding() ? 1 : 0));
        if (theMV is not null)
        {
            w.Tok(p + "c", () => Hex(theMV.GetCurvature1()) + "," + Hex(theMV.GetCurvature2()) + "," + Hex(theMV.GetPhysicalRatio()));
        }
        w.Tok(p + "ref", () => Hex(theB.SlidingOfReference()));
        w.Tok(p + "curve", () => Curve(theB.Curve()));
    }

    private static void Compute(Writer w, Batten theB, string thePrefix, int theNbIter, double theTol)
    {
        w.Tok(thePrefix, () =>
        {
            bool anOk = theB.Compute(out AnalysisCode aCode, theNbIter, theTol);
            return Str(anOk ? 1 : 0) + "," + Str((int)aCode);
        });
    }

    private static void RunFair(Reader r, Writer w, bool theMV)
    {
        Point2D P1 = r.Pnt(), P2 = r.Pnt();
        double h = r.D(), s = r.D();
        double ratio = theMV ? r.D() : 0.0;

        Batten aB;
        MinimalVariation? aMV = null;
        try
        {
            if (theMV)
            {
                aMV = new MinimalVariation(P1, P2, h, s, ratio);
                aB = aMV;
            }
            else
            {
                aB = new Batten(P1, P2, h, s);
            }
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            w.Put("ctor:!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anExc).GetType()));
            return;
        }

        double a1 = r.Angle(), a2 = r.Angle();
        int o1 = r.I(), o2 = r.I();
        bool fs = r.B();
        double f = r.D();
        aB.SetAngle1(a1);
        aB.SetAngle2(a2);
        aB.SetConstraintOrder1(o1);
        aB.SetConstraintOrder2(o2);
        aB.SetFreeSliding(fs);
        aB.SetSlidingFactor(f);
        if (aMV is not null)
        {
            double c1 = r.D(), c2 = r.D();
            aMV.SetCurvature1(c1);
            aMV.SetCurvature2(c2);
        }
        w.Tok("ref0", () => Hex(aB.SlidingOfReference()));
        int n = r.I();
        double t = r.D();
        Compute(w, aB, "c1", n, t);
        State(w, aB, "", aMV);

        if (r.B())
        {
            Point2D Q2 = r.Pnt();
            double b1 = r.Angle();
            double h2 = r.D();
            double s2 = r.D();
            w.Tok("setp2", () =>
            {
                aB.SetP2(Q2);
                return "1";
            });
            aB.SetAngle1(b1);
            w.Tok("seth", () =>
            {
                aB.SetHeight(h2);
                return "1";
            });
            aB.SetSlope(s2);
            if (aMV is not null)
            {
                double r2 = r.D();
                w.Tok("setr", () =>
                {
                    aMV.SetPhysicalRatio(r2);
                    return "1";
                });
            }
            Compute(w, aB, "c2", 50, 1.0e-3);
            State(w, aB, "2", aMV);
        }
    }

    private static void RunDraw(Reader r, Writer w, bool theMV)
    {
        Point2D P1 = r.Pnt(), P2 = r.Pnt();
        double a1 = r.D(), a2 = r.D();
        double h = r.D();

        Batten aB;
        MinimalVariation? aMV = null;
        try
        {
            if (theMV)
            {
                aMV = new MinimalVariation(P1, P2, h);
                aB = aMV;
            }
            else
            {
                aB = new Batten(P1, P2, h);
            }
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            w.Put("ctor:!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anExc).GetType()));
            return;
        }
        aB.SetAngle1(a1 * M_PI / 180);
        aB.SetAngle2(a2 * M_PI / 180);
        Compute(w, aB, "c0", 50, 1.0e-3);
        Compute(w, aB, "c1", 50, 1.0e-2);
        State(w, aB, "", aMV);

        int aNbOps = r.I();
        for (int k = 0; k < aNbOps; k++)
        {
            int anOp = r.I();
            int aSide = r.I();
            double aValue = r.D();
            string aName = "op" + Str(k);
            w.Tok(aName, () =>
            {
                switch (anOp)
                {
                    case 0: // setangle side angle
                        if (aSide == 1)
                        {
                            aB.SetAngle1(aValue * M_PI / 180);
                            if (aB.GetConstraintOrder1() == 0)
                            {
                                aB.SetConstraintOrder1(1);
                            }
                        }
                        else
                        {
                            aB.SetAngle2(aValue * M_PI / 180);
                            if (aB.GetConstraintOrder2() == 0)
                            {
                                aB.SetConstraintOrder2(1);
                            }
                        }
                        break;
                    case 1: // setslide
                        aB.SetFreeSliding(false);
                        aB.SetSlidingFactor(Math.Abs(aValue) / 30.0);
                        break;
                    case 2: // freeslide
                        aB.SetFreeSliding(true);
                        break;
                    case 3: // freeangle
                        if (aSide == 1)
                        {
                            aB.SetConstraintOrder1(0);
                        }
                        else
                        {
                            aB.SetConstraintOrder2(0);
                        }
                        break;
                    case 4: // setheight
                        aB.SetHeight(aValue / 60.0);
                        break;
                    case 5: // setslope
                        aB.SetSlope(aValue / 60.0);
                        break;
                    case 6: // setcurvature
                        if (aSide == 1)
                        {
                            aMV!.SetCurvature1(aValue / 60.0);
                            aMV.SetConstraintOrder1(2);
                        }
                        else
                        {
                            aMV!.SetCurvature2(aValue / 60.0);
                            aMV.SetConstraintOrder2(2);
                        }
                        break;
                    case 7: // freecurvature
                        if (aSide == 1)
                        {
                            if (aMV!.GetConstraintOrder1() > 1)
                            {
                                aMV.SetConstraintOrder1(1);
                            }
                        }
                        else
                        {
                            if (aMV!.GetConstraintOrder2() > 1)
                            {
                                aMV.SetConstraintOrder2(1);
                            }
                        }
                        break;
                    case 8: // setphysicalratio
                        aMV!.SetPhysicalRatio(Math.Abs(aValue) / 40.0);
                        break;
                }
                bool anOk = aB.Compute(out AnalysisCode aCode, 50, 1.0e-2);
                return Str(anOk ? 1 : 0) + "," + Str((int)aCode);
            });
            State(w, aB, aName + ".", aMV);
        }
    }

    private static void RunEnergy(Reader r, Writer w, bool theMVC)
    {
        const int Degree = 9;
        int aNbKnots = r.I();
        NCollection_Array1<double> aKnots = new(1, aNbKnots);
        NCollection_Array1<int> aMults = new(1, aNbKnots);
        for (int i = 1; i <= aNbKnots; i++)
        {
            aKnots[i] = (double)(i - 1) / (aNbKnots - 1);
            aMults[i] = (i == 1 || i == aNbKnots) ? Degree + 1 : Degree - 3;
        }
        NCollection_HArray1<double> aFlat = new(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(aMults, Degree, false));
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(aKnots, aMults, Degree, false, aFlat.ChangeArray1());
        int aNbPoles = aFlat.Length() - Degree - 1;
        NCollection_HArray1<Point2D> aPoles = new(1, aNbPoles);
        double aLength = r.D();
        double aBend = r.D();
        for (int i = 1; i <= aNbPoles; i++)
        {
            double x = r.D();
            double y = r.D();
            aPoles.SetValue(i, new Point2D(x, y));
        }
        int o1 = r.I(), o2 = r.I();
        bool fs = r.B();
        double a1 = r.Angle(), a2 = r.Angle();
        double h = r.D(), s = r.D();
        double aSliding = r.D();
        BattenLaw aLaw = new(h, s, aSliding);

        Energy anE;
        try
        {
            if (theMVC)
            {
                double ratio = r.D();
                double c1 = r.D(), c2 = r.D();
                anE = new EnergyOfMvc(Degree + 1, aFlat, aPoles, o1, o2, aLaw, ratio, aSliding, fs, a1, a2, c1, c2);
            }
            else
            {
                anE = new EnergyOfBatten(Degree + 1, aFlat, aPoles, o1, o2, aLaw, aSliding, fs, a1, a2);
            }
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            w.Put("ctor:!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anExc).GetType()));
            return;
        }
        int n = anE.VariableCount;
        math_Vector X = new(1, n), G = new(1, n);
        Matrix H = new(1, n, 1, n);
        w.Tok("var", () =>
        {
            bool anOk = anE.Variable(X);
            return Str(anOk ? 1 : 0) + "," + Vec(X);
        });
        for (int k = 0; k < 2; k++)
        {
            string p = Str(k);
            w.Tok("v" + p, () =>
            {
                bool anOk = anE.Value(X, out double E);
                return Str(anOk ? 1 : 0) + "," + Hex(E);
            });
            w.Tok("g" + p, () =>
            {
                bool anOk = anE.Values(X, out double E, G);
                return Str(anOk ? 1 : 0) + "," + Hex(E) + "," + Vec(G);
            });
            w.Tok("h" + p, () =>
            {
                bool anOk = anE.Values(X, out double E, G, H);
                return Str(anOk ? 1 : 0) + "," + Hex(E) + "," + Vec(G) + "," + Mat(H);
            });
            w.Tok("p" + p, () =>
            {
                StringBuilder aS = new();
                for (int i = 1; i <= aNbPoles; i++)
                {
                    aS.Append(i > 1 ? "," : "").Append(Hex(anE.Poles().Value(i)));
                }
                return aS.ToString();
            });
            for (int i = 1; i <= n; i++)
            {
                X[i] += r.D();
            }
        }
    }

    /// <summary>Replays a case on the port: its tokens, in fairrec's format.</summary>
    public static string[] Run(Case theCase)
    {
        Reader r = new(theCase.Inputs);
        Writer w = new();
        try
        {
            switch (theCase.Kind)
            {
                case "Batten":
                    RunFair(r, w, false);
                    break;
                case "MinVar":
                    RunFair(r, w, true);
                    break;
                case "DrawBatten":
                    RunDraw(r, w, false);
                    break;
                case "DrawMinVar":
                    RunDraw(r, w, true);
                    break;
                case "EBatten":
                    RunEnergy(r, w, false);
                    break;
                case "EMVC":
                    RunEnergy(r, w, true);
                    break;
                default:
                    throw new InvalidOperationException(theCase.Kind);
            }
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            w.Put("fail:!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anExc).GetType()));
        }
        if (!r.AtEnd)
        {
            throw new InvalidOperationException($"{theCase.Kind}: the replay did not read every input");
        }
        return [.. w.Tokens];
    }
}
