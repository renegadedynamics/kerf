// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Offsets.Bisectors;
using Kerf.Geometry;
using System.Globalization;
using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Native OCCT's bisectors recorded by tools/bisrec (unit 5M): Bisector_Bisec::Perform between lines, circular and
/// elliptic arcs, B-splines and points, and Bisector_Inter between the bisectors of two neighbouring corners of a
/// chain. Each line holds a case's inputs as text (both sides build their geometry from it) and native's results as
/// named tokens, every double as its 64 bits in hex. <see cref="Run"/> replays a case on the port and writes the same
/// tokens, mirroring tools/bisrec/main.cpp.
/// </summary>
internal static class BisectorRecordings
{
    public sealed record Case(string Input, string[] Native);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    public static List<Case> Cases => theCases.Value;

    private static List<Case> Load()
    {
        using Stream aRes = typeof(BisectorRecordings).Assembly.GetManifestResourceStream("Bisector.bisrec.txt.gz")
                            ?? throw new InvalidOperationException("Bisector.bisrec.txt.gz");
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
            string[] aOut = aLine[(aSemi + 1)..].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            aCases.Add(new Case(aLine[..aSemi].Trim(), aOut));
        }
        return aCases;
    }

    //=============================================================================================
    // Parsing the inputs, as bisrec's Reader

    public sealed class Reader(string theText)
    {
        private readonly string[] myWords = theText.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        private int myPos;

        public string Word() => myWords[myPos++];

        public double D() => double.Parse(Word(), CultureInfo.InvariantCulture);

        public Geometry2D Elt()
        {
            string aKind = Word();
            if (aKind == "L")
            {
                double x = D(), y = D(), dx = D(), dy = D(), u1 = D(), u2 = D();
                LineCurve2D aL = new(new Point2D(x, y), new Direction2D(dx, dy));
                return new TrimmedCurve2D(aL, u1, u2);
            }
            if (aKind == "C")
            {
                double cx = D(), cy = D(), r = D(), s = D(), u1 = D(), u2 = D();
                CircleCurve2D aC = new(new Circle2D(new Axis2D(new Point2D(cx, cy), new Direction2D(1.0, 0.0)), r,
                                                     s != 0.0));
                return new TrimmedCurve2D(aC, u1, u2);
            }
            if (aKind == "E")
            {
                double cx = D(), cy = D(), dx = D(), dy = D(), a = D(), b = D(), u1 = D(), u2 = D();
                EllipseCurve2D aE = new(new Ellipse2D(new Axis2D(new Point2D(cx, cy), new Direction2D(dx, dy)), a, b));
                return new TrimmedCurve2D(aE, u1, u2);
            }
            if (aKind == "S")
            {
                NCollection_Array1<Point2D> aPoles = new(1, 4);
                for (int i = 1; i <= 4; i++)
                {
                    double x = D(), y = D();
                    aPoles[i] = new Point2D(x, y);
                }
                NCollection_Array1<double> aKnots = new(1, 2);
                aKnots[1] = 0.0;
                aKnots[2] = 1.0;
                NCollection_Array1<int> aMults = new(1, 2);
                aMults[1] = 4;
                aMults[2] = 4;
                return new BSplineCurve2D(aPoles, aKnots, aMults, 3);
            }
            double px = D(), py = D(); // "P"
            return new CartesianPoint2(px, py);
        }
    }

    //=============================================================================================
    // Results, as bisrec writes them

    public static string Hex(double theValue) =>
        BitConverter.DoubleToInt64Bits(theValue).ToString("x", CultureInfo.InvariantCulture);

    private static string Hex(Point2D theP) => Hex(theP.X()) + "," + Hex(theP.Y());

    private static string Hex(Vector2D theV) => Hex(theV.X()) + "," + Hex(theV.Y());

    private static string ExcName(Exception theE) => theE is FailureException ? "!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theE.GetType()) : "!SEH";

    private static void Tok(List<string> theOut, string theName, Func<string> theF)
    {
        string aVal;
        try
        {
            aVal = theF();
        }
        catch (Exception e)
        {
            aVal = ExcName(e);
        }
        theOut.Add(theName + ":" + aVal);
    }

    private static void Range(double theF, double theL, out double theA, out double theB)
    {
        theA = theF;
        theB = theL;
        if (theA < -1.0e50 && theB > 1.0e50)
        {
            theA = -50.0;
            theB = 50.0;
        }
        else if (theA < -1.0e50)
        {
            theA = theB - 100.0;
        }
        else if (theB > 1.0e50)
        {
            theB = theA + 100.0;
        }
    }

    private static void PutBisec(List<string> theOut, string thePre, TrimmedCurve2D? theT)
    {
        if (theT is null)
        {
            theOut.Add(thePre + "V:null");
            return;
        }
        Curve2D aBasis = theT.ChangeBasisCurve();
        BisectorCurve? aBis = aBasis as BisectorCurve;
        theOut.Add(thePre + "T:" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(aBasis.GetType()));
        if (aBasis is BisecAna anAna)
        {
            Tok(theOut, thePre + "G", () => global::Kerf.Engine.Foundation.OcctTypeNames.Of(anAna.Geom2dCurve().GetType()));
        }
        Tok(theOut, thePre + "F", () => Hex(theT.FirstParameter()));
        Tok(theOut, thePre + "L", () => Hex(theT.LastParameter()));
        if (aBis is null)
        {
            return;
        }
        Tok(theOut, thePre + "BF", () => Hex(aBis.FirstParameter()));
        Tok(theOut, thePre + "BL", () => Hex(aBis.LastParameter()));
        Tok(theOut, thePre + "X", () => (aBis.IsExtendAtStart ? "1" : "0") + (aBis.IsExtendAtEnd ? "1" : "0"));
        Tok(theOut, thePre + "N", () =>
        {
            StringBuilder aS = new(aBis.IntervalCount.ToString(CultureInfo.InvariantCulture));
            for (int i = 1; i <= aBis.IntervalCount; i++)
            {
                aS.Append(',').Append(Hex(aBis.IntervalFirst(i))).Append(',').Append(Hex(aBis.IntervalLast(i)));
            }
            return aS.ToString();
        });
        Range(theT.FirstParameter(), theT.LastParameter(), out double aA, out double aB);
        for (int k = 0; k <= 6; k++)
        {
            double u = aA + k * (aB - aA) / 6.0;
            Tok(theOut, thePre + "P" + k, () => Hex(theT.Value(u)));
            Tok(theOut, thePre + "D" + k, () =>
            {
                theT.D1(u, out Point2D _, out Vector2D aV);
                return Hex(aV);
            });
        }
        Tok(theOut, thePre + "Pm", () => Hex(aBis.Parameter(theT.Value(0.5 * (aA + aB)))));
    }

    /// <summary>Bisector_Bisec::Perform as MAT2d_Tool2d::CreateBisector calls it, by the kinds of the elements.</summary>
    public static void Perform(Bisec theB, Geometry2D theE1, Geometry2D theE2, in Point2D theP,
                               in Vector2D theV1, in Vector2D theV2, double theSense, JoinKind theJoin,
                               double theTol, bool theOnCurve)
    {
        Point2? aP1 = theE1 as Point2;
        Point2? aP2 = theE2 as Point2;
        Curve2D? aC1 = theE1 as Curve2D;
        Curve2D? aC2 = theE2 as Curve2D;
        if (aC1 is not null && aC2 is not null)
        {
            theB.Perform(aC1, aC2, theP, theV1, theV2, theSense, theJoin, theTol, theOnCurve);
        }
        else if (aP1 is not null && aP2 is not null)
        {
            theB.Perform(aP1, aP2, theP, theV1, theV2, theSense, theTol, theOnCurve);
        }
        else if (aP1 is not null)
        {
            theB.Perform(aP1, aC2!, theP, theV1, theV2, theSense, theTol, theOnCurve);
        }
        else
        {
            theB.Perform(aC1!, aP2!, theP, theV1, theV2, theSense, theTol, theOnCurve);
        }
    }

    /// <summary>MAT2d_Tool2d's static Domain.</summary>
    public static TwoDDomain Domain(TrimmedCurve2D Bisector1, double Tolerance)
    {
        double Param1 = Bisector1.FirstParameter();
        double Param2 = Bisector1.LastParameter();
        if (Param2 > 10000.0)
        {
            Param2 = 10000.0;
            Curve2D BasisCurve = Bisector1.ChangeBasisCurve();
            Type Type1 = BasisCurve.GetType();
            if (Type1 == typeof(BisecAna))
            {
                BasisCurve = ((BisecAna)Bisector1.ChangeBasisCurve()).Geom2dCurve();
                Type1 = BasisCurve.GetType();
            }
            double Limit = 50000.0;
            if (Type1 == typeof(ParabolaCurve2D))
            {
                Parabola2D gpParabola = ((ParabolaCurve2D)BasisCurve).Parab2d();
                double Focus = gpParabola.Focal();
                double Val1 = Math.Sqrt(Limit * Focus);
                double Val2 = Math.Sqrt(Limit * Limit);
                Param2 = (Val1 <= Val2 ? Val1 : Val2);
            }
            else if (Type1 == typeof(HyperbolaCurve2D))
            {
                Hyperbola2D gpHyperbola = ((HyperbolaCurve2D)BasisCurve).Hypr2d();
                double Majr = gpHyperbola.MajorRadius();
                double Minr = gpHyperbola.MinorRadius();
                double Valu1 = Limit / Majr;
                double Valu2 = Limit / Minr;
                double Val1 = Math.Log(Valu1 + Math.Sqrt(Valu1 * Valu1 - 1));
                double Val2 = Math.Log(Valu2 + Math.Sqrt(Valu2 * Valu2 + 1));
                Param2 = (Val1 <= Val2 ? Val1 : Val2);
            }
        }
        TwoDDomain Domain1 = new(Bisector1.Value(Param1), Param1, Tolerance, Bisector1.Value(Param2), Param2,
                                      Tolerance);
        if (Bisector1.ChangeBasisCurve().IsPeriodic)
        {
            Domain1 = Domain1.WithEquivalentParameters(0.0, 2.0 * Math.PI);
        }
        return Domain1;
    }

    private static void PutInter(List<string> theOut, string thePre, Inter theI)
    {
        Tok(theOut, thePre + "D", () => theI.IsDone ? "1" : "0");
        if (!theI.IsDone)
        {
            return;
        }
        Tok(theOut, thePre + "NP", () => theI.PointCount.ToString(CultureInfo.InvariantCulture));
        for (int i = 1; i <= theI.PointCount; i++)
        {
            int j = i;
            Tok(theOut, thePre + "p" + i, () =>
            {
                IntersectionPoint2D aP = theI.Point(j);
                return Hex(aP.Value()) + "," + Hex(aP.ParamOnFirst()) + "," + Hex(aP.ParamOnSecond());
            });
        }
        Tok(theOut, thePre + "NS", () => theI.SegmentCount.ToString(CultureInfo.InvariantCulture));
    }

    /// <summary>The port's tokens for the case theInput.</summary>
    public static List<string> Run(string theInput)
    {
        List<string> aOut = [];
        Reader aR = new(theInput);
        string anOp = aR.Word();
        try
        {
            if (anOp == "B")
            {
                Geometry2D aE1 = aR.Elt();
                Geometry2D aE2 = aR.Elt();
                double px = aR.D(), py = aR.D(), v1x = aR.D(), v1y = aR.D(), v2x = aR.D(), v2y = aR.D();
                double aSense = aR.D();
                int aJoin = (int)aR.D();
                double aTol = aR.D();
                bool anOn = aR.D() != 0.0;
                Bisec aB = new();
                try
                {
                    Perform(aB, aE1, aE2, new Point2D(px, py), new Vector2D(v1x, v1y), new Vector2D(v2x, v2y), aSense,
                            (JoinKind)aJoin, aTol, anOn);
                }
                catch (FailureException e)
                {
                    aOut.Add("E:" + ExcName(e));
                    return aOut;
                }
                PutBisec(aOut, "", aB.Value());
            }
            else
            {
                Curve2D aA = (Curve2D)aR.Elt();
                Curve2D aBc = (Curve2D)aR.Elt();
                Curve2D aC = (Curve2D)aR.Elt();
                double aSense = aR.D();
                int aJoin = (int)aR.D();
                double aTol = aR.D();
                Bisec aB1 = new(), aB2 = new();
                try
                {
                    Point2D aP1 = aA.Value(aA.LastParameter());
                    aB1.Perform(aA, aBc, aP1, aA.DN(aA.LastParameter(), 1), aBc.DN(aBc.FirstParameter(), 1).Reversed(),
                                aSense, (JoinKind)aJoin, aTol, true);
                    Point2D aP2 = aBc.Value(aBc.LastParameter());
                    aB2.Perform(aBc, aC, aP2, aBc.DN(aBc.LastParameter(), 1), aC.DN(aC.FirstParameter(), 1).Reversed(),
                                aSense, (JoinKind)aJoin, aTol, true);
                }
                catch (FailureException e)
                {
                    aOut.Add("E:" + ExcName(e));
                    return aOut;
                }
                PutBisec(aOut, "a", aB1.Value());
                PutBisec(aOut, "b", aB2.Value());
                for (int aComun = 1; aComun >= 0; aComun--)
                {
                    string aPre = aComun != 0 ? "i" : "j";
                    try
                    {
                        TwoDDomain aD1 = Domain(aB1.Value(), aTol);
                        TwoDDomain aD2 = Domain(aB2.Value(), aTol);
                        Inter anInter = new();
                        anInter.Perform(aB1, aD1, aB2, aD2, aTol, aTol, aComun != 0);
                        PutInter(aOut, aPre, anInter);
                    }
                    catch (Exception e)
                    {
                        aOut.Add(aPre + "E:" + ExcName(e));
                    }
                }
            }
        }
        catch (Exception e)
        {
            aOut.Add("X:" + ExcName(e));
        }
        return aOut;
    }
}
