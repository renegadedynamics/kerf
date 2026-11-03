// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Helix;
using Kerf.Engine.Construction.Helix.Geometry;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using Xunit.Abstractions;
using static Kerf.Geometry.Continuity;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// TKHelix against native OCCT's results recorded once by tools/helixrec (unit 7D; build123d.NET doesn't bind TKHelix):
/// HelixBRep_BuilderHelix through each SetParameters overload the helix commands use, on the parameters of every
/// command the corpus's helix grid calls (with its setaxis axes), with the commands' approximation parameters and
/// with 1e-5, degree 8, C2; and per part HelixGeom_BuilderHelix, HelixGeom_BuilderHelixCoil and
/// HelixGeom_Tools::ApprHelix. The test writes the records the recorder writes, from the recorded inputs' bits, and
/// compares them byte for byte: statuses, tolerances reached (as bits), the wires' BRep text (format 3; in full up to
/// 250 lines, by its FNV-1a hash beyond), and the curves' degrees, poles, knots and multiplicities (as bits).
/// </summary>
public class HelixNativeTests(ITestOutputHelper theOutput)
{
    private sealed record Case(string Id, string Command, string Axis, string In, List<string> Native);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    private static List<Case> Load()
    {
        using Stream aRes = typeof(HelixNativeTests).Assembly.GetManifestResourceStream("HelixBRep.helixrec.txt.gz")
                            ?? throw new InvalidOperationException("HelixBRep.helixrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        List<Case> aCases = [];
        List<string>? aLines = null;
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            if (aLine.StartsWith("CASE ", StringComparison.Ordinal))
            {
                string[] aWords = aLine.Split(' ');
                string anAxis = aReader.ReadLine()!;
                string anIn = aReader.ReadLine()!;
                aLines = [aLine, anAxis, anIn];
                aCases.Add(new Case(aWords[1], aWords[2], anAxis, anIn, aLines));
            }
            else
            {
                aLines!.Add(aLine);
            }
        }
        return aCases;
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture);

    private static double FromHex(string theHex) =>
        BitConverter.Int64BitsToDouble(unchecked((long)ulong.Parse(theHex, NumberStyles.HexNumber, CultureInfo.InvariantCulture)));

    private static string Fnv(string theText)
    {
        ulong aH = 14695981039346656037ul;
        foreach (byte c in Encoding.ASCII.GetBytes(theText))
        {
            aH = unchecked((aH ^ c) * 1099511628211ul);
        }
        return aH.ToString("x16", CultureInfo.InvariantCulture);
    }

    // ---- the port's side: helixrec's records ----

    private static string CurveText(Curve3D theC)
    {
        if (theC is not BSplineCurve3D aB)
        {
            return "none";
        }
        var aS = new StringBuilder();
        aS.Append(CultureInfo.InvariantCulture, $"{aB.Degree()} {aB.PoleCount} {aB.KnotCount}");
        for (int i = 1; i <= aB.PoleCount; ++i)
        {
            Point3D aP = aB.Pole(i);
            aS.Append(' ').Append(Hex(aP.X())).Append(' ').Append(Hex(aP.Y())).Append(' ').Append(Hex(aP.Z()));
        }
        for (int i = 1; i <= aB.KnotCount; ++i)
        {
            aS.Append(' ').Append(Hex(aB.Knot(i)));
        }
        for (int i = 1; i <= aB.KnotCount; ++i)
        {
            aS.Append(' ').Append(aB.Multiplicity(i).ToString(CultureInfo.InvariantCulture));
        }
        return aS.ToString();
    }

    private static void Curve(List<string> theOut, Curve3D theC, int theK, int theN)
    {
        string aText = CurveText(theC);
        theOut.Add(theK <= 2 || theK == theN ? "CURVE " + aText : "CURVEH " + Fnv(aText));
    }

    private static NCollection_Array1<double> Array(double[] theV, int theFrom, int theNb)
    {
        NCollection_Array1<double> anA = new(1, theNb);
        for (int i = 1; i <= theNb; ++i)
        {
            anA[i] = theV[theFrom + i - 1];
        }
        return anA;
    }

    private static NCollection_Array1<bool> Flags(double[] theV, int theFrom, int theNb)
    {
        NCollection_Array1<bool> anA = new(1, theNb);
        for (int i = 1; i <= theNb; ++i)
        {
            anA[i] = (int)theV[theFrom + i - 1] != 0;
        }
        return anA;
    }

    private sealed class Parts
    {
        public readonly List<double> D = [], H = [], P = [];
    }

    private static void SetParameters(BuilderHelix theB, string theCmd, in CoordinateSystem theAxis, int theNb, double[] theV,
                                      Parts theParts)
    {
        if (theCmd == "comphelix")
        {
            NCollection_Array1<double> aD = Array(theV, 0, theNb + 1), aH = Array(theV, theNb + 1, theNb),
                                       aP = Array(theV, 2 * theNb + 1, theNb);
            NCollection_Array1<bool> aF = Flags(theV, 3 * theNb + 1, theNb);
            theParts.D.AddRange(theV.Take(theNb + 1));
            for (int i = 1; i <= theNb; ++i)
            {
                theParts.H.Add(aH[i]);
                theParts.P.Add(aF[i] ? aP[i] : aH[i] / aP[i]);
            }
            theB.SetParameters(theAxis, aD, aH, aP, aF);
        }
        else if (theCmd is "helix" or "spiral")
        {
            int aNbD = theCmd == "helix" ? 1 : 2;
            NCollection_Array1<double> aH = Array(theV, aNbD, theNb), aP = Array(theV, aNbD + theNb, theNb);
            NCollection_Array1<bool> aF = Flags(theV, aNbD + 2 * theNb, theNb);
            theParts.D.AddRange(theV.Take(aNbD));
            for (int i = 1; i <= theNb; ++i)
            {
                theParts.H.Add(aH[i]);
                theParts.P.Add(aF[i] ? aP[i] : aH[i] / aP[i]);
            }
            if (aNbD == 1)
            {
                theB.SetParameters(theAxis, theV[0], aH, aP, aF);
            }
            else
            {
                theB.SetParameters(theAxis, theV[0], theV[1], aH, aP, aF);
            }
        }
        else
        {
            int aNbD = theCmd == "comphelix2" ? theNb + 1 : theCmd == "helix2" ? 1 : 2;
            NCollection_Array1<double> aP = Array(theV, aNbD, theNb), aN = Array(theV, aNbD + theNb, theNb);
            theParts.D.AddRange(theV.Take(aNbD));
            for (int i = 1; i <= theNb; ++i)
            {
                theParts.H.Add(aP[i] * aN[i]);
                theParts.P.Add(aP[i]);
            }
            if (theCmd == "comphelix2")
            {
                theB.SetParameters(theAxis, Array(theV, 0, theNb + 1), aP, aN);
            }
            else if (theCmd == "helix2")
            {
                theB.SetParameters(theAxis, theV[0], aP, aN);
            }
            else
            {
                theB.SetParameters(theAxis, theV[0], theV[1], aP, aN);
            }
        }
    }

    private const int MaxLines = 250;

    private static void RecordBRep(List<string> theOut, int theSet, string theCmd, in CoordinateSystem theAxis, int theNb, double[] theV,
                                   Parts theParts)
    {
        try
        {
            var aB = new BuilderHelix();
            SetParameters(aB, theCmd, theAxis, theNb, theV, theParts);
            if (theSet == 1)
            {
                aB.SetApproxParameters(1.0e-5, 8, C2);
            }
            aB.Perform();
            string aText = "";
            if (aB.ErrorStatus() == 0)
            {
                var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
                BrepTools.Write(aB.Shape(), aS, true, false, FormatVersion.FormatVersion_VERSION_3);
                aText = aS.ToString();
            }
            int aNb = aText.Count(c => c == '\n') + (aText.Length > 0 && aText[^1] != '\n' ? 1 : 0);
            theOut.Add($"BREP {theSet} {aB.ErrorStatus()} {aB.WarningStatus()} {Hex(aB.ToleranceReached())} {aNb} {Fnv(aText)}");
            if (aNb <= MaxLines && aText.Length > 0)
            {
                theOut.AddRange((aText[^1] == '\n' ? aText[..^1] : aText).Split('\n'));
            }
        }
        catch (FailureException e)
        {
            theOut.Add("THROW " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
        }
    }

    private static void RecordGeom(List<string> theOut, in CoordinateSystem theAxis, Parts theParts)
    {
        double aTwoPI = 2.0 * Math.PI;
        for (int i = 0; i < theParts.H.Count; ++i)
        {
            int aLast = theParts.D.Count - 1;
            double aD1 = theParts.D[i < aLast ? i : aLast], aD2 = theParts.D[i + 1 < aLast ? i + 1 : aLast];
            double aT2 = aTwoPI * theParts.H[i] / theParts.P[i];
            double aR = 0.5 * aD1, aTaper = Math.Atan((aD2 - aD1) / (2.0 * theParts.H[i]));
            bool aCW = i % 2 == 0;
            theOut.Add($"GEOM {i + 1} {Hex(0.0)} {Hex(aT2)} {Hex(theParts.P[i])} {Hex(aR)} {Hex(aTaper)} {(aCW ? 1 : 0)}");
            try
            {
                var aBH = new GeometryBuilderHelix();
                aBH.SetPosition(theAxis.Ax2());
                aBH.SetCurveParameters(0.0, aT2, theParts.P[i], aR, aTaper, aCW);
                aBH.SetTolerance(1.0e-4);
                aBH.SetApproxParameters(C1, 8, 1000);
                aBH.Perform();
                var aCurves = aBH.Curves();
                theOut.Add($"HELIX {aBH.ErrorStatus()} {Hex(aBH.ToleranceReached())} {aCurves.Count}");
                for (int k = 1; k <= aCurves.Count; ++k)
                {
                    Curve(theOut, aCurves[k - 1], k, aCurves.Count);
                }
            }
            catch (FailureException e)
            {
                theOut.Add("THROW " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
            }
            double aT2c = aT2 < aTwoPI ? aT2 : aTwoPI;
            try
            {
                var aBC = new BuilderHelixCoil();
                aBC.SetCurveParameters(0.0, aT2c, theParts.P[i], aR, aTaper, aCW);
                aBC.Perform();
                var aCurves = aBC.Curves();
                theOut.Add($"COIL {aBC.ErrorStatus()} {Hex(aBC.ToleranceReached())} {aCurves.Count}");
                for (int k = 1; k <= aCurves.Count; ++k)
                {
                    Curve(theOut, aCurves[k - 1], k, aCurves.Count);
                }
            }
            catch (FailureException e)
            {
                theOut.Add("THROW " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
            }
            try
            {
                BSplineCurve3D? aBS = null;
                double aMaxErr = -1.0;
                int iErr = GeometryTools.ApprHelix(0.0, aT2c, theParts.P[i], aR, aTaper, aCW, 1.0e-4, ref aBS, ref aMaxErr);
                theOut.Add($"APPR {iErr} {Hex(aMaxErr)} {(aBS is null ? 0 : 1)}");
                if (aBS is not null)
                {
                    Curve(theOut, aBS, 1, 1);
                }
            }
            catch (FailureException e)
            {
                theOut.Add("THROW " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
            }
        }
    }

    private static List<string> Port(Case theCase)
    {
        List<string> anOut = [$"CASE {theCase.Id} {theCase.Command}", theCase.Axis, theCase.In];
        // the axis: OCCT's default, then the case's setaxis replayed as the command does
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        double[] aSet = theCase.Axis.Split(' ', StringSplitOptions.RemoveEmptyEntries).Skip(1).Select(FromHex).ToArray();
        if (aSet.Length == 9)
        {
            anAxis.SetLocation(new Point3D(aSet[0], aSet[1], aSet[2]));
            anAxis.SetDirection(new Direction3D(aSet[3], aSet[4], aSet[5]));
            anAxis.SetXDirection(new Direction3D(aSet[6], aSet[7], aSet[8]));
        }
        string[] anIn = theCase.In.Split(' ');
        int aNb = int.Parse(anIn[1], CultureInfo.InvariantCulture);
        double[] aV = anIn.Skip(2).Select(FromHex).ToArray();
        var aParts = new Parts();
        RecordBRep(anOut, 0, theCase.Command, anAxis, aNb, aV, aParts);
        RecordBRep(anOut, 1, theCase.Command, anAxis, aNb, aV, new Parts());
        if (aParts.H.Count > 0)
        {
            RecordGeom(anOut, anAxis, aParts);
        }
        return anOut;
    }

    [Fact]
    public void The_recording_covers_the_helix_grid()
    {
        List<Case> aCases = theCases.Value;
        Assert.Equal(56, aCases.Count);
        Assert.Equal(56, aCases.Select(c => c.Id).Distinct().Count());
        Assert.Equal(2, aCases.Count(c => c.Axis != "AXIS"));
        Assert.Equal(["comphelix", "comphelix2", "helix", "helix2", "spiral", "spiral2"],
                     aCases.Select(c => c.Command).Distinct().Order(StringComparer.Ordinal));
        // every builder succeeded natively
        Assert.DoesNotContain(aCases.SelectMany(c => c.Native), l => l.StartsWith("THROW", StringComparison.Ordinal));
        Assert.All(aCases.SelectMany(c => c.Native).Where(l => l.StartsWith("BREP", StringComparison.Ordinal)),
                   l => Assert.Equal("0", l.Split(' ')[2]));
    }

    [Fact]
    public void The_port_writes_the_native_records_byte_for_byte()
    {
        List<Case> aCases = theCases.Value;
        var aDiffs = new List<string>();
        int aLines = 0;
        foreach (Case aCase in aCases)
        {
            List<string> aPort = Port(aCase);
            aLines += aCase.Native.Count;
            int aN = Math.Max(aPort.Count, aCase.Native.Count);
            for (int i = 0; i < aN; ++i)
            {
                string? aP = i < aPort.Count ? aPort[i] : null;
                string? aR = i < aCase.Native.Count ? aCase.Native[i] : null;
                if (aP != aR)
                {
                    aDiffs.Add($"{aCase.Id} line {i + 1}:\n  native {Cut(aR)}\n  port   {Cut(aP)}");
                    break;
                }
            }
        }
        theOutput.WriteLine($"{aCases.Count} cases, {aLines} lines; {aCases.Count - aDiffs.Count} identical");
        foreach (string aDiff in aDiffs)
        {
            theOutput.WriteLine(aDiff);
        }
        Assert.Empty(aDiffs);
    }

    private static string Cut(string? theLine) => theLine is null ? "<none>" : theLine.Length > 200 ? theLine[..200] + "..." : theLine;
}
