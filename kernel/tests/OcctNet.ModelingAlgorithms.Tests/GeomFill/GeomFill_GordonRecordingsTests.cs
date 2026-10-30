// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Text;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Native OCCT's GeomFill_Gordon and GeomFill_NetworkSurface on reproducible random networks, recorded by
/// tools/gordonrec (unit 5L3): the port rebuilds each case's curves from the recorded B-spline data, runs the same
/// algorithm and writes its results in the recorder's format, every double as its bits; the tokens must be equal.
/// </summary>
public class GeomFill_GordonRecordingsTests(ITestOutputHelper output)
{
    private static readonly Lazy<string[]> Lines = new(Load);

    private static string[] Load()
    {
        using Stream aRes = typeof(GeomFill_GordonRecordingsTests).Assembly.GetManifestResourceStream("GeomFill.gordonrec.txt.gz")
                            ?? throw new InvalidOperationException("GeomFill.gordonrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        return aReader.ReadToEnd().Split('\n', StringSplitOptions.RemoveEmptyEntries);
    }

    private static string Hex(double theValue) =>
        BitConverter.DoubleToInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture);

    private sealed class Reader(string[] theTokens)
    {
        private int myPos;

        public string Next() => theTokens[myPos++];

        public int Int() => int.Parse(Next(), CultureInfo.InvariantCulture);

        public double Real() => double.Parse(Next(), NumberStyles.Float, CultureInfo.InvariantCulture);

        public void Expect(string theToken) => Assert.Equal(theToken, Next());

        public Geom_BSplineCurve Curve()
        {
            int aDegree = Int();
            bool isPeriodic = Int() != 0;
            bool isRational = Int() != 0;
            int aNbPoles = Int();
            NCollection_Array1<gp_Pnt> aPoles = new(1, aNbPoles);
            NCollection_Array1<double> aWeights = new(1, aNbPoles);
            for (int i = 1; i <= aNbPoles; ++i)
            {
                aPoles[i] = new gp_Pnt(Real(), Real(), Real());
                aWeights[i] = isRational ? Real() : 1.0;
            }
            int aNbKnots = Int();
            NCollection_Array1<double> aKnots = new(1, aNbKnots);
            NCollection_Array1<int> aMults = new(1, aNbKnots);
            for (int i = 1; i <= aNbKnots; ++i)
            {
                aKnots[i] = Real();
                aMults[i] = Int();
            }
            return isRational ? new Geom_BSplineCurve(aPoles, aWeights, aKnots, aMults, aDegree, isPeriodic)
                              : new Geom_BSplineCurve(aPoles, aKnots, aMults, aDegree, isPeriodic);
        }
    }

    private static void PutSurface(List<string> theOut, Geom_BSplineSurface theSurface)
    {
        theOut.Add("ud:" + theSurface.UDegree());
        theOut.Add("vd:" + theSurface.VDegree());
        theOut.Add("up:" + (theSurface.IsUPeriodic() ? "1" : "0"));
        theOut.Add("vp:" + (theSurface.IsVPeriodic() ? "1" : "0"));
        theOut.Add("ur:" + (theSurface.IsURational() ? "1" : "0"));
        theOut.Add("vr:" + (theSurface.IsVRational() ? "1" : "0"));
        theOut.Add("nu:" + theSurface.NbUPoles());
        theOut.Add("nv:" + theSurface.NbVPoles());
        bool isRational = theSurface.IsURational() || theSurface.IsVRational();
        ulong aHash = 14695981039346656037ul;
        int aCount = 0;
        for (int i = 1; i <= theSurface.NbUPoles(); ++i)
        {
            for (int j = 1; j <= theSurface.NbVPoles(); ++j)
            {
                gp_Pnt aP = theSurface.Pole(i, j);
                double aW = isRational ? theSurface.Weight(i, j) : 1.0;
                double[] aVals = [aP.X(), aP.Y(), aP.Z(), aW];
                for (int k = 0; k < (isRational ? 4 : 3); ++k)
                {
                    ulong aBits = (ulong)BitConverter.DoubleToInt64Bits(aVals[k]);
                    for (int b = 0; b < 8; ++b)
                    {
                        aHash ^= (aBits >> (8 * b)) & 0xff;
                        aHash *= 1099511628211ul;
                    }
                }
                if (aCount++ < 12)
                {
                    theOut.Add("p:" + Hex(aP.X()) + "," + Hex(aP.Y()) + "," + Hex(aP.Z()));
                    if (isRational)
                    {
                        theOut.Add("w:" + Hex(aW));
                    }
                }
            }
        }
        theOut.Add("ph:" + aHash.ToString("x16", CultureInfo.InvariantCulture));
        for (int i = 1; i <= theSurface.NbUKnots(); ++i)
        {
            theOut.Add("uk:" + Hex(theSurface.UKnot(i)) + "," + theSurface.UMultiplicity(i));
        }
        for (int i = 1; i <= theSurface.NbVKnots(); ++i)
        {
            theOut.Add("vk:" + Hex(theSurface.VKnot(i)) + "," + theSurface.VMultiplicity(i));
        }
        double aU0 = theSurface.UKnot(1), aU1 = theSurface.UKnot(theSurface.NbUKnots());
        double aV0 = theSurface.VKnot(1), aV1 = theSurface.VKnot(theSurface.NbVKnots());
        for (int i = 0; i <= 4; ++i)
        {
            for (int j = 0; j <= 4; ++j)
            {
                gp_Pnt aP = theSurface.Value(aU0 + (aU1 - aU0) * i / 4.0, aV0 + (aV1 - aV0) * j / 4.0);
                theOut.Add("g:" + Hex(aP.X()) + "," + Hex(aP.Y()) + "," + Hex(aP.Z()));
            }
        }
    }

    /// <summary>Replays one recorded line; returns the port's tokens and the recorded ones.</summary>
    internal static (List<string> Port, string[] Native) Replay(string theLine)
    {
        string[] aHalves = theLine.Split(" ; ");
        Reader aIn = new(aHalves[0].Split(' '));
        string[] aNative = aHalves[1].Split(' ');
        List<string> anOut = [];
        string aKind = aIn.Next();
        aIn.Next(); // name
        if (aKind == "G")
        {
            bool isParallel = aIn.Int() != 0;
            bool isApprox = aIn.Int() != 0;
            double aTol = aIn.Real();
            aIn.Expect("P");
            int n = aIn.Int();
            NCollection_Array1<Geom_Curve> aProfiles = new(1, n);
            for (int i = 1; i <= n; ++i)
            {
                aProfiles[i] = aIn.Curve();
            }
            aIn.Expect("Q");
            int m = aIn.Int();
            NCollection_Array1<Geom_Curve> aGuides = new(1, m);
            for (int j = 1; j <= m; ++j)
            {
                aGuides[j] = aIn.Curve();
            }
            try
            {
                GeomFill_Gordon aGordon = new();
                aGordon.SetParallelMode(isParallel);
                if (isApprox)
                {
                    aGordon.SetApproximationMode(GeomFill_Gordon.ApproximationMode.AllowApproximateFallback);
                }
                aGordon.Init(aProfiles, aGuides, aTol);
                aGordon.Perform();
                GeomFill_Gordon.BuildReport aReport = aGordon.Report();
                anOut.Add("st:" + (int)aReport.Status);
                anOut.Add("fs:" + (int)aReport.FailedStage);
                anOut.Add("ap:" + (aReport.IsApproximate ? "1" : "0"));
                anOut.Add("gap:" + Hex(aReport.MaxContactGap));
                anOut.Add("rep:" + Hex(aReport.MaxReparametrizationDeviation));
                anOut.Add("pd:" + Hex(aReport.MaxProfileDeviation));
                anOut.Add("gd:" + Hex(aReport.MaxGuideDeviation));
                anOut.Add("ad:" + Hex(aReport.MaxApproximationDeviation));
                if (aGordon.IsDone())
                {
                    PutSurface(anOut, aGordon.Surface());
                }
            }
            catch (Standard_Failure)
            {
                anOut.Add("exc");
            }
        }
        else
        {
            double aTol = aIn.Real();
            bool isUClosed = aIn.Int() != 0;
            bool isVClosed = aIn.Int() != 0;
            aIn.Expect("P");
            int n = aIn.Int();
            NCollection_Array1<Geom_BSplineCurve?> aProfiles = new(1, n);
            for (int i = 1; i <= n; ++i)
            {
                aProfiles[i] = aIn.Curve();
            }
            aIn.Expect("Q");
            int m = aIn.Int();
            NCollection_Array1<Geom_BSplineCurve?> aGuides = new(1, m);
            for (int j = 1; j <= m; ++j)
            {
                aGuides[j] = aIn.Curve();
            }
            aIn.Expect("V");
            NCollection_Array1<double> aV = new(1, aIn.Int());
            for (int i = 1; i <= aV.Length(); ++i)
            {
                aV[i] = aIn.Real();
            }
            aIn.Expect("U");
            NCollection_Array1<double> aU = new(1, aIn.Int());
            for (int j = 1; j <= aU.Length(); ++j)
            {
                aU[j] = aIn.Real();
            }
            aIn.Expect("X");
            NCollection_Array2<gp_Pnt> aGrid = new(1, m, 1, n);
            for (int j = 1; j <= m; ++j)
            {
                for (int i = 1; i <= n; ++i)
                {
                    aGrid[j, i] = new gp_Pnt(aIn.Real(), aIn.Real(), aIn.Real());
                }
            }
            aIn.Expect("W");
            NCollection_Array2<double> aWeights = new(1, m, 1, n);
            for (int j = 1; j <= m; ++j)
            {
                for (int i = 1; i <= n; ++i)
                {
                    aWeights[j, i] = aIn.Real();
                }
            }
            try
            {
                GeomFill_NetworkSurface aNetwork = new();
                aNetwork.Init(aProfiles, aGuides, aV, aU, aGrid, aWeights, aTol, isUClosed, isVClosed);
                aNetwork.Perform();
                anOut.Add("st:" + (int)aNetwork.Status());
                if (aNetwork.IsDone())
                {
                    PutSurface(anOut, aNetwork.Surface());
                }
            }
            catch (Standard_Failure)
            {
                anOut.Add("exc");
            }
        }
        return (anOut, aNative);
    }

    [Fact]
    public void Gordon_and_NetworkSurface_reproduce_native_bit_for_bit()
    {
        string[] aLines = Lines.Value;
        int aCases = 0, anIdentical = 0, aTokens = 0, aDone = 0;
        Dictionary<string, int> aDiffByKey = [];
        List<string> aFailures = [];
        foreach (string aLine in aLines)
        {
            ++aCases;
            (List<string> aPort, string[] aNative) = Replay(aLine);
            aTokens += aNative.Length;
            if (aNative[0] == "st:1")
            {
                ++aDone;
            }
            bool isSame = aPort.Count == aNative.Length;
            for (int k = 0; k < Math.Min(aPort.Count, aNative.Length); ++k)
            {
                if (aPort[k] != aNative[k])
                {
                    isSame = false;
                    string aKey = aNative[k].Split(':')[0];
                    aDiffByKey[aKey] = aDiffByKey.GetValueOrDefault(aKey) + 1;
                }
            }
            if (isSame)
            {
                ++anIdentical;
            }
            else
            {
                string aName = aLine.Split(' ')[1];
                aFailures.Add($"{aName}: port {aPort.Count} tokens, native {aNative.Length}; first port {string.Join(' ', aPort.Take(3))}, native {string.Join(' ', aNative.Take(3))}");
            }
        }
        output.WriteLine($"{aCases} cases ({aDone} done natively), {aTokens} tokens; {anIdentical} identical");
        foreach (KeyValuePair<string, int> aDiff in aDiffByKey.OrderBy(d => d.Key, StringComparer.Ordinal))
        {
            output.WriteLine($"  differing {aDiff.Key}: {aDiff.Value}");
        }
        foreach (string aFailure in aFailures.Take(30))
        {
            output.WriteLine("  " + aFailure);
        }
        Assert.True(aCases >= 380);
        Assert.Empty(aFailures);
    }

    [Fact]
    public void Parallel_replay_gives_the_serial_tokens()
    {
        // Every recorded Gordon case, run again with the other parallel mode: the port's result must not change.
        string[] aLines = Lines.Value;
        int aChecked = 0;
        foreach (string aLine in aLines.Where(l => l.StartsWith("G ", StringComparison.Ordinal)).Take(120))
        {
            string[] aTok = aLine.Split(' ');
            aTok[2] = aTok[2] == "1" ? "0" : "1";
            (List<string> aFlipped, _) = Replay(string.Join(' ', aTok));
            (List<string> aPort, _) = Replay(aLine);
            Assert.Equal(aPort, aFlipped);
            ++aChecked;
        }
        Assert.True(aChecked > 100);
    }
}
