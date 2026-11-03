// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Geometry;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using static Kerf.Engine.Approximation.ParametrizationType;
using static Kerf.Geometry.Continuity;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Native OCCT's surfaces through sections recorded by tools/appsurfrec (unit 5L3): GeomFill_Profiler, GeomFill_Generator
/// and GeomFill_AppSurf (Bezier pieces joined, the B-spline least squares, the variational smoothing, pieces of at most
/// NbMaxP sections). Each case's inputs are round-trip decimals and native's results named tokens, every double as its
/// 64 bits in hex (pole grids as an FNV-1a digest of their bits). <see cref="Run"/> replays a case on the port and
/// writes its results in the same format.
/// </summary>
internal static class AppSurfRecordings
{
    public sealed record Case(string Kind, double[] Inputs, string[] Native);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    public static List<Case> Cases => theCases.Value;

    private static List<Case> Load()
    {
        using Stream aRes = typeof(AppSurfRecordings).Assembly.GetManifestResourceStream("GeomFill.appsurfrec.txt.gz")
                            ?? throw new InvalidOperationException("GeomFill.appsurfrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        List<Case> aCases = [];
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            aCases.Add(Parse(aLine));
        }
        return aCases;
    }

    public static Case Parse(string theLine)
    {
        int aSemi = theLine.IndexOf(';');
        string aHead = aSemi < 0 ? theLine : theLine[..aSemi];
        string[] aIn = aHead.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        double[] aValues = aIn.Skip(1).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
        string[] aOut = aSemi < 0 ? [] : theLine[(aSemi + 1)..].Split(' ', StringSplitOptions.RemoveEmptyEntries);
        return new Case(aIn[0], aValues, aOut);
    }

    //=============================================================================================
    // Output, as appsurfrec writes it

    private sealed class Writer
    {
        public readonly List<string> Tokens = [];

        public void Put(string theTok) => Tokens.Add(theTok);
    }

    private static string ExcName(global::Kerf.Engine.Foundation.FailureException theF) => "!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theF.GetType());

    private static string Hex(double theValue) =>
        ((ulong)BitConverter.DoubleToInt64Bits(theValue)).ToString("x", CultureInfo.InvariantCulture);

    private static string Hex(Point3D theP) => Hex(theP.X()) + "," + Hex(theP.Y()) + "," + Hex(theP.Z());

    private static string Int(int theValue) => theValue.ToString(CultureInfo.InvariantCulture);

    private static string Join(string theA, string theB) => theA.Length == 0 ? theB : theA + "," + theB;

    // FNV-1a over the 64 bits of each value
    private sealed class Digest
    {
        private ulong myH = 14695981039346656037UL;
        private int myN;

        public void Add(double theValue)
        {
            ulong aBits = (ulong)BitConverter.DoubleToInt64Bits(theValue);
            for (int i = 0; i < 8; i++)
            {
                myH ^= (aBits >> (8 * i)) & 0xff;
                myH *= 1099511628211UL;
            }
            myN++;
        }

        public void Add(Point3D theP)
        {
            Add(theP.X());
            Add(theP.Y());
            Add(theP.Z());
        }

        public string Str() => Int(myN) + "," + myH.ToString("x", CultureInfo.InvariantCulture);
    }

    //=============================================================================================
    // Inputs

    private sealed class Reader(double[] theInputs)
    {
        private int myPos;

        public double Next() => theInputs[myPos++];

        public int NextInt() => (int)theInputs[myPos++];
    }

    private static Curve3D MakeSection(Reader theR)
    {
        int aType = theR.NextInt();
        if (aType == 0)
        {
            int aDeg = theR.NextInt();
            bool aRat = theR.NextInt() != 0;
            bool aPer = theR.NextInt() != 0;
            int aNbP = theR.NextInt();
            NCollection_Array1<Point3D> aPoles = new(1, aNbP);
            NCollection_Array1<double> aW = new(1, aNbP);
            for (int i = 1; i <= aNbP; i++)
            {
                double x = theR.Next(), y = theR.Next(), z = theR.Next();
                aPoles[i] = new Point3D(x, y, z);
                if (aRat)
                {
                    aW[i] = theR.Next();
                }
            }
            int aNbK = theR.NextInt();
            NCollection_Array1<double> aKnots = new(1, aNbK);
            NCollection_Array1<int> aMults = new(1, aNbK);
            for (int i = 1; i <= aNbK; i++)
            {
                aKnots[i] = theR.Next();
                aMults[i] = theR.NextInt();
            }
            return aRat ? new BSplineCurve3D(aPoles, aW, aKnots, aMults, aDeg, aPer)
                        : new BSplineCurve3D(aPoles, aKnots, aMults, aDeg, aPer);
        }
        if (aType == 1)
        {
            bool aRat = theR.NextInt() != 0;
            int aNbP = theR.NextInt();
            NCollection_Array1<Point3D> aPoles = new(1, aNbP);
            NCollection_Array1<double> aW = new(1, aNbP);
            for (int i = 1; i <= aNbP; i++)
            {
                double x = theR.Next(), y = theR.Next(), z = theR.Next();
                aPoles[i] = new Point3D(x, y, z);
                if (aRat)
                {
                    aW[i] = theR.Next();
                }
            }
            return aRat ? new BezierCurve3D(aPoles, aW) : new BezierCurve3D(aPoles);
        }
        if (aType == 2)
        {
            double cx = theR.Next(), h = theR.Next(), cz = theR.Next(), r = theR.Next(), a = theR.Next(), b = theR.Next();
            CircleCurve3D aC = new(new Frame3D(new Point3D(cx, h, cz), new Direction3D(0, 1, 0)), r);
            return new TrimmedCurve3D(aC, a, b);
        }
        {
            double x = theR.Next(), y = theR.Next(), z = theR.Next();
            double dx = theR.Next(), dy = theR.Next(), dz = theR.Next(), l = theR.Next();
            LineCurve3D aL = new(new Point3D(x, y, z), new Direction3D(dx, dy, dz));
            return new TrimmedCurve3D(aL, 0.0, l);
        }
    }

    //=============================================================================================
    // Results

    // As appsurfrec's PutProfile: false when the curves have not been brought to one number of poles.
    private static bool PutProfile(Writer theW, List<Curve3D> theCurves)
    {
        GeomFill_Profiler aP = new();
        foreach (Curve3D aC in theCurves)
        {
            aP.AddCurve(aC);
        }
        aP.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        int aDeg = aP.Degree(), aNbP = aP.NbPoles(), aNbK = aP.NbKnots();
        theW.Put("prof:" + Int(aDeg) + "," + Int(aNbP) + "," + Int(aNbK) + "," + (aP.IsPeriodic() ? "1" : "0"));
        NCollection_Array1<double> aK = new(1, aNbK);
        NCollection_Array1<int> aM = new(1, aNbK);
        aP.KnotsAndMults(aK, aM);
        string aS = "";
        for (int i = 1; i <= aNbK; i++)
        {
            aS = Join(aS, Hex(aK[i]) + "," + Int(aM[i]));
        }
        theW.Put("pk:" + aS);
        string aCounts = "";
        bool anEqual = true;
        for (int c = 1; c <= theCurves.Count; c++)
        {
            BSplineCurve3D aC = (BSplineCurve3D)aP.Curve(c);
            aCounts = Join(aCounts, Int(aC.PoleCount) + "," + Int(aC.KnotCount));
            anEqual = anEqual && aC.PoleCount == aNbP && aC.KnotCount == aNbK;
        }
        theW.Put("pn:" + aCounts);
        if (!anEqual)
        {
            return false;
        }
        for (int c = 1; c <= theCurves.Count; c++)
        {
            NCollection_Array1<Point3D> aPoles = new(1, aNbP);
            NCollection_Array1<double> aW = new(1, aNbP);
            aP.Poles(c, aPoles);
            aP.Weights(c, aW);
            Digest aDP = new(), aDW = new();
            for (int i = 1; i <= aNbP; i++)
            {
                aDP.Add(aPoles[i]);
                aDW.Add(aW[i]);
            }
            theW.Put("pp" + Int(c) + ":" + aDP.Str() + "," + aDW.Str() + "," + Hex(aPoles[1]));
        }
        return true;
    }

    private static void PutBSplineSurface(Writer theW, BSplineSurface theS)
    {
        theW.Put("sdeg:" + Int(theS.UDegree()) + "," + Int(theS.VDegree()) + "," + Int(theS.UPoleCount) + ","
                 + Int(theS.VPoleCount) + "," + (theS.IsURational ? "1" : "0") + (theS.IsVRational ? "1" : "0")
                 + (theS.IsUPeriodic ? "1" : "0") + (theS.IsVPeriodic ? "1" : "0"));
        Digest aP = new(), aW = new();
        for (int i = 1; i <= theS.UPoleCount; i++)
        {
            for (int j = 1; j <= theS.VPoleCount; j++)
            {
                aP.Add(theS.Pole(i, j));
                aW.Add(theS.Weight(i, j));
            }
        }
        theW.Put("spol:" + aP.Str());
        theW.Put("swt:" + aW.Str());
        string aU = "", aV = "";
        for (int i = 1; i <= theS.UKnotCount; i++)
        {
            aU = Join(aU, Hex(theS.UKnot(i)) + "," + Int(theS.UMultiplicity(i)));
        }
        for (int i = 1; i <= theS.VKnotCount; i++)
        {
            aV = Join(aV, Hex(theS.VKnot(i)) + "," + Int(theS.VMultiplicity(i)));
        }
        theW.Put("suk:" + aU);
        theW.Put("svk:" + aV);
        theS.Bounds(out double u1, out double u2, out double v1, out double v2);
        string aD = "";
        for (int i = 0; i <= 2; i++)
        {
            for (int j = 0; j <= 2; j++)
            {
                aD = Join(aD, Hex(theS.Value(u1 + (u2 - u1) * i / 2.0, v1 + (v2 - v1) * j / 2.0)));
            }
        }
        theW.Put("sd0:" + aD);
    }

    private static void PutApprox(Writer theW, GeomFill_AppSurf theA)
    {
        theW.Put("done:" + (theA.IsDone() ? "1" : "0"));
        if (!theA.IsDone())
        {
            return;
        }
        theA.SurfShape(out int ud, out int vd, out int nup, out int nvp, out int nuk, out int nvk);
        theW.Put("shape:" + Int(ud) + "," + Int(vd) + "," + Int(nup) + "," + Int(nvp) + "," + Int(nuk) + "," + Int(nvk));
        Digest aP = new(), aW = new();
        NCollection_Array2<Point3D> aPoles = theA.SurfPoles();
        NCollection_Array2<double> aWts = theA.SurfWeights();
        for (int i = aPoles.LowerRow(); i <= aPoles.UpperRow(); i++)
        {
            for (int j = aPoles.LowerCol(); j <= aPoles.UpperCol(); j++)
            {
                aP.Add(aPoles[i, j]);
                aW.Add(aWts[i, j]);
            }
        }
        theW.Put("apol:" + aP.Str() + "," + Hex(aPoles[aPoles.LowerRow(), aPoles.LowerCol()]));
        theW.Put("awt:" + aW.Str());
        string aU = "", aV = "";
        for (int i = theA.SurfUKnots().Lower(); i <= theA.SurfUKnots().Upper(); i++)
        {
            aU = Join(aU, Hex(theA.SurfUKnots()[i]) + "," + Int(theA.SurfUMults()[i]));
        }
        for (int i = theA.SurfVKnots().Lower(); i <= theA.SurfVKnots().Upper(); i++)
        {
            aV = Join(aV, Hex(theA.SurfVKnots()[i]) + "," + Int(theA.SurfVMults()[i]));
        }
        theW.Put("auk:" + aU);
        theW.Put("avk:" + aV);
        theA.TolReached(out double t3, out double t2);
        theW.Put("tol:" + Hex(t3) + "," + Hex(t2) + "," + Hex(theA.TolCurveOnSurf(1)) + "," + Int(theA.NbCurves2d()));
        // the surface Draw's appsurf builds
        try
        {
            BSplineSurface aS = new(theA.SurfPoles(), theA.SurfWeights(), theA.SurfUKnots(), theA.SurfVKnots(),
                                         theA.SurfUMults(), theA.SurfVMults(), theA.UDegree(), theA.VDegree());
            PutBSplineSurface(theW, aS);
        }
        catch (global::Kerf.Engine.Foundation.FailureException anExc)
        {
            theW.Put("srf:" + ExcName(anExc));
        }
    }

    //=============================================================================================
    // The cases

    /// <summary>Replays a case on the port; the tokens as appsurfrec writes them.</summary>
    public static List<string> Run(Case theCase)
    {
        Writer aW = new();
        Reader aR = new(theCase.Inputs);
        int aMode = theCase.Kind switch
        {
            "GEN" => -1,
            "APP" => 0,
            "APPSP" => 1,
            "APPSM" => 2,
            "APPMAXP" => 3,
            _ => throw new InvalidOperationException(theCase.Kind),
        };
        int aNb = aR.NextInt();
        List<Curve3D> aCurves = [];
        for (int i = 0; i < aNb; i++)
        {
            aCurves.Add(MakeSection(aR));
        }
        if (aMode < 0)
        {
            try
            {
                if (!PutProfile(aW, aCurves))
                {
                    return aW.Tokens;
                }
                GeomFill_Generator aGen = new();
                foreach (Curve3D aC in aCurves)
                {
                    aGen.AddCurve(aC);
                }
                aGen.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
                PutBSplineSurface(aW, (BSplineSurface)aGen.Surface()!);
            }
            catch (global::Kerf.Engine.Foundation.FailureException anExc)
            {
                aW.Put("exc:" + ExcName(anExc));
            }
            return aW.Tokens;
        }

        int aDMin = aR.NextInt();
        int aDMax = aR.NextInt();
        double aTol3d = aR.Next();
        double aTol2d = aR.Next();
        int aNbIt = aR.NextInt();
        bool aKnown = aR.NextInt() != 0;
        int aCont = aR.NextInt();
        int aParT = aR.NextInt();
        double aW1 = aR.Next();
        double aW2 = aR.Next();
        double aW3 = aR.Next();
        int aNbMaxP = aR.NextInt();
        bool aDraw = aMode == 0 && aR.NextInt() != 0;
        List<double> aParams = [];
        double aU = aR.Next();
        for (int i = 0; i < aNb; i++)
        {
            aParams.Add(aU);
            aU = aR.Next();
        }
        try
        {
            if (!PutProfile(aW, aCurves))
            {
                return aW.Tokens;
            }
            GeomFill_SectionGenerator aSec = new();
            foreach (Curve3D aC in aCurves)
            {
                aSec.AddCurve(aC);
            }
            aSec.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
            NCollection_HArray1<double> aPar = new(1, aNb);
            for (int i = 1; i <= aNb; i++)
            {
                aPar.SetValue(i, aParams[i - 1]);
            }
            aSec.SetParam(aPar);
            GeomFill_Line aLine = new(aNb);
            GeomFill_AppSurf anApp = new();
            if (aDraw)
            {
                anApp.Init(3, 8, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion(), 0);
            }
            else
            {
                anApp.Init(aDMin, aDMax, aTol3d, aTol2d, aNbIt, aKnown);
                Continuity[] aConts = [C0, C1, C2, C3];
                ParametrizationType[] aPars = [ChordLength, Centripetal, IsoParametric];
                anApp.SetContinuity(aConts[aCont]);
                anApp.SetParType(aPars[aParT]);
                anApp.SetCriteriumWeight(aW1, aW2, aW3);
            }
            if (aMode == 0)
            {
                anApp.Perform(aLine, aSec);
            }
            else if (aMode == 1)
            {
                anApp.Perform(aLine, aSec, true);
            }
            else if (aMode == 2)
            {
                anApp.PerformSmoothing(aLine, aSec);
            }
            else
            {
                anApp.Perform(aLine, aSec, aNbMaxP);
            }
            PutApprox(aW, anApp);
        }
        catch (global::Kerf.Engine.Foundation.FailureException anExc)
        {
            aW.Put("exc:" + ExcName(anExc));
        }
        return aW.Tokens;
    }
}
