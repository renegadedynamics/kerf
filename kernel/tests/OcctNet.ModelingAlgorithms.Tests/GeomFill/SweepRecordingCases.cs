// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using System.IO.Compression;
using System.Text;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Construction.Surfaces.ApproxStyle;
using static Kerf.Engine.Construction.Surfaces.Trihedron;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The port's side of tools/sweeprec (unit 5L2): the same cases, built by the same generator in the same order, and
/// written in the same text, so that each case's result compares with native OCCT's recording as a string. Kept line
/// for line with tools/sweeprec/main.cpp.
/// </summary>
internal static class SweepRecordingCases
{
    /// <summary>The kinds of case and their counts, as in main.cpp.</summary>
    public static readonly (string Name, int Count)[] Kinds =
    [
        ("pipe-radius", 40), ("pipe-cfrenet", 40), ("pipe-frenet", 30), ("pipe-fixed", 30), ("pipe-cnormal", 30),
        ("pipe-dir", 30), ("pipe-darboux", 30), ("pipe-2sect", 30), ("pipe-nsect", 30), ("pipe-guide", 30),
        ("pipe-edge-old", 30), ("pipe-edge", 30), ("kpart", 80), ("sweep-evolved", 40), ("nsections", 40),
        ("blendfunc", 30), ("secgen", 40), ("package", 30), ("ruled", 30), ("pipe-2sect-plane", 40),
    ];

    private static readonly Lazy<Dictionary<string, string>> theNative = new(LoadNative);

    /// <summary>Native OCCT's text of each case, keyed "kind seed".</summary>
    public static Dictionary<string, string> Native => theNative.Value;

    private static Dictionary<string, string> LoadNative()
    {
        using Stream aRes = typeof(SweepRecordingCases).Assembly.GetManifestResourceStream("GeomFill.sweeprec.txt.gz")
                            ?? throw new InvalidOperationException("GeomFill.sweeprec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        Dictionary<string, string> aCases = [];
        string? aKey = null;
        StringBuilder aText = new();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            if (aLine.StartsWith("CASE ", StringComparison.Ordinal))
            {
                if (aKey is not null)
                {
                    aCases[aKey] = aText.ToString();
                }
                aKey = aLine[5..];
                aText.Clear();
            }
            aText.Append(aLine).Append('\n');
        }
        if (aKey is not null)
        {
            aCases[aKey] = aText.ToString();
        }
        return aCases;
    }

    /// <summary>The port's text of the case <paramref name="theSeed"/> of the kind <paramref name="theKind"/>.</summary>
    public static string Run(string theKind, int theSeed)
    {
        int aKindIndex = Array.FindIndex(Kinds, k => k.Name == theKind);
        Rng r = new((ulong)(aKindIndex * 1000 + theSeed));
        theOut.Clear();
        Line($"CASE {theKind} {theSeed}");
        switch (theKind)
        {
            case "pipe-radius": PipeRadius(r); break;
            case "pipe-cfrenet": PipeSection(r, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsCorrectedFrenet); break;
            case "pipe-frenet": PipeSection(r, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsFrenet); break;
            case "pipe-fixed": PipeSection(r, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsFixed); break;
            case "pipe-cnormal": PipeSection(r, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsConstantNormal); break;
            case "pipe-dir": PipeDir(r); break;
            case "pipe-darboux": PipeDarboux(r); break;
            case "pipe-2sect": PipeTwoSections(r, false); break;
            case "pipe-2sect-plane": PipeTwoSections(r, true); break;
            case "pipe-nsect": PipeNSections(r); break;
            case "pipe-guide": PipeGuide(r); break;
            case "pipe-edge-old": PipeEdge(r, true); break;
            case "pipe-edge": PipeEdge(r, false); break;
            case "kpart": KPart(r); break;
            case "sweep-evolved": SweepEvolved(r); break;
            case "nsections": NSectionsLaw(r); break;
            case "blendfunc": BlendFunc(r); break;
            case "secgen": SectionGenerator(r); break;
            case "package": PackageFunctions(r); break;
            case "ruled": Ruled(r); break;
            default: throw new ArgumentException(theKind);
        }
        return theOut.ToString();
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16");

    private static string Hex(Point3D theP) => Hex(theP.X()) + "," + Hex(theP.Y()) + "," + Hex(theP.Z());

    private static string Hex(Vector3D theV) => Hex(theV.X()) + "," + Hex(theV.Y()) + "," + Hex(theV.Z());

    private static string Hex(Point2D theP) => Hex(theP.X()) + "," + Hex(theP.Y());

    private static string Int(int theValue) => theValue.ToString(System.Globalization.CultureInfo.InvariantCulture);

    private static string Int(bool theValue) => theValue ? "1" : "0";

    private static string Fnv(string theText)
    {
        ulong aHash = 14695981039346656037UL;
        foreach (char aChar in theText)
        {
            aHash ^= (byte)aChar;
            aHash = unchecked(aHash * 1099511628211UL);
        }
        return aHash.ToString("x16");
    }

    /// <summary>The generator of main.cpp: a 64-bit LCG; U() has 32 significant bits.</summary>
    private sealed class Rng(ulong theSeed)
    {
        private ulong s = unchecked(theSeed * 2862933555777941757UL + 3037000493UL);

        public ulong Next()
        {
            s = unchecked(s * 6364136223846793005UL + 1442695040888963407UL);
            return s;
        }

        public double U() => (double)(Next() >> 32) / 4294967296.0;

        public double R(double a, double b) => a + (b - a) * U();

        public int I(int a, int b) => a + (int)((Next() >> 33) % (ulong)(b - a + 1));
    }

    [ThreadStatic] private static StringBuilder? theOutStorage;

    private static StringBuilder theOut => theOutStorage ??= new StringBuilder();

    private static void Line(string theLine) => theOut.Append(theLine).Append('\n');

    //==================================================================================================
    // Inputs
    //==================================================================================================

    private static BSplineCurve3D MakePath(Rng r, bool theRational, double theWiggle)
    {
        int aDeg = r.I(2, 4);
        int aNb = r.I(aDeg + 1, aDeg + 4);
        NCollection_Array1<Point3D> aPoles = new(1, aNb);
        NCollection_Array1<double> aW = new(1, aNb);
        for (int i = 1; i <= aNb; i++)
        {
            double x = 10.0 * (i - 1) + r.R(-2, 2);
            double y = r.R(-theWiggle, theWiggle);
            double z = r.R(-theWiggle, theWiggle);
            aPoles[i] = new Point3D(x, y, z);
            aW[i] = theRational ? r.R(0.7, 1.4) : 1.0;
        }
        int aNbK = aNb - aDeg + 1;
        NCollection_Array1<double> aK = new(1, aNbK);
        NCollection_Array1<int> aM = new(1, aNbK);
        for (int i = 1; i <= aNbK; i++)
        {
            aK[i] = (double)(i - 1);
            aM[i] = 1;
        }
        aM[1] = aDeg + 1;
        aM[aNbK] = aDeg + 1;
        if (theRational)
        {
            return new BSplineCurve3D(aPoles, aW, aK, aM, aDeg);
        }
        return new BSplineCurve3D(aPoles, aK, aM, aDeg);
    }

    private static Curve3D MakeSection(Rng r, int theKind, Point3D P, Vector3D T, double theScale)
    {
        Frame3D anAx = new(P, new Direction3D(T));
        switch (theKind)
        {
            case 0:
                return new CircleCurve3D(anAx, theScale * r.R(0.5, 1.5));
            case 1:
                {
                    double aMaj = theScale * r.R(1.0, 2.0);
                    return new EllipseCurve3D(anAx, aMaj, aMaj * r.R(0.3, 0.9));
                }
            case 2:
                {
                    Vector3D aX = new(anAx.XDirection());
                    Point3D aP1 = P.Translated(aX * (-theScale * r.R(0.5, 1.5)));
                    Vector3D aD = aX * theScale * r.R(1.0, 2.5);
                    return new TrimmedCurve3D(new LineCurve3D(aP1, new Direction3D(aD)), 0.0, aD.Magnitude());
                }
            default:
                {
                    bool aPer = theKind == 4;
                    int aNb = r.I(5, 8);
                    NCollection_Array1<Point3D> aPoles = new(1, aNb);
                    Vector3D aX = new(anAx.XDirection()), aY = new(anAx.YDirection());
                    for (int i = 1; i <= aNb; i++)
                    {
                        double anA = 2.0 * M_PI * (i - 1) / (aPer ? aNb : aNb - 1) * (aPer ? 1.0 : 0.6);
                        double aR = theScale * r.R(0.7, 1.3);
                        aPoles[i] = P.Translated(aX * (aR * Math.Cos(anA)) + aY * (aR * Math.Sin(anA)));
                    }
                    int aDeg = 3;
                    if (aPer)
                    {
                        NCollection_Array1<double> aKp = new(1, aNb + 1);
                        NCollection_Array1<int> aMp = new(1, aNb + 1);
                        for (int i = 1; i <= aNb + 1; i++)
                        {
                            aKp[i] = (double)(i - 1);
                            aMp[i] = 1;
                        }
                        return new BSplineCurve3D(aPoles, aKp, aMp, aDeg, true);
                    }
                    int aNbK = aNb - aDeg + 1;
                    NCollection_Array1<double> aK = new(1, aNbK);
                    NCollection_Array1<int> aM = new(1, aNbK);
                    for (int i = 1; i <= aNbK; i++)
                    {
                        aK[i] = (double)(i - 1);
                        aM[i] = 1;
                    }
                    aM[1] = aDeg + 1;
                    aM[aNbK] = aDeg + 1;
                    return new BSplineCurve3D(aPoles, aK, aM, aDeg);
                }
        }
    }

    //==================================================================================================
    // Outputs
    //==================================================================================================

    private static double Clamp(double theV) => theV < -10.0 ? -10.0 : (theV > 10.0 ? 10.0 : theV);

    private static void DumpSurface(Surface? theS)
    {
        if (theS is null)
        {
            Line("SURF null");
            return;
        }
        theS.Bounds(out double u1, out double u2, out double v1, out double v2);
        Line("SURF " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theS.GetType()) + " " + Hex(u1) + " " + Hex(u2) + " " + Hex(v1) + " " + Hex(v2));
        Surface aBasis = theS;
        if (theS is TrimmedSurface aTrim)
        {
            aBasis = aTrim.BasisSurface();
            Line("BASIS " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(aBasis.GetType()));
        }
        if (aBasis is BSplineSurface aBS)
        {
            Line("BS " + Int(aBS.UDegree()) + " " + Int(aBS.VDegree()) + " " + Int(aBS.UPoleCount) + " "
                 + Int(aBS.VPoleCount) + " " + Int(aBS.UKnotCount) + " " + Int(aBS.VKnotCount) + " "
                 + Int(aBS.IsUPeriodic) + " " + Int(aBS.IsVPeriodic) + " " + Int(aBS.IsURational) + " "
                 + Int(aBS.IsVRational));
            string aK = "UK";
            for (int i = 1; i <= aBS.UKnotCount; i++)
            {
                aK += " " + Hex(aBS.UKnot(i)) + "x" + Int(aBS.UMultiplicity(i));
            }
            Line(aK);
            aK = "VK";
            for (int i = 1; i <= aBS.VKnotCount; i++)
            {
                aK += " " + Hex(aBS.VKnot(i)) + "x" + Int(aBS.VMultiplicity(i));
            }
            Line(aK);
            StringBuilder aPoles = new();
            for (int i = 1; i <= aBS.UPoleCount; i++)
            {
                for (int j = 1; j <= aBS.VPoleCount; j++)
                {
                    aPoles.Append(Hex(aBS.Pole(i, j))).Append(',').Append(Hex(aBS.Weight(i, j))).Append(';');
                }
            }
            Line("PR " + Fnv(aPoles.ToString()) + " " + Hex(aBS.Pole(1, 1)) + " "
                 + Hex(aBS.Pole(aBS.UPoleCount, aBS.VPoleCount)));
        }
        StringBuilder aGrid = new("GRID");
        double a1 = Clamp(u1), a2 = Clamp(u2), b1 = Clamp(v1), b2 = Clamp(v2);
        for (int i = 0; i <= 4; i++)
        {
            for (int j = 0; j <= 4; j++)
            {
                double u = a1 + (a2 - a1) * i / 4.0, v = b1 + (b2 - b1) * j / 4.0;
                aGrid.Append(' ').Append(Hex(theS.Value(u, v)));
            }
        }
        Line(aGrid.ToString());
    }

    private static void DumpCurve2d(string theTag, Curve2D? theC)
    {
        if (theC is null)
        {
            Line(theTag + " null");
            return;
        }
        double f = Clamp(theC.FirstParameter()), l = Clamp(theC.LastParameter());
        string aLine = theTag + " " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theC.GetType()) + " " + Hex(theC.FirstParameter()) + " "
                       + Hex(theC.LastParameter());
        for (int i = 0; i <= 4; i++)
        {
            aLine += " " + Hex(theC.Value(f + (l - f) * i / 4.0));
        }
        Line(aLine);
    }

    private static void DumpPipe(global::Kerf.Engine.Construction.Surfaces.SurfacePipe thePipe)
    {
        Line("PIPE " + Int(thePipe.IsDone) + " " + Int((int)thePipe.GetStatus()) + " " + Hex(thePipe.ErrorOnSurf())
             + " " + Int(thePipe.ExchangeUV()));
        DumpSurface(thePipe.Surface());
    }

    private static void DumpArrays(string theTag, NCollection_Array1<Point3D> theP, NCollection_Array1<double> theW)
    {
        StringBuilder aLine = new(theTag);
        for (int i = theP.Lower(); i <= theP.Upper(); i++)
        {
            aLine.Append(' ').Append(Hex(theP[i])).Append(',').Append(Hex(theW[i]));
        }
        Line(aLine.ToString());
    }

    private static void DumpArrays(string theTag, NCollection_Array1<Vector3D> theP, NCollection_Array1<double> theW)
    {
        StringBuilder aLine = new(theTag);
        for (int i = theP.Lower(); i <= theP.Upper(); i++)
        {
            aLine.Append(' ').Append(Hex(theP[i])).Append(',').Append(Hex(theW[i]));
        }
        Line(aLine.ToString());
    }

    private static void Guard(Action theBody)
    {
        try
        {
            theBody();
        }
        catch (Exception anExc)
        {
            Line("EXC " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(anExc.GetType()));
        }
    }

    //==================================================================================================
    // Cases
    //==================================================================================================

    private static void PipeRadius(Rng r)
    {
        bool aRat = r.I(0, 3) == 0;
        double aWig = r.R(0.5, 5.0);
        BSplineCurve3D aPath = MakePath(r, aRat, aWig);
        double aR = r.R(0.3, 2.0);
        Line("IN " + Hex(aPath.Pole(1)) + " " + Hex(aR));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aPath, aR);
            aPipe.GenerateParticularCase(true);
            aPipe.Perform(1.0e-4, false, C2);
            DumpPipe(aPipe);
        });
    }

    private static void PipeSection(Rng r, global::Kerf.Engine.Construction.Surfaces.Trihedron theOption)
    {
        bool aRat = r.I(0, 3) == 0;
        double aWig = r.R(0.5, 5.0);
        BSplineCurve3D aPath = MakePath(r, aRat, aWig);
        int aKind = r.I(0, 4);
        aPath.D1(aPath.FirstParameter(), out Point3D aP, out Vector3D aT);
        double sx = r.R(-0.5, 0.5);
        double sy = r.R(-0.5, 0.5);
        double sz = r.R(-0.5, 0.5);
        Vector3D aShift = new(sx, sy, sz);
        Curve3D aSect = MakeSection(r, aKind, aP.Translated(aShift), aT, 1.0);
        int aDeg = r.I(0, 2) == 0 ? 8 : 11;
        Line("IN " + Hex(aPath.Pole(1)) + " " + Int(aKind) + " " + Hex(aSect.Value(aSect.FirstParameter())));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new();
            aPipe.GenerateParticularCase(true);
            aPipe.Init(aPath, aSect, theOption);
            aPipe.Perform(1.0e-4, false, C2, aDeg, 30);
            DumpPipe(aPipe);
        });
    }

    private static void PipeDir(Rng r)
    {
        BSplineCurve3D aPath = MakePath(r, false, r.R(0.5, 3.0));
        int aKind = r.I(0, 4);
        aPath.D1(aPath.FirstParameter(), out Point3D aP, out Vector3D aT);
        Curve3D aSect = MakeSection(r, aKind, aP, aT, 1.0);
        double dx = r.R(-0.3, 0.3);
        double dy = r.R(0.5, 1.0);
        double dz = r.R(-0.3, 0.3);
        Direction3D aDir = new(dx, dy, dz);
        Line("IN " + Hex(aPath.Pole(1)) + " " + Int(aKind));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new();
            aPipe.GenerateParticularCase(true);
            aPipe.Init(aPath, aSect, aDir);
            aPipe.Perform(1.0e-4, false, C2, 11, 30);
            DumpPipe(aPipe);
        });
    }

    private static void PipeDarboux(Rng r)
    {
        // a 2d path on a plane, a cylinder or a B-spline surface
        int aSupKind = r.I(0, 2);
        Surface aSup;
        if (aSupKind == 0)
        {
            double dx = r.R(-0.2, 0.2);
            double dy = r.R(-0.2, 0.2);
            aSup = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(dx, dy, 1.0)));
        }
        else if (aSupKind == 1)
        {
            aSup = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), r.R(5.0, 10.0));
        }
        else
        {
            NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 4);
            for (int i = 1; i <= 4; i++)
            {
                for (int j = 1; j <= 4; j++)
                {
                    aPoles[i, j] = new Point3D(10.0 * (i - 1), 10.0 * (j - 1), r.R(-2.0, 2.0));
                }
            }
            NCollection_Array1<double> aKs = new(1, 2);
            NCollection_Array1<int> aMs = new(1, 2);
            aKs[1] = 0.0;
            aKs[2] = 1.0;
            aMs[1] = aMs[2] = 4;
            aSup = new BSplineSurface(aPoles, aKs, aKs, aMs, aMs, 3, 3);
        }
        double aScaleU = aSupKind == 1 ? 1.0 : (aSupKind == 2 ? 0.1 : 10.0);
        NCollection_Array1<Point2D> aP2 = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aP2[i] = new Point2D((0.1 + 0.25 * (i - 1)) * aScaleU * (aSupKind == 1 ? 2.0 : 1.0),
                                  (0.2 + r.R(0.0, 0.6)) * (aSupKind == 1 ? 10.0 : aScaleU));
        }
        NCollection_Array1<double> aK = new(1, 2);
        NCollection_Array1<int> aM = new(1, 2);
        aK[1] = 0.0;
        aK[2] = 1.0;
        aM[1] = aM[2] = 4;
        BSplineCurve2D aPath = new(aP2, aK, aM, 3);
        CurveAdaptor2D aC2 = new(aPath);
        Point2D aQ = aC2.Value(0.0);
        aSup.D1(aQ.X(), aQ.Y(), out Point3D aP, out Vector3D aDU, out Vector3D aDV);
        Vector2D aT2 = aC2.DN(0.0, 1);
        Vector3D aT = aDU * aT2.X() + aDV * aT2.Y();
        int aKind = r.I(0, 3);
        Curve3D aSect = MakeSection(r, aKind, aP, aT, 0.5);
        Line("IN " + Int(aSupKind) + " " + Hex(aP) + " " + Int(aKind));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new();
            aPipe.GenerateParticularCase(true);
            aPipe.Init(aPath, aSup, aSect);
            aPipe.Perform(1.0e-4, false, C2, 11, 30);
            DumpPipe(aPipe);
        });
    }

    // The path's poles moved onto a convex planar arc: GeomFill_Pipe's evolving sections fail where the corrected
    // Frenet trihedron needs correcting (occt-issues.md).
    private static BSplineCurve3D Flatten(BSplineCurve3D thePath)
    {
        NCollection_Array1<Point3D> aPoles = new(1, thePath.PoleCount);
        for (int i = 1; i <= thePath.PoleCount; i++)
        {
            double anA = 0.3 * (i - 1);
            aPoles[i] = new Point3D(20.0 * Math.Sin(anA), 20.0 - 20.0 * Math.Cos(anA), 0.0);
        }
        return new BSplineCurve3D(aPoles, thePath.InternalKnots(), thePath.InternalMultiplicities(), thePath.Degree());
    }

    private static void PipeTwoSections(Rng r, bool thePlanar)
    {
        BSplineCurve3D aPath = MakePath(r, false, r.R(0.5, 4.0));
        if (thePlanar)
        {
            aPath = Flatten(aPath);
        }
        int aKind = r.I(0, 1) == 0 ? 0 : 3;
        aPath.D1(aPath.FirstParameter(), out Point3D aP, out Vector3D aT);
        Rng aR2 = new(r.Next());
        Curve3D aS1 = MakeSection(r, aKind, aP, aT, 1.0);
        Curve3D aS2 = MakeSection(aR2, aKind, aP, aT, r.R(0.5, 2.0));
        Line("IN " + Hex(aPath.Pole(1)) + " " + Int(aKind));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new();
            aPipe.GenerateParticularCase(true);
            aPipe.Init(aPath, aS1, aS2);
            aPipe.Perform(1.0e-4, false, C2);
            DumpPipe(aPipe);
        });
    }

    private static void PipeNSections(Rng r)
    {
        BSplineCurve3D aPath = MakePath(r, false, r.R(0.5, 3.0));
        int aNb = r.I(2, 4);
        int aKind = r.I(0, 1) == 0 ? 0 : 3;
        NCollection_Sequence<Curve3D> aSeq = new();
        double f = aPath.FirstParameter(), l = aPath.LastParameter();
        for (int i = 0; i < aNb; i++)
        {
            double t = f + (l - f) * i / (aNb - 1.0);
            aPath.D1(t, out Point3D aP, out Vector3D aT);
            Rng aRi = new(r.Next());
            aSeq.Append(MakeSection(aRi, aKind, aP, aT, r.R(0.6, 1.6)));
        }
        Line("IN " + Hex(aPath.Pole(1)) + " " + Int(aNb) + " " + Int(aKind));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new();
            aPipe.GenerateParticularCase(true);
            aPipe.Init(aPath, aSeq);
            aPipe.Perform(1.0e-4, false, C2);
            DumpPipe(aPipe);
        });
    }

    private static void PipeGuide(Rng r)
    {
        BSplineCurve3D aPath = MakePath(r, false, r.R(0.2, 1.5));
        // the guide: the path's poles shifted sideways and wiggled
        NCollection_Array1<Point3D> aPoles = new(1, aPath.PoleCount);
        for (int i = 1; i <= aPath.PoleCount; i++)
        {
            double x = r.R(-0.5, 0.5);
            double y = 3.0 + r.R(-0.5, 0.5);
            double z = r.R(-0.5, 0.5);
            aPoles[i] = aPath.Pole(i).Translated(new Vector3D(x, y, z));
        }
        BSplineCurve3D aGuide = new(aPoles, aPath.InternalKnots(), aPath.InternalMultiplicities(), aPath.Degree());
        aPath.D1(aPath.FirstParameter(), out Point3D aP, out Vector3D aT);
        int aKind = r.I(0, 3);
        Curve3D aSect = MakeSection(r, aKind, aP, aT, 1.0);
        bool byACR = r.I(0, 1) == 1, rotat = r.I(0, 1) == 1;
        Line("IN " + Hex(aPath.Pole(1)) + " " + Int(aKind) + " " + Int(byACR) + " " + Int(rotat));
        Guard(() =>
        {
            CurveAdaptor aG = new(aGuide);
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aPath, aG, aSect, byACR, rotat);
            aPipe.GenerateParticularCase(true);
            aPipe.Perform(1.0e-4, false, C2, 11, 30);
            DumpPipe(aPipe);
        });
    }

    private static void EdgeCurves(Rng r, out BSplineCurve3D thePath, out BSplineCurve3D theC1,
                                   out BSplineCurve3D theC2, out double theR)
    {
        thePath = MakePath(r, false, r.R(0.2, 2.0));
        theR = r.R(1.0, 3.0);
        double anAng = r.R(0.6, 2.6);
        NCollection_Array1<Point3D> aP1 = new(1, thePath.PoleCount), aP2 = new(1, thePath.PoleCount);
        for (int i = 1; i <= thePath.PoleCount; i++)
        {
            Point3D aP = thePath.Pole(i);
            double x1 = r.R(-0.1, 0.1);
            double z1 = r.R(-0.1, 0.1);
            aP1[i] = aP.Translated(new Vector3D(x1, theR, z1));
            double x2 = r.R(-0.1, 0.1);
            aP2[i] = aP.Translated(new Vector3D(x2, theR * Math.Cos(anAng), theR * Math.Sin(anAng)));
        }
        theC1 = new BSplineCurve3D(aP1, thePath.InternalKnots(), thePath.InternalMultiplicities(), thePath.Degree());
        theC2 = new BSplineCurve3D(aP2, thePath.InternalKnots(), thePath.InternalMultiplicities(), thePath.Degree());
    }

    private static void PipeEdge(Rng r, bool theOld)
    {
        EdgeCurves(r, out BSplineCurve3D aPath, out BSplineCurve3D aC1, out BSplineCurve3D aC2, out double aR);
        bool aPoly = r.I(0, 2) == 0;
        Line("IN " + Hex(aPath.Pole(1)) + " " + Hex(aR) + " " + Int(aPoly));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new(aPath, aC1, aC2, aR);
            aPipe.GenerateParticularCase(true);
            if (theOld)
            {
                aPipe.Perform(true, aPoly);
            }
            else
            {
                aPipe.Perform(1.0e-4, aPoly, C2);
            }
            DumpPipe(aPipe);
        });
    }

    private static void KPart(Rng r)
    {
        int aCase = r.I(0, 7);
        Curve3D aPath;
        if (aCase < 4)
        {
            double dx = r.R(-0.2, 0.2);
            double dy = r.R(-0.2, 0.2);
            Direction3D aD = new(dx, dy, 1.0);
            double px = r.R(-1, 1);
            double py = r.R(-1, 1);
            double aL = r.R(5.0, 20.0);
            aPath = new TrimmedCurve3D(new LineCurve3D(new Point3D(px, py, 0), aD), 0.0, aL);
        }
        else
        {
            double dx = r.R(-0.1, 0.1);
            double dy = r.R(-0.1, 0.1);
            Frame3D anAx = new(new Point3D(0, 0, 0), new Direction3D(dx, dy, 1.0));
            double anEnd = aCase == 7 ? 2 * M_PI : r.R(1.0, 5.0);
            double aRad = r.R(5.0, 10.0);
            aPath = new TrimmedCurve3D(new CircleCurve3D(anAx, aRad), 0.0, anEnd);
        }
        aPath.D1(aPath.FirstParameter(), out Point3D aP, out Vector3D aT);
        Curve3D? aSect = null;
        int aSectKind = 0;
        switch (aCase)
        {
            case 0:
            case 4:
            case 7:
                aSectKind = 0; // circle: cylinder, torus, sphere when centred on the axis
                if (aCase == 7 && r.I(0, 1) == 1)
                {
                    // a half circle in the plane of the axis: a sphere
                    Frame3D anAx = new(new Point3D(0, 0, 0), new Direction3D(aT));
                    aSect = new TrimmedCurve3D(new CircleCurve3D(anAx, r.R(1.0, 3.0)), -M_PI / 2, M_PI / 2);
                }
                break;
            case 1:
            case 5:
                aSectKind = 2; // segment: plane, cone, cylinder
                break;
            case 2:
                aSectKind = 1; // ellipse: extrusion
                break;
            case 3:
                aSectKind = 3; // B-spline: extrusion
                break;
            default:
                aSectKind = 4; // revolution
                break;
        }
        aSect ??= MakeSection(r, aSectKind, aP, aT, 1.0);
        bool aTwo = aCase == 0 && r.I(0, 1) == 1; // two coaxial circles: a conical law
        Line("IN " + Int(aCase) + " " + Int(aSectKind) + " " + Int(aTwo));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfacePipe aPipe = new();
            aPipe.GenerateParticularCase(true);
            if (aTwo)
            {
                CircleCurve3D aC2 = (CircleCurve3D)aSect.Copy();
                aC2.SetRadius(aC2.Radius() * r.R(0.5, 2.0));
                aPipe.Init(aPath, aSect, aC2);
            }
            else
            {
                aPipe.Init(aPath, aSect, global::Kerf.Engine.Construction.Surfaces.Trihedron.IsCorrectedFrenet);
            }
            aPipe.Perform(1.0e-4, false, C2);
            DumpPipe(aPipe);
        });
    }

    private static void SweepEvolved(Rng r)
    {
        BSplineCurve3D aPath = MakePath(r, false, r.R(0.5, 4.0));
        aPath.D1(aPath.FirstParameter(), out Point3D _, out Vector3D _);
        int aKind = r.I(0, 4);
        Curve3D aSect = MakeSection(r, aKind, new Point3D(0, 0, 0), new Vector3D(0, 0, 1), 1.0);
        LawsFunction aLaw;
        double f = aPath.FirstParameter(), l = aPath.LastParameter();
        if (r.I(0, 1) == 0)
        {
            Linear aLin = new();
            double v1 = r.R(0.5, 1.5);
            double v2 = r.R(0.5, 1.5);
            aLin.Set(f, v1, l, v2);
            aLaw = aLin;
        }
        else
        {
            S anS = new();
            double v1 = r.R(0.5, 1.5);
            double v2 = r.R(0.5, 1.5);
            anS.Set(f, v1, l, v2);
            aLaw = anS;
        }
        bool aKPart = r.I(0, 1) == 1;
        Continuity aCont = r.I(0, 1) == 0 ? C1 : C2;
        bool aForce = r.I(0, 3) == 0;
        Line("IN " + Hex(aPath.Pole(1)) + " " + Int(aKind) + " " + Int(aKPart) + " " + Int((int)aCont));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.CurveAndTrihedron aLoc = new(new global::Kerf.Engine.Construction.Surfaces.CorrectedFrenet());
            aLoc.SetCurve(new CurveAdaptor(aPath));
            global::Kerf.Engine.Construction.Surfaces.EvolvedSection aSec = new(aSect, aLaw);
            global::Kerf.Engine.Construction.Surfaces.SurfaceSweep aSweep = new(aLoc, aKPart);
            aSweep.SetTolerance(1.0e-4, 1.0e-4, 1.0e-5, 0.01);
            aSweep.SetForceApproxC1(aForce);
            aSweep.Build(aSec, global::Kerf.Engine.Construction.Surfaces.ApproxStyle.Location, aCont, 10, 30);
            Line("SWEEP " + Int(aSweep.IsDone) + " " + Hex(aSweep.ErrorOnSurface()) + " " + Int(aSweep.ExchangeUV())
                 + " " + Int(aSweep.UReversed()) + " " + Int(aSweep.VReversed()) + " " + Int(aSweep.NumberOfTrace()));
            if (aSweep.IsDone)
            {
                DumpSurface(aSweep.Surface());
                DumpCurve2d("R1", aSweep.Restriction(true));
                DumpCurve2d("R2", aSweep.Restriction(false));
                aSweep.ErrorOnRestriction(true, out double aU, out double aV);
                Line("ER " + Hex(aU) + " " + Hex(aV));
            }
        });
    }

    private static void NSectionsLaw(Rng r)
    {
        int aNb = r.I(2, 4);
        int aKind = r.I(0, 4);
        NCollection_Sequence<Curve3D> aSeq = new();
        NCollection_Sequence<double> aPar = new();
        for (int i = 0; i < aNb; i++)
        {
            Rng aRi = new(r.Next());
            double x = r.R(-1, 1);
            double y = r.R(-1, 1);
            double s = r.R(0.5, 2.0);
            aSeq.Append(MakeSection(aRi, aKind, new Point3D(x, y, 5.0 * i), new Vector3D(0, 0, 1), s));
            aPar.Append(i == 0 ? 0.0 : aPar.Last() + r.R(0.5, 2.0));
        }
        bool aWithPar = r.I(0, 2) != 0;
        Line("IN " + Int(aNb) + " " + Int(aKind) + " " + Int(aWithPar));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SurfaceNSections aLaw = aWithPar ? new global::Kerf.Engine.Construction.Surfaces.SurfaceNSections(aSeq, aPar, 0.0, 1.0, aPar.First(), aPar.Last())
                                               : new global::Kerf.Engine.Construction.Surfaces.SurfaceNSections(aSeq);
            BSplineSurface? aS = aLaw.BSplineSurface();
            DumpSurface(aS);
            if (aS is null)
            {
                return;
            }
            aLaw.SectionShape(out int aNbP, out int aNbK, out int aDeg);
            double anErr = -1.0;
            bool aConst = aLaw.IsConstant(ref anErr);
            bool aConic = aLaw.IsConicalLaw(ref anErr);
            Line("SHAPE " + Int(aNbP) + " " + Int(aNbK) + " " + Int(aDeg) + " " + Int(aLaw.IsRational) + " "
                 + Int(aLaw.IsUPeriodic) + " " + Int(aLaw.IsVPeriodic) + " " + Int(aConst) + " " + Int(aConic) + " "
                 + Hex(aLaw.MaximalSection()) + " " + Hex(aLaw.BarycentreOfSurf()));
            int aNbI = aLaw.NbIntervals(C2);
            NCollection_Array1<double> anI = new(1, aNbI + 1);
            aLaw.Intervals(anI, C2);
            string aLine = "INT " + Int(aNbI);
            for (int i = 1; i <= aNbI + 1; i++)
            {
                aLine += " " + Hex(anI[i]);
            }
            Line(aLine);
            NCollection_Array1<Point3D> aP = new(1, aNbP);
            NCollection_Array1<Vector3D> aD1 = new(1, aNbP), aD2 = new(1, aNbP);
            NCollection_Array1<double> aW = new(1, aNbP), aDW = new(1, aNbP), aD2W = new(1, aNbP);
            aLaw.GetMinimalWeight(aW);
            DumpArrays("MINW", aP, aW);
            aLaw.GetDomain(out double f, out double l);
            for (int k = 0; k < 4; k++)
            {
                double v = f + (l - f) * r.U();
                Line("V " + Hex(v) + " " + Int(aLaw.D0(v, aP, aW)));
                DumpArrays("D0", aP, aW);
                bool ok1 = aLaw.D1(v, aP, aD1, aW, aDW);
                Line("D1 " + Int(ok1));
                DumpArrays("D1P", aP, aW);
                DumpArrays("D1V", aD1, aDW);
                bool ok2 = aLaw.D2(v, aP, aD1, aD2, aW, aDW, aD2W);
                Line("D2 " + Int(ok2));
                DumpArrays("D2V", aD2, aD2W);
            }
        });
    }

    private static void BlendFunc(Rng r)
    {
        EdgeCurves(r, out BSplineCurve3D aPath, out BSplineCurve3D aC1, out BSplineCurve3D aC2, out double aR);
        bool aPoly = r.I(0, 2) == 0;
        Line("IN " + Hex(aPath.Pole(1)) + " " + Hex(aR) + " " + Int(aPoly));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.CircularBlendFunc aF = new(new CurveAdaptor(aPath), new CurveAdaptor(aC1),
                                                new CurveAdaptor(aC2), aR, aPoly);
            aF.SectionShape(out int aNbP, out int aNbK, out int aDeg);
            Line("SHAPE " + Int(aNbP) + " " + Int(aNbK) + " " + Int(aDeg) + " " + Int(aF.IsRational) + " "
                 + Hex(aF.MaximalSection()) + " " + Hex(aF.BarycentreOfSurf()));
            NCollection_Array1<double> aK = new(1, aNbK);
            NCollection_Array1<int> aM = new(1, aNbK);
            aF.Knots(aK);
            aF.Mults(aM);
            string aLine = "KM";
            for (int i = 1; i <= aNbK; i++)
            {
                aLine += " " + Hex(aK[i]) + "x" + Int(aM[i]);
            }
            Line(aLine);
            int aNbI = aF.NbIntervals(C1);
            Line("NBI " + Int(aNbI));
            NCollection_Array1<double> aTol = new(1, aNbP);
            aF.GetTolerance(1.0e-4, 1.0e-3, 0.01, aTol);
            aLine = "TOL";
            for (int i = 1; i <= aNbP; i++)
            {
                aLine += " " + Hex(aTol[i]);
            }
            Line(aLine);
            NCollection_Array1<Point3D> aP = new(1, aNbP);
            NCollection_Array1<Vector3D> aD1 = new(1, aNbP), aD2 = new(1, aNbP);
            NCollection_Array1<double> aW = new(1, aNbP), aDW = new(1, aNbP), aD2W = new(1, aNbP);
            NCollection_Array1<Point2D> aP2 = new(1, 1);
            NCollection_Array1<Vector2D> aV2 = new(1, 1), aV22 = new(1, 1);
            aF.GetMinimalWeight(aW);
            DumpArrays("MINW", aP, aW);
            double f = aPath.FirstParameter(), l = aPath.LastParameter();
            for (int k = 0; k < 3; k++)
            {
                double v = f + (l - f) * r.U();
                Line("V " + Hex(v));
                aF.D0(v, f, l, aP, aP2, aW);
                DumpArrays("D0", aP, aW);
                aF.D1(v, f, l, aP, aD1, aP2, aV2, aW, aDW);
                DumpArrays("D1P", aP, aW);
                DumpArrays("D1V", aD1, aDW);
                aF.D2(v, f, l, aP, aD1, aD2, aP2, aV2, aV22, aW, aDW, aD2W);
                DumpArrays("D2V", aD2, aD2W);
            }
        });
    }

    private static void SectionGenerator(Rng r)
    {
        int aType = r.I(1, 6);
        Curve3D aPath;
        BSplineCurve3D aBPath = MakePath(r, false, r.R(0.5, 3.0));
        if (aType >= 4)
        {
            double aRad = r.R(5, 10);
            double anEnd = r.R(1.0, 4.0);
            aPath = new TrimmedCurve3D(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), aRad),
                                          0.0, anEnd);
        }
        else
        {
            aPath = aBPath;
        }
        aPath.D1(aPath.FirstParameter(), out Point3D aP, out Vector3D aT);
        int aKind = r.I(0, 1) == 0 ? 0 : 3;
        Rng aR2 = new(r.Next());
        Curve3D aS1 = MakeSection(r, aKind, aP, aT, 1.0);
        Curve3D aS2 = MakeSection(aR2, aKind, aP, aT, 1.5);
        Line("IN " + Int(aType) + " " + Int(aKind));
        Guard(() =>
        {
            global::Kerf.Engine.Construction.Surfaces.SweepSectionGenerator aGen = new();
            if (aType == 1 || aType == 4)
            {
                aGen.Init(aPath, r.R(0.5, 2.0));
            }
            else if (aType == 2 || aType == 5)
            {
                aGen.Init(aPath, aS1);
            }
            else
            {
                aGen.Init(aPath, aS1, aS2);
            }
            aGen.Perform(false);
            aGen.GetShape(out int aNbP, out int aNbK, out int aDeg, out int aNb2d);
            int aNbS = aGen.SectionCount;
            Line("SHAPE " + Int(aNbP) + " " + Int(aNbK) + " " + Int(aDeg) + " " + Int(aNb2d) + " " + Int(aNbS));
            NCollection_Array1<Point3D> aPo = new(1, aNbP);
            NCollection_Array1<Vector3D> aD = new(1, aNbP);
            NCollection_Array1<double> aW = new(1, aNbP), aDW = new(1, aNbP);
            NCollection_Array1<Point2D> aP2 = new(1, 1);
            NCollection_Array1<Vector2D> aV2 = new(1, 1);
            int[] aIdx = [1, 2, aNbS / 2, aNbS];
            for (int k = 0; k < 4; k++)
            {
                aGen.Section(aIdx[k], aPo, aP2, aW);
                Line("S " + Int(aIdx[k]) + " " + Hex(aGen.Parameter(aIdx[k])));
                DumpArrays("SP", aPo, aW);
            }
            for (int k = 0; k < 4; k += 3)
            {
                bool ok = aGen.Section(aIdx[k], aPo, aD, aP2, aV2, aW, aDW);
                Line("SD " + Int(ok));
                if (ok)
                {
                    DumpArrays("SDV", aD, aDW);
                }
            }
        });
    }

    private static void PackageFunctions(Rng r)
    {
        Line("IN");
        Guard(() =>
        {
            // GetShape, Knots, Mults, GetTolerance, GetMinimalWeights
            double anAng = r.R(0.05, 2.0 * M_PI);
            ParameterisationType[] aConvs = [QuasiAngular, Polynomial, TgtThetaOver2];
            foreach (ParameterisationType aConv0 in aConvs)
            {
                ParameterisationType aConv = aConv0;
                global::Kerf.Engine.Construction.Surfaces.GeomFill.GetShape(anAng, out int aNbP, out int aNbK, out int aDeg, ref aConv);
                NCollection_Array1<double> aK = new(1, aNbK);
                NCollection_Array1<int> aM = new(1, aNbK);
                global::Kerf.Engine.Construction.Surfaces.GeomFill.Knots(aConv, aK);
                global::Kerf.Engine.Construction.Surfaces.GeomFill.Mults(aConv, aM);
                string aLine = "GS " + Int((int)aConv) + " " + Int(aNbP) + " " + Int(aNbK) + " " + Int(aDeg);
                for (int i = 1; i <= aNbK; i++)
                {
                    aLine += " " + Hex(aK[i]) + "x" + Int(aM[i]);
                }
                Line(aLine);
                double aMin = r.R(0.01, anAng);
                Line("TOL " + Hex(global::Kerf.Engine.Construction.Surfaces.GeomFill.GetTolerance(aConv, aMin, r.R(0.5, 3.0), 0.01, 1.0e-4)));
                if (aConv != TgtThetaOver2 || aNbP <= 7)
                {
                    NCollection_Array1<double> aWm = new(1, aNbP);
                    NCollection_Array1<Point3D> aDummy = new(1, aNbP);
                    global::Kerf.Engine.Construction.Surfaces.GeomFill.GetMinimalWeights(aConv, aMin, anAng, aWm);
                    DumpArrays("MINW", aDummy, aWm);
                }

                // GetCircle and its derivatives on a random arc and random derivatives
                double aRad = r.R(0.5, 3.0);
                double cx = r.R(-1, 1);
                double cy = r.R(-1, 1);
                double cz = r.R(-1, 1);
                Point3D aC = new(cx, cy, cz);
                double nx = r.R(-0.3, 0.3);
                double ny = r.R(-0.3, 0.3);
                Direction3D aN = new(nx, ny, 1.0);
                Frame3D anAx = new(aC, aN);
                double a1 = r.R(0.0, 1.0), a2 = a1 + anAng;
                Vector3D aX = new(anAx.XDirection()), aY = new(anAx.YDirection());
                Vector3D aU1 = aX * Math.Cos(a1) + aY * Math.Sin(a1), aU2 = aX * Math.Cos(a2) + aY * Math.Sin(a2);
                Point3D aP1 = aC.Translated(aU1 * aRad), aP2 = aC.Translated(aU2 * aRad);
                Vector3D ns1 = -aU1, ns2 = -aU2, nplan = new(aN);
                Vector3D aRV()
                {
                    double x = r.R(-0.2, 0.2);
                    double y = r.R(-0.2, 0.2);
                    double z = r.R(-0.2, 0.2);
                    return new Vector3D(x, y, z);
                }
                Vector3D dn1 = aRV(), dn2 = aRV(), dnp = aRV(), d2n1 = aRV(), d2n2 = aRV(), d2np = aRV();
                Vector3D t1 = aRV(), t2 = aRV(), dt1 = aRV(), dt2 = aRV(), dc = aRV(), d2c = aRV();
                NCollection_Array1<Point3D> aP = new(1, aNbP);
                NCollection_Array1<Vector3D> aD1 = new(1, aNbP), aD2 = new(1, aNbP);
                NCollection_Array1<double> aW = new(1, aNbP), aDW = new(1, aNbP), aD2W = new(1, aNbP);
                global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(aConv, ns1, ns2, nplan, aP1, aP2, aRad, aC, aP, aW);
                DumpArrays("C0", aP, aW);
                bool ok1 = global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(aConv, ns1, ns2, dn1, dn2, nplan, dnp, aP1, aP2, t1, t2, aRad, 0.1, aC, dc,
                                              aP, aD1, aW, aDW);
                Line("C1 " + Int(ok1));
                DumpArrays("C1P", aP, aW);
                DumpArrays("C1V", aD1, aDW);
                bool ok2 = global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(aConv, ns1, ns2, dn1, dn2, d2n1, d2n2, nplan, dnp, d2np, aP1, aP2, t1, t2,
                                              dt1, dt2, aRad, 0.1, 0.05, aC, dc, d2c, aP, aD1, aD2, aW, aDW, aD2W);
                Line("C2 " + Int(ok2));
                DumpArrays("C2P", aP, aW);
                DumpArrays("C2V", aD1, aDW);
                DumpArrays("C2W", aD2, aD2W);
            }
        });
    }

    private static void Ruled(Rng r)
    {
        int aCase = r.I(0, 4);
        Curve3D aC1, aC2;
        switch (aCase)
        {
            case 0:
                { // two parallel segments (a plane), the second reversed or not
                    double dx = r.R(-1, 1);
                    double dy = r.R(-1, 1);
                    Direction3D aD = new(dx, dy, 1.0);
                    double px = r.R(-1, 1);
                    double py = r.R(-1, 1);
                    Point3D aP = new(px, py, 0);
                    double ox = r.R(1, 3);
                    double oy = r.R(1, 3);
                    Vector3D aOff = new(ox, oy, 0);
                    double a = r.R(-2, 0), b = r.R(1, 3);
                    aC1 = new TrimmedCurve3D(new LineCurve3D(aP, aD), a, b);
                    Vector3D aDV = new(aD);
                    double proj = aOff.Dot(aDV);
                    if (r.I(0, 1) == 0)
                    {
                        aC2 = new TrimmedCurve3D(new LineCurve3D(aP.Translated(aOff), aD), a - proj, b - proj);
                    }
                    else
                    {
                        aC2 = new TrimmedCurve3D(new LineCurve3D(aP.Translated(aOff), aD.Reversed()), proj - b, proj - a);
                    }
                    break;
                }
            case 1:
            case 2:
                { // coaxial circles: a cylinder or a cone
                    double px = r.R(-1, 1);
                    double dx = r.R(-0.2, 0.2);
                    Frame3D anAx = new(new Point3D(px, 0, 0), new Direction3D(dx, 0, 1));
                    double aR = r.R(1, 3);
                    aC1 = new CircleCurve3D(anAx, aR);
                    Frame3D anAx2 = anAx.Translated(new Vector3D(anAx.Direction()) * r.R(-5, 5));
                    aC2 = new CircleCurve3D(anAx2, aCase == 1 ? aR : aR * r.R(0.3, 2.0));
                    break;
                }
            default:
                { // B-splines and others: GeomFill_Generator
                    aC1 = MakePath(r, false, 2.0);
                    BSplineCurve3D aB = MakePath(r, false, 2.0);
                    aC2 = aCase == 3 ? aB : MakeSection(r, 1, new Point3D(0, 5, 0), new Vector3D(0, 1, 0), 2.0);
                    break;
                }
        }
        Line("IN " + Int(aCase));
        Guard(() => DumpSurface(global::Kerf.Engine.Construction.Surfaces.GeomFill.Surface(aC1, aC2)));
    }
}
