// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using System.Globalization;
using System.IO.Compression;
using System.Text;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The native recordings of unit 5F (decision 680): walking lines of IntPatch_Intersection on nine surface pairs,
/// and native OCCT 8.0.1's approximations of them by GeomInt_WLApprox, BRepApprox_Approx and ApproxInt_KnotTools,
/// read from <c>ApproxIntRecordings.txt.gz</c> (an embedded resource).
/// </summary>
internal static class ApproxIntRecordings
{
    internal sealed class Approx
    {
        public char Tag;
        public string Algo = "";
        public bool OnSurfaces;
        public double Tol3d, Tol2d;
        public int DegMin, DegMax, NbIter, NbPntMax;
        public bool Tangency;
        public Approx_ParametrizationType ParType;
        public bool Xyz, U1V1, U2V2;
        public int IMin, IMax;

        public bool IsDone;
        public double TolReached3d, TolReached2d;
        public int NbMultiCurves;
        public int NbCurves, Degree, NbPoles;
        public double[] Knots = [];
        public int[] Mults = [];
        public List<(int Dim, double[] Coords)> Poles = [];

        public override string ToString() =>
            $"{Tag} {Algo} {(OnSurfaces ? "SS" : "L")} [{IMin}, {IMax}] xyz={Xyz} u1v1={U1V1} u2v2={U2V2}";
    }

    internal sealed class Line
    {
        public int Index;
        public double[][] Points = [];
        public bool PtXyz, PtU1V1, PtU2V2;
        public Approx_ParametrizationType ParType;
        public List<Approx> Approxes = [];
    }

    internal sealed class Case
    {
        public string Name = "";
        public string Surface1 = "", Surface2 = "";
        public List<Line> Lines = [];
    }

    internal sealed class KnotToolsRun
    {
        public int MinNbPnts;
        public int[] Knots = [];
    }

    internal sealed class Recording
    {
        public List<Case> Cases = [];
        public List<KnotToolsRun> KnotTools = [];
    }

    private static Recording? theRecording;

    internal static Recording Load()
    {
        if (theRecording is not null)
        {
            return theRecording;
        }

        using Stream aRes = typeof(ApproxIntRecordings).Assembly.GetManifestResourceStream("ApproxIntRecordings.txt.gz")
                            ?? throw new InvalidOperationException("ApproxIntRecordings.txt.gz isn't embedded");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz);
        theRecording = Parse(aReader);
        return theRecording;
    }

    private static double D(string s) => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);

    private static int I(string s) => int.Parse(s, CultureInfo.InvariantCulture);

    private static Recording Parse(TextReader theReader)
    {
        Recording aRec = new();
        Case? aCase = null;
        Line? aLine = null;
        Approx? anApprox = null;
        string? aStr;
        while ((aStr = theReader.ReadLine()) is not null)
        {
            if (aStr.Length == 0)
            {
                continue;
            }
            string[] t = aStr.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            switch (t[0])
            {
                case "CASE":
                    aCase = new Case { Name = t[1] };
                    aRec.Cases.Add(aCase);
                    break;
                case "SURFACE1":
                case "SURFACE2":
                {
                    StringBuilder aText = new();
                    while ((aStr = theReader.ReadLine()) != "ENDSURFACE")
                    {
                        aText.Append(aStr).Append('\n');
                    }
                    if (t[0] == "SURFACE1")
                    {
                        aCase!.Surface1 = aText.ToString();
                    }
                    else
                    {
                        aCase!.Surface2 = aText.ToString();
                    }
                    break;
                }
                case "LINE":
                {
                    int n = I(t[2]);
                    aLine = new Line { Index = I(t[1]), Points = new double[n][] };
                    for (int i = 0; i < n; i++)
                    {
                        aLine.Points[i] = theReader.ReadLine()!.Split(' ', StringSplitOptions.RemoveEmptyEntries)
                                                   .Select(D).ToArray();
                    }
                    aCase!.Lines.Add(aLine);
                    break;
                }
                case "PARTYPE":
                    aLine!.PtXyz = t[1] == "1";
                    aLine.PtU1V1 = t[2] == "1";
                    aLine.PtU2V2 = t[3] == "1";
                    aLine.ParType = (Approx_ParametrizationType)I(t[6]);
                    break;
                case "APPROX":
                    anApprox = new Approx
                    {
                        Tag = t[1][0],
                        Algo = t[2],
                        OnSurfaces = t[3] == "SS",
                        Tol3d = D(t[4]),
                        Tol2d = D(t[5]),
                        DegMin = I(t[6]),
                        DegMax = I(t[7]),
                        NbIter = I(t[8]),
                        NbPntMax = I(t[9]),
                        Tangency = t[10] == "1",
                        ParType = (Approx_ParametrizationType)I(t[11]),
                        Xyz = t[12] == "1",
                        U1V1 = t[13] == "1",
                        U2V2 = t[14] == "1",
                        IMin = I(t[15]),
                        IMax = I(t[16]),
                    };
                    aLine!.Approxes.Add(anApprox);
                    break;
                case "RESULT":
                    if (t[1] == "EXCEPTION")
                    {
                        throw new InvalidDataException("A recorded approximation threw: " + aStr);
                    }
                    anApprox!.IsDone = t[1] == "1";
                    anApprox.TolReached3d = D(t[2]);
                    anApprox.TolReached2d = D(t[3]);
                    anApprox.NbMultiCurves = I(t[4]);
                    break;
                case "MBSP":
                    anApprox!.NbCurves = I(t[1]);
                    anApprox.Degree = I(t[2]);
                    anApprox.NbPoles = I(t[3]);
                    break;
                case "KNOTS":
                    anApprox!.Knots = t.Skip(1).Select(D).ToArray();
                    break;
                case "MULTS":
                    anApprox!.Mults = t.Skip(1).Select(I).ToArray();
                    break;
                case "POLES":
                    anApprox!.Poles.Add((I(t[2]), t.Skip(3).Select(D).ToArray()));
                    break;
                case "KNOTTOOLS":
                    aRec.KnotTools.Add(new KnotToolsRun { MinNbPnts = I(t[1]) });
                    break;
                case "RESULTKNOTS":
                    aRec.KnotTools[^1].Knots = t.Skip(2).Select(I).ToArray();
                    break;
                case "ENDAPPROX":
                case "ENDCASE":
                    break;
                default:
                    throw new InvalidDataException("Unexpected line: " + aStr);
            }
        }
        return aRec;
    }

    internal static Geom_Surface ReadSurface(string theText)
    {
        using StringReader aReader = new(theText);
        GeomTools.Read(out Geom_Surface? aSurf, aReader);
        return aSurf!;
    }

    internal static IntPatch_WLine MakeWLine(Line theLine)
    {
        IntSurf_LineOn2S aLin = new();
        foreach (double[] p in theLine.Points)
        {
            aLin.Add(new IntSurf_PntOn2S().WithValue(new Point3D(p[0], p[1], p[2]), p[3], p[4], p[5], p[6]));
        }
        return new IntPatch_WLine(aLin, false);
    }

    /// <summary>The line of approximation H: degree-1 B-splines through the points, knots 1..n.</summary>
    internal static BRepApprox_ApproxLine MakeCurvesLine(Line theLine)
    {
        int n = theLine.Points.Length;
        NCollection_Array1<Point3D> p3 = new(1, n);
        NCollection_Array1<Point2D> pa = new(1, n), pb = new(1, n);
        NCollection_Array1<double> kn = new(1, n);
        NCollection_Array1<int> mu = new(1, n);
        for (int i = 1; i <= n; i++)
        {
            double[] p = theLine.Points[i - 1];
            p3[i] = new Point3D(p[0], p[1], p[2]);
            pa[i] = new Point2D(p[3], p[4]);
            pb[i] = new Point2D(p[5], p[6]);
            kn[i] = i;
            mu[i] = 1;
        }
        mu[1] = 2;
        mu[n] = 2;
        return new BRepApprox_ApproxLine(new Geom_BSplineCurve(p3, kn, mu, 1), new Geom2d_BSplineCurve(pa, kn, mu, 1),
                                         new Geom2d_BSplineCurve(pb, kn, mu, 1));
    }
}
