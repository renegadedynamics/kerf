// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Planar;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ChFi2d_FilletAPI (and through it ChFi2d_AnaFilletAlgo and ChFi2d_FilletAlgo) and ChFi2d_ChamferAPI against native
/// OCCT's results recorded once by tools/chfi2drec (unit 5R): random pairs of edges of a plane (lines, arcs of circles
/// and ellipses, cubic B-splines; sharing a vertex, meeting at a point, or a small gap apart; some reversed; in the XY
/// plane or a random one, some left-handed), given as a wire or as two edges, with random radii and lengths. Each
/// case's edges are rebuilt from the recorded doubles; Perform, NbResults, Result (nearest, first and second solution)
/// and the trimmed edges are compared token by token: each edge's curve type, range, points at its ends and middle,
/// orientation and vertices, every double as its 64 bits. The recorder builds its directions as the port reads them
/// (gp_Dir of the recorded components, normalised again), so both sides start from the same bits.
/// </summary>
public class ChFi2d_NativeTests(ITestOutputHelper theOutput)
{
    public sealed record Case(string Kind, double[] Inputs, string[] Native);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    private static List<Case> Load()
    {
        using Stream aRes = typeof(ChFi2d_NativeTests).Assembly.GetManifestResourceStream("ChFi2d.chfi2drec.txt.gz")
                            ?? throw new InvalidOperationException("ChFi2d.chfi2drec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        List<Case> aCases = [];
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            int aSemi = aLine.IndexOf(';', StringComparison.Ordinal);
            string[] aIn = aLine[..aSemi].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            double[] aValues = aIn.Skip(1)
                                  .Select(s => BitConverter.Int64BitsToDouble(unchecked((long)ulong.Parse(s, NumberStyles.HexNumber,
                                                                                                         CultureInfo.InvariantCulture))))
                                  .ToArray();
            string[] aOut = aLine[(aSemi + 1)..].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            aCases.Add(new Case(aIn[0], aValues, aOut));
        }
        return aCases;
    }

    public static IEnumerable<object[]> Kinds() => [["fw"], ["fe"], ["cw"], ["ce"]];

    // ---- the port's side: chfi2drec's RunCase ----

    private sealed class Reader(double[] theIn)
    {
        private int myPos;

        public double D() => theIn[myPos++];

        public bool B() => D() != 0.0;

        public Point3D P()
        {
            double x = D(), y = D(), z = D();
            return new Point3D(x, y, z);
        }

        public Direction3D Dir()
        {
            double x = D(), y = D(), z = D();
            return new Direction3D(x, y, z);
        }

        public bool AtEnd => myPos == theIn.Length;
    }

    private sealed class EdgeData
    {
        public int Type;
        public Point3D P1, P2, Center;
        public Direction3D N = new(), X = new();
        public double R1, R2;
        public Point3D[] Poles = [];
    }

    private static EdgeData ReadEdge(Reader r)
    {
        var d = new EdgeData { Type = (int)r.D() };
        if (d.Type == 0)
        {
            d.P1 = r.P();
            d.P2 = r.P();
        }
        else if (d.Type is 1 or 2)
        {
            d.Center = r.P();
            d.N = r.Dir();
            d.X = r.Dir();
            d.R1 = r.D();
            if (d.Type == 2)
            {
                d.R2 = r.D();
            }
            d.P1 = r.P();
            d.P2 = r.P();
        }
        else
        {
            d.Poles = [r.P(), r.P(), r.P(), r.P()];
            d.P1 = d.Poles[0];
            d.P2 = d.Poles[3];
        }
        return d;
    }

    private static Edge MakeEdge(EdgeData d, in Vertex V1, in Vertex V2)
    {
        MakeEdge aMaker;
        switch (d.Type)
        {
            case 0:
                aMaker = new MakeEdge(V1, V2);
                break;
            case 1:
                aMaker = new MakeEdge(new Circle3D(new Frame3D(d.Center, d.N, d.X), d.R1), V1, V2);
                break;
            case 2:
                aMaker = new MakeEdge(new Ellipse3D(new Frame3D(d.Center, d.N, d.X), d.R1, d.R2), V1, V2);
                break;
            default:
                {
                    NCollection_Array1<Point3D> aPoles = new(1, 4);
                    for (int i = 1; i <= 4; i++)
                    {
                        aPoles[i] = d.Poles[i - 1];
                    }
                    NCollection_Array1<double> aKnots = new(1, 2);
                    aKnots[1] = 0.0;
                    aKnots[2] = 1.0;
                    NCollection_Array1<int> aMults = new(1, 2);
                    aMults[1] = 4;
                    aMults[2] = 4;
                    var aC = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
                    aMaker = new MakeEdge(aC, V1, V2);
                    break;
                }
        }
        return aMaker.IsDone ? aMaker.Edge() : new Edge();
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x", CultureInfo.InvariantCulture);

    private static string Hex(in Point3D theP) => Hex(theP.X) + "," + Hex(theP.Y) + "," + Hex(theP.Z);

    private static string ExcName(Exception theE) => "!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theE.GetType());

    private static string Desc(in Edge theE)
    {
        if (theE.IsNull)
        {
            return "null";
        }
        Curve3D? aC = Tool.Curve(theE, out double f, out double l);
        if (aC is null)
        {
            return "nocurve";
        }
        string s = global::Kerf.Engine.Foundation.OcctTypeNames.Of(aC.GetType());
        s += "," + Hex(f) + "," + Hex(l) + "," + Hex(aC.Value(f)) + "," + Hex(aC.Value(0.5 * (f + l))) + "," + Hex(aC.Value(l));
        s += theE.Orientation == Orientation.Forward ? ",F"
           : theE.Orientation == Orientation.Reversed ? ",R" : ",O";
        Exploration.Vertices(theE, out Vertex aV1, out Vertex aV2);
        s += "," + (aV1.IsNull ? "nv" : Hex(Tool.Pnt(aV1)));
        s += "," + (aV2.IsNull ? "nv" : Hex(Tool.Pnt(aV2)));
        return s;
    }

    /// <summary>Replays a case on the port: the tokens chfi2drec writes after the ';'.</summary>
    internal static List<string> Run(Case theCase)
    {
        var aOut = new List<string>();
        void Tok(string theName, Func<string> theF)
        {
            string aVal;
            try
            {
                aVal = theF();
            }
            catch (Exception e) when (Refusal.IsFailure(e))
            {
                aVal = ExcName(e);
            }
            aOut.Add(theName + ":" + aVal);
        }

        bool isFillet = theCase.Kind[0] == 'f';
        bool isWire = theCase.Kind[1] == 'w';
        var r = new Reader(theCase.Inputs);
        Point3D aLoc = r.P();
        Direction3D aN = r.Dir(), aX = r.Dir();
        bool isDirect = r.B();
        var anAx3 = new CoordinateSystem(aLoc, aN, aX);
        if (!isDirect)
        {
            anAx3 = anAx3.YReversed();
        }
        var aPln = new Plane3D(anAx3);
        bool isShared = r.B(), isRev1 = r.B(), isRev2 = r.B();
        Point3D aC = r.P();
        EdgeData d1 = ReadEdge(r), d2 = ReadEdge(r);
        double aRadius = 0.0, aL1 = 0.0, aL2 = 0.0;
        if (isFillet)
        {
            aRadius = r.D();
        }
        else
        {
            aL1 = r.D();
            aL2 = r.D();
        }
        Assert.True(r.AtEnd, "inputs not all read");

        Vertex aV11 = new MakeVertex(d1.P1).Vertex();
        Vertex aV12 = new MakeVertex(d1.P2).Vertex();
        Vertex aV21 = isShared ? aV12 : new MakeVertex(d2.P1).Vertex();
        Vertex aV22 = new MakeVertex(d2.P2).Vertex();

        Edge anE1 = new(), anE2 = new();
        try
        {
            anE1 = MakeEdge(d1, aV11, aV12);
            anE2 = MakeEdge(d2, aV21, aV22);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            aOut.Add("edges:" + ExcName(e));
        }
        if (anE1.IsNull || anE2.IsNull)
        {
            aOut.Add("edges:0");
            return aOut;
        }
        if (isRev1)
        {
            anE1 = ShapeCasts.Edge(anE1.Reversed());
        }
        if (isRev2)
        {
            anE2 = ShapeCasts.Edge(anE2.Reversed());
        }
        Wire aWire = new();
        if (isWire)
        {
            var aMW = new MakeWire(anE1, anE2);
            if (!aMW.IsDone)
            {
                aOut.Add("wire:0");
                return aOut;
            }
            aWire = aMW.Wire();
        }

        if (isFillet)
        {
            var anAlgo = new FilletApi();
            bool isInit = false;
            Tok("init", () =>
            {
                if (isWire)
                {
                    anAlgo.Init(aWire, aPln);
                }
                else
                {
                    anAlgo.Init(anE1, anE2, aPln);
                }
                isInit = true;
                return "1";
            });
            if (!isInit)
            {
                return aOut;
            }
            bool isDone = false;
            Tok("perf", () =>
            {
                isDone = anAlgo.Perform(aRadius);
                return isDone ? "1" : "0";
            });
            if (isDone)
            {
                Tok("nb", () => anAlgo.NbResults(aC).ToString(CultureInfo.InvariantCulture));
                foreach (int iSol in new[] { -1, 1, 2 })
                {
                    Edge aM1 = new(), aM2 = new();
                    bool isRes = false;
                    Tok(iSol == -1 ? "res" : (iSol == 1 ? "res1" : "res2"), () =>
                    {
                        Edge aF = anAlgo.Result(aC, ref aM1, ref aM2, iSol);
                        isRes = true;
                        return Desc(aF);
                    });
                    if (isRes)
                    {
                        aOut.Add("m1:" + Desc(aM1));
                        aOut.Add("m2:" + Desc(aM2));
                    }
                }
            }
        }
        else
        {
            var anAlgo = new ChamferApi();
            if (isWire)
            {
                anAlgo.Init(aWire);
            }
            else
            {
                anAlgo.Init(anE1, anE2);
            }
            bool isDone = false;
            Tok("perf", () =>
            {
                isDone = anAlgo.Perform();
                return isDone ? "1" : "0";
            });
            if (isDone)
            {
                Edge aM1 = new(), aM2 = new();
                bool isRes = false;
                Tok("res", () =>
                {
                    Edge aCh = anAlgo.Result(ref aM1, ref aM2, aL1, aL2);
                    isRes = true;
                    return Desc(aCh);
                });
                if (isRes)
                {
                    aOut.Add("m1:" + Desc(aM1));
                    aOut.Add("m2:" + Desc(aM2));
                }
            }
        }
        return aOut;
    }

    /// <summary>Every case gives native's tokens, bit for bit (1,200 cases; all identical when recorded).</summary>
    [Theory]
    [MemberData(nameof(Kinds))]
    public void Port_matches_native_recordings(string theKind)
    {
        List<Case> aCases = theCases.Value.Where(c => c.Kind == theKind).ToList();
        Assert.NotEmpty(aCases);
        int aIdentical = 0, aTokens = 0;
        var aDifferences = new List<string>();
        for (int i = 0; i < aCases.Count; i++)
        {
            Case aCase = aCases[i];
            List<string> aPort;
            try
            {
                aPort = Run(aCase);
            }
            catch (Exception e)
            {
                aDifferences.Add($"{theKind} #{i}: the port threw {Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(e).GetType())}: {Refusal.Unwrapped(e).Message}");
                continue;
            }
            aTokens += aCase.Native.Length;
            if (aPort.SequenceEqual(aCase.Native))
            {
                aIdentical++;
                continue;
            }
            int k = 0;
            while (k < Math.Min(aPort.Count, aCase.Native.Length) && aPort[k] == aCase.Native[k])
            {
                k++;
            }
            aDifferences.Add($"{theKind} #{i}: native {(k < aCase.Native.Length ? aCase.Native[k] : "(end)")}, "
                             + $"port {(k < aPort.Count ? aPort[k] : "(end)")}");
        }
        theOutput.WriteLine($"{theKind}: {aCases.Count} cases ({aTokens} tokens): {aIdentical} identical token for token, "
                            + $"{aDifferences.Count} different");
        foreach (string aDiff in aDifferences.Take(40))
        {
            theOutput.WriteLine("  " + aDiff);
        }
        Assert.Empty(aDifferences);
    }
}
