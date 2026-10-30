// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Text;
using Xunit.Abstractions;

namespace OcctNet.ModelingAlgorithms.Tests;

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

        public gp_Pnt P()
        {
            double x = D(), y = D(), z = D();
            return new gp_Pnt(x, y, z);
        }

        public gp_Dir Dir()
        {
            double x = D(), y = D(), z = D();
            return new gp_Dir(x, y, z);
        }

        public bool AtEnd => myPos == theIn.Length;
    }

    private sealed class EdgeData
    {
        public int Type;
        public gp_Pnt P1, P2, Center;
        public gp_Dir N = new(), X = new();
        public double R1, R2;
        public gp_Pnt[] Poles = [];
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

    private static TopoDS_Edge MakeEdge(EdgeData d, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        BRepBuilderAPI_MakeEdge aMaker;
        switch (d.Type)
        {
            case 0:
                aMaker = new BRepBuilderAPI_MakeEdge(V1, V2);
                break;
            case 1:
                aMaker = new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(d.Center, d.N, d.X), d.R1), V1, V2);
                break;
            case 2:
                aMaker = new BRepBuilderAPI_MakeEdge(new gp_Elips(new gp_Ax2(d.Center, d.N, d.X), d.R1, d.R2), V1, V2);
                break;
            default:
                {
                    NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
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
                    var aC = new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
                    aMaker = new BRepBuilderAPI_MakeEdge(aC, V1, V2);
                    break;
                }
        }
        return aMaker.IsDone() ? aMaker.Edge() : new TopoDS_Edge();
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x", CultureInfo.InvariantCulture);

    private static string Hex(in gp_Pnt theP) => Hex(theP.X()) + "," + Hex(theP.Y()) + "," + Hex(theP.Z());

    private static string ExcName(Exception theE) => "!" + theE.GetType().Name;

    private static string Desc(in TopoDS_Edge theE)
    {
        if (theE.IsNull())
        {
            return "null";
        }
        Geom_Curve? aC = BRep_Tool.Curve(theE, out double f, out double l);
        if (aC is null)
        {
            return "nocurve";
        }
        string s = aC.GetType().Name;
        s += "," + Hex(f) + "," + Hex(l) + "," + Hex(aC.Value(f)) + "," + Hex(aC.Value(0.5 * (f + l))) + "," + Hex(aC.Value(l));
        s += theE.Orientation() == TopAbs_Orientation.TopAbs_FORWARD ? ",F"
           : theE.Orientation() == TopAbs_Orientation.TopAbs_REVERSED ? ",R" : ",O";
        TopExp.Vertices(theE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        s += "," + (aV1.IsNull() ? "nv" : Hex(BRep_Tool.Pnt(aV1)));
        s += "," + (aV2.IsNull() ? "nv" : Hex(BRep_Tool.Pnt(aV2)));
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
            catch (Standard_Failure e)
            {
                aVal = ExcName(e);
            }
            aOut.Add(theName + ":" + aVal);
        }

        bool isFillet = theCase.Kind[0] == 'f';
        bool isWire = theCase.Kind[1] == 'w';
        var r = new Reader(theCase.Inputs);
        gp_Pnt aLoc = r.P();
        gp_Dir aN = r.Dir(), aX = r.Dir();
        bool isDirect = r.B();
        var anAx3 = new gp_Ax3(aLoc, aN, aX);
        if (!isDirect)
        {
            anAx3.YReverse();
        }
        var aPln = new gp_Pln(anAx3);
        bool isShared = r.B(), isRev1 = r.B(), isRev2 = r.B();
        gp_Pnt aC = r.P();
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

        TopoDS_Vertex aV11 = new BRepBuilderAPI_MakeVertex(d1.P1).Vertex();
        TopoDS_Vertex aV12 = new BRepBuilderAPI_MakeVertex(d1.P2).Vertex();
        TopoDS_Vertex aV21 = isShared ? aV12 : new BRepBuilderAPI_MakeVertex(d2.P1).Vertex();
        TopoDS_Vertex aV22 = new BRepBuilderAPI_MakeVertex(d2.P2).Vertex();

        TopoDS_Edge anE1 = new(), anE2 = new();
        try
        {
            anE1 = MakeEdge(d1, aV11, aV12);
            anE2 = MakeEdge(d2, aV21, aV22);
        }
        catch (Standard_Failure e)
        {
            aOut.Add("edges:" + ExcName(e));
        }
        if (anE1.IsNull() || anE2.IsNull())
        {
            aOut.Add("edges:0");
            return aOut;
        }
        if (isRev1)
        {
            anE1 = TopoDS.Edge(anE1.Reversed());
        }
        if (isRev2)
        {
            anE2 = TopoDS.Edge(anE2.Reversed());
        }
        TopoDS_Wire aWire = new();
        if (isWire)
        {
            var aMW = new BRepBuilderAPI_MakeWire(anE1, anE2);
            if (!aMW.IsDone())
            {
                aOut.Add("wire:0");
                return aOut;
            }
            aWire = aMW.Wire();
        }

        if (isFillet)
        {
            var anAlgo = new ChFi2d_FilletAPI();
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
                    TopoDS_Edge aM1 = new(), aM2 = new();
                    bool isRes = false;
                    Tok(iSol == -1 ? "res" : (iSol == 1 ? "res1" : "res2"), () =>
                    {
                        TopoDS_Edge aF = anAlgo.Result(aC, ref aM1, ref aM2, iSol);
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
            var anAlgo = new ChFi2d_ChamferAPI();
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
                TopoDS_Edge aM1 = new(), aM2 = new();
                bool isRes = false;
                Tok("res", () =>
                {
                    TopoDS_Edge aCh = anAlgo.Result(ref aM1, ref aM2, aL1, aL2);
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
                aDifferences.Add($"{theKind} #{i}: the port threw {e.GetType().Name}: {e.Message}");
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
