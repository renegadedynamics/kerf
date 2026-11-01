// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Text;
using Xunit.Abstractions;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepFeat's makers that build123d.NET doesn't bind (MakePrism, MakeRevol, MakePipe, MakeCylindricalHole, Gluer)
/// against native OCCT's recording (tools/featrec): the same operations on the same inputs (read from the recording's
/// BRep texts, frozen as a caller's), compared by the result's BRep text (Free and Checked masked), its canonical record
/// (decision 1017), the status and the history (IsDeleted and the number of Modified faces of the basis shape's faces,
/// Generated of the profile's edges); every input is checked unchanged and every result frozen.
/// </summary>
public class BRepFeatNativeTests(ITestOutputHelper output)
{
    private static List<string> Recording()
    {
        using Stream aRes = typeof(BRepFeatNativeTests).Assembly.GetManifestResourceStream("BRepFeat.featrec.txt.gz")
                            ?? throw new InvalidOperationException("no recording");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz);
        var aLines = new List<string>();
        while (aReader.ReadLine() is string aLine)
        {
            aLines.Add(aLine);
        }
        return aLines;
    }

    private static ulong Fnv(string theText)
    {
        ulong aHash = 14695981039346656037ul;
        foreach (byte c in Encoding.UTF8.GetBytes(theText))
        {
            aHash ^= c;
            aHash *= 1099511628211ul;
        }
        return aHash;
    }

    private static string Hex(ulong theValue) => theValue.ToString("x16", CultureInfo.InvariantCulture);

    private static string Text(in TopoDS_Shape theShape)
    {
        var aW = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BRepTools.Write(theShape, aW, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3);
        return aW.ToString();
    }

    private static string Masked(string theText)
    {
        var aRes = new StringBuilder();
        foreach (string aLine0 in theText.Split('\n'))
        {
            char[] aLine = aLine0.TrimEnd('\r').ToCharArray();
            if (aLine.Length == 7 && aLine.All(c => c is '0' or '1'))
            {
                aLine[0] = 'x';
                aLine[2] = 'x';
            }
            aRes.Append(aLine).Append('\n');
        }
        // the recorder's getline loop gives no line after the last newline
        string s = aRes.ToString();
        return s.EndsWith("\n\n", StringComparison.Ordinal) ? s[..^1] : s;
    }

    private static string Q(double theValue) =>
        ((long)Math.Round(theValue * 1e6, MidpointRounding.AwayFromZero)).ToString(CultureInfo.InvariantCulture) + ";";

    private static string P(gp_Pnt p) => Q(p.X()) + Q(p.Y()) + Q(p.Z());

    private static List<TopoDS_Shape> Map(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        var aList = new List<TopoDS_Shape>();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    private static string Coarse(in TopoDS_Shape theShape)
    {
        var aLines = new List<string>();
        foreach (TopoDS_Shape aF in Map(theShape, TopAbs_FACE))
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(aF, aProps);
            aLines.Add($"F {Q(aProps.Mass())} {Map(aF, TopAbs_EDGE).Count}");
        }
        foreach (TopoDS_Shape anE in Map(theShape, TopAbs_EDGE))
        {
            TopExp.Vertices(TopoDS.Edge(anE), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            string[] anEnds = [aV1.IsNull() ? "" : P(BRep_Tool.Pnt(aV1)), aV2.IsNull() ? "" : P(BRep_Tool.Pnt(aV2))];
            if (string.CompareOrdinal(anEnds[1], anEnds[0]) < 0)
            {
                (anEnds[0], anEnds[1]) = (anEnds[1], anEnds[0]);
            }
            aLines.Add($"E {Q(BRep_Tool.Tolerance(TopoDS.Edge(anE)))} {anEnds[0]} {anEnds[1]}");
        }
        foreach (TopoDS_Shape aV in Map(theShape, TopAbs_VERTEX))
        {
            aLines.Add($"V {P(BRep_Tool.Pnt(TopoDS.Vertex(aV)))} {Q(BRep_Tool.Tolerance(TopoDS.Vertex(aV)))}");
        }
        aLines.Sort(StringComparer.Ordinal);
        var aVolume = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aVolume);
        var aJoined = new StringBuilder();
        foreach (string aL in aLines)
        {
            aJoined.Append(aL).Append('\n');
        }
        aJoined.Append("volume ").Append(Q(aVolume.Mass()));
        return Hex(Fnv(aJoined.ToString()));
    }

    private sealed class Tally
    {
        public int Compared, Identical, Coarse, Unchanged;
        public readonly List<string> Differences = [];
    }

    // Compares one record: native's "<done> <status> <hash> <coarse> <history>"
    private static void Compare(Tally theTally, string theWhat, string theNative, string thePort)
    {
        theTally.Compared++;
        if (theNative == thePort)
        {
            theTally.Identical++;
            return;
        }
        string[] aN = theNative.Split(' '), aP = thePort.Split(' ');
        // the BRep texts differ but the canonical records agree (and everything else)
        if (aN.Length == aP.Length && aN.Length > 3 && aN.Where((w, i) => i != 2).SequenceEqual(aP.Where((w, i) => i != 2)))
        {
            theTally.Coarse++;
            return;
        }
        theTally.Differences.Add($"{theWhat}: native {theNative}, port {thePort}");
    }

    private static string Result(BRepBuilderAPI_MakeShape theMaker, string theStatus, List<TopoDS_Shape> theFaces,
                                 List<TopoDS_Shape> theEdges)
    {
        if (!theMaker.IsDone())
        {
            return "0 " + theStatus;
        }
        TopoDS_Shape aRes = theMaker.Shape();
        if (!aRes.IsNull() && !aRes.TShape()!.IsFrozen())
        {
            throw new InvalidOperationException("the result isn't frozen");
        }
        var aLine = new StringBuilder("1 " + theStatus + " " + (aRes.IsNull() ? "NULL NULL" : Hex(Fnv(Masked(Text(aRes)))) + " " + Coarse(aRes)) + " ");
        foreach (TopoDS_Shape aF in theFaces)
        {
            aLine.Append(theMaker.IsDeleted(aF) ? "d " : theMaker.Modified(aF).Count.ToString(CultureInfo.InvariantCulture) + " ");
        }
        aLine.Append('|');
        foreach (TopoDS_Shape anE in theEdges)
        {
            aLine.Append(' ').Append(theMaker.Generated(anE).Count.ToString(CultureInfo.InvariantCulture));
        }
        return aLine.ToString();
    }

    private static string Run(Func<string> theRun)
    {
        try
        {
            return theRun();
        }
        catch (Standard_Failure e)
        {
            return "THROW " + e.OcctType;
        }
    }

    [Fact]
    public void The_makers_on_primitives_with_profiles_as_native()
    {
        List<string> aLines = Recording();
        var anInputs = new Dictionary<int, TopoDS_Shape>();
        var aTexts = new Dictionary<int, string>();
        var aParams = new Dictionary<int, double[]>();
        var aTally = new Tally();
        for (int i = 0; i < aLines.Count; i++)
        {
            string aLine = aLines[i];
            if (aLine.StartsWith("INPUT ", StringComparison.Ordinal))
            {
                int k = int.Parse(aLine.Split(' ')[1], CultureInfo.InvariantCulture);
                var aText = new StringBuilder();
                for (i++; aLines[i] != "END"; i++)
                {
                    aText.Append(aLines[i]).Append('\n');
                }
                BRepTools.Read(out TopoDS_Shape aShape, new StringReader(aText.ToString()));
                aShape.TShape()!.FreezeGraph(); // a caller's shape
                anInputs[k] = aShape;
                aTexts[k] = Text(aShape);
                continue;
            }
            if (aLine.StartsWith("PARAMS ", StringComparison.Ordinal))
            {
                string[] aW = aLine.Split(' ');
                aParams[int.Parse(aW[1], CultureInfo.InvariantCulture)] = aW.Skip(2).Select(w => double.Parse(w, CultureInfo.InvariantCulture)).ToArray();
                continue;
            }
            string[] aParts = aLine.Split(" => ");
            string[] aHead = aParts[0].Split(' ');
            int aK = int.Parse(aHead[1], CultureInfo.InvariantCulture);
            TopoDS_Shape anIn = anInputs[aK];
            double[] aPar = aParams[aK];
            var aC = new gp_Pnt(aPar[0], aPar[1], aPar[2]);
            var aN = new gp_Dir(aPar[3], aPar[4], aPar[5]);
            var aX = new gp_Dir(aPar[6], aPar[7], aPar[8]);
            double aSize = aPar[9];
            var anIt = new TopoDS_Iterator(anIn);
            TopoDS_Shape aS = anIt.Value();
            anIt.Next();
            TopoDS_Shape aSecond = anIt.Value();
            List<TopoDS_Shape> aFaces = Map(aS, TopAbs_FACE);
            string aPort;
            switch (aHead[0])
            {
                case "PRISM":
                case "REVOL":
                case "PIPE":
                {
                    TopoDS_Face aP = TopoDS.Face(aSecond);
                    List<TopoDS_Shape> anEdges = Map(aP, TopAbs_EDGE);
                    int iF = int.Parse(aLines.Take(i).Last(l => l.StartsWith("INPUT " + aK + " ", StringComparison.Ordinal)).Split(' ')[2].Split('.')[1], CultureInfo.InvariantCulture);
                    TopoDS_Face aSk = TopoDS.Face(aFaces[iF]);
                    int iUntil = -1;
                    for (int j = aFaces.Count - 1; j >= 0; j--)
                    {
                        if (j != iF && new BRepAdaptor_Surface(TopoDS.Face(aFaces[j])).GetType() == GeomAbs_SurfaceType.GeomAbs_Plane)
                        {
                            iUntil = j;
                            break;
                        }
                    }
                    int aFuse = int.Parse(aHead[3], CultureInfo.InvariantCulture);
                    bool isMod = aHead[4] == "1";
                    string anOp = aHead[2];
                    aPort = aHead[0] switch
                    {
                        "PRISM" => Run(() =>
                        {
                            var aM = new BRepFeat_MakePrism(aS, aP, aSk, aFuse != 0 ? aN : aN.Reversed(), aFuse, isMod);
                            switch (anOp)
                            {
                                case "length": aM.Perform(5.0); break;
                                case "thruall": aM.PerformThruAll(); break;
                                case "until": aM.Perform(aFaces[iUntil]); break;
                                case "untilend": aM.PerformUntilEnd(); break;
                                default: aM.PerformFromEnd(aFaces[iUntil]); break;
                            }
                            return Result(aM, ((int)aM.CurrentStatusError()).ToString(CultureInfo.InvariantCulture), aFaces, anEdges);
                        }),
                        "REVOL" => Run(() =>
                        {
                            var anAxis = new gp_Ax1(aC.Translated(new gp_Vec(aN.Crossed(aX)) * (2.5 * aSize)), aX);
                            var aM = new BRepFeat_MakeRevol(aS, aP, aSk, anAxis, aFuse, isMod);
                            switch (anOp)
                            {
                                case "angle": aM.Perform(30 * Math.PI / 180); break;
                                case "thruall": aM.PerformThruAll(); break;
                                default: aM.Perform(aFaces[iUntil]); break;
                            }
                            return Result(aM, ((int)aM.CurrentStatusError()).ToString(CultureInfo.InvariantCulture), aFaces, anEdges);
                        }),
                        _ => Run(() =>
                        {
                            gp_Dir aDir = aFuse != 0 ? aN : aN.Reversed();
                            TopoDS_Edge aSpine = new BRepBuilderAPI_MakeEdge(aC, aC.Translated(new gp_Vec(aDir) * 5.0)).Edge();
                            TopoDS_Wire aW = new BRepBuilderAPI_MakeWire(aSpine).Wire();
                            var aM = new BRepFeat_MakePipe(aS, aP, aSk, aW, aFuse, isMod);
                            aM.Perform();
                            return Result(aM, ((int)aM.CurrentStatusError()).ToString(CultureInfo.InvariantCulture), aFaces, anEdges);
                        }),
                    };
                    break;
                }
                case "HOLE":
                {
                    int aMode = int.Parse(aHead[2], CultureInfo.InvariantCulture);
                    aPort = Run(() =>
                    {
                        var aM = new BRepFeat_MakeCylindricalHole();
                        aM.Init(aS, new gp_Ax1(aC.Translated(new gp_Vec(aN) * 1.0), aN.Reversed()));
                        switch (aMode)
                        {
                            case 0: aM.Perform(aSize / 2); break;
                            case 1: aM.PerformThruNext(aSize / 2); break;
                            case 2: aM.PerformUntilEnd(aSize / 2); break;
                            case 3: aM.PerformBlind(aSize / 2, 4.0); break;
                            default: aM.Perform(aSize / 2, 0.5, 4.0); break;
                        }
                        aM.Build();
                        string aRes = $"{(int)aM.Status()} {(aM.HasErrors() ? 1 : 0)}";
                        if (aM.Status() == BRepFeat_Status.BRepFeat_NoError && !aM.HasErrors())
                        {
                            TopoDS_Shape aShape = aM.Shape();
                            aRes += " " + (aShape.IsNull() ? "NULL NULL" : Hex(Fnv(Masked(Text(aShape)))) + " " + Coarse(aShape));
                        }
                        return aRes;
                    });
                    break;
                }
                default: // GLUE
                {
                    aPort = Run(() =>
                    {
                        var aG = new BRepFeat_Gluer(aSecond, aS);
                        TopoDS_Face aFnew = new();
                        foreach (TopoDS_Shape aF in Map(aSecond, TopAbs_FACE))
                        {
                            var aProps = new GProp_GProps();
                            BRepGProp.SurfaceProperties(aF, aProps);
                            if (aProps.CentreOfMass().Distance(aC) < 1e-7)
                            {
                                aFnew = TopoDS.Face(aF);
                            }
                        }
                        aG.Bind(aFnew, TopoDS.Face(aFaces[0]));
                        var aFE = new LocOpe_FindEdges(aFnew, aFaces[0]);
                        for (aFE.InitIterator(); aFE.More(); aFE.Next())
                        {
                            aG.Bind(aFE.EdgeFrom(), aFE.EdgeTo());
                        }
                        aG.Build();
                        return ((int)aG.OpeType()).ToString(CultureInfo.InvariantCulture) + " " + Result(aG, "-", aFaces, []);
                    });
                    break;
                }
            }
            Compare(aTally, aParts[0], aParts[1], aPort);
            if (Text(anIn) == aTexts[aK])
            {
                aTally.Unchanged++;
            }
            else
            {
                aTally.Differences.Add($"{aParts[0]}: the input changed");
            }
        }
        output.WriteLine($"{aTally.Compared} records: {aTally.Identical} identical, {aTally.Coarse} equal by the canonical record only, "
                         + $"{aTally.Differences.Count} different; inputs unchanged {aTally.Unchanged}");
        foreach (string aD in aTally.Differences.Take(80))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Empty(aTally.Differences);
    }
}
