// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.IO.Compression;
using System.Text;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Native OCCT's edge intersectors recorded by tools/edgerec (unit 6C, decision 754): IntTools_EdgeEdge and
/// IntTools_EdgeFace on pairs of edges and faces of data/occ shapes, every double as its 64 bits in hex. The port's
/// runs are written in the same format, so a result compares as a string.
/// </summary>
internal static class EdgeIntersectorRecordings
{
    /// <summary>An edge/edge run: the indices of the edges in the case's maps, the variant, native's result.</summary>
    public sealed record EERun(int I, int J, int Variant, string Native);

    /// <summary>
    /// An edge/face run: the indices, whether the intersector localises on the face, native's result with a fresh
    /// context, and native's result with one context for the case's runs in order where that differs (else null).
    /// </summary>
    public sealed record EFRun(int I, int J, bool Localized, string Native, string? NativeShared);

    public sealed class Case
    {
        public required string Name { get; init; }
        public required Shape A { get; init; }
        public required Shape B { get; init; }
        public NCollection_IndexedMap<Shape> EA { get; } = new(ShapeHasher.Instance);
        public NCollection_IndexedMap<Shape> EB { get; } = new(ShapeHasher.Instance);
        public NCollection_IndexedMap<Shape> FB { get; } = new(ShapeHasher.Instance);
        public List<EERun> EE { get; } = [];
        public List<EFRun> EF { get; } = [];
    }

    private static readonly Lazy<Dictionary<string, Case>> theCases = new(Load);

    public static Dictionary<string, Case> Cases => theCases.Value;

    public static IEnumerable<object[]> CaseNames() => Cases.Keys.Select(k => new object[] { k });

    private static double FromHex(string theHex) => BitConverter.Int64BitsToDouble(Convert.ToInt64(theHex, 16));

    private static Dictionary<string, Case> Load()
    {
        using Stream aRes = typeof(EdgeIntersectorRecordings).Assembly.GetManifestResourceStream("IntTools.edgerec.txt.gz")
                            ?? throw new InvalidOperationException("IntTools.edgerec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        Dictionary<string, Case> aCases = [];
        Case? aCase = null;
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            string[] aTok = aLine.Split(' ', 5);
            switch (aTok[0])
            {
                case "CASE":
                    {
                        string[] f = aLine.Split(' ');
                        Shape aA, aB;
                        if (f[2] == "#probes")
                        {
                            string[] aHead = aReader.ReadLine()!.Split(' ');
                            int aNb = int.Parse(aHead[1], System.Globalization.CultureInfo.InvariantCulture);
                            StringBuilder aText = new();
                            for (int i = 0; i < aNb; i++)
                            {
                                aText.Append(aReader.ReadLine()).Append('\n');
                            }
                            BrepTools.Read(out aA, new StringReader(aText.ToString()));
                            aB = BRepMeshModelKit.ReadDataOcc(f[3]);
                        }
                        else
                        {
                            aA = BRepMeshModelKit.ReadDataOcc(f[2]);
                            aB = f[3] == "-" ? aA : BRepMeshModelKit.ReadDataOcc(f[3]);
                            Vector3D aShift = new(FromHex(f[4]), FromHex(f[5]), FromHex(f[6]));
                            if (aShift.SquareMagnitude() != 0.0)
                            {
                                Transform3D aT = new();
                                aT.SetTranslation(aShift);
                                aB = aB.Moved(new Placement(aT));
                            }
                        }
                        aCase = new Case { Name = f[1], A = aA, B = aB };
                        Exploration.MapShapes(aA, global::Kerf.Topology.ShapeType.Edge, aCase.EA);
                        Exploration.MapShapes(aB, global::Kerf.Topology.ShapeType.Edge, aCase.EB);
                        Exploration.MapShapes(aB, global::Kerf.Topology.ShapeType.Face, aCase.FB);
                        aCases.Add(aCase.Name, aCase);
                        break;
                    }
                case "EE":
                    aCase!.EE.Add(new EERun(int.Parse(aTok[1]), int.Parse(aTok[2]), int.Parse(aTok[3]), aTok[4]));
                    break;
                case "EF":
                    aCase!.EF.Add(new EFRun(int.Parse(aTok[1]), int.Parse(aTok[2]), aTok[3] == "1", aTok[4], null));
                    break;
                case "EFS":
                    aCase!.EF[^1] = aCase.EF[^1] with { NativeShared = aLine[4..] };
                    break;
                default:
                    throw new InvalidDataException(aLine);
            }
        }
        return aCases;
    }

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16");

    private static string CommonPart(CommonPrt theCP, in Edge theE1, bool theWithPoints)
    {
        StringBuilder aS = new();
        aS.Append(" | ").Append((int)theCP.Type()).Append(theCP.Edge1().IsSame(theE1) ? " 1" : " 2");
        theCP.Range1(out double f, out double l);
        aS.Append(' ').Append(Hex(f)).Append(' ').Append(Hex(l));
        NCollection_Sequence<ParameterRange> aR2 = theCP.Ranges2();
        aS.Append(' ').Append(aR2.Length());
        for (int k = 1; k <= aR2.Length(); k++)
        {
            aS.Append(' ').Append(Hex(aR2.Value(k).First())).Append(' ').Append(Hex(aR2.Value(k).Last()));
        }
        aS.Append(' ').Append(Hex(theCP.VertexParameter1())).Append(' ').Append(Hex(theCP.VertexParameter2()));
        aS.Append(theCP.AllNullFlag() ? " 1" : " 0");
        if (theWithPoints)
        {
            theCP.BoundingPoints(out Point3D aP1, out Point3D aP2);
            aS.Append(' ').Append(Hex(aP1.X())).Append(' ').Append(Hex(aP1.Y())).Append(' ').Append(Hex(aP1.Z()))
              .Append(' ').Append(Hex(aP2.X())).Append(' ').Append(Hex(aP2.Y())).Append(' ').Append(Hex(aP2.Z()));
        }
        return aS.ToString();
    }

    /// <summary>
    /// An edge/edge run as edgerec makes it: 0 whole ranges; 1 sub-ranges; 2 the quick coincidence check; 3 a fuzzy
    /// value of 1e-4.
    /// </summary>
    public static string RunEE(in Edge theE1, in Edge theE2, int theVariant)
    {
        EdgeEdge anEE = new();
        anEE.SetEdge1(theE1);
        anEE.SetEdge2(theE2);
        if (theVariant == 1)
        {
            Tool.Range(theE1, out double f1, out double l1);
            Tool.Range(theE2, out double f2, out double l2);
            anEE.SetRange1(f1 + 0.125 * (l1 - f1), l1 - 0.0625 * (l1 - f1));
            anEE.SetRange2(f2 + 0.0625 * (l2 - f2), l2 - 0.125 * (l2 - f2));
        }
        if (theVariant == 2)
        {
            anEE.UseQuickCoincidenceCheck(true);
        }
        if (theVariant == 3)
        {
            anEE.SetFuzzyValue(1.0e-4);
        }
        try
        {
            anEE.Perform();
            StringBuilder aS = new();
            aS.Append(anEE.IsDone ? "1" : "0").Append(' ').Append(anEE.CommonParts().Length());
            for (int k = 1; k <= anEE.CommonParts().Length(); k++)
            {
                aS.Append(CommonPart(anEE.CommonParts().Value(k), theE1, false));
            }
            return aS.ToString();
        }
        catch (FailureException anEx)
        {
            return "X " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(anEx.GetType());
        }
    }

    /// <summary>An edge/face run on the edge's range with the context and the fuzzy value (0: the default).</summary>
    public static string RunEF(in Edge theE, in Face theF, IntersectionContext theCtx, double theFuzz = 0.0)
    {
        EdgeFace anEF = new();
        anEF.SetEdge(theE);
        anEF.SetFace(theF);
        Tool.Range(theE, out double f, out double l);
        anEF.SetRange(f, l);
        anEF.SetContext(theCtx);
        if (theFuzz > 0.0)
        {
            anEF.SetFuzzyValue(theFuzz);
        }
        try
        {
            anEF.Perform();
            StringBuilder aS = new();
            aS.Append(anEF.IsDone ? "1" : "0").Append(' ').Append(anEF.ErrorStatus()).Append(' ')
              .Append(Hex(anEF.MinimalDistance())).Append(' ').Append(anEF.CommonParts().Length());
            for (int k = 1; k <= anEF.CommonParts().Length(); k++)
            {
                aS.Append(CommonPart(anEF.CommonParts().Value(k), theE, true));
            }
            return aS.ToString();
        }
        catch (FailureException anEx)
        {
            return "X " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(anEx.GetType());
        }
    }

    public static Edge Edge(Case theCase, NCollection_IndexedMap<Shape> theMap, int theIndex) =>
        ShapeCasts.Edge(theMap.FindKey(theIndex));

    public static Face Face(Case theCase, int theIndex) => ShapeCasts.Face(theCase.FB.FindKey(theIndex));
}
