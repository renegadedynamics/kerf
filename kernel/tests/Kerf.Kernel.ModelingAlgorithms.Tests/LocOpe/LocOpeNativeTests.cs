// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using Kerf.Kernel.ModelingData.Tests;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// LocOpe's local operations against native OCCT's recordings (tools/locoperec, unit 7H) on primitives, data/occ's
/// shapes up to 60 kB and the feat group's data files: LocOpe_SplitShape (vertices on edges, open and closed wires on
/// planar faces), LocOpe_WiresOnShape with LocOpe_Spliter (bound and added iso edges), LocOpe_Prism, LocOpe_Revol,
/// LocOpe_LinearForm, LocOpe_DPrism, LocOpe_FindEdges and LocOpe_FindEdgesInFace, with their descendants. The port reads
/// each input frozen, as a caller's, runs every tool on it and writes the recorder's records; every input is checked
/// unchanged and every result frozen. See tools/locoperec/main.cpp for the records.
/// </summary>
public class LocOpeNativeTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(LocOpeNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static List<(string Name, List<string> Lines)> Sections()
    {
        using Stream aRes = typeof(LocOpeNativeTests).Assembly.GetManifestResourceStream("LocOpe.locoperec.txt.gz")
                            ?? throw new InvalidOperationException("LocOpe.locoperec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var aSections = new List<(string, List<string>)>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            if (aLine.StartsWith("FILE ", StringComparison.Ordinal))
            {
                aSections.Add((aLine[5..], []));
            }
            else
            {
                aSections[^1].Item2.Add(aLine);
            }
        }
        return aSections;
    }

    private static string Hex(ulong theValue) => theValue.ToString("x16", CultureInfo.InvariantCulture);

    private static string D(double theValue) => Hex(BitConverter.DoubleToUInt64Bits(theValue));

    private static ulong Fnv(string theText)
    {
        ulong aHash = 14695981039346656037UL;
        foreach (char c in theText)
        {
            aHash ^= (byte)c;
            aHash *= 1099511628211UL;
        }
        return aHash;
    }

    // OCCTNET_LOCOPE_DUMP: a directory the shapes' texts are written to, named as the recorder's (LOCOPEREC_DUMP)
    private static readonly string? s_dump = Environment.GetEnvironmentVariable("OCCTNET_LOCOPE_DUMP");

    [ThreadStatic]
    private static string? s_tag;

    [ThreadStatic]
    private static int s_nbDumped;

    private static ulong ShapeHash(in Shape theShape)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theShape, aS, true, false, FormatVersion.FormatVersion_VERSION_3);
        if (s_dump is not null && s_tag is not null)
        {
            File.WriteAllText(Path.Combine(s_dump, $"{s_tag}.{s_nbDumped++}.brep"), aS.ToString());
        }
        return ShapeUpgradeNativeTests.Hash(aS.ToString());
    }

    private static int Count(in Shape theShape, ShapeType theType) =>
        theShape.IsNull ? 0 : LocOpeTests.Count(theShape, theType);

    private static string Rec(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return "NULL";
        }
        if (!theShape.TShape()!.IsFrozen)
        {
            throw new InvalidOperationException("a result isn't frozen");
        }
        return string.Create(CultureInfo.InvariantCulture,
                             $"{Hex(ShapeHash(theShape))} {Count(theShape, ShapeType.Face)} {Count(theShape, ShapeType.Edge)} {Count(theShape, ShapeType.Vertex)}");
    }

    private static string SetHash(NCollection_List<Shape> theList)
    {
        var aHashes = new List<string>();
        foreach (Shape aS in theList)
        {
            aHashes.Add(Hex(ShapeHash(aS)));
        }
        aHashes.Sort(StringComparer.Ordinal);
        return Hex(Fnv(string.Concat(aHashes.Select(h => h + ","))));
    }

    [ThreadStatic]
    private static string? s_name;

    private static string? Run(string theTool, int theIndex, Func<string?> theRun)
    {
        s_tag = $"{s_name}.{theTool}.{theIndex}";
        s_nbDumped = 0;
        string? aLine;
        try
        {
            aLine = theRun();
        }
        catch (Exception e) when ((Refusal.IsFailure(e) || e is NullReferenceException) && e is not ImmutableObjectException)
        {
            // an access violation natively (decision 977)
            aLine = "THROW " + (e is NullReferenceException ? "OSD_Exception_ACCESS_VIOLATION" : global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(e).GetType()));
        }
        return aLine is null ? null : string.Create(CultureInfo.InvariantCulture, $"{theTool} {theIndex} {aLine}");
    }

    /// <summary>The recorder's inputs: a primitive, or a file (a file of wires stands for the face of its first wire).</summary>
    internal static Shape Make(string theName, IReadOnlyDictionary<string, string> thePaths)
    {
        switch (theName)
        {
            case "PRIM_BOX":
                return new MakeBox(10, 20, 30).Shape();
            case "PRIM_CYLINDER":
                return new ApiMakeCylinder(5, 10).Shape();
            case "PRIM_CONE":
                return new ApiMakeCone(5, 2, 8).Shape();
            case "PRIM_SPHERE":
                return new MakeSphere(6).Shape();
            case "PRIM_TORUS":
                return new MakeTorus(10, 3).Shape();
            case "PRIM_BOXHOLE":
            {
                Shape aBox = new MakeBox(10, 20, 30).Shape();
                Shape aCyl = new ApiMakeCylinder(new Frame3D(new Point3D(5, 10, -1), Constants.DZ()), 3, 32).Shape();
                return new Cut(aBox, aCyl).Shape();
            }
            case "PRIM_HEXAGON":
            {
                var aPoly = new MakePolygon();
                for (int i = 0; i < 6; i++)
                {
                    aPoly.Add(new Point3D(10 * Math.Cos(i * Math.PI / 3), 10 * Math.Sin(i * Math.PI / 3), 0));
                }
                aPoly.Close();
                return new MakeFace(aPoly.Wire(), true).Shape();
            }
        }

        if (!thePaths.TryGetValue(theName, out string? aPath) || !BrepTools.Read(out Shape aShape, aPath))
        {
            return new Shape();
        }
        aShape.TShape()?.FreezeGraph(); // a caller's shape
        if (!aShape.IsNull && Count(aShape, ShapeType.Face) == 0)
        {
            var anExp = new Explorer(aShape, ShapeType.Wire);
            if (anExp.More())
            {
                var aMF = new MakeFace(ShapeCasts.Wire(anExp.Current()), true);
                return aMF.IsDone ? aMF.Shape() : new Shape();
            }
            return new Shape();
        }
        return aShape;
    }

    private static IndexedSet<Shape> Map(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap;
    }

    private static bool IsPlanar(in Face theFace)
    {
        Surface? aS = Tool.Surface(theFace);
        if (aS is null)
        {
            return false;
        }
        if (aS is TrimmedSurface aTrimmed)
        {
            aS = aTrimmed.BasisSurface;
        }
        return aS is PlaneSurface;
    }

    private static bool Middle(in Edge theEdge, out double thePrm, out Point3D thePnt)
    {
        thePrm = 0;
        thePnt = new Point3D();
        if (Tool.Degenerated(theEdge))
        {
            return false;
        }
        Curve3D? aC = Tool.Curve(theEdge, out double f, out double l);
        if (aC is null)
        {
            return false;
        }
        thePrm = (f + l) / 2;
        thePnt = aC.Value(thePrm);
        return true;
    }

    private static Edge IsoEdge(in Face theFace)
    {
        Surface aS = Tool.Surface(theFace)!;
        BrepTools.UVBounds(theFace, out double u1, out double u2, out double v1, out double v2);
        Curve3D anIso = aS.UIso((u1 + u2) / 2);
        var aME = new MakeEdge(anIso, v1, v2);
        return aME.IsDone ? aME.Edge() : new Edge();
    }

    private static string Spliter(WiresOnShape theWOnS, in Shape theShape)
    {
        var aSpliter = new Spliter(theShape);
        aSpliter.Perform(theWOnS);
        if (!aSpliter.IsDone)
        {
            return "NOTDONE";
        }
        IndexedSet<Shape> aFaces = Map(theShape, ShapeType.Face);
        var aDesc = new StringBuilder();
        for (int i = 1; i <= Math.Min(30, aFaces.Count); i++)
        {
            aDesc.Append(i > 1 ? "," : "").Append(aSpliter.DescendantShapes(aFaces[i - 1]).Extent().ToString(CultureInfo.InvariantCulture));
        }
        return string.Create(CultureInfo.InvariantCulture,
                             $"{Rec(aSpliter.ResultingShape())} {aSpliter.DirectLeft().Extent()} {aSpliter.Left().Extent()} {SetHash(aSpliter.Left())} {(aDesc.Length == 0 ? "-" : aDesc.ToString())}");
    }

    // The recorder's rounding of the numbers of a canonical record (llround(x * 1e6))
    private static string Q(double theValue) =>
        ((long)Math.Round(theValue * 1e6, MidpointRounding.AwayFromZero)).ToString(CultureInfo.InvariantCulture) + ";";

    // The recorder's canonical record of a shape whose BRep text natively varies run to run (DPrism's)
    private static string Canonical(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return "NULL";
        }
        if (!theShape.TShape()!.IsFrozen)
        {
            throw new InvalidOperationException("a result isn't frozen");
        }
        var aLines = new List<string>();
        IndexedSet<Shape> aMap = Map(theShape, ShapeType.Face);
        for (int i = 0; i < aMap.Count; i++)
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(aMap[i], aProps);
            aLines.Add($"F {Q(aProps.Mass())} {Count(aMap[i], ShapeType.Edge).ToString(CultureInfo.InvariantCulture)}");
        }
        aMap = Map(theShape, ShapeType.Edge);
        for (int i = 0; i < aMap.Count; i++)
        {
            Edge anE = ShapeCasts.Edge(aMap[i]);
            Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
            string[] anEnds = ["", ""];
            for (int k = 0; k < 2; k++)
            {
                Vertex aV = k == 0 ? aV1 : aV2;
                if (!aV.IsNull)
                {
                    Point3D aP = Tool.Pnt(aV);
                    anEnds[k] = Q(aP.X) + Q(aP.Y) + Q(aP.Z);
                }
            }
            if (string.CompareOrdinal(anEnds[1], anEnds[0]) < 0)
            {
                (anEnds[0], anEnds[1]) = (anEnds[1], anEnds[0]);
            }
            aLines.Add($"E {Q(Tool.Tolerance(anE))} {anEnds[0]} {anEnds[1]}");
        }
        aMap = Map(theShape, ShapeType.Vertex);
        for (int i = 0; i < aMap.Count; i++)
        {
            Vertex aV = ShapeCasts.Vertex(aMap[i]);
            Point3D aP = Tool.Pnt(aV);
            aLines.Add($"V {Q(aP.X)}{Q(aP.Y)}{Q(aP.Z)} {Q(Tool.Tolerance(aV))}");
        }
        aLines.Sort(StringComparer.Ordinal);
        string aJoined = string.Concat(aLines.Select(l => l + "\n"));
        return string.Create(CultureInfo.InvariantCulture,
                             $"C{Hex(Fnv(aJoined))} {Count(theShape, ShapeType.Face)} {Count(theShape, ShapeType.Edge)} {Count(theShape, ShapeType.Vertex)}");
    }

    private static string Swept(Shape theShape, Shape theFirst, Shape theLast,
                                Func<Shape, NCollection_List<Shape>> theShapes, in Face theFace,
                                bool theCanonical = false)
    {
        var aRes = new StringBuilder(theCanonical ? $"{Canonical(theShape)} {Canonical(theFirst)} {Canonical(theLast)} "
                                                  : $"{Rec(theShape)} {Rec(theFirst)} {Rec(theLast)} ");
        int i = 0;
        for (var anExp = new Explorer(theFace, ShapeType.Edge); anExp.More() && i < 16; anExp.Next(), i++)
        {
            aRes.Append(i > 0 ? "," : "").Append(theShapes(anExp.Current()).Extent().ToString(CultureInfo.InvariantCulture));
        }
        return aRes.ToString();
    }

    private static string Curves(NCollection_Sequence<Curve3D> theCurves, Curve3D? theBary)
    {
        string aRes = " " + theCurves.Length().ToString(CultureInfo.InvariantCulture);
        if (theBary is LineCurve3D aLine)
        {
            Point3D aP = aLine.Line.Location;
            aRes += $" {D(aP.X)} {D(aP.Y)} {D(aP.Z)}";
        }
        return aRes;
    }

    private static string Split(NCollection_List<Shape> theRes) =>
        theRes.Extent() == 1 ? Rec(theRes.First()) : "N" + theRes.Extent().ToString(CultureInfo.InvariantCulture);

    /// <summary>The recorder's runs on the frozen shape <paramref name="S"/>.</summary>
    internal static List<string> Tools(Shape S)
    {
        var aRecords = new List<string>();
        void add(string? theLine)
        {
            if (theLine is not null)
            {
                aRecords.Add(theLine);
            }
        }

        IndexedSet<Shape> aFMap = Map(S, ShapeType.Face), anEMap = Map(S, ShapeType.Edge);
        int aNbF = Math.Min(8, aFMap.Count);
        int aNbE = Math.Min(12, anEMap.Count);

        add(Run("SPLITE", 0, () =>
        {
            if (aNbE == 0)
            {
                return null;
            }
            var aSplit = new LocalSplitShape(S);
            for (int i = 1; i <= aNbE; i++)
            {
                Edge anE = ShapeCasts.Edge(anEMap[i - 1]);
                if (Middle(anE, out double aPrm, out Point3D aP) && aSplit.CanSplit(anE))
                {
                    aSplit.Add(new MakeVertex(aP).Vertex(), aPrm, anE);
                }
            }
            string aLine = Split(aSplit.DescendantShapes(S)) + " ";
            for (int i = 1; i <= aNbE; i++)
            {
                aLine += (i > 1 ? "," : "") + aSplit.DescendantShapes(anEMap[i - 1]).Extent().ToString(CultureInfo.InvariantCulture);
            }
            return aLine;
        }));

        for (int f = 1; f <= aNbF; f++)
        {
            Face aF = ShapeCasts.Face(aFMap[f - 1]);
            if (!IsPlanar(aF))
            {
                continue;
            }
            add(Run("SPLITW", f, () =>
            {
                var anEdges = new List<Edge>();
                for (var anExp = new Explorer(BrepTools.OuterWire(aF), ShapeType.Edge); anExp.More(); anExp.Next())
                {
                    Edge anE = ShapeCasts.Edge(anExp.Current());
                    bool isNew = true;
                    foreach (Edge anOther in anEdges)
                    {
                        isNew = isNew && !anOther.IsSame(anE);
                    }
                    if (isNew && Middle(anE, out _, out _))
                    {
                        anEdges.Add(anE);
                    }
                }
                if (anEdges.Count < 3)
                {
                    return null;
                }
                Edge anE1 = anEdges[0];
                Edge anE2 = anEdges[anEdges.Count / 2];
                Middle(anE1, out double aPrm1, out Point3D aP1);
                Middle(anE2, out double aPrm2, out Point3D aP2);
                if (aP1.DistanceTo(aP2) < 1e-3)
                {
                    return null;
                }
                Vertex aV1 = new MakeVertex(aP1).Vertex();
                Vertex aV2 = new MakeVertex(aP2).Vertex();
                var aSplit = new LocalSplitShape(S);
                aSplit.Add(aV1, aPrm1, anE1);
                aSplit.Add(aV2, aPrm2, anE2);
                Wire aW = new MakeWire(new MakeEdge(aV1, aV2).Edge()).Wire();
                bool isOk = aSplit.Add(aW, aF);
                string aLine = (isOk ? "1 " : "0 ") + Split(aSplit.DescendantShapes(S));
                return aLine + string.Create(CultureInfo.InvariantCulture,
                                             $" {aSplit.DescendantShapes(aF).Extent()} {aSplit.LeftOf(aW, aF).Extent()}");
            }));

            add(Run("SPLITC", f, () =>
            {
                BrepTools.UVBounds(aF, out double u1, out double u2, out double v1, out double v2);
                var aUV = new Point2D((u1 + u2) / 2, (v1 + v2) / 2);
                var aClass = new FClass2d(aF, 1e-9);
                if (aClass.Perform(aUV) != In)
                {
                    return null;
                }
                Surface aSurf = Tool.Surface(aF)!;
                if (aSurf is TrimmedSurface aTrimmed)
                {
                    aSurf = aTrimmed.BasisSurface;
                }
                Plane3D aPln = ((PlaneSurface)aSurf).Plane;
                Point3D aC = Tool.Surface(aF)!.Value(aUV.X, aUV.Y);
                double aR = 0.1 * Math.Min(u2 - u1, v2 - v1);
                var aCirc = new Circle3D(new Frame3D(aC, aPln.Axis.Direction), aR);
                Wire aW = new MakeWire(new MakeEdge(aCirc).Edge()).Wire();
                var aSplit = new LocalSplitShape(S);
                bool isOk = aSplit.Add(aW, aF);
                return (isOk ? "1 " : "0 ") + Split(aSplit.DescendantShapes(S))
                                            + " " + aSplit.DescendantShapes(aF).Extent().ToString(CultureInfo.InvariantCulture);
            }));
        }

        for (int f = 1; f <= aNbF; f++)
        {
            Face aF = ShapeCasts.Face(aFMap[f - 1]);
            add(Run("SPLITTER", f, () =>
            {
                Edge anE = IsoEdge(aF);
                if (anE.IsNull)
                {
                    return null;
                }
                var aWOnS = new WiresOnShape(S);
                aWOnS.Bind(anE, aF);
                return Spliter(aWOnS, S);
            }));
        }

        add(Run("SPLITADD", 0, () =>
        {
            var anEdges = new NCollection_Sequence<Shape>();
            for (int f = 1; f <= Math.Min(4, aNbF); f++)
            {
                Edge anE = IsoEdge(ShapeCasts.Face(aFMap[f - 1]));
                if (!anE.IsNull)
                {
                    anEdges.Append(anE);
                }
            }
            if (anEdges.IsEmpty())
            {
                return null;
            }
            var aWOnS = new WiresOnShape(S);
            bool isAdded = aWOnS.Add(anEdges);
            return (isAdded ? "1 " : "0 ") + Spliter(aWOnS, S);
        }));

        for (int f = 1; f <= Math.Min(6, aNbF); f++)
        {
            Face aF = ShapeCasts.Face(aFMap[f - 1]);
            add(Run("PRISM", f, () =>
            {
                var aPrism = new LocalPrism(aF, new Vector3D(3, 4, 12));
                var aCurves = new NCollection_Sequence<Curve3D>();
                aPrism.Curves(aCurves);
                return Swept(aPrism.Shape(), aPrism.FirstShape(), aPrism.LastShape(), s => aPrism.Shapes(s), aF)
                       + Curves(aCurves, aPrism.BarycCurve());
            }));
            add(Run("PRISMT", f, () =>
            {
                var aPrism = new LocalPrism(aF, new Vector3D(3, 4, 12), new Vector3D(1, 1, 1));
                return Swept(aPrism.Shape(), aPrism.FirstShape(), aPrism.LastShape(), s => aPrism.Shapes(s), aF);
            }));
            for (int aDec = 0; aDec <= 1; aDec++)
            {
                int aDec1 = aDec;
                add(Run(aDec1 == 1 ? "REVOLD" : "REVOL", f, () =>
                {
                    double[] aMin = [1e100, 1e100, 1e100];
                    for (var anExp = new Explorer(aF, ShapeType.Vertex); anExp.More(); anExp.Next())
                    {
                        Point3D aP = Tool.Pnt(ShapeCasts.Vertex(anExp.Current()));
                        aMin[0] = Min(aMin[0], aP.X);
                        aMin[1] = Min(aMin[1], aP.Y);
                        aMin[2] = Min(aMin[2], aP.Z);
                    }
                    if (aMin[0] > 1e99)
                    {
                        return null;
                    }
                    var anAxis = new Axis3D(new Point3D(aMin[0] - 5, aMin[1] - 5, aMin[2] - 5), Constants.DZ());
                    var aRevol = new LocalRevol();
                    if (aDec1 == 1)
                    {
                        aRevol.Perform(aF, anAxis, Math.PI / 6, Math.PI / 12);
                    }
                    else
                    {
                        aRevol.Perform(aF, anAxis, Math.PI / 6);
                    }
                    return Swept(aRevol.Shape(), aRevol.FirstShape(), aRevol.LastShape(), s => aRevol.Shapes(s), aF);
                }));
            }
            add(Run("LFORM", f, () =>
            {
                var aForm = new LinearForm(aF, new Vector3D(3, 4, 12), new Point3D(0, 0, 0), new Point3D(1, 0, 0));
                return Swept(aForm.Shape(), aForm.FirstShape(), aForm.LastShape(), s => aForm.Shapes(s), aF);
            }));
        }

        for (int f = 1; f <= aNbF; f++)
        {
            Face aF = ShapeCasts.Face(aFMap[f - 1]);
            if (!IsPlanar(aF) || Count(aF, ShapeType.Edge) > 16)
            {
                continue;
            }
            for (int aTwo = 0; aTwo <= 1; aTwo++)
            {
                int aTwo1 = aTwo;
                add(Run(aTwo1 == 1 ? "DPRISM2" : "DPRISM", f, () =>
                {
                    DPrism aDPrism = aTwo1 == 1 ? new DPrism(aF, 10, 4, 3 * Math.PI / 180)
                                                       : new DPrism(aF, 10, 3 * Math.PI / 180);
                    if (!aDPrism.IsDone)
                    {
                        return "NOTDONE";
                    }
                    var aCurves = new NCollection_Sequence<Curve3D>();
                    string aLine = Swept(aDPrism.Shape(), aDPrism.FirstShape(), aDPrism.LastShape(), s => aDPrism.Shapes(s), aF, true);
                    aDPrism.Curves(aCurves);
                    return aLine + Curves(aCurves, aDPrism.BarycCurve());
                }));
            }
        }

        for (int f = 1; f <= aNbF; f++)
        {
            Face aF = ShapeCasts.Face(aFMap[f - 1]);
            if (!IsPlanar(aF))
            {
                continue;
            }
            add(Run("DRAFT", f, () =>
            {
                Edge anE = IsoEdge(aF);
                if (anE.IsNull)
                {
                    return null;
                }
                double aZMin = 1e100;
                for (var anExp = new Explorer(aF, ShapeType.Vertex); anExp.More(); anExp.Next())
                {
                    aZMin = Min(aZMin, Tool.Pnt(ShapeCasts.Vertex(anExp.Current())).Z);
                }
                var aDrafts = new SplitDrafts(S);
                aDrafts.Perform(aF, new MakeWire(anE).Wire(), Constants.DZ(), new Plane3D(new Point3D(0, 0, aZMin), Constants.DZ()),
                                5 * Math.PI / 180);
                if (!aDrafts.IsDone)
                {
                    return "NOTDONE";
                }
                return Rec(aDrafts.Shape()) + " " + aDrafts.ShapesFromShape(aF).Extent().ToString(CultureInfo.InvariantCulture);
            }));
        }

        for (int f = 1; f <= aNbF; f++)
        {
            Face aF = ShapeCasts.Face(aFMap[f - 1]);
            add(Run("FIND", f, () =>
            {
                var aFind = new FindEdges(aF, S);
                var aRes = new StringBuilder();
                int aNb = 0;
                for (aFind.InitIterator(); aFind.More(); aFind.Next(), aNb++)
                {
                    aRes.Append(CultureInfo.InvariantCulture, $" {(anEMap.IndexOf(aFind.EdgeFrom()) + 1)}:{(anEMap.IndexOf(aFind.EdgeTo()) + 1)}");
                }
                return aNb.ToString(CultureInfo.InvariantCulture) + aRes;
            }));
            add(Run("FIF", f, () =>
            {
                var aFind = new FindEdgesInFace(S, aF);
                var aRes = new StringBuilder();
                int aNb = 0;
                for (aFind.Init(); aFind.More(); aFind.Next(), aNb++)
                {
                    aRes.Append(CultureInfo.InvariantCulture, $" {(anEMap.IndexOf(aFind.Edge()) + 1)}");
                }
                return aNb.ToString(CultureInfo.InvariantCulture) + aRes;
            }));
        }
        return aRecords;
    }

    // The records natively different, attributed (decisions 1013-1017)
    private static readonly HashSet<string> s_attributed = new(StringComparer.Ordinal);

    [Fact]
    public void LocalOperations_AsNativeOCCT()
    {
        var aByName = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (string aFile in Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep"))
        {
            aByName[Path.GetFileName(aFile)] = aFile;
        }
        string aData = Metadata("OcctTestDataDir");
        if (!string.IsNullOrEmpty(aData) && Directory.Exists(aData))
        {
            foreach (string aFile in Directory.EnumerateFiles(aData, "*", SearchOption.AllDirectories))
            {
                aByName.TryAdd(Path.GetFileName(aFile), aFile);
            }
        }

        List<(string Name, List<string> Lines)> aSections = Sections();
        var aPorted = new ConcurrentDictionary<int, List<string>>();
        int aMissing = 0;
        string? aTrace = Environment.GetEnvironmentVariable("OCCTNET_LOCOPE_TRACE");
        Parallel.For(0, aSections.Count, new ParallelOptions { MaxDegreeOfParallelism = aTrace is null ? -1 : 1 }, i =>
        {
            if (aTrace is not null)
            {
                File.AppendAllText(aTrace, aSections[i].Name + "\n");
            }
            Shape aShape = Make(aSections[i].Name, aByName);
            if (aShape.IsNull)
            {
                Interlocked.Increment(ref aMissing);
                return;
            }
            aShape.TShape()!.FreezeGraph(); // a caller's shape
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);
            s_name = aSections[i].Name;
            List<string> aRecords = Tools(aShape);
            aSnap.AssertUnchanged(aShape);
            aPorted[i] = aRecords;
        });

        int aFiles = 0, aRecordsCount = 0, anAttributed = 0;
        var aCounts = new SortedDictionary<string, int>(StringComparer.Ordinal);
        var aDifferences = new List<string>();
        var aDiffByTool = new SortedDictionary<string, int>(StringComparer.Ordinal);
        for (int i = 0; i < aSections.Count; i++)
        {
            if (!aPorted.TryGetValue(i, out List<string>? aPort))
            {
                continue;
            }
            aFiles++;
            List<string> aNative = aSections[i].Lines;
            var aPortByKey = aPort.ToDictionary(l => string.Join(' ', l.Split(' ').Take(2)), l => l, StringComparer.Ordinal);
            foreach (string aLine in aNative)
            {
                string aKey = string.Join(' ', aLine.Split(' ').Take(2));
                string aTool = aLine.Split(' ')[0];
                aCounts[aTool] = aCounts.GetValueOrDefault(aTool) + 1;
                aRecordsCount++;
                aPortByKey.TryGetValue(aKey, out string? aPortLine);
                aPortByKey.Remove(aKey);
                if (aLine == aPortLine)
                {
                    continue;
                }
                if (s_attributed.Contains($"{aSections[i].Name} {aKey}"))
                {
                    anAttributed++;
                    continue;
                }
                aDiffByTool[aTool] = aDiffByTool.GetValueOrDefault(aTool) + 1;
                aDifferences.Add($"{aSections[i].Name}: '{aLine}' natively, '{aPortLine ?? "<none>"}' in the port");
            }
            foreach (string aExtra in aPortByKey.Values)
            {
                aDifferences.Add($"{aSections[i].Name}: none natively, '{aExtra}' in the port");
            }
        }
        output.WriteLine($"{aFiles} inputs ({aMissing} not found), {aRecordsCount} records ({string.Join(", ", aCounts.Select(c => $"{c.Key} {c.Value}"))}): "
                         + $"{aRecordsCount - aDifferences.Count - anAttributed} identical, {anAttributed} attributed, {aDifferences.Count} different "
                         + $"({string.Join(", ", aDiffByTool.Select(c => $"{c.Key} {c.Value}"))})");
        foreach (string aD in aDifferences.Take(300))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Empty(aDifferences);
        Assert.True(aFiles > 100);
    }
}
