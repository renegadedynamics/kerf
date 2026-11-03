// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using OcctNet.ModelingData.Tests;
using Xunit.Abstractions;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

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

    private static ulong ShapeHash(in TopoDS_Shape theShape)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BRepTools.Write(theShape, aS, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3);
        if (s_dump is not null && s_tag is not null)
        {
            File.WriteAllText(Path.Combine(s_dump, $"{s_tag}.{s_nbDumped++}.brep"), aS.ToString());
        }
        return ShapeUpgradeNativeTests.Hash(aS.ToString());
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType) =>
        theShape.IsNull() ? 0 : LocOpeTests.Count(theShape, theType);

    private static string Rec(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return "NULL";
        }
        if (!theShape.TShape()!.IsFrozen)
        {
            throw new InvalidOperationException("a result isn't frozen");
        }
        return string.Create(CultureInfo.InvariantCulture,
                             $"{Hex(ShapeHash(theShape))} {Count(theShape, TopAbs_FACE)} {Count(theShape, TopAbs_EDGE)} {Count(theShape, TopAbs_VERTEX)}");
    }

    private static string SetHash(NCollection_List<TopoDS_Shape> theList)
    {
        var aHashes = new List<string>();
        foreach (TopoDS_Shape aS in theList)
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
        catch (Exception e) when (e is FailureException or NullReferenceException && e is not (ImmutableObjectException or TopoDS_LockedShape))
        {
            // an access violation natively (decision 977)
            aLine = "THROW " + (e is NullReferenceException ? "OSD_Exception_ACCESS_VIOLATION" : global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
        }
        return aLine is null ? null : string.Create(CultureInfo.InvariantCulture, $"{theTool} {theIndex} {aLine}");
    }

    /// <summary>The recorder's inputs: a primitive, or a file (a file of wires stands for the face of its first wire).</summary>
    internal static TopoDS_Shape Make(string theName, IReadOnlyDictionary<string, string> thePaths)
    {
        switch (theName)
        {
            case "PRIM_BOX":
                return new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
            case "PRIM_CYLINDER":
                return new BRepPrimAPI_MakeCylinder(5, 10).Shape();
            case "PRIM_CONE":
                return new BRepPrimAPI_MakeCone(5, 2, 8).Shape();
            case "PRIM_SPHERE":
                return new BRepPrimAPI_MakeSphere(6).Shape();
            case "PRIM_TORUS":
                return new BRepPrimAPI_MakeTorus(10, 3).Shape();
            case "PRIM_BOXHOLE":
            {
                TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
                TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(5, 10, -1), Constants.DZ()), 3, 32).Shape();
                return new BRepAlgoAPI_Cut(aBox, aCyl).Shape();
            }
            case "PRIM_HEXAGON":
            {
                var aPoly = new BRepBuilderAPI_MakePolygon();
                for (int i = 0; i < 6; i++)
                {
                    aPoly.Add(new Point3D(10 * Math.Cos(i * Math.PI / 3), 10 * Math.Sin(i * Math.PI / 3), 0));
                }
                aPoly.Close();
                return new BRepBuilderAPI_MakeFace(aPoly.Wire(), true).Shape();
            }
        }

        if (!thePaths.TryGetValue(theName, out string? aPath) || !BRepTools.Read(out TopoDS_Shape aShape, aPath))
        {
            return new TopoDS_Shape();
        }
        aShape.TShape()?.FreezeGraph(); // a caller's shape
        if (!aShape.IsNull() && Count(aShape, TopAbs_FACE) == 0)
        {
            var anExp = new TopExp_Explorer(aShape, TopAbs_WIRE);
            if (anExp.More())
            {
                var aMF = new BRepBuilderAPI_MakeFace(TopoDS.Wire(anExp.Current()), true);
                return aMF.IsDone() ? aMF.Shape() : new TopoDS_Shape();
            }
            return new TopoDS_Shape();
        }
        return aShape;
    }

    private static NCollection_IndexedMap<TopoDS_Shape> Map(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap;
    }

    private static bool IsPlanar(in TopoDS_Face theFace)
    {
        Geom_Surface? aS = BRep_Tool.Surface(theFace);
        if (aS is null)
        {
            return false;
        }
        if (aS is Geom_RectangularTrimmedSurface aTrimmed)
        {
            aS = aTrimmed.BasisSurface();
        }
        return aS is Geom_Plane;
    }

    private static bool Middle(in TopoDS_Edge theEdge, out double thePrm, out Point3D thePnt)
    {
        thePrm = 0;
        thePnt = new Point3D();
        if (BRep_Tool.Degenerated(theEdge))
        {
            return false;
        }
        Geom_Curve? aC = BRep_Tool.Curve(theEdge, out double f, out double l);
        if (aC is null)
        {
            return false;
        }
        thePrm = (f + l) / 2;
        thePnt = aC.Value(thePrm);
        return true;
    }

    private static TopoDS_Edge IsoEdge(in TopoDS_Face theFace)
    {
        Geom_Surface aS = BRep_Tool.Surface(theFace)!;
        BRepTools.UVBounds(theFace, out double u1, out double u2, out double v1, out double v2);
        Geom_Curve anIso = aS.UIso((u1 + u2) / 2);
        var aME = new BRepBuilderAPI_MakeEdge(anIso, v1, v2);
        return aME.IsDone() ? aME.Edge() : new TopoDS_Edge();
    }

    private static string Spliter(LocOpe_WiresOnShape theWOnS, in TopoDS_Shape theShape)
    {
        var aSpliter = new LocOpe_Spliter(theShape);
        aSpliter.Perform(theWOnS);
        if (!aSpliter.IsDone())
        {
            return "NOTDONE";
        }
        NCollection_IndexedMap<TopoDS_Shape> aFaces = Map(theShape, TopAbs_FACE);
        var aDesc = new StringBuilder();
        for (int i = 1; i <= Math.Min(30, aFaces.Extent()); i++)
        {
            aDesc.Append(i > 1 ? "," : "").Append(aSpliter.DescendantShapes(aFaces[i]).Extent().ToString(CultureInfo.InvariantCulture));
        }
        return string.Create(CultureInfo.InvariantCulture,
                             $"{Rec(aSpliter.ResultingShape())} {aSpliter.DirectLeft().Extent()} {aSpliter.Left().Extent()} {SetHash(aSpliter.Left())} {(aDesc.Length == 0 ? "-" : aDesc.ToString())}");
    }

    // The recorder's rounding of the numbers of a canonical record (llround(x * 1e6))
    private static string Q(double theValue) =>
        ((long)Math.Round(theValue * 1e6, MidpointRounding.AwayFromZero)).ToString(CultureInfo.InvariantCulture) + ";";

    // The recorder's canonical record of a shape whose BRep text natively varies run to run (DPrism's)
    private static string Canonical(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return "NULL";
        }
        if (!theShape.TShape()!.IsFrozen)
        {
            throw new InvalidOperationException("a result isn't frozen");
        }
        var aLines = new List<string>();
        NCollection_IndexedMap<TopoDS_Shape> aMap = Map(theShape, TopAbs_FACE);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(aMap[i], aProps);
            aLines.Add($"F {Q(aProps.Mass())} {Count(aMap[i], TopAbs_EDGE).ToString(CultureInfo.InvariantCulture)}");
        }
        aMap = Map(theShape, TopAbs_EDGE);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            TopoDS_Edge anE = TopoDS.Edge(aMap[i]);
            TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            string[] anEnds = ["", ""];
            for (int k = 0; k < 2; k++)
            {
                TopoDS_Vertex aV = k == 0 ? aV1 : aV2;
                if (!aV.IsNull())
                {
                    Point3D aP = BRep_Tool.Pnt(aV);
                    anEnds[k] = Q(aP.X()) + Q(aP.Y()) + Q(aP.Z());
                }
            }
            if (string.CompareOrdinal(anEnds[1], anEnds[0]) < 0)
            {
                (anEnds[0], anEnds[1]) = (anEnds[1], anEnds[0]);
            }
            aLines.Add($"E {Q(BRep_Tool.Tolerance(anE))} {anEnds[0]} {anEnds[1]}");
        }
        aMap = Map(theShape, TopAbs_VERTEX);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(aMap[i]);
            Point3D aP = BRep_Tool.Pnt(aV);
            aLines.Add($"V {Q(aP.X())}{Q(aP.Y())}{Q(aP.Z())} {Q(BRep_Tool.Tolerance(aV))}");
        }
        aLines.Sort(StringComparer.Ordinal);
        string aJoined = string.Concat(aLines.Select(l => l + "\n"));
        return string.Create(CultureInfo.InvariantCulture,
                             $"C{Hex(Fnv(aJoined))} {Count(theShape, TopAbs_FACE)} {Count(theShape, TopAbs_EDGE)} {Count(theShape, TopAbs_VERTEX)}");
    }

    private static string Swept(TopoDS_Shape theShape, TopoDS_Shape theFirst, TopoDS_Shape theLast,
                                Func<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theShapes, in TopoDS_Face theFace,
                                bool theCanonical = false)
    {
        var aRes = new StringBuilder(theCanonical ? $"{Canonical(theShape)} {Canonical(theFirst)} {Canonical(theLast)} "
                                                  : $"{Rec(theShape)} {Rec(theFirst)} {Rec(theLast)} ");
        int i = 0;
        for (var anExp = new TopExp_Explorer(theFace, TopAbs_EDGE); anExp.More() && i < 16; anExp.Next(), i++)
        {
            aRes.Append(i > 0 ? "," : "").Append(theShapes(anExp.Current()).Extent().ToString(CultureInfo.InvariantCulture));
        }
        return aRes.ToString();
    }

    private static string Curves(NCollection_Sequence<Geom_Curve> theCurves, Geom_Curve? theBary)
    {
        string aRes = " " + theCurves.Length().ToString(CultureInfo.InvariantCulture);
        if (theBary is Geom_Line aLine)
        {
            Point3D aP = aLine.Lin().Location();
            aRes += $" {D(aP.X())} {D(aP.Y())} {D(aP.Z())}";
        }
        return aRes;
    }

    private static string Split(NCollection_List<TopoDS_Shape> theRes) =>
        theRes.Extent() == 1 ? Rec(theRes.First()) : "N" + theRes.Extent().ToString(CultureInfo.InvariantCulture);

    /// <summary>The recorder's runs on the frozen shape <paramref name="S"/>.</summary>
    internal static List<string> Tools(TopoDS_Shape S)
    {
        var aRecords = new List<string>();
        void add(string? theLine)
        {
            if (theLine is not null)
            {
                aRecords.Add(theLine);
            }
        }

        NCollection_IndexedMap<TopoDS_Shape> aFMap = Map(S, TopAbs_FACE), anEMap = Map(S, TopAbs_EDGE);
        int aNbF = Math.Min(8, aFMap.Extent());
        int aNbE = Math.Min(12, anEMap.Extent());

        add(Run("SPLITE", 0, () =>
        {
            if (aNbE == 0)
            {
                return null;
            }
            var aSplit = new LocOpe_SplitShape(S);
            for (int i = 1; i <= aNbE; i++)
            {
                TopoDS_Edge anE = TopoDS.Edge(anEMap[i]);
                if (Middle(anE, out double aPrm, out Point3D aP) && aSplit.CanSplit(anE))
                {
                    aSplit.Add(new BRepBuilderAPI_MakeVertex(aP).Vertex(), aPrm, anE);
                }
            }
            string aLine = Split(aSplit.DescendantShapes(S)) + " ";
            for (int i = 1; i <= aNbE; i++)
            {
                aLine += (i > 1 ? "," : "") + aSplit.DescendantShapes(anEMap[i]).Extent().ToString(CultureInfo.InvariantCulture);
            }
            return aLine;
        }));

        for (int f = 1; f <= aNbF; f++)
        {
            TopoDS_Face aF = TopoDS.Face(aFMap[f]);
            if (!IsPlanar(aF))
            {
                continue;
            }
            add(Run("SPLITW", f, () =>
            {
                var anEdges = new List<TopoDS_Edge>();
                for (var anExp = new TopExp_Explorer(BRepTools.OuterWire(aF), TopAbs_EDGE); anExp.More(); anExp.Next())
                {
                    TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
                    bool isNew = true;
                    foreach (TopoDS_Edge anOther in anEdges)
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
                TopoDS_Edge anE1 = anEdges[0];
                TopoDS_Edge anE2 = anEdges[anEdges.Count / 2];
                Middle(anE1, out double aPrm1, out Point3D aP1);
                Middle(anE2, out double aPrm2, out Point3D aP2);
                if (aP1.Distance(aP2) < 1e-3)
                {
                    return null;
                }
                TopoDS_Vertex aV1 = new BRepBuilderAPI_MakeVertex(aP1).Vertex();
                TopoDS_Vertex aV2 = new BRepBuilderAPI_MakeVertex(aP2).Vertex();
                var aSplit = new LocOpe_SplitShape(S);
                aSplit.Add(aV1, aPrm1, anE1);
                aSplit.Add(aV2, aPrm2, anE2);
                TopoDS_Wire aW = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aV1, aV2).Edge()).Wire();
                bool isOk = aSplit.Add(aW, aF);
                string aLine = (isOk ? "1 " : "0 ") + Split(aSplit.DescendantShapes(S));
                return aLine + string.Create(CultureInfo.InvariantCulture,
                                             $" {aSplit.DescendantShapes(aF).Extent()} {aSplit.LeftOf(aW, aF).Extent()}");
            }));

            add(Run("SPLITC", f, () =>
            {
                BRepTools.UVBounds(aF, out double u1, out double u2, out double v1, out double v2);
                var aUV = new Point2D((u1 + u2) / 2, (v1 + v2) / 2);
                var aClass = new BRepTopAdaptor_FClass2d(aF, 1e-9);
                if (aClass.Perform(aUV) != TopAbs_IN)
                {
                    return null;
                }
                Geom_Surface aSurf = BRep_Tool.Surface(aF)!;
                if (aSurf is Geom_RectangularTrimmedSurface aTrimmed)
                {
                    aSurf = aTrimmed.BasisSurface();
                }
                Plane3D aPln = ((Geom_Plane)aSurf).Pln();
                Point3D aC = BRep_Tool.Surface(aF)!.Value(aUV.X(), aUV.Y());
                double aR = 0.1 * Math.Min(u2 - u1, v2 - v1);
                var aCirc = new Circle3D(new Frame3D(aC, aPln.Axis().Direction()), aR);
                TopoDS_Wire aW = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aCirc).Edge()).Wire();
                var aSplit = new LocOpe_SplitShape(S);
                bool isOk = aSplit.Add(aW, aF);
                return (isOk ? "1 " : "0 ") + Split(aSplit.DescendantShapes(S))
                                            + " " + aSplit.DescendantShapes(aF).Extent().ToString(CultureInfo.InvariantCulture);
            }));
        }

        for (int f = 1; f <= aNbF; f++)
        {
            TopoDS_Face aF = TopoDS.Face(aFMap[f]);
            add(Run("SPLITTER", f, () =>
            {
                TopoDS_Edge anE = IsoEdge(aF);
                if (anE.IsNull())
                {
                    return null;
                }
                var aWOnS = new LocOpe_WiresOnShape(S);
                aWOnS.Bind(anE, aF);
                return Spliter(aWOnS, S);
            }));
        }

        add(Run("SPLITADD", 0, () =>
        {
            var anEdges = new NCollection_Sequence<TopoDS_Shape>();
            for (int f = 1; f <= Math.Min(4, aNbF); f++)
            {
                TopoDS_Edge anE = IsoEdge(TopoDS.Face(aFMap[f]));
                if (!anE.IsNull())
                {
                    anEdges.Append(anE);
                }
            }
            if (anEdges.IsEmpty())
            {
                return null;
            }
            var aWOnS = new LocOpe_WiresOnShape(S);
            bool isAdded = aWOnS.Add(anEdges);
            return (isAdded ? "1 " : "0 ") + Spliter(aWOnS, S);
        }));

        for (int f = 1; f <= Math.Min(6, aNbF); f++)
        {
            TopoDS_Face aF = TopoDS.Face(aFMap[f]);
            add(Run("PRISM", f, () =>
            {
                var aPrism = new LocOpe_Prism(aF, new Vector3D(3, 4, 12));
                var aCurves = new NCollection_Sequence<Geom_Curve>();
                aPrism.Curves(aCurves);
                return Swept(aPrism.Shape(), aPrism.FirstShape(), aPrism.LastShape(), s => aPrism.Shapes(s), aF)
                       + Curves(aCurves, aPrism.BarycCurve());
            }));
            add(Run("PRISMT", f, () =>
            {
                var aPrism = new LocOpe_Prism(aF, new Vector3D(3, 4, 12), new Vector3D(1, 1, 1));
                return Swept(aPrism.Shape(), aPrism.FirstShape(), aPrism.LastShape(), s => aPrism.Shapes(s), aF);
            }));
            for (int aDec = 0; aDec <= 1; aDec++)
            {
                int aDec1 = aDec;
                add(Run(aDec1 == 1 ? "REVOLD" : "REVOL", f, () =>
                {
                    double[] aMin = [1e100, 1e100, 1e100];
                    for (var anExp = new TopExp_Explorer(aF, TopAbs_VERTEX); anExp.More(); anExp.Next())
                    {
                        Point3D aP = BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current()));
                        aMin[0] = Min(aMin[0], aP.X());
                        aMin[1] = Min(aMin[1], aP.Y());
                        aMin[2] = Min(aMin[2], aP.Z());
                    }
                    if (aMin[0] > 1e99)
                    {
                        return null;
                    }
                    var anAxis = new Axis3D(new Point3D(aMin[0] - 5, aMin[1] - 5, aMin[2] - 5), Constants.DZ());
                    var aRevol = new LocOpe_Revol();
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
                var aForm = new LocOpe_LinearForm(aF, new Vector3D(3, 4, 12), new Point3D(0, 0, 0), new Point3D(1, 0, 0));
                return Swept(aForm.Shape(), aForm.FirstShape(), aForm.LastShape(), s => aForm.Shapes(s), aF);
            }));
        }

        for (int f = 1; f <= aNbF; f++)
        {
            TopoDS_Face aF = TopoDS.Face(aFMap[f]);
            if (!IsPlanar(aF) || Count(aF, TopAbs_EDGE) > 16)
            {
                continue;
            }
            for (int aTwo = 0; aTwo <= 1; aTwo++)
            {
                int aTwo1 = aTwo;
                add(Run(aTwo1 == 1 ? "DPRISM2" : "DPRISM", f, () =>
                {
                    LocOpe_DPrism aDPrism = aTwo1 == 1 ? new LocOpe_DPrism(aF, 10, 4, 3 * Math.PI / 180)
                                                       : new LocOpe_DPrism(aF, 10, 3 * Math.PI / 180);
                    if (!aDPrism.IsDone())
                    {
                        return "NOTDONE";
                    }
                    var aCurves = new NCollection_Sequence<Geom_Curve>();
                    string aLine = Swept(aDPrism.Shape(), aDPrism.FirstShape(), aDPrism.LastShape(), s => aDPrism.Shapes(s), aF, true);
                    aDPrism.Curves(aCurves);
                    return aLine + Curves(aCurves, aDPrism.BarycCurve());
                }));
            }
        }

        for (int f = 1; f <= aNbF; f++)
        {
            TopoDS_Face aF = TopoDS.Face(aFMap[f]);
            if (!IsPlanar(aF))
            {
                continue;
            }
            add(Run("DRAFT", f, () =>
            {
                TopoDS_Edge anE = IsoEdge(aF);
                if (anE.IsNull())
                {
                    return null;
                }
                double aZMin = 1e100;
                for (var anExp = new TopExp_Explorer(aF, TopAbs_VERTEX); anExp.More(); anExp.Next())
                {
                    aZMin = Min(aZMin, BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current())).Z());
                }
                var aDrafts = new LocOpe_SplitDrafts(S);
                aDrafts.Perform(aF, new BRepBuilderAPI_MakeWire(anE).Wire(), Constants.DZ(), new Plane3D(new Point3D(0, 0, aZMin), Constants.DZ()),
                                5 * Math.PI / 180);
                if (!aDrafts.IsDone())
                {
                    return "NOTDONE";
                }
                return Rec(aDrafts.Shape()) + " " + aDrafts.ShapesFromShape(aF).Extent().ToString(CultureInfo.InvariantCulture);
            }));
        }

        for (int f = 1; f <= aNbF; f++)
        {
            TopoDS_Face aF = TopoDS.Face(aFMap[f]);
            add(Run("FIND", f, () =>
            {
                var aFind = new LocOpe_FindEdges(aF, S);
                var aRes = new StringBuilder();
                int aNb = 0;
                for (aFind.InitIterator(); aFind.More(); aFind.Next(), aNb++)
                {
                    aRes.Append(CultureInfo.InvariantCulture, $" {anEMap.FindIndex(aFind.EdgeFrom())}:{anEMap.FindIndex(aFind.EdgeTo())}");
                }
                return aNb.ToString(CultureInfo.InvariantCulture) + aRes;
            }));
            add(Run("FIF", f, () =>
            {
                var aFind = new LocOpe_FindEdgesInFace(S, aF);
                var aRes = new StringBuilder();
                int aNb = 0;
                for (aFind.Init(); aFind.More(); aFind.Next(), aNb++)
                {
                    aRes.Append(CultureInfo.InvariantCulture, $" {anEMap.FindIndex(aFind.Edge())}");
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
            TopoDS_Shape aShape = Make(aSections[i].Name, aByName);
            if (aShape.IsNull())
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
