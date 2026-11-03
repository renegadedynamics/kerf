// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Decision 581, step 2: where native's own output varies from run to run (address-ordered maps, plan section 2.4)
// or the BRep bytes differ, a result is compared canonically: the same number of each shape type; every sub-shape
// matched to one of the other side by its geometry, then its tolerance and flags; volume, area and centroid; validity.
//
// How later units use it: CanonicalShapeComparer.Compare(native, port) gives a CanonicalComparison; a case is
// "canonically identical" when IsIdentical holds, and its ToString() says which sub-shapes differ and by how much.
// CanonicalKeys.Build(shape) gives each sub-shape's key, which HistoryComparison uses to compare histories.

using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.TestHarness;

/// <summary>Options of the canonical comparison.</summary>
internal sealed class CanonicalOptions
{
    /// <summary>
    /// 0 (the default): vertices match by their points' exact bits. Otherwise a port vertex with no exact match takes
    /// the key of the nearest unmatched native vertex within this distance. Curves and surfaces always compare exactly.
    /// </summary>
    public double VertexTolerance { get; init; }

    /// <summary>
    /// False (the default): a reversed sub-shape is the same sub-shape. True: the orientations of the edges in wires,
    /// the faces in shells, the children of every container and the top-level shape count too.
    /// </summary>
    public bool CheckOrientation { get; init; }

    /// <summary>Compare volume, area and centroid (true by default).</summary>
    public bool CheckProperties { get; init; } = true;

    /// <summary>The relative tolerance of the properties (1e-9 by default).</summary>
    public double PropertyRelativeTolerance { get; init; } = 1.0e-9;

    /// <summary>Require both shapes valid by BRepCheck_Analyzer (true by default).</summary>
    public bool CheckValidity { get; init; } = true;
}

/// <summary>The result of a canonical comparison.</summary>
internal sealed class CanonicalComparison
{
    /// <summary>The number of unique sub-shapes (TShape and location) of each type: native, port.</summary>
    public SortedDictionary<TopAbs_ShapeEnum, (int Native, int Port)> Counts { get; } = new();

    /// <summary>Sub-shapes with no match on the other side: type, key description, count native, count port.</summary>
    public List<string> Unmatched { get; } = [];

    /// <summary>Matched sub-shapes whose tolerances or flags differ.</summary>
    public List<string> AttributeDifferences { get; } = [];

    /// <summary>Volume, area or centroid beyond the relative tolerance.</summary>
    public List<string> PropertyDifferences { get; } = [];

    /// <summary>Validity by BRepCheck_Analyzer: native, port (true when not checked).</summary>
    public (bool Native, bool Port) Validity { get; set; } = (true, true);

    /// <summary>The top-level orientations, when checked and different.</summary>
    public string? OrientationDifference { get; set; }

    public bool CountsEqual => Counts.Values.All(c => c.Native == c.Port);

    /// <summary>True if the shapes are canonically identical.</summary>
    public bool IsIdentical =>
        CountsEqual && Unmatched.Count == 0 && AttributeDifferences.Count == 0 && PropertyDifferences.Count == 0
        && Validity.Native && Validity.Port && OrientationDifference is null;

    public override string ToString()
    {
        if (IsIdentical)
        {
            return "canonically identical";
        }
        var aText = new StringBuilder("canonically different:");
        foreach ((TopAbs_ShapeEnum aType, (int aNative, int aPort)) in Counts)
        {
            if (aNative != aPort)
            {
                aText.Append(CultureInfo.InvariantCulture, $"\n  {aType}: {aNative} native, {aPort} port");
            }
        }
        foreach (string aLine in Unmatched.Take(20))
        {
            aText.Append("\n  unmatched ").Append(aLine);
        }
        if (Unmatched.Count > 20)
        {
            aText.Append(CultureInfo.InvariantCulture, $"\n  ... {Unmatched.Count - 20} more unmatched");
        }
        foreach (string aLine in AttributeDifferences.Take(20))
        {
            aText.Append("\n  ").Append(aLine);
        }
        foreach (string aLine in PropertyDifferences)
        {
            aText.Append("\n  ").Append(aLine);
        }
        if (!Validity.Native || !Validity.Port)
        {
            aText.Append(CultureInfo.InvariantCulture, $"\n  valid: native {Validity.Native}, port {Validity.Port}");
        }
        if (OrientationDifference is not null)
        {
            aText.Append("\n  ").Append(OrientationDifference);
        }
        return aText.ToString();
    }
}

/// <summary>
/// The canonical keys of a shape's sub-shapes: a key names a sub-shape by its geometry, independently of the order in
/// which the shape stores it and of its TShape's identity.
/// </summary>
/// <remarks>
/// <list type="bullet">
/// <item>A vertex: its point (location applied) as the bits of its coordinates.</item>
/// <item>An edge: its 3D curve's text (GeomTools_CurveSet.PrintCurve of the curve transformed by the edge's
/// location), its range's bits and its vertices' keys, sorted. A degenerated edge, or one without a 3D curve: its
/// range and vertices.</item>
/// <item>A face: its surface's text (location applied) and its edges' keys, sorted.</item>
/// <item>A wire, shell, solid, compsolid or compound: its type and its children's keys, sorted.</item>
/// </list>
/// Keys ignore orientation unless <see cref="CanonicalOptions.CheckOrientation"/> is set (then the children's
/// orientations are part of their containers' keys). A key is the first 16 bytes of the SHA-256 of its text, so
/// long keys stay short; <see cref="Describe"/> gives a readable name for reports.
/// </remarks>
internal sealed class CanonicalKeys
{
    private readonly Dictionary<TopoDS_Shape, string> myKeys = new(TopTools_ShapeMapHasher.Instance);
    private readonly Dictionary<string, string> myDescriptions = new(StringComparer.Ordinal);
    private readonly bool myCheckOrientation;
    private readonly Func<TopoDS_Vertex, string?>? myVertexKeyOverride;

    private CanonicalKeys(bool theCheckOrientation, Func<TopoDS_Vertex, string?>? theVertexKeyOverride)
    {
        myCheckOrientation = theCheckOrientation;
        myVertexKeyOverride = theVertexKeyOverride;
    }

    /// <summary>The keys of every sub-shape of <paramref name="theShape"/>.</summary>
    public static CanonicalKeys Build(TopoDS_Shape theShape, bool theCheckOrientation = false,
                                      Func<TopoDS_Vertex, string?>? theVertexKeyOverride = null)
    {
        var aKeys = new CanonicalKeys(theCheckOrientation, theVertexKeyOverride);
        if (!theShape.IsNull())
        {
            aKeys.Key(theShape);
        }
        return aKeys;
    }

    /// <summary>The key of a shape (computed and remembered on first use; any shape, not only sub-shapes).</summary>
    public string Key(TopoDS_Shape theShape)
    {
        if (myKeys.TryGetValue(theShape, out string? aKey))
        {
            return aKey;
        }
        string aDescription;
        string aText = theShape.ShapeType() switch
        {
            TopAbs_VERTEX => VertexText(TopoDS.Vertex(theShape), out aDescription),
            TopAbs_EDGE => EdgeText(TopoDS.Edge(theShape), out aDescription),
            TopAbs_FACE => FaceText(TopoDS.Face(theShape), out aDescription),
            _ => ContainerText(theShape, out aDescription),
        };
        aKey = theShape.ShapeType() == TopAbs_VERTEX && aText.StartsWith('#') ? aText[1..] : Hash(aText);
        myKeys[theShape] = aKey;
        myDescriptions.TryAdd(aKey, aDescription);
        return aKey;
    }

    /// <summary>A readable description of the shape a key names.</summary>
    public string Describe(string theKey) => myDescriptions.TryGetValue(theKey, out string? aText) ? aText : theKey;

    /// <summary>The exact vertex key of a point (the bits of its coordinates).</summary>
    public static string PointKey(in Point3D theP) =>
        "V" + Bits(theP.X()) + Bits(theP.Y()) + Bits(theP.Z());

    private string VertexText(TopoDS_Vertex theV, out string theDescription)
    {
        Point3D aP = BRep_Tool.Pnt(theV);
        theDescription = string.Create(CultureInfo.InvariantCulture, $"vertex ({aP.X():R} {aP.Y():R} {aP.Z():R})");
        // '#' marks a ready key, not hashed: overriding keys are other vertices' keys
        return "#" + (myVertexKeyOverride?.Invoke(theV) ?? PointKey(aP));
    }

    private string EdgeText(TopoDS_Edge theE, out string theDescription)
    {
        var aText = new StringBuilder("E ");
        BRep_Tool.Range(theE, out double aFirst, out double aLast);
        Geom_Curve? aCurve = BRep_Tool.Curve(theE, out Placement aLoc, out _, out _);
        string aCurveName;
        if (BRep_Tool.Degenerated(theE))
        {
            aText.Append("degenerated");
            aCurveName = "degenerated";
        }
        else if (aCurve is null)
        {
            aText.Append("no 3D curve");
            aCurveName = "no 3D curve";
        }
        else
        {
            Geom_Curve aGlobal = aLoc.IsIdentity ? aCurve : (Geom_Curve)aCurve.Transformed(aLoc.Transformation());
            using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
            GeomTools_CurveSet.PrintCurve(aGlobal, aWriter, true);
            aText.Append(aWriter.ToString());
            aCurveName = global::Kerf.Engine.Foundation.OcctTypeNames.Of(aCurve.GetType());
        }
        aText.Append(" range ").Append(Bits(aFirst)).Append(Bits(aLast));
        aText.Append(" vertices ").Append(ChildKeys(theE, TopAbs_VERTEX, false));
        theDescription = string.Create(CultureInfo.InvariantCulture, $"edge {aCurveName} [{aFirst:R}, {aLast:R}]");
        return aText.ToString();
    }

    private string FaceText(TopoDS_Face theF, out string theDescription)
    {
        var aText = new StringBuilder("F ");
        Geom_Surface? aSurface = BRep_Tool.Surface(theF, out Placement aLoc);
        if (aSurface is not null)
        {
            Geom_Surface aGlobal = aLoc.IsIdentity ? aSurface : (Geom_Surface)aSurface.Transformed(aLoc.Transformation());
            using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
            GeomTools_SurfaceSet.PrintSurface(aGlobal, aWriter, true);
            aText.Append(aWriter.ToString());
        }
        // the face's edges as a multiset of keys (each unique edge once); with orientations, its wires
        var anEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theF, TopAbs_EDGE, anEdges);
        var aKeys = new List<string>();
        for (int i = 1; i <= anEdges.Extent(); ++i)
        {
            aKeys.Add(Key(anEdges.FindKey(i)));
        }
        aKeys.Sort(StringComparer.Ordinal);
        aText.Append(" edges ").AppendJoin(',', aKeys);
        if (myCheckOrientation)
        {
            aText.Append(" wires ").Append(ChildKeys(theF, TopAbs_WIRE, true));
        }
        int aNbEdges = aKeys.Count;
        theDescription = $"face {aSurface?.GetType().Name ?? "without surface"}, {aNbEdges} edges";
        return aText.ToString();
    }

    private string ContainerText(TopoDS_Shape theS, out string theDescription)
    {
        string aChildren = ChildKeys(theS, null, myCheckOrientation);
        int aNb = 0;
        for (var anIt = new TopoDS_Iterator(theS); anIt.More(); anIt.Next())
        {
            ++aNb;
        }
        theDescription = $"{theS.ShapeType()} of {aNb} children";
        return theS.ShapeType() + " " + aChildren;
    }

    // The sorted keys of the direct children (of the given type), with their orientations if asked.
    private string ChildKeys(TopoDS_Shape theS, TopAbs_ShapeEnum? theType, bool theWithOrientation)
    {
        var aKeys = new List<string>();
        for (var anIt = new TopoDS_Iterator(theS); anIt.More(); anIt.Next())
        {
            TopoDS_Shape aChild = anIt.Value();
            if (theType is not null && aChild.ShapeType() != theType)
            {
                continue;
            }
            string aKey = Key(aChild);
            aKeys.Add(theWithOrientation ? aKey + ":" + (int)aChild.Orientation() : aKey);
        }
        aKeys.Sort(StringComparer.Ordinal);
        return string.Join(",", aKeys);
    }

    private static string Bits(double theValue) =>
        BitConverter.DoubleToInt64Bits(theValue).ToString("X16", CultureInfo.InvariantCulture);

    private static string Hash(string theText) =>
        Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(theText)), 0, 16);
}

/// <summary>Compares two shapes canonically (decision 581).</summary>
internal static class CanonicalShapeComparer
{
    private static readonly TopAbs_ShapeEnum[] Types =
        [TopAbs_VERTEX, TopAbs_EDGE, TopAbs_WIRE, TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID, TopAbs_COMPSOLID, TopAbs_COMPOUND];

    /// <summary>Compares <paramref name="theNative"/> with <paramref name="thePort"/>.</summary>
    public static CanonicalComparison Compare(TopoDS_Shape theNative, TopoDS_Shape thePort, CanonicalOptions? theOptions = null)
    {
        CanonicalOptions anOptions = theOptions ?? new CanonicalOptions();
        var aResult = new CanonicalComparison();

        CanonicalKeys aNativeKeys = CanonicalKeys.Build(theNative, anOptions.CheckOrientation);
        CanonicalKeys aPortKeys = CanonicalKeys.Build(thePort, anOptions.CheckOrientation,
                                                      VertexMatcher(theNative, aNativeKeys, anOptions.VertexTolerance));

        foreach (TopAbs_ShapeEnum aType in Types)
        {
            List<TopoDS_Shape> aNativeShapes = SubShapes(theNative, aType);
            List<TopoDS_Shape> aPortShapes = SubShapes(thePort, aType);
            aResult.Counts[aType] = (aNativeShapes.Count, aPortShapes.Count);

            Dictionary<string, List<string>> aNativeGroups = Groups(aNativeShapes, aNativeKeys);
            Dictionary<string, List<string>> aPortGroups = Groups(aPortShapes, aPortKeys);
            foreach (string aKey in aNativeGroups.Keys.Union(aPortGroups.Keys).Order(StringComparer.Ordinal))
            {
                List<string> aNativeAttributes = aNativeGroups.GetValueOrDefault(aKey) ?? [];
                List<string> aPortAttributes = aPortGroups.GetValueOrDefault(aKey) ?? [];
                string aDescription = aNativeAttributes.Count > 0 ? aNativeKeys.Describe(aKey) : aPortKeys.Describe(aKey);
                if (aNativeAttributes.Count != aPortAttributes.Count)
                {
                    aResult.Unmatched.Add($"{aType} {aDescription} [{aKey}]: {aNativeAttributes.Count} native, {aPortAttributes.Count} port");
                    continue;
                }
                aNativeAttributes.Sort(StringComparer.Ordinal);
                aPortAttributes.Sort(StringComparer.Ordinal);
                if (!aNativeAttributes.SequenceEqual(aPortAttributes, StringComparer.Ordinal))
                {
                    aResult.AttributeDifferences.Add(
                        $"{aType} {aDescription}: native {string.Join(" | ", aNativeAttributes)}; port {string.Join(" | ", aPortAttributes)}");
                }
            }
        }

        if (anOptions.CheckOrientation && !theNative.IsNull() && !thePort.IsNull()
            && theNative.Orientation() != thePort.Orientation())
        {
            aResult.OrientationDifference = $"top-level orientation: native {theNative.Orientation()}, port {thePort.Orientation()}";
        }

        if (anOptions.CheckProperties && !theNative.IsNull() && !thePort.IsNull())
        {
            CompareProperties(theNative, thePort, anOptions.PropertyRelativeTolerance, aResult);
        }

        if (anOptions.CheckValidity)
        {
            aResult.Validity = (IsValid(theNative), IsValid(thePort));
        }
        return aResult;
    }

    /// <summary>Asserts that the shapes are canonically identical, with the comparison's report as the message.</summary>
    public static void AssertIdentical(TopoDS_Shape theNative, TopoDS_Shape thePort, CanonicalOptions? theOptions = null)
    {
        CanonicalComparison aComparison = Compare(theNative, thePort, theOptions);
        Assert.True(aComparison.IsIdentical, aComparison.ToString());
    }

    /// <summary>The unique sub-shapes of the type (TShape and location), in TopExp.MapShapes order.</summary>
    internal static List<TopoDS_Shape> SubShapes(TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aResult = new List<TopoDS_Shape>();
        if (theShape.IsNull())
        {
            return aResult;
        }
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        for (int i = 1; i <= aMap.Extent(); ++i)
        {
            aResult.Add(aMap.FindKey(i));
        }
        return aResult;
    }

    // Key -> the attributes (tolerances and flags) of each sub-shape with that key.
    private static Dictionary<string, List<string>> Groups(List<TopoDS_Shape> theShapes, CanonicalKeys theKeys)
    {
        var aGroups = new Dictionary<string, List<string>>(StringComparer.Ordinal);
        foreach (TopoDS_Shape aShape in theShapes)
        {
            string aKey = theKeys.Key(aShape);
            if (!aGroups.TryGetValue(aKey, out List<string>? aList))
            {
                aGroups[aKey] = aList = [];
            }
            aList.Add(Attributes(aShape));
        }
        return aGroups;
    }

    // The tolerance (bits) and flags of a sub-shape, but not Free, Modified, Checked or Locked.
    private static string Attributes(TopoDS_Shape theShape)
    {
        string aFlags = $"closed={B(theShape.Closed())} orientable={B(theShape.Orientable())} convex={B(theShape.Convex())} infinite={B(theShape.Infinite())}";
        return theShape.ShapeType() switch
        {
            TopAbs_VERTEX => $"tol={R(BRep_Tool.Tolerance(TopoDS.Vertex(theShape)))} {aFlags}",
            TopAbs_EDGE => $"tol={R(BRep_Tool.Tolerance(TopoDS.Edge(theShape)))} degenerated={B(BRep_Tool.Degenerated(TopoDS.Edge(theShape)))} sameparameter={B(BRep_Tool.SameParameter(TopoDS.Edge(theShape)))} samerange={B(BRep_Tool.SameRange(TopoDS.Edge(theShape)))} {aFlags}",
            TopAbs_FACE => $"tol={R(BRep_Tool.Tolerance(TopoDS.Face(theShape)))} naturalrestriction={B(BRep_Tool.NaturalRestriction(TopoDS.Face(theShape)))} {aFlags}",
            _ => aFlags,
        };

        static string B(bool theFlag) => theFlag ? "1" : "0";
        static string R(double theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);
    }

    // With a vertex tolerance, a port vertex without an exact native match takes the nearest unmatched native
    // vertex's key within the tolerance.
    private static Func<TopoDS_Vertex, string?>? VertexMatcher(TopoDS_Shape theNative, CanonicalKeys theNativeKeys, double theTolerance)
    {
        if (theTolerance <= 0.0)
        {
            return null;
        }
        var aNative = SubShapes(theNative, TopAbs_VERTEX)
                      .Select(v => (Point: BRep_Tool.Pnt(TopoDS.Vertex(v)), Key: theNativeKeys.Key(v)))
                      .ToList();
        var anExact = new HashSet<string>(aNative.Select(v => v.Key), StringComparer.Ordinal);
        var aUsed = new HashSet<int>();
        return theV =>
        {
            Point3D aP = BRep_Tool.Pnt(theV);
            string anExactKey = CanonicalKeys.PointKey(aP);
            if (anExact.Contains(anExactKey))
            {
                return anExactKey;
            }
            int aBest = -1;
            double aBestDistance = theTolerance;
            for (int i = 0; i < aNative.Count; ++i)
            {
                double aDistance = aNative[i].Point.Distance(aP);
                if (!aUsed.Contains(i) && aDistance <= aBestDistance)
                {
                    aBest = i;
                    aBestDistance = aDistance;
                }
            }
            if (aBest < 0)
            {
                return null;
            }
            aUsed.Add(aBest);
            return aNative[aBest].Key;
        };
    }

    private static void CompareProperties(TopoDS_Shape theNative, TopoDS_Shape thePort, double theRelTol, CanonicalComparison theResult)
    {
        var aNativeVolume = new GProp_GProps();
        var aPortVolume = new GProp_GProps();
        BRepGProp.VolumeProperties(theNative, aNativeVolume);
        BRepGProp.VolumeProperties(thePort, aPortVolume);
        var aNativeArea = new GProp_GProps();
        var aPortArea = new GProp_GProps();
        BRepGProp.SurfaceProperties(theNative, aNativeArea);
        BRepGProp.SurfaceProperties(thePort, aPortArea);

        Check("volume", aNativeVolume.Mass(), aPortVolume.Mass());
        Check("area", aNativeArea.Mass(), aPortArea.Mass());
        // the centroid of the volume if there is one, else of the area; compared relative to the shape's size
        bool aHasVolume = Math.Abs(aNativeVolume.Mass()) > 0.0;
        Point3D aNativeCentre = aHasVolume ? aNativeVolume.CentreOfMass() : aNativeArea.CentreOfMass();
        Point3D aPortCentre = aHasVolume ? aPortVolume.CentreOfMass() : aPortArea.CentreOfMass();
        if (Math.Abs(aNativeArea.Mass()) > 0.0)
        {
            double aScale = Math.Max(Math.Sqrt(Math.Abs(aNativeArea.Mass())), aNativeCentre.XYZ().Modulus());
            double aDistance = aNativeCentre.Distance(aPortCentre);
            if (aDistance > theRelTol * aScale)
            {
                theResult.PropertyDifferences.Add(string.Create(CultureInfo.InvariantCulture,
                    $"centroid: native ({aNativeCentre.X():R} {aNativeCentre.Y():R} {aNativeCentre.Z():R}), port ({aPortCentre.X():R} {aPortCentre.Y():R} {aPortCentre.Z():R}), distance {aDistance:R}"));
            }
        }

        void Check(string theName, double theNativeValue, double thePortValue)
        {
            double aScale = Math.Max(Math.Abs(theNativeValue), Math.Abs(thePortValue));
            if (Math.Abs(theNativeValue - thePortValue) > theRelTol * aScale)
            {
                theResult.PropertyDifferences.Add(string.Create(CultureInfo.InvariantCulture,
                    $"{theName}: native {theNativeValue:R}, port {thePortValue:R}"));
            }
        }
    }

    private static bool IsValid(TopoDS_Shape theShape) => theShape.IsNull() || new BRepCheck_Analyzer(theShape).IsValid();
}
