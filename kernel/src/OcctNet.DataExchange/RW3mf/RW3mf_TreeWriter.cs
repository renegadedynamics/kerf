// New in OCCT.NET (unit 8T): 3MF written fresh over the assembly tree (decisions 1166, 1235); OCCT has no 3MF.
// The 3MF Core Specification 1.3: an OPC package (a zip) of the content types, the root relationship and the model
// part; objects with meshes or components, build items with transforms, colours as core base materials.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using System.Xml;

namespace Occt;

/// <summary>The options of <see cref="RW3mf_TreeWriter"/>.</summary>
public sealed record RW3mf_WriteOptions
{
    /// <summary>The defaults.</summary>
    public static RW3mf_WriteOptions Default { get; } = new();

    /// <summary>
    /// The size of the model's length unit in metres; null to take the tree's <see cref="XdeDoc_Tree.LengthUnit"/>, or
    /// millimetres if it has none. A unit 3MF names (micron, millimetre, centimetre, inch, foot, metre) is written as
    /// is; another is converted to millimetres.
    /// </summary>
    public double? InputLengthUnit { get; init; }

    /// <summary>Write hidden parts too.</summary>
    public bool IncludeHidden { get; init; }

    /// <summary>If set, the tree is meshed first (<see cref="XdeDoc_Tree.Meshed"/>).</summary>
    public ToolParameters? MeshParameters { get; init; }
}

/// <summary>
/// Writes an assembly tree as a 3MF package.
/// </summary>
/// <remarks>
/// <para>
/// Each part becomes a mesh object (its faces' triangulations, in the part's coordinates), each assembly an object of
/// components with their transforms, each root a build item; objects are written before the objects using them, as
/// the specification requires. Colours are core base materials (sRGB with alpha): an object's default material is its
/// instance's colour, and triangles of faces with a sub-shape colour name theirs. A product placed with different
/// colours becomes an object per colour; parts without triangles are left out.
/// </para>
/// <para>
/// The package is deterministic (fixed entry times). Streams first; the stream must be writable (and is left open).
/// The tree isn't changed.
/// </para>
/// </remarks>
public sealed class RW3mf_TreeWriter
{
    /// <summary>The namespace of the 3MF core model.</summary>
    public const string CoreNamespace = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";

    private const string ContentTypesNamespace = "http://schemas.openxmlformats.org/package/2006/content-types";
    private const string RelationshipsNamespace = "http://schemas.openxmlformats.org/package/2006/relationships";
    private const string ModelRelationshipType = "http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel";
    private static readonly DateTimeOffset theEntryTime = new(1980, 1, 1, 0, 0, 0, TimeSpan.Zero);

    /// <summary>A writer with the options (the defaults if null).</summary>
    public RW3mf_TreeWriter(RW3mf_WriteOptions? theOptions = null) => Options = theOptions ?? RW3mf_WriteOptions.Default;

    /// <summary>The options.</summary>
    public RW3mf_WriteOptions Options { get; }

    /// <summary>Writes the tree as a 3MF package to the stream. False if cancelled or the stream fails.</summary>
    public bool Write(XdeDoc_Tree theTree, Stream theStream, ProgressRange theProgress = default)
    {
        ArgumentNullException.ThrowIfNull(theTree);
        ArgumentNullException.ThrowIfNull(theStream);
        using var aPS = new ProgressScope(theProgress, "Writing 3MF", Options.MeshParameters is null ? 1 : 2);
        XdeDoc_Tree aTree = theTree;
        if (Options.MeshParameters is ToolParameters aParams)
        {
            XdeDoc_Tree? aMeshed = theTree.Meshed(aParams, aPS.Next());
            if (aMeshed is null)
            {
                return false;
            }
            aTree = aMeshed;
        }
        byte[]? aModel = buildModel(aTree, aPS.Next());
        if (aModel is null)
        {
            return false;
        }

        try
        {
            using var aZip = new ZipArchive(theStream, ZipArchiveMode.Create, true);
            writeEntry(aZip, "[Content_Types].xml", contentTypes());
            writeEntry(aZip, "_rels/.rels", relationships());
            writeEntry(aZip, "3D/3dmodel.model", aModel);
        }
        catch (IOException)
        {
            return false;
        }
        return true;
    }

    /// <summary>Writes the tree to a 3MF file.</summary>
    public bool Perform(XdeDoc_Tree theTree, string theFilePath, ProgressRange theProgress = default)
    {
        using FileStream aFile = File.Create(theFilePath);
        return Write(theTree, aFile, theProgress);
    }

    private static void writeEntry(ZipArchive theZip, string theName, byte[] theData)
    {
        ZipArchiveEntry anEntry = theZip.CreateEntry(theName, CompressionLevel.Optimal);
        anEntry.LastWriteTime = theEntryTime;
        using Stream aStream = anEntry.Open();
        aStream.Write(theData);
    }

    private static XmlWriterSettings settings() => new()
    {
        Encoding = new UTF8Encoding(false),
        Indent = false,
        NewLineChars = "\n",
    };

    private static byte[] contentTypes()
    {
        using var aStream = new MemoryStream();
        using (var aW = XmlWriter.Create(aStream, settings()))
        {
            aW.WriteStartDocument();
            aW.WriteStartElement("Types", ContentTypesNamespace);
            aW.WriteStartElement("Default", ContentTypesNamespace);
            aW.WriteAttributeString("Extension", "rels");
            aW.WriteAttributeString("ContentType", "application/vnd.openxmlformats-package.relationships+xml");
            aW.WriteEndElement();
            aW.WriteStartElement("Default", ContentTypesNamespace);
            aW.WriteAttributeString("Extension", "model");
            aW.WriteAttributeString("ContentType", "application/vnd.ms-package.3dmanufacturing-3dmodel+xml");
            aW.WriteEndElement();
            aW.WriteEndElement();
            aW.WriteEndDocument();
        }
        return aStream.ToArray();
    }

    private static byte[] relationships()
    {
        using var aStream = new MemoryStream();
        using (var aW = XmlWriter.Create(aStream, settings()))
        {
            aW.WriteStartDocument();
            aW.WriteStartElement("Relationships", RelationshipsNamespace);
            aW.WriteStartElement("Relationship", RelationshipsNamespace);
            aW.WriteAttributeString("Target", "/3D/3dmodel.model");
            aW.WriteAttributeString("Id", "rel0");
            aW.WriteAttributeString("Type", ModelRelationshipType);
            aW.WriteEndElement();
            aW.WriteEndElement();
            aW.WriteEndDocument();
        }
        return aStream.ToArray();
    }

    /// <summary>The 3MF unit name of a unit size in metres, or null if 3MF has none.</summary>
    internal static string? UnitName(double theUnitValue)
    {
        static bool near(double a, double b) => Math.Abs(a - b) <= 1e-9 * b;
        return near(theUnitValue, 1e-6) ? "micron"
             : near(theUnitValue, 0.001) ? "millimeter"
             : near(theUnitValue, 0.01) ? "centimeter"
             : near(theUnitValue, 0.0254) ? "inch"
             : near(theUnitValue, 0.3048) ? "foot"
             : near(theUnitValue, 1.0) ? "meter"
             : null;
    }

    private sealed class ObjectDef
    {
        public int Id;
        public string? Name;
        public RWMesh_ProductMesh? Mesh;
        public int DefaultMaterial = -1;
        public List<int>? GroupMaterials;
        public List<(int ObjectId, Transform3D? Trsf)>? Components;
    }

    private byte[]? buildModel(XdeDoc_Tree theTree, ProgressRange theProgress)
    {
        double anInputUnit = Options.InputLengthUnit ?? theTree.LengthUnit?.UnitValue ?? 0.001;
        string? aUnitName = UnitName(anInputUnit);
        double aScale = 1.0;
        if (aUnitName is null)
        {
            aUnitName = "millimeter";
            aScale = anInputUnit / 0.001;
        }

        var aMaterials = new List<ColorWithAlpha>();
        var aMaterialIndex = new Dictionary<ColorWithAlpha, int>();
        int materialOf(ColorWithAlpha theColor)
        {
            if (!aMaterialIndex.TryGetValue(theColor, out int anIndex))
            {
                anIndex = aMaterials.Count;
                aMaterials.Add(theColor);
                aMaterialIndex.Add(theColor, anIndex);
            }
            return anIndex;
        }

        var anObjects = new List<ObjectDef>();
        var aKnown = new Dictionary<(XdeDoc_Product, ColorWithAlpha?, ColorWithAlpha?), ObjectDef?>();
        var aMeshes = new Dictionary<XdeDoc_Product, RWMesh_ProductMesh>(ReferenceEqualityComparer.Instance);
        var aBuild = new List<(int ObjectId, Transform3D? Trsf)>();
        int aNextId = 2; // 1 is the base materials
        int aNbParts = theTree.Products().Count(p => !p.IsAssembly);
        using var aPS = new ProgressScope(theProgress, "Objects", Math.Max(1, aNbParts));
        bool isCancelled = false;

        ObjectDef? objectOf(XdeDoc_Product theProduct, XdeDoc_Style theStyle)
        {
            ColorWithAlpha? aColor = theStyle.SurfaceColor ?? theStyle.CurveColor;
            if (aKnown.TryGetValue((theProduct, theStyle.SurfaceColor, theStyle.CurveColor), out ObjectDef? aDef))
            {
                return aDef;
            }
            if (theProduct.IsAssembly)
            {
                var aComponents = new List<(int, Transform3D?)>();
                foreach (XdeDoc_Instance aComponent in theProduct.Components)
                {
                    XdeDoc_Style aStyle = componentStyle(theTree, aComponent, theStyle);
                    if (!aStyle.IsVisible && !Options.IncludeHidden)
                    {
                        continue;
                    }
                    ObjectDef? aChild = objectOf(aComponent.Product, aStyle);
                    if (isCancelled)
                    {
                        return null;
                    }
                    if (aChild is not null)
                    {
                        aComponents.Add((aChild.Id, aComponent.Location.IsIdentity ? null : aComponent.Location.Transformation()));
                    }
                }
                aDef = aComponents.Count == 0 ? null : new ObjectDef { Name = theProduct.Name, Components = aComponents };
            }
            else
            {
                if (!aMeshes.TryGetValue(theProduct, out RWMesh_ProductMesh? aMesh))
                {
                    if (!aPS.More())
                    {
                        isCancelled = true;
                        return null;
                    }
                    aPS.Next();
                    aMesh = RWMesh_ProductMesh.Make(theProduct, false);
                    aMeshes.Add(theProduct, aMesh);
                }
                if (aMesh.NbTriangles == 0)
                {
                    aDef = null;
                }
                else
                {
                    aDef = new ObjectDef { Name = theProduct.Name, Mesh = aMesh };
                    bool hasColor = aColor is not null || aMesh.Groups.Any(g => g.Color is not null);
                    if (hasColor)
                    {
                        // a part with colours but none of its own takes white where its faces have none
                        aDef.DefaultMaterial = materialOf(aColor ?? new ColorWithAlpha(1.0f, 1.0f, 1.0f, 1.0f));
                        aDef.GroupMaterials = aMesh.Groups.Select(g => g.Color is ColorWithAlpha c ? materialOf(c) : -1).ToList();
                    }
                }
            }
            if (aDef is not null)
            {
                aDef.Id = aNextId++;
                anObjects.Add(aDef);
            }
            aKnown[(theProduct, theStyle.SurfaceColor, theStyle.CurveColor)] = aDef;
            return aDef;
        }

        foreach (XdeDoc_Instance aRoot in theTree.Roots)
        {
            XdeDoc_Style aStyle = componentStyle(theTree, aRoot, XdeDoc_Style.Default);
            if (!aStyle.IsVisible && !Options.IncludeHidden)
            {
                continue;
            }
            ObjectDef? aDef = objectOf(aRoot.Product, aStyle);
            if (isCancelled)
            {
                return null;
            }
            if (aDef is not null)
            {
                aBuild.Add((aDef.Id, aRoot.Location.IsIdentity ? null : aRoot.Location.Transformation()));
            }
        }

        using var aStream = new MemoryStream();
        using (var aW = XmlWriter.Create(aStream, settings()))
        {
            aW.WriteStartDocument();
            aW.WriteStartElement("model", CoreNamespace);
            aW.WriteAttributeString("unit", aUnitName);
            aW.WriteAttributeString("xml", "lang", null, "en-US");
            aW.WriteStartElement("metadata", CoreNamespace);
            aW.WriteAttributeString("name", "Application");
            aW.WriteString("OCCT.NET");
            aW.WriteEndElement();

            aW.WriteStartElement("resources", CoreNamespace);
            if (aMaterials.Count > 0)
            {
                aW.WriteStartElement("basematerials", CoreNamespace);
                aW.WriteAttributeString("id", "1");
                for (int i = 0; i < aMaterials.Count; i++)
                {
                    aW.WriteStartElement("base", CoreNamespace);
                    aW.WriteAttributeString("name", "mat_" + i.ToString(CultureInfo.InvariantCulture));
                    aW.WriteAttributeString("displaycolor", DisplayColor(aMaterials[i]));
                    aW.WriteEndElement();
                }
                aW.WriteEndElement();
            }
            foreach (ObjectDef anObject in anObjects)
            {
                aW.WriteStartElement("object", CoreNamespace);
                aW.WriteAttributeString("id", anObject.Id.ToString(CultureInfo.InvariantCulture));
                aW.WriteAttributeString("type", "model");
                if (anObject.Name is not null)
                {
                    aW.WriteAttributeString("name", anObject.Name);
                }
                if (anObject.DefaultMaterial >= 0)
                {
                    aW.WriteAttributeString("pid", "1");
                    aW.WriteAttributeString("pindex", anObject.DefaultMaterial.ToString(CultureInfo.InvariantCulture));
                }
                if (anObject.Mesh is RWMesh_ProductMesh aMesh)
                {
                    writeMesh(aW, aMesh, anObject, aScale);
                }
                else
                {
                    aW.WriteStartElement("components", CoreNamespace);
                    foreach ((int anId, Transform3D? aTrsf) in anObject.Components!)
                    {
                        aW.WriteStartElement("component", CoreNamespace);
                        aW.WriteAttributeString("objectid", anId.ToString(CultureInfo.InvariantCulture));
                        if (aTrsf is Transform3D aT)
                        {
                            aW.WriteAttributeString("transform", Transform(aT, aScale));
                        }
                        aW.WriteEndElement();
                    }
                    aW.WriteEndElement();
                }
                aW.WriteEndElement();
            }
            aW.WriteEndElement();

            aW.WriteStartElement("build", CoreNamespace);
            foreach ((int anId, Transform3D? aTrsf) in aBuild)
            {
                aW.WriteStartElement("item", CoreNamespace);
                aW.WriteAttributeString("objectid", anId.ToString(CultureInfo.InvariantCulture));
                if (aTrsf is Transform3D aT)
                {
                    aW.WriteAttributeString("transform", Transform(aT, aScale));
                }
                aW.WriteEndElement();
            }
            aW.WriteEndElement();
            aW.WriteEndElement();
            aW.WriteEndDocument();
        }
        return aStream.ToArray();
    }

    private static XdeDoc_Style componentStyle(XdeDoc_Tree theTree, XdeDoc_Instance theInstance, XdeDoc_Style theParent)
    {
        XdeDoc_Style aStyle = theParent.Merged(theInstance.Product.Attributes).Merged(theInstance.Attributes);
        if (theTree.IsInHiddenLayer(theInstance.Product.Attributes) || theTree.IsInHiddenLayer(theInstance.Attributes))
        {
            aStyle = aStyle with { IsVisible = false };
        }
        return aStyle;
    }

    private static void writeMesh(XmlWriter theW, RWMesh_ProductMesh theMesh, ObjectDef theObject, double theScale)
    {
        theW.WriteStartElement("mesh", CoreNamespace);
        theW.WriteStartElement("vertices", CoreNamespace);
        foreach (RWMesh_ProductMesh.Group aGroup in theMesh.Groups)
        {
            for (int i = 0; i < aGroup.NbNodes; i++)
            {
                theW.WriteStartElement("vertex", CoreNamespace);
                theW.WriteAttributeString("x", Number(aGroup.Positions[3 * i] * theScale));
                theW.WriteAttributeString("y", Number(aGroup.Positions[3 * i + 1] * theScale));
                theW.WriteAttributeString("z", Number(aGroup.Positions[3 * i + 2] * theScale));
                theW.WriteEndElement();
            }
        }
        theW.WriteEndElement();
        theW.WriteStartElement("triangles", CoreNamespace);
        int aBase = 0;
        for (int g = 0; g < theMesh.Groups.Count; g++)
        {
            RWMesh_ProductMesh.Group aGroup = theMesh.Groups[g];
            int aMaterial = theObject.GroupMaterials is null ? -1 : theObject.GroupMaterials[g];
            for (int t = 0; t < aGroup.NbTriangles; t++)
            {
                int a = aGroup.Indices[3 * t], b = aGroup.Indices[3 * t + 1], c = aGroup.Indices[3 * t + 2];
                if (a == b || b == c || a == c)
                {
                    continue; // 3MF: a triangle's three vertices must differ
                }
                theW.WriteStartElement("triangle", CoreNamespace);
                theW.WriteAttributeString("v1", (aBase + a).ToString(CultureInfo.InvariantCulture));
                theW.WriteAttributeString("v2", (aBase + b).ToString(CultureInfo.InvariantCulture));
                theW.WriteAttributeString("v3", (aBase + c).ToString(CultureInfo.InvariantCulture));
                if (aMaterial >= 0 && aMaterial != theObject.DefaultMaterial)
                {
                    theW.WriteAttributeString("pid", "1");
                    theW.WriteAttributeString("p1", aMaterial.ToString(CultureInfo.InvariantCulture));
                }
                theW.WriteEndElement();
            }
            aBase += aGroup.NbNodes;
        }
        theW.WriteEndElement();
        theW.WriteEndElement();
    }

    /// <summary>A colour as 3MF writes it: "#RRGGBB" (sRGB), with "AA" if not opaque.</summary>
    internal static string DisplayColor(in ColorWithAlpha theColor)
    {
        theColor.GetRGB().Values(out double aR, out double aG, out double aB, ColorSpace.TOC_sRGB);
        static int toByte(double v) => (int)Math.Round(Math.Clamp(v, 0.0, 1.0) * 255.0);
        string aHex = string.Create(CultureInfo.InvariantCulture, $"#{toByte(aR):X2}{toByte(aG):X2}{toByte(aB):X2}");
        float anAlpha = theColor.Alpha();
        return anAlpha >= 1.0f ? aHex : aHex + toByte(anAlpha).ToString("X2", CultureInfo.InvariantCulture);
    }

    /// <summary>
    /// A transformation as 3MF writes it: "m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32", applied to row vectors
    /// (the transpose of OCCT's matrix), the translation scaled.
    /// </summary>
    internal static string Transform(in Transform3D theTrsf, double theScale)
    {
        double[] aValues =
        [
            theTrsf.Value(1, 1), theTrsf.Value(2, 1), theTrsf.Value(3, 1),
            theTrsf.Value(1, 2), theTrsf.Value(2, 2), theTrsf.Value(3, 2),
            theTrsf.Value(1, 3), theTrsf.Value(2, 3), theTrsf.Value(3, 3),
            theTrsf.Value(1, 4) * theScale, theTrsf.Value(2, 4) * theScale, theTrsf.Value(3, 4) * theScale,
        ];
        return string.Join(" ", aValues.Select(Number));
    }

    internal static string Number(double theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);
}
