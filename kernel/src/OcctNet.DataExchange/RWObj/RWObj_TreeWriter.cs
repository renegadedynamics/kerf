// New in OCCT.NET (unit 8T): Wavefront OBJ written fresh over the assembly tree (decisions 1166, 1234). Not a port of
// OCCT's RWObj_CafWriter, which needs XCAF; its conventions are kept where they show: a group per placed part named by
// its path, MTL diffuse colours in sRGB, transparency as "Tr".
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using System.Globalization;
using System.Text;

namespace Occt;

/// <summary>The options of <see cref="RWObj_TreeWriter"/>.</summary>
public sealed record RWObj_WriteOptions
{
    /// <summary>The defaults.</summary>
    public static RWObj_WriteOptions Default { get; } = new();

    /// <summary>The factor applied to coordinates (1: the model's unit).</summary>
    public double Scale { get; init; } = 1.0;

    /// <summary>Turn OCCT's Z-up system into the Y-up one most OBJ readers expect (off by default).</summary>
    public bool ZUpToYUp { get; init; }

    /// <summary>Write normals (<c>vn</c>): the triangulations', or computed from the surfaces.</summary>
    public bool WriteNormals { get; init; } = true;

    /// <summary>Write hidden parts too.</summary>
    public bool IncludeHidden { get; init; }

    /// <summary>If set, the tree is meshed first (<see cref="XdeDoc_Tree.Meshed"/>).</summary>
    public ToolParameters? MeshParameters { get; init; }
}

/// <summary>
/// Writes an assembly tree as Wavefront OBJ, with its colours as an MTL material library.
/// </summary>
/// <remarks>
/// OBJ has no instances: each placed part (<see cref="XdeDoc_Tree.Flatten"/>) is written in world coordinates as an
/// object (<c>o</c>) named by its path ("assembly/part"), with a <c>usemtl</c> per colour group. Materials are named
/// <c>mat_N</c>: <c>Kd</c> the colour in sRGB, <c>d</c> and <c>Tr</c> its opacity and transparency; faces without colour
/// after faces with one take a white material, <c>default</c>. Streams first; the
/// file form writes the MTL beside the OBJ. Numbers in the invariant culture, LF line ends. The tree isn't changed.
/// </remarks>
public sealed class RWObj_TreeWriter
{
    /// <summary>A writer with the options (the defaults if null).</summary>
    public RWObj_TreeWriter(RWObj_WriteOptions? theOptions = null) => Options = theOptions ?? RWObj_WriteOptions.Default;

    /// <summary>The options.</summary>
    public RWObj_WriteOptions Options { get; }

    /// <summary>
    /// Writes the tree's OBJ to <paramref name="theObj"/> and its materials to <paramref name="theMtl"/> (if not null),
    /// which the OBJ names as <paramref name="theMtlName"/>. False if cancelled or a stream fails.
    /// </summary>
    public bool Write(XdeDoc_Tree theTree, Stream theObj, Stream? theMtl, string theMtlName = "model.mtl", ProgressRange theProgress = default)
    {
        ArgumentNullException.ThrowIfNull(theTree);
        ArgumentNullException.ThrowIfNull(theObj);
        using var aPS = new ProgressScope(theProgress, "Writing OBJ", Options.MeshParameters is null ? 1 : 2);
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

        var aConverter = new RWMesh_CoordinateConverter(Options.Scale, Options.ZUpToYUp);
        IReadOnlyList<XdeDoc_FlatItem> anItems = aTree.Flatten(Options.IncludeHidden);
        var aMeshes = new Dictionary<XdeDoc_Product, RWMesh_ProductMesh>(ReferenceEqualityComparer.Instance);
        var aMaterials = new List<ColorWithAlpha>();
        var aMaterialIndex = new Dictionary<ColorWithAlpha, int>();
        var anObj = new StringBuilder();
        anObj.Append("# OCCT.NET\n");
        if (theMtl is not null)
        {
            anObj.Append("mtllib ").Append(theMtlName).Append('\n');
        }

        int aNodeBase = 1, aNormalBase = 1;
        string? aCurrentMaterial = null;
        bool hasDefault = false;
        using var anItemsPS = new ProgressScope(aPS.Next(), "Parts", Math.Max(1, anItems.Count));
        foreach (XdeDoc_FlatItem anItem in anItems)
        {
            if (!anItemsPS.More())
            {
                return false;
            }
            anItemsPS.Next();
            if (!aMeshes.TryGetValue(anItem.Product, out RWMesh_ProductMesh? aMesh))
            {
                aMesh = RWMesh_ProductMesh.Make(anItem.Product, Options.WriteNormals);
                aMeshes.Add(anItem.Product, aMesh);
            }
            if (aMesh.Groups.Count == 0)
            {
                continue;
            }
            Transform3D aTrsf = anItem.Location.Transformation();
            bool isIdentity = anItem.Location.IsIdentity;
            bool isMirrored = aTrsf.VectorialPart().Determinant() < 0.0;
            anObj.Append("o ").Append(objName(anItem)).Append('\n');
            ColorWithAlpha? aNodeColor = anItem.Style.SurfaceColor ?? anItem.Style.CurveColor;
            foreach (RWMesh_ProductMesh.Group aGroup in aMesh.Groups)
            {
                for (int i = 0; i < aGroup.NbNodes; i++)
                {
                    var aP = new Point3D(aGroup.Positions[3 * i], aGroup.Positions[3 * i + 1], aGroup.Positions[3 * i + 2]);
                    if (!isIdentity)
                    {
                        aP = aP.Transformed(aTrsf);
                    }
                    double aX = aP.X(), aY = aP.Y(), aZ = aP.Z();
                    aConverter.Map(ref aX, ref aY, ref aZ);
                    anObj.Append("v ").Append(num(aX)).Append(' ').Append(num(aY)).Append(' ').Append(num(aZ)).Append('\n');
                }
                bool hasNormals = aGroup.Normals.Count == aGroup.Positions.Count && aGroup.NbNodes > 0;
                if (hasNormals)
                {
                    for (int i = 0; i < aGroup.NbNodes; i++)
                    {
                        var aN = new Direction3D(aGroup.Normals[3 * i], aGroup.Normals[3 * i + 1], aGroup.Normals[3 * i + 2]);
                        if (!isIdentity)
                        {
                            aN = aN.Transformed(aTrsf);
                        }
                        double aX = aN.X(), aY = aN.Y(), aZ = aN.Z();
                        aConverter.Map(ref aX, ref aY, ref aZ, true);
                        anObj.Append("vn ").Append(num((float)aX)).Append(' ').Append(num((float)aY)).Append(' ').Append(num((float)aZ)).Append('\n');
                    }
                }
                if (theMtl is not null && (aGroup.Color ?? aNodeColor) is ColorWithAlpha aColor)
                {
                    if (!aMaterialIndex.TryGetValue(aColor, out int aMat))
                    {
                        aMat = aMaterials.Count;
                        aMaterials.Add(aColor);
                        aMaterialIndex.Add(aColor, aMat);
                    }
                    string aName = "mat_" + aMat.ToString(CultureInfo.InvariantCulture);
                    if (aName != aCurrentMaterial)
                    {
                        anObj.Append("usemtl ").Append(aName).Append('\n');
                        aCurrentMaterial = aName;
                    }
                }
                else if (theMtl is not null && aCurrentMaterial is not null && aCurrentMaterial != "default")
                {
                    // a group without colour after one with: OBJ keeps the material, so a neutral one is named
                    anObj.Append("usemtl default\n");
                    aCurrentMaterial = "default";
                    hasDefault = true;
                }
                for (int t = 0; t < aGroup.NbTriangles; t++)
                {
                    int a = aGroup.Indices[3 * t], b = aGroup.Indices[3 * t + 1], c = aGroup.Indices[3 * t + 2];
                    if (isMirrored)
                    {
                        (b, c) = (c, b);
                    }
                    anObj.Append('f');
                    foreach (int aNode in new[] { a, b, c })
                    {
                        anObj.Append(' ').Append((aNodeBase + aNode).ToString(CultureInfo.InvariantCulture));
                        if (hasNormals)
                        {
                            anObj.Append("//").Append((aNormalBase + aNode).ToString(CultureInfo.InvariantCulture));
                        }
                    }
                    anObj.Append('\n');
                }
                aNodeBase += aGroup.NbNodes;
                if (hasNormals)
                {
                    aNormalBase += aGroup.NbNodes;
                }
            }
        }

        try
        {
            theObj.Write(Encoding.UTF8.GetBytes(anObj.ToString()));
            theObj.Flush();
            if (theMtl is not null)
            {
                var aMtl = new StringBuilder("# OCCT.NET\n");
                for (int i = 0; i < aMaterials.Count; i++)
                {
                    Color aRgb = aMaterials[i].GetRGB();
                    aRgb.Values(out double aR, out double aG, out double aB, ColorSpace.TOC_sRGB);
                    float anAlpha = aMaterials[i].Alpha();
                    aMtl.Append("newmtl mat_").Append(i.ToString(CultureInfo.InvariantCulture)).Append('\n');
                    aMtl.Append("Kd ").Append(num((float)aR)).Append(' ').Append(num((float)aG)).Append(' ').Append(num((float)aB)).Append('\n');
                    aMtl.Append("d ").Append(num(anAlpha)).Append('\n');
                    if (anAlpha < 1.0f)
                    {
                        aMtl.Append("Tr ").Append(num(1.0f - anAlpha)).Append('\n');
                    }
                }
                if (hasDefault)
                {
                    aMtl.Append("newmtl default\nKd 1 1 1\nd 1\n");
                }
                theMtl.Write(Encoding.UTF8.GetBytes(aMtl.ToString()));
                theMtl.Flush();
            }
        }
        catch (IOException)
        {
            return false;
        }
        return true;
    }

    /// <summary>Writes the tree to an OBJ file, with its MTL file of the same name beside it.</summary>
    public bool Perform(XdeDoc_Tree theTree, string theFilePath, ProgressRange theProgress = default)
    {
        string aMtlPath = Path.ChangeExtension(theFilePath, ".mtl");
        using FileStream anObj = File.Create(theFilePath);
        using FileStream aMtl = File.Create(aMtlPath);
        return Write(theTree, anObj, aMtl, Path.GetFileName(aMtlPath), theProgress);
    }

    private static string objName(XdeDoc_FlatItem theItem)
    {
        string aName = string.Join("/", theItem.NamePath.Select(n => n ?? ""));
        var aBuilder = new StringBuilder(aName.Length);
        foreach (char aChar in aName)
        {
            aBuilder.Append(char.IsWhiteSpace(aChar) || char.IsControl(aChar) ? '_' : aChar);
        }
        return aBuilder.Length == 0 ? "part" : aBuilder.ToString();
    }

    private static string num(double theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);

    private static string num(float theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);
}
