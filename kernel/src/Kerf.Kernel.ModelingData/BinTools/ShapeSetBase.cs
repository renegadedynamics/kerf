// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeSetBase.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeSetBase.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// The base of the binary shape writers and readers: the format version and whether triangulations and their normals
/// are written. <see cref="BinaryShapeSet"/> writes the table-based format (versions 1 to 4);
/// <see cref="ShapeWriter"/> and <see cref="ShapeReader"/> the position-based one.
/// </summary>
internal class ShapeSetBase
{
    // the header line of each format version (index 0 is unused)
    internal static readonly string[] THE_ASCII_VERSIONS =
    [
        "",
        "Open CASCADE Topology V1 (c)",
        "Open CASCADE Topology V2 (c)",
        "Open CASCADE Topology V3 (c)",
        "Open CASCADE Topology V4, (c) Open Cascade",
    ];

    private int _formatNb;
    private bool _withTriangles;
    private bool _withNormals;

    /// <summary>The current format version, without triangulations or normals.</summary>
    public ShapeSetBase()
    {
        _formatNb = (int)BinaryFormatVersion.FormatVersion_CURRENT;
        _withTriangles = false;
        _withNormals = false;
    }

    /// <summary>True if shapes are written with their triangulations.</summary>
    public bool IsWithTriangles => _withTriangles;

    /// <summary>True if triangulations are written with their normals.</summary>
    public bool IsWithNormals => _withNormals;

    /// <summary>
    /// Sets whether shapes are written with their triangulations (always, for a face with only a triangulation).
    /// </summary>
    public void SetWithTriangles(bool withTriangles) => _withTriangles = withTriangles;

    /// <summary>
    /// Sets whether triangulations are written with their normals (always, for a face with only a triangulation).
    /// </summary>
    public void SetWithNormals(bool withNormals) => _withNormals = withNormals;

    /// <summary>Sets the <see cref="BinaryFormatVersion"/>; a version out of range is ignored, as in OCCT.</summary>
    public void SetFormatNb(int formatNb)
    {
        // Standard_ASSERT_RETURN: reports (in debug builds) and returns
        if (!(formatNb >= (int)BinaryFormatVersionBounds.BinTools_FormatVersion_LOWER
              && formatNb <= (int)BinaryFormatVersionBounds.BinTools_FormatVersion_UPPER))
        {
            return;
        }

        _formatNb = formatNb;
    }

    /// <summary>The <see cref="BinaryFormatVersion"/>.</summary>
    public int FormatNb() => _formatNb;

    /// <summary>Clears the content of the set.</summary>
    public virtual void Clear()
    {
    }

    /// <summary>Writes the content of the set in binary format.</summary>
    public virtual void Write(Stream os, ProgressRange range = default)
    {
    }

    /// <summary>Clears the set and reads its content from the binary stream.</summary>
    public virtual void Read(Stream IS, ProgressRange range = default)
    {
    }

    /// <summary>Writes a shape: its orientation, the index of its TShape and the index of its location.</summary>
    public virtual void Write(in Shape shape, Stream stream)
    {
    }

    /// <summary>Reads a shape (redefined by <see cref="ShapeReader"/>). Leaves the shape unchanged here.</summary>
    public virtual void Read(Stream stream, ref Shape shape)
    {
    }

    // operator<<(Standard_OStream&, const gp_Pnt&)
    internal static void Write(Stream os, in Point3D p)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, p.X);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, p.Y);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutReal(os, p.Z);
    }
}
