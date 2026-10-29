// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeSetBase.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_ShapeSetBase.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The base of the binary shape writers and readers: the format version and whether triangulations and their normals
/// are written. <see cref="BinTools_ShapeSet"/> writes the table-based format (versions 1 to 4);
/// <see cref="BinTools_ShapeWriter"/> and <see cref="BinTools_ShapeReader"/> the position-based one.
/// </summary>
public class BinTools_ShapeSetBase
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

    private int myFormatNb;
    private bool myWithTriangles;
    private bool myWithNormals;

    /// <summary>The current format version, without triangulations or normals.</summary>
    public BinTools_ShapeSetBase()
    {
        myFormatNb = (int)BinTools_FormatVersion.BinTools_FormatVersion_CURRENT;
        myWithTriangles = false;
        myWithNormals = false;
    }

    /// <summary>True if shapes are written with their triangulations.</summary>
    public bool IsWithTriangles() => myWithTriangles;

    /// <summary>True if triangulations are written with their normals.</summary>
    public bool IsWithNormals() => myWithNormals;

    /// <summary>
    /// Sets whether shapes are written with their triangulations (always, for a face with only a triangulation).
    /// </summary>
    public void SetWithTriangles(bool theWithTriangles) => myWithTriangles = theWithTriangles;

    /// <summary>
    /// Sets whether triangulations are written with their normals (always, for a face with only a triangulation).
    /// </summary>
    public void SetWithNormals(bool theWithNormals) => myWithNormals = theWithNormals;

    /// <summary>Sets the <see cref="BinTools_FormatVersion"/>; a version out of range is ignored, as in OCCT.</summary>
    public void SetFormatNb(int theFormatNb)
    {
        // Standard_ASSERT_RETURN: reports (in debug builds) and returns
        if (!(theFormatNb >= (int)BinTools_FormatVersionBounds.BinTools_FormatVersion_LOWER
              && theFormatNb <= (int)BinTools_FormatVersionBounds.BinTools_FormatVersion_UPPER))
        {
            return;
        }

        myFormatNb = theFormatNb;
    }

    /// <summary>The <see cref="BinTools_FormatVersion"/>.</summary>
    public int FormatNb() => myFormatNb;

    /// <summary>Clears the content of the set.</summary>
    public virtual void Clear()
    {
    }

    /// <summary>Writes the content of the set in binary format.</summary>
    public virtual void Write(Stream OS, Message_ProgressRange theRange = default)
    {
    }

    /// <summary>Clears the set and reads its content from the binary stream.</summary>
    public virtual void Read(Stream IS, Message_ProgressRange theRange = default)
    {
    }

    /// <summary>Writes a shape: its orientation, the index of its TShape and the index of its location.</summary>
    public virtual void Write(in TopoDS_Shape theShape, Stream theStream)
    {
    }

    /// <summary>Reads a shape (redefined by <see cref="BinTools_ShapeReader"/>). Leaves the shape unchanged here.</summary>
    public virtual void Read(Stream theStream, ref TopoDS_Shape theShape)
    {
    }

    // operator<<(Standard_OStream&, const gp_Pnt&)
    internal static void Write(Stream OS, in gp_Pnt P)
    {
        BinTools.PutReal(OS, P.X());
        BinTools.PutReal(OS, P.Y());
        BinTools.PutReal(OS, P.Z());
    }
}
