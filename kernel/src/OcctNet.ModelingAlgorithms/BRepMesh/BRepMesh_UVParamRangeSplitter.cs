// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_UVParamRangeSplitter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Extends the default range splitter with maps of U and V parameters, kept in the order they are added
/// (<c>IMeshData::IMapOfReal</c>).
/// </summary>
public class BRepMesh_UVParamRangeSplitter : BRepMesh_DefaultRangeSplitter
{
    private readonly NCollection_IndexedMap<double> myUParams = new();
    private readonly NCollection_IndexedMap<double> myVParams = new();

    /// <summary>A splitter.</summary>
    public BRepMesh_UVParamRangeSplitter()
    {
    }

    /// <summary>Resets the splitter for the face. Must be called before the first use.</summary>
    public override void Reset(IMeshData_Face theDFace, in IMeshTools_Parameters theParameters)
    {
        base.Reset(theDFace, theParameters);
        myUParams.Clear();
        myVParams.Clear();
    }

    /// <summary>The U parameters (the splitter's own map: OCCT's const and non-const accessors are one).</summary>
    public NCollection_IndexedMap<double> GetParametersU() => myUParams;

    /// <summary>The V parameters (the splitter's own map: OCCT's const and non-const accessors are one).</summary>
    public NCollection_IndexedMap<double> GetParametersV() => myVParams;
}
