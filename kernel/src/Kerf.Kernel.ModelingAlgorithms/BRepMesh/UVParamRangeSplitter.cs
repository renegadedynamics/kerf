// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_UVParamRangeSplitter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Meshing.Model;

using Kerf.Engine.Meshing.Tools;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Extends the default range splitter with maps of U and V parameters, kept in the order they are added
/// (<c>IMeshData::IMapOfReal</c>).
/// </summary>
internal class UVParamRangeSplitter : DefaultRangeSplitter
{
    private readonly IndexedSet<double> _uParams = new();
    private readonly IndexedSet<double> _vParams = new();

    /// <summary>A splitter.</summary>
    public UVParamRangeSplitter()
    {
    }

    /// <summary>Resets the splitter for the face. Must be called before the first use.</summary>
    public override void Reset(FaceModel dFace, in ToolParameters parameters)
    {
        base.Reset(dFace, parameters);
        _uParams.Clear();
        _vParams.Clear();
    }

    /// <summary>The U parameters (the splitter's own map: OCCT's const and non-const accessors are one).</summary>
    public IndexedSet<double> GetParametersU() => _uParams;

    /// <summary>The V parameters (the splitter's own map: OCCT's const and non-const accessors are one).</summary>
    public IndexedSet<double> GetParametersV() => _vParams;
}
