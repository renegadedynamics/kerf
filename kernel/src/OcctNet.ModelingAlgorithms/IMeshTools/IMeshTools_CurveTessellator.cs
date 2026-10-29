// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_CurveTessellator.hxx
// Port of: src/ModelingAlgorithms/TKMesh/IMeshTools/IMeshTools_CurveTessellator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An edge tessellation tool: the points of a tessellation, indexed from 1.</summary>
public abstract class IMeshTools_CurveTessellator : Standard_Transient
{
    /// <summary>A tessellator.</summary>
    protected IMeshTools_CurveTessellator()
    {
    }

    /// <summary>The number of tessellation points.</summary>
    public abstract int PointsNb();

    /// <summary>
    /// The tessellation point with the given index and its parameter on the pcurve; false if the result isn't valid.
    /// </summary>
    public abstract bool Value(int theIndex, out gp_Pnt thePoint, out double theParameter);
}
