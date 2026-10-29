// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_BoundaryParamsRangeSplitter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The NURBS range splitter whose grid parameters are those of the border points.</summary>
public class BRepMesh_BoundaryParamsRangeSplitter : BRepMesh_NURBSRangeSplitter
{
    /// <summary>A splitter.</summary>
    public BRepMesh_BoundaryParamsRangeSplitter()
    {
    }

    /// <summary>Registers a border point and its parameters.</summary>
    public override void AddPoint(in gp_Pnt2d thePoint)
    {
        base.AddPoint(thePoint);
        GetParametersU().Add(thePoint.X());
        GetParametersV().Add(thePoint.Y());
    }

    /// <summary>The parameters are the border points': nothing to initialise.</summary>
    protected override bool initParameters() => true;
}
