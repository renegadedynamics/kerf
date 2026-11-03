// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ExtrusionRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ExtrusionRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>The NURBS range splitter for surfaces of linear extrusion.</summary>
public class BRepMesh_ExtrusionRangeSplitter : BRepMesh_NURBSRangeSplitter
{
    /// <summary>A splitter.</summary>
    public BRepMesh_ExtrusionRangeSplitter()
    {
    }

    /// <summary>
    /// The number of intervals of the basis curve; its number of poles less one when that is a single B-spline or
    /// Bezier interval.
    /// </summary>
    protected override int getUndefinedIntervalNb(Adaptor3d_Surface theSurface,
                                                  bool isU,
                                                  Continuity theContinuity)
    {
        // Here we need just a regular grid along dimension with no
        // geometrical data regarding intervals like extrusion surface.
        Adaptor3d_Curve aCurve = theSurface.BasisCurve();
        int aIntervalsNb = aCurve.NbIntervals(theContinuity);
        if (aIntervalsNb == 1)
        {
            CurveKind aCurveType = aCurve.GetType();
            bool isBSplineCurve =
                aCurveType == BezierCurve || aCurveType == BSplineCurve;

            if (isBSplineCurve)
            {
                aIntervalsNb = aCurve.NbPoles() - 1;
            }
        }

        return aIntervalsNb;
    }
}
