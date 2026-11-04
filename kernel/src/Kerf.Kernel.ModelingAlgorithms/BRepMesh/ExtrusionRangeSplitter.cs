// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ExtrusionRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ExtrusionRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Meshing;

/// <summary>The NURBS range splitter for surfaces of linear extrusion.</summary>
internal class ExtrusionRangeSplitter : NurbsRangeSplitter
{
    /// <summary>A splitter.</summary>
    public ExtrusionRangeSplitter()
    {
    }

    /// <summary>
    /// The number of intervals of the basis curve; its number of poles less one when that is a single B-spline or
    /// Bezier interval.
    /// </summary>
    protected override int GetUndefinedIntervalNb(SurfaceAdaptorBase surface,
                                                  bool isU,
                                                  Continuity continuity)
    {
        // Here we need just a regular grid along dimension with no
        // geometrical data regarding intervals like extrusion surface.
        CurveAdaptorBase curve = surface.BasisCurve();
        int intervalsNb = curve.NbIntervals(continuity);
        if (intervalsNb == 1)
        {
            CurveKind curveType = curve.GetType();
            bool isBSplineCurve =
                curveType == BezierCurve || curveType == BSplineCurve;

            if (isBSplineCurve)
            {
                intervalsNb = curve.PoleCount - 1;
            }
        }

        return intervalsNb;
    }
}
