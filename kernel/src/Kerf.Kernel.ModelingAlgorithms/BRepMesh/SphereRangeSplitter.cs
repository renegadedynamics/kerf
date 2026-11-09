// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_SphereRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_SphereRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>Extends the default range splitter to generate the internal nodes of a spherical surface.</summary>
internal class SphereRangeSplitter : DefaultRangeSplitter
{
    /// <summary>A splitter.</summary>
    public SphereRangeSplitter()
    {
    }

    /// <summary>The nodes of a staggered grid over the face's range.</summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in ToolParameters parameters)
    {
        // Calculate parameters for iteration in V direction
        double step =
            0.7
            * TangentialDeflection.ArcAngularStep(GetDFace().GetSurface().Sphere().Radius,
                                                         GetDFace().GetDeflection(),
                                                         parameters.Angle,
                                                         parameters.MinSize);

        // aRange[0] is V, aRange[1] is U
        (double first, double second) range0 = GetRangeV();
        (double first, double second) range1 = GetRangeU();

        ComputeStep(range0, step, out (double first, double second) stepAndOffset0);
        ComputeStep(range1, step, out (double first, double second) stepAndOffset1);

        NCollection_List<Point2D> nodes = new();

        double halfDu = stepAndOffset1.first * 0.5;
        bool shift = false;
        double pasV = range0.first + stepAndOffset0.first;
        for (; pasV < stepAndOffset0.second; pasV = StepLimit.Moved(pasV, pasV + stepAndOffset0.first))
        {
            shift = !shift;
            double d = (shift) ? halfDu : 0.0;
            double pasU = range1.first + d;
            for (; pasU < stepAndOffset1.second; pasU = StepLimit.Moved(pasU, pasU + stepAndOffset1.first))
            {
                if ((nodes.Size() & 1023) == 0)
                {
                    parameters.Budget?.Check(nodes.Size());
                }

                nodes.Append(new Point2D(pasU, pasV));
            }
        }

        return nodes;
    }

    /// <summary>
    /// Kerf: the parameters are scaled as the sphere is at its latitude nearest the equator, not by its average lengths
    /// (decision 1953). On a whole sphere the average takes a length along u as 0.64 of what it is at the equator, so
    /// Delaunay joins nodes two of the grid's steps apart there and loses grid nodes beside the seam, and the mesh is up to
    /// twice the deflection from the sphere. Scaled so, a length along u is never taken as shorter than it is. At a relative
    /// deflection too (decision 2056).
    /// </summary>
    protected override void ComputeDelta(double lengthU, double lengthV)
    {
        base.ComputeDelta(lengthU, lengthV);
        (double first, double second) rangeV = GetRangeV();
        double nearest = rangeV.first > 0 ? rangeV.first : rangeV.second < 0 ? rangeV.second : 0;
        double cos = Math.Cos(nearest);
        if (cos > 1e-3)
        {
            _delta.first = _delta.second / cos;
        }
    }

    // Computes the step for the given range.
    private static void ComputeStep((double first, double second) range,
                                    double defaultStep,
                                    out (double first, double second) stepAndOffset)
    {
        double diff = range.second - range.first;
        stepAndOffset.first = diff / ((int)(diff / defaultStep) + 1);
        stepAndOffset.second = range.second - global::Kerf.Engine.Foundation.Precision.PConfusion();
    }
}
