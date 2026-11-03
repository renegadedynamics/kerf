// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_SphereRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_SphereRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Geometry;

namespace Occt;

/// <summary>Extends the default range splitter to generate the internal nodes of a spherical surface.</summary>
public class BRepMesh_SphereRangeSplitter : BRepMesh_DefaultRangeSplitter
{
    /// <summary>A splitter.</summary>
    public BRepMesh_SphereRangeSplitter()
    {
    }

    /// <summary>The nodes of a staggered grid over the face's range.</summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in IMeshTools_Parameters theParameters)
    {
        // Calculate parameters for iteration in V direction
        double aStep =
            0.7
            * TangentialDeflection.ArcAngularStep(GetDFace().GetSurface().Sphere().Radius(),
                                                         GetDFace().GetDeflection(),
                                                         theParameters.Angle,
                                                         theParameters.MinSize);

        // aRange[0] is V, aRange[1] is U
        (double first, double second) aRange0 = GetRangeV();
        (double first, double second) aRange1 = GetRangeU();

        computeStep(aRange0, aStep, out (double first, double second) aStepAndOffset0);
        computeStep(aRange1, aStep, out (double first, double second) aStepAndOffset1);

        NCollection_List<Point2D> aNodes = new();

        double aHalfDu = aStepAndOffset1.first * 0.5;
        bool Shift = false;
        double aPasV = aRange0.first + aStepAndOffset0.first;
        for (; aPasV < aStepAndOffset0.second; aPasV += aStepAndOffset0.first)
        {
            Shift = !Shift;
            double d = (Shift) ? aHalfDu : 0.0;
            double aPasU = aRange1.first + d;
            for (; aPasU < aStepAndOffset1.second; aPasU += aStepAndOffset1.first)
            {
                aNodes.Append(new Point2D(aPasU, aPasV));
            }
        }

        return aNodes;
    }

    // Computes the step for the given range.
    private static void computeStep((double first, double second) theRange,
                                    double theDefaultStep,
                                    out (double first, double second) theStepAndOffset)
    {
        double aDiff = theRange.second - theRange.first;
        theStepAndOffset.first = aDiff / ((int)(aDiff / theDefaultStep) + 1);
        theStepAndOffset.second = theRange.second - global::Kerf.Engine.Foundation.Precision.PConfusion();
    }
}
