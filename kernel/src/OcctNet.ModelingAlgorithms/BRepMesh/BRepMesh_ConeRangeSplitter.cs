// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ConeRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ConeRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>Extends the default range splitter to generate the internal nodes of a conical surface.</summary>
/// <remarks>
/// One of unit 4D's range splitters, ported with unit 4C because <see cref="BRepMesh_ModelPreProcessor"/>'s seam
/// amplifier takes its steps.
/// </remarks>
public class BRepMesh_ConeRangeSplitter : BRepMesh_DefaultRangeSplitter
{
    /// <summary>A splitter.</summary>
    public BRepMesh_ConeRangeSplitter()
    {
    }

    /// <summary>
    /// The split intervals along U and V for the parameters, with the number of steps along each direction in
    /// <paramref name="theStepsNb"/>.
    /// </summary>
    public (double first, double second) GetSplitSteps(in IMeshTools_Parameters theParameters,
                                                       out (int first, int second) theStepsNb)
    {
        (double first, double second) aRangeU = GetRangeU();
        (double first, double second) aRangeV = GetRangeV();

        Cone aCone = GetDFace().GetSurface().Cone();
        double aRefR = aCone.RefRadius();
        double aSAng = aCone.SemiAngle();
        double aRadius = Max(Math.Abs(aRefR + aRangeV.first * Math.Sin(aSAng)),
                             Math.Abs(aRefR + aRangeV.second * Math.Sin(aSAng)));

        double Dv, Du = GCPnts_TangentialDeflection.ArcAngularStep(aRadius,
                                                                   GetDFace().GetDeflection(),
                                                                   theParameters.Angle,
                                                                   theParameters.MinSize);

        double aDiffU = aRangeU.second - aRangeU.first;
        double aDiffV = aRangeV.second - aRangeV.first;
        double aScale = (Du * aRadius);
        double aRatio = Max(1.0, Math.Log(aDiffV / aScale));
        int nbU = (int)(aDiffU / Du);
        int nbV = (int)(aDiffV / aScale / aRatio);

        Du = aDiffU / (nbU + 1);
        Dv = aDiffV / (nbV + (int)aRatio);

        theStepsNb.first = nbU;
        theStepsNb.second = nbV;
        return (Du, Dv);
    }

    /// <summary>The nodes of a regular grid over the face's range, with the steps of <see cref="GetSplitSteps"/>.</summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in IMeshTools_Parameters theParameters)
    {
        (double first, double second) aRangeU = GetRangeU();
        (double first, double second) aRangeV = GetRangeV();

        (double first, double second) aSteps = GetSplitSteps(theParameters, out (int first, int second) _);

        NCollection_List<Point2D> aNodes = new();

        double aPasMaxV = aRangeV.second - aSteps.second * 0.5;
        double aPasMaxU = aRangeU.second - aSteps.first * 0.5;
        for (double aPasV = aRangeV.first + aSteps.second; aPasV < aPasMaxV; aPasV += aSteps.second)
        {
            for (double aPasU = aRangeU.first + aSteps.first; aPasU < aPasMaxU; aPasU += aSteps.first)
            {
                aNodes.Append(new Point2D(aPasU, aPasV));
            }
        }

        return aNodes;
    }
}
