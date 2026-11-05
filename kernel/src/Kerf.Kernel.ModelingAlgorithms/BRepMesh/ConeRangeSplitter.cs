// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ConeRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ConeRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>Extends the default range splitter to generate the internal nodes of a conical surface.</summary>
/// <remarks>
/// One of unit 4D's range splitters, ported with unit 4C because <see cref="ModelPreProcessor"/>'s seam
/// amplifier takes its steps.
/// </remarks>
internal class ConeRangeSplitter : DefaultRangeSplitter
{
    /// <summary>A splitter.</summary>
    public ConeRangeSplitter()
    {
    }

    /// <summary>
    /// The split intervals along U and V for the parameters, with the number of steps along each direction in
    /// <paramref name="stepsNb"/>.
    /// </summary>
    public (double first, double second) GetSplitSteps(in ToolParameters parameters,
                                                       out (int first, int second) stepsNb)
    {
        (double first, double second) rangeU = GetRangeU();
        (double first, double second) rangeV = GetRangeV();

        Cone3D cone = GetDFace().GetSurface().Cone();
        double refR = cone.RefRadius;
        double sAng = cone.SemiAngle;
        double radius = Max(Math.Abs(refR + rangeV.first * Math.Sin(sAng)),
                             Math.Abs(refR + rangeV.second * Math.Sin(sAng)));

        double dv, du = TangentialDeflection.ArcAngularStep(radius,
                                                                   GetDFace().GetDeflection(),
                                                                   parameters.Angle,
                                                                   parameters.MinSize);

        double diffU = rangeU.second - rangeU.first;
        double diffV = rangeV.second - rangeV.first;
        double scale = (du * radius);
        double ratio = Max(1.0, Math.Log(diffV / scale));
        int nbU = (int)(diffU / du);
        int nbV = (int)(diffV / scale / ratio);

        du = diffU / (nbU + 1);
        dv = diffV / (nbV + (int)ratio);

        stepsNb.first = nbU;
        stepsNb.second = nbV;
        return (du, dv);
    }

    /// <summary>The nodes of a regular grid over the face's range, with the steps of <see cref="GetSplitSteps"/>.</summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in ToolParameters parameters)
    {
        (double first, double second) rangeU = GetRangeU();
        (double first, double second) rangeV = GetRangeV();

        (double first, double second) steps = GetSplitSteps(parameters, out (int first, int second) _);

        NCollection_List<Point2D> nodes = new();

        double pasMaxV = rangeV.second - steps.second * 0.5;
        double pasMaxU = rangeU.second - steps.first * 0.5;
        for (double pasV = rangeV.first + steps.second; pasV < pasMaxV; pasV += steps.second)
        {
            for (double pasU = rangeU.first + steps.first; pasU < pasMaxU; pasU += steps.first)
            {
                nodes.Append(new Point2D(pasU, pasV));
            }
        }

        return nodes;
    }
}
