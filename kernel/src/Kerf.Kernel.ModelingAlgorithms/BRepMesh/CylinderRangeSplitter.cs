// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CylinderRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CylinderRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>Extends the default range splitter to generate the internal nodes of a cylindrical surface.</summary>
internal class CylinderRangeSplitter : DefaultRangeSplitter
{
    private double _du;

    /// <summary>A splitter.</summary>
    public CylinderRangeSplitter()
    {
        _du = 1.0;
    }

    /// <summary>Resets the splitter for the face. Must be called before the first use.</summary>
    public override void Reset(FaceModel dFace, in ToolParameters parameters)
    {
        base.Reset(dFace, parameters);

        double radius = GetDFace().GetSurface().Cylinder().Radius;
        _du = TangentialDeflection.ArcAngularStep(radius,
                                                          GetDFace().GetDeflection(),
                                                          parameters.Angle,
                                                          parameters.MinSize);
    }

    /// <summary>The nodes of a grid along U over the face's range (OCCT leaves the V subdivision out).</summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in ToolParameters parameters)
    {
        (double first, double second) rangeU = GetRangeU();
        (double first, double second) rangeV = GetRangeV();

        double radius = GetDFace().GetSurface().Cylinder().Radius;

        int nbU = 0;
        int nbV = 0;
        double su = rangeU.second - rangeU.first;
        double sv = rangeV.second - rangeV.first;
        double arcLen = su * radius;
        if (arcLen > GetDFace().GetDeflection())
        {
            // Calculate parameters for iteration in U direction
            nbU = (int)(su / _du);

            // OCCT has the computation of nbV commented out here: nbV stays 0.
        }

        double du = su / (nbU + 1);
        double dv = sv / (nbV + 1);

        NCollection_List<Point2D> nodes = new();

        double pasMaxV = rangeV.second - dv * 0.5;
        double pasMaxU = rangeU.second - du * 0.5;
        for (double pasV = rangeV.first + dv; pasV < pasMaxV; pasV += dv)
        {
            for (double pasU = rangeU.first + du; pasU < pasMaxU; pasU += du)
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

    /// <summary>Computes the parametric delta from the angular step and the length along V.</summary>
    protected override void ComputeDelta(double lengthU, double lengthV)
    {
        (double first, double second) rangeV = GetRangeV();
        _delta.first = _du / Max(lengthV, rangeV.second - rangeV.first);
        _delta.second = 1.0;
    }
}
