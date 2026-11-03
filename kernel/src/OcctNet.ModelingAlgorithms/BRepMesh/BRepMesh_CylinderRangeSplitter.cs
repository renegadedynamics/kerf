// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CylinderRangeSplitter.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CylinderRangeSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>Extends the default range splitter to generate the internal nodes of a cylindrical surface.</summary>
public class BRepMesh_CylinderRangeSplitter : BRepMesh_DefaultRangeSplitter
{
    private double myDu;

    /// <summary>A splitter.</summary>
    public BRepMesh_CylinderRangeSplitter()
    {
        myDu = 1.0;
    }

    /// <summary>Resets the splitter for the face. Must be called before the first use.</summary>
    public override void Reset(IMeshData_Face theDFace, in IMeshTools_Parameters theParameters)
    {
        base.Reset(theDFace, theParameters);

        double aRadius = GetDFace().GetSurface().Cylinder().Radius();
        myDu = GCPnts_TangentialDeflection.ArcAngularStep(aRadius,
                                                          GetDFace().GetDeflection(),
                                                          theParameters.Angle,
                                                          theParameters.MinSize);
    }

    /// <summary>The nodes of a grid along U over the face's range (OCCT leaves the V subdivision out).</summary>
    public override NCollection_List<Point2D>? GenerateSurfaceNodes(in IMeshTools_Parameters theParameters)
    {
        (double first, double second) aRangeU = GetRangeU();
        (double first, double second) aRangeV = GetRangeV();

        double aRadius = GetDFace().GetSurface().Cylinder().Radius();

        int nbU = 0;
        int nbV = 0;
        double su = aRangeU.second - aRangeU.first;
        double sv = aRangeV.second - aRangeV.first;
        double aArcLen = su * aRadius;
        if (aArcLen > GetDFace().GetDeflection())
        {
            // Calculate parameters for iteration in U direction
            nbU = (int)(su / myDu);

            // OCCT has the computation of nbV commented out here: nbV stays 0.
        }

        double Du = su / (nbU + 1);
        double Dv = sv / (nbV + 1);

        NCollection_List<Point2D> aNodes = new();

        double aPasMaxV = aRangeV.second - Dv * 0.5;
        double aPasMaxU = aRangeU.second - Du * 0.5;
        for (double aPasV = aRangeV.first + Dv; aPasV < aPasMaxV; aPasV += Dv)
        {
            for (double aPasU = aRangeU.first + Du; aPasU < aPasMaxU; aPasU += Du)
            {
                aNodes.Append(new Point2D(aPasU, aPasV));
            }
        }

        return aNodes;
    }

    /// <summary>Computes the parametric delta from the angular step and the length along V.</summary>
    protected override void computeDelta(double theLengthU, double theLengthV)
    {
        (double first, double second) aRangeV = GetRangeV();
        myDelta.first = myDu / Max(theLengthV, aRangeV.second - aRangeV.first);
        myDelta.second = 1.0;
    }
}
