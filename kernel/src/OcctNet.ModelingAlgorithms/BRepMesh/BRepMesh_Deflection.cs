// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Deflection.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Deflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The mesher's computations of the deflections of shapes and of their discrete models.</summary>
/// <remarks>OCCT's class is a <c>Standard_Transient</c> with only statics; the port's is a static class.</remarks>
public static class BRepMesh_Deflection
{
    /// <summary>
    /// The absolute deflection of the shape for the relative deflection, from its bounding box and
    /// <paramref name="theMaxShapeSize"/> (the size of the whole shape; the shape's own if not positive).
    /// </summary>
    public static double ComputeAbsoluteDeflection(in TopoDS_Shape theShape,
                                                   double theRelativeDeflection,
                                                   double theMaxShapeSize)
    {
        if (theShape.IsNull())
        {
            return theRelativeDeflection;
        }

        Bnd_Box aBox = new();
        BRepBndLib.Add(theShape, ref aBox, false);

        double aShapeSize = theRelativeDeflection;
        BRepMesh_ShapeTool.BoxMaxDimension(aBox, ref aShapeSize);

        // Adjust resulting value in relation to the total size

        aBox.Get(out double aX1, out double aY1, out double aZ1, out double aX2, out double aY2, out double aZ2);
        double aMaxShapeSize =
            (theMaxShapeSize > 0.0) ? theMaxShapeSize : Max(aX2 - aX1, Max(aY2 - aY1, aZ2 - aZ1));

        double anAdjustmentCoefficient = aMaxShapeSize / (2 * aShapeSize);
        if (anAdjustmentCoefficient < 0.5)
        {
            anAdjustmentCoefficient = 0.5;
        }
        else if (anAdjustmentCoefficient > 2.0)
        {
            anAdjustmentCoefficient = 2.0;
        }

        return (anAdjustmentCoefficient * aShapeSize * theRelativeDeflection);
    }

    /// <summary>
    /// Computes and sets the deflection and the angular deflection of the discrete edge: the parameters' (relative to
    /// the edge's size if asked), at least the distance of its vertices from its curve's ends.
    /// </summary>
    public static void ComputeDeflection(IMeshData_Edge theDEdge,
                                         double theMaxShapeSize,
                                         in IMeshTools_Parameters theParameters)
    {
        double aAngDeflection = theParameters.Angle;
        double aLinDeflection =
            !theParameters.Relative
                ? theParameters.Deflection
                : ComputeAbsoluteDeflection(theDEdge.GetEdge(), theParameters.Deflection, theMaxShapeSize);

        TopoDS_Edge anEdge = theDEdge.GetEdge();

        TopExp.Vertices(anEdge, out TopoDS_Vertex aFirstVertex, out TopoDS_Vertex aLastVertex);

        if (BRepMesh_ShapeTool.Range(anEdge, out Geom_Curve? aCurve, out double aFirstParam, out double aLastParam))
        {
            double aDistF = aFirstVertex.IsNull()
                                ? -1.0
                                : BRep_Tool.Pnt(aFirstVertex).Distance(aCurve!.Value(aFirstParam));
            double aDistL =
                aLastVertex.IsNull() ? -1.0 : BRep_Tool.Pnt(aLastVertex).Distance(aCurve!.Value(aLastParam));

            double aVertexAdjustDistance = Max(aDistF, aDistL);

            aLinDeflection = Max(aVertexAdjustDistance, aLinDeflection);
        }

        theDEdge.SetDeflection(aLinDeflection);
        theDEdge.SetAngularDeflection(aAngDeflection);
    }

    /// <summary>Computes and sets the deflection of the discrete wire: the mean of its edges' (the parameters' if none).</summary>
    public static void ComputeDeflection(IMeshData_Wire theDWire, in IMeshTools_Parameters theParameters)
    {
        double aWireDeflection = 0.0;
        if (theDWire.EdgesNb() > 0)
        {
            for (int aEdgeIt = 0; aEdgeIt < theDWire.EdgesNb(); ++aEdgeIt)
            {
                aWireDeflection += theDWire.GetEdge(aEdgeIt).GetDeflection();
            }

            aWireDeflection /= theDWire.EdgesNb();
        }
        else
        {
            aWireDeflection = theParameters.Deflection;
        }

        theDWire.SetDeflection(aWireDeflection);
    }

    /// <summary>
    /// Computes and sets the deflection of the discrete face: the interior deflection (relative if asked), and unless
    /// <see cref="IMeshTools_Parameters.ForceFaceDeflection"/>, at least the mean of its wires' and twice the face's
    /// maximum tolerance.
    /// </summary>
    public static void ComputeDeflection(IMeshData_Face theDFace, in IMeshTools_Parameters theParameters)
    {
        double aDeflection = theParameters.DeflectionInterior;
        if (theParameters.Relative)
        {
            aDeflection = ComputeAbsoluteDeflection(theDFace.GetFace(), aDeflection, -1.0);
        }

        double aFaceDeflection = 0.0;
        if (!theParameters.ForceFaceDeflection)
        {
            if (theDFace.WiresNb() > 0)
            {
                for (int aWireIt = 0; aWireIt < theDFace.WiresNb(); ++aWireIt)
                {
                    aFaceDeflection += theDFace.GetWire(aWireIt).GetDeflection();
                }

                aFaceDeflection /= theDFace.WiresNb();
            }

            aFaceDeflection = Max(2.0 * BRepMesh_ShapeTool.MaxFaceTolerance(theDFace.GetFace()), aFaceDeflection);
        }
        aFaceDeflection = Max(aDeflection, aFaceDeflection);

        theDFace.SetDeflection(aFaceDeflection);
    }

    /// <summary>
    /// True if the current deflection of a polygonal representation fits the required one: below it (with the ratio
    /// as margin), and, if <paramref name="theAllowDecrease"/>, also about the same.
    /// </summary>
    public static bool IsConsistent(double theCurrent,
                                    double theRequired,
                                    bool theAllowDecrease,
                                    double theRatio = 0.1)
    {
        // Check if the deflection of existing polygonal representation
        // fits the required deflection.
        bool isConsistent = theCurrent < (1.0 + theRatio) * theRequired
                            && (!theAllowDecrease || theCurrent > (1.0 - theRatio) * theRequired);
        return isConsistent;
    }
}
