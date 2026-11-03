// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Deflection.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Deflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>The mesher's computations of the deflections of shapes and of their discrete models.</summary>
/// <remarks>OCCT's class is a <c>Standard_Transient</c> with only statics; the port's is a static class.</remarks>
public static class Deflection
{
    /// <summary>
    /// The absolute deflection of the shape for the relative deflection, from its bounding box and
    /// <paramref name="maxShapeSize"/> (the size of the whole shape; the shape's own if not positive).
    /// </summary>
    public static double ComputeAbsoluteDeflection(in Shape shape,
                                                   double relativeDeflection,
                                                   double maxShapeSize)
    {
        if (shape.IsNull)
        {
            return relativeDeflection;
        }

        Box box = new();
        Bounds.Add(shape, ref box, false);

        double shapeSize = relativeDeflection;
        ShapeTool.BoxMaxDimension(box, ref shapeSize);

        // Adjust resulting value in relation to the total size

        box.Get(out double x1, out double y1, out double z1, out double x2, out double y2, out double z2);
        double aMaxShapeSize =
            (maxShapeSize > 0.0) ? maxShapeSize : Max(x2 - x1, Max(y2 - y1, z2 - z1));

        double adjustmentCoefficient = aMaxShapeSize / (2 * shapeSize);
        if (adjustmentCoefficient < 0.5)
        {
            adjustmentCoefficient = 0.5;
        }
        else if (adjustmentCoefficient > 2.0)
        {
            adjustmentCoefficient = 2.0;
        }

        return (adjustmentCoefficient * shapeSize * relativeDeflection);
    }

    /// <summary>
    /// Computes and sets the deflection and the angular deflection of the discrete edge: the parameters' (relative to
    /// the edge's size if asked), at least the distance of its vertices from its curve's ends.
    /// </summary>
    public static void ComputeDeflection(EdgeModel dEdge,
                                         double maxShapeSize,
                                         in ToolParameters parameters)
    {
        double angDeflection = parameters.Angle;
        double linDeflection =
            !parameters.Relative
                ? parameters.Deflection
                : ComputeAbsoluteDeflection(dEdge.GetEdge(), parameters.Deflection, maxShapeSize);

        Edge edge = dEdge.GetEdge();

        Exploration.Vertices(edge, out Vertex firstVertex, out Vertex lastVertex);

        if (ShapeTool.Range(edge, out Curve3D? curve, out double firstParam, out double lastParam))
        {
            double distF = firstVertex.IsNull
                                ? -1.0
                                : Tool.Pnt(firstVertex).Distance(curve!.Value(firstParam));
            double distL =
                lastVertex.IsNull ? -1.0 : Tool.Pnt(lastVertex).Distance(curve!.Value(lastParam));

            double vertexAdjustDistance = Max(distF, distL);

            linDeflection = Max(vertexAdjustDistance, linDeflection);
        }

        dEdge.SetDeflection(linDeflection);
        dEdge.SetAngularDeflection(angDeflection);
    }

    /// <summary>Computes and sets the deflection of the discrete wire: the mean of its edges' (the parameters' if none).</summary>
    public static void ComputeDeflection(WireModel dWire, in ToolParameters parameters)
    {
        double wireDeflection = 0.0;
        if (dWire.EdgesNb() > 0)
        {
            for (int edgeIt = 0; edgeIt < dWire.EdgesNb(); ++edgeIt)
            {
                wireDeflection += dWire.GetEdge(edgeIt).GetDeflection();
            }

            wireDeflection /= dWire.EdgesNb();
        }
        else
        {
            wireDeflection = parameters.Deflection;
        }

        dWire.SetDeflection(wireDeflection);
    }

    /// <summary>
    /// Computes and sets the deflection of the discrete face: the interior deflection (relative if asked), and unless
    /// <see cref="ToolParameters.ForceFaceDeflection"/>, at least the mean of its wires' and twice the face's
    /// maximum tolerance.
    /// </summary>
    public static void ComputeDeflection(FaceModel dFace, in ToolParameters parameters)
    {
        double deflection = parameters.DeflectionInterior;
        if (parameters.Relative)
        {
            deflection = ComputeAbsoluteDeflection(dFace.GetFace(), deflection, -1.0);
        }

        double faceDeflection = 0.0;
        if (!parameters.ForceFaceDeflection)
        {
            if (dFace.WiresNb() > 0)
            {
                for (int wireIt = 0; wireIt < dFace.WiresNb(); ++wireIt)
                {
                    faceDeflection += dFace.GetWire(wireIt).GetDeflection();
                }

                faceDeflection /= dFace.WiresNb();
            }

            faceDeflection = Max(2.0 * ShapeTool.MaxFaceTolerance(dFace.GetFace()), faceDeflection);
        }
        faceDeflection = Max(deflection, faceDeflection);

        dFace.SetDeflection(faceDeflection);
    }

    /// <summary>
    /// True if the current deflection of a polygonal representation fits the required one: below it (with the ratio
    /// as margin), and, if <paramref name="allowDecrease"/>, also about the same.
    /// </summary>
    public static bool IsConsistent(double current,
                                    double required,
                                    bool allowDecrease,
                                    double ratio = 0.1)
    {
        // Check if the deflection of existing polygonal representation
        // fits the required deflection.
        bool isConsistent = current < (1.0 + ratio) * required
                            && (!allowDecrease || current > (1.0 - ratio) * required);
        return isConsistent;
    }
}
