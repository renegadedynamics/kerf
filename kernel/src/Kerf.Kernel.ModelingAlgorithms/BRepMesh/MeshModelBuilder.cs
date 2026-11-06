// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelBuilder.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing.Data;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Topology;
using static Kerf.Engine.Foundation.Status;

namespace Kerf.Engine.Meshing;

/// <summary>Builds the default discrete model (<see cref="ModelData"/>) of a shape.</summary>
/// <remarks>
/// Statuses: <see cref="Done1"/>, the model is built; <see cref="Fail1"/>, an empty shape (a void
/// bounding box); <see cref="Fail2"/>, the model wasn't built for an unexpected reason.
/// </remarks>
internal class MeshModelBuilder : ModelBuilder
{
    /// <summary>A model builder.</summary>
    public MeshModelBuilder()
    {
    }

    /// <summary>The discrete model of the shape; null for an empty shape.</summary>
    protected override ModelModel? PerformInternal(in Shape shape, in ToolParameters parameters)
    {
        ModelData? model = null;

        Box box = new();
        Bounds.Add(shape, ref box, false);

        if (!box.IsVoid)
        {
            // Build data model for further processing.
            model = new ModelData(shape);

            if (parameters.Relative)
            {
                double maxSize = 0.0; // OCCT leaves it uninitialized; the box isn't void, so it is set
                ShapeTool.BoxMaxDimension(box, ref maxSize);
                model.SetMaxSize(maxSize);
            }
            else
            {
                model.SetMaxSize(Max(parameters.Deflection, parameters.DeflectionInterior));
            }

            ShapeVisitor visitor = new MeshShapeVisitor(model);

            ShapeExplorer explorer = new(shape);
            explorer.Accept(visitor);
            SetStatus(Done1);
        }
        else
        {
            SetStatus(Fail1);
        }

        return model;
    }
}
