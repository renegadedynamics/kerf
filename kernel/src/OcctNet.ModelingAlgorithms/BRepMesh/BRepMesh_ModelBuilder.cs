// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelBuilder.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ModelBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Status;

namespace Occt;

/// <summary>Builds the default discrete model (<see cref="BRepMeshData_Model"/>) of a shape.</summary>
/// <remarks>
/// Statuses: <see cref="Message_Done1"/>, the model is built; <see cref="Message_Fail1"/>, an empty shape (a void
/// bounding box); <see cref="Message_Fail2"/>, the model wasn't built for an unexpected reason.
/// </remarks>
public class BRepMesh_ModelBuilder : IMeshTools_ModelBuilder
{
    /// <summary>A model builder.</summary>
    public BRepMesh_ModelBuilder()
    {
    }

    /// <summary>The discrete model of the shape; null for an empty shape.</summary>
    protected override IMeshData_Model? performInternal(in TopoDS_Shape theShape, in IMeshTools_Parameters theParameters)
    {
        BRepMeshData_Model? aModel = null;

        Bnd_Box aBox = new();
        BRepBndLib.Add(theShape, ref aBox, false);

        if (!aBox.IsVoid())
        {
            // Build data model for further processing.
            aModel = new BRepMeshData_Model(theShape);

            if (theParameters.Relative)
            {
                double aMaxSize = 0.0; // OCCT leaves it uninitialized; the box isn't void, so it is set
                BRepMesh_ShapeTool.BoxMaxDimension(aBox, ref aMaxSize);
                aModel.SetMaxSize(aMaxSize);
            }
            else
            {
                aModel.SetMaxSize(Max(theParameters.Deflection, theParameters.DeflectionInterior));
            }

            IMeshTools_ShapeVisitor aVisitor = new BRepMesh_ShapeVisitor(aModel);

            IMeshTools_ShapeExplorer aExplorer = new(theShape);
            aExplorer.Accept(aVisitor);
            SetStatus(Message_Done1);
        }
        else
        {
            SetStatus(Message_Fail1);
        }

        return aModel;
    }
}
