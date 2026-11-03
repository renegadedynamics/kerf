// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_MakeTransformed.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_MakeTransformed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.TransformForm;

namespace Occt;

/// <summary>Produces instances by transformation of a basic item.</summary>
public sealed class StepToTopoDS_MakeTransformed : StepToTopoDS_Root
{
    private Transform3D theTrsf = new Transform3D();

    public StepToTopoDS_MakeTransformed()
    {
    }

    /// <summary>
    /// Computes the transformation from an origin placement to a target placement; true when done. If not done, the
    /// transformation is the identity.
    /// </summary>
    public bool Compute(StepGeom_Axis2Placement3d Origin, StepGeom_Axis2Placement3d Target, in StepData_Factors theLocalFactors)
    {
        theTrsf = new Transform3D(); // reinit
        if (Origin is null || Target is null)
        {
            return false;
        }

        // sln 23.10.2001 : If the directions have not been created do nothing.
        Axis2Placement theOrig = StepToGeom.MakeAxis2Placement(Origin, theLocalFactors);
        if (theOrig is null)
        {
            return false;
        }

        Axis2Placement theTarg = StepToGeom.MakeAxis2Placement(Target, theLocalFactors);
        if (theTarg is null)
        {
            return false;
        }

        CoordinateSystem ax3Orig = new CoordinateSystem(theOrig.Ax2());
        CoordinateSystem ax3Targ = new CoordinateSystem(theTarg.Ax2());

        //  ne pas se tromper de sens !
        theTrsf.SetTransformation(ax3Targ, ax3Orig);
        return true;
    }

    public bool Compute(StepGeom_Axis2Placement3d Origin, StepGeom_Axis2Placement3d Target)
        => Compute(Origin, Target, new StepData_Factors());

    /// <summary>Computes a transformation defined by a 3D operator.</summary>
    public bool Compute(StepGeom_CartesianTransformationOperator3d Operator, in StepData_Factors theLocalFactors)
        => StepToGeom.MakeTransformation3d(Operator, ref theTrsf, theLocalFactors);

    public bool Compute(StepGeom_CartesianTransformationOperator3d Operator) => Compute(Operator, new StepData_Factors());

    /// <summary>The computed transformation (the identity if not yet computed or if that failed).</summary>
    public Transform3D Transformation() => theTrsf;

    /// <summary>Applies the computed transformation to a shape; false if the transformation is the identity.</summary>
    public bool Transform(ref Shape shape)
    {
        if (theTrsf.Form() == Identity)
        {
            return false;
        }

        Placement theLoc = new Placement(theTrsf);
        shape.Move(theLoc);
        return true;
    }

    /// <summary>
    /// Translates a mapped item: the transformation from its mapping origin to its mapping target is computed, the
    /// mapped representation converted to a shape (through the transient process), then moved as an instance of it.
    /// </summary>
    public Shape TranslateMappedItem(StepRepr_MappedItem mapit, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors,
                                            ProgressRange theProgress = default)
    {
        Shape theResult = new Shape();

        //  Positionnement : 2 formules
        //  1/ Ax2 dans Source et comme Target  : passage de Source a Target
        //  2/ CartesianOperator3d comme Target : on applique

        StepGeom_Axis2Placement3d Origin = mapit.MappingSource().MappingOrigin() as StepGeom_Axis2Placement3d;
        StepGeom_Axis2Placement3d Target = mapit.MappingTarget() as StepGeom_Axis2Placement3d;

        StepGeom_CartesianTransformationOperator3d CartOp = mapit.MappingTarget() as StepGeom_CartesianTransformationOperator3d;

        bool ok = false;
        if (Origin is not null && Target is not null)
        {
            ok = Compute(Origin, Target, theLocalFactors);
        }
        else if (CartOp is not null)
        {
            ok = Compute(CartOp, theLocalFactors);
        }

        if (!ok)
        {
            TP.AddWarning(mapit, "Mapped Item, case not recognized, location ignored");
        }

        //  La Shape, et la mise en position
        StepRepr_Representation maprep = mapit.MappingSource().MappedRepresentation();
        Transfer_Binder binder = TP.Find(maprep);
        if (binder is null)
        {
            binder = TP.Transferring(maprep, theProgress);
        }

        TransferBRep_ShapeBinder shbinder = binder as TransferBRep_ShapeBinder;
        if (shbinder is null)
        {
            TP.AddWarning(mapit, "No Shape Produced");
        }
        else
        {
            theResult = shbinder.Result();
            Transform(ref theResult);
        }

        return theResult;
    }

    public Shape TranslateMappedItem(StepRepr_MappedItem mapit, Transfer_TransientProcess TP)
        => TranslateMappedItem(mapit, TP, new StepData_Factors());
}
