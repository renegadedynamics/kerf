// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_MakeTransformed.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_MakeTransformed.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Produces instances by transformation of a basic item.</summary>
public sealed class MakeTransformed : ImportRoot
{
    private Transform3D _trsf = new Transform3D();

    public MakeTransformed()
    {
    }

    /// <summary>
    /// Computes the transformation from an origin placement to a target placement; true when done. If not done, the
    /// transformation is the identity.
    /// </summary>
    public bool Compute(StepAxis2Placement3d origin, StepAxis2Placement3d target, in Factors localFactors)
    {
        _trsf = new Transform3D(); // reinit
        if (origin is null || target is null)
        {
            return false;
        }

        // sln 23.10.2001 : If the directions have not been created do nothing.
        Axis2Placement theOrig = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(origin, localFactors);
        if (theOrig is null)
        {
            return false;
        }

        Axis2Placement theTarg = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(target, localFactors);
        if (theTarg is null)
        {
            return false;
        }

        CoordinateSystem ax3Orig = new CoordinateSystem(theOrig.Ax2());
        CoordinateSystem ax3Targ = new CoordinateSystem(theTarg.Ax2());

        //  ne pas se tromper de sens !
        _trsf = _trsf.WithTransformation(ax3Targ, ax3Orig);
        return true;
    }

    public bool Compute(StepAxis2Placement3d origin, StepAxis2Placement3d target)
        => Compute(origin, target, new Factors());

    /// <summary>Computes a transformation defined by a 3D operator.</summary>
    public bool Compute(StepCartesianTransformationOperator3d Operator, in Factors localFactors)
        => global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation3d(Operator, ref _trsf, localFactors);

    public bool Compute(StepCartesianTransformationOperator3d Operator) => Compute(Operator, new Factors());

    /// <summary>The computed transformation (the identity if not yet computed or if that failed).</summary>
    public Transform3D Transformation() => _trsf;

    /// <summary>Applies the computed transformation to a shape; false if the transformation is the identity.</summary>
    public bool Transform(ref Shape shape)
    {
        if (_trsf.Form == Identity)
        {
            return false;
        }

        Placement theLoc = new Placement(_trsf);
        shape.Move(theLoc);
        return true;
    }

    /// <summary>
    /// Translates a mapped item: the transformation from its mapping origin to its mapping target is computed, the
    /// mapped representation converted to a shape (through the transient process), then moved as an instance of it.
    /// </summary>
    public Shape TranslateMappedItem(MappedItem mapit, TransientProcess tp, in Factors localFactors,
                                            ProgressRange progress = default)
    {
        Shape theResult = new Shape();

        //  Positionnement : 2 formules
        //  1/ Ax2 dans Source et comme Target  : passage de Source a Target
        //  2/ CartesianOperator3d comme Target : on applique

        StepAxis2Placement3d origin = mapit.MappingSource().MappingOrigin() as StepAxis2Placement3d;
        StepAxis2Placement3d target = mapit.MappingTarget() as StepAxis2Placement3d;

        StepCartesianTransformationOperator3d cartOp = mapit.MappingTarget() as StepCartesianTransformationOperator3d;

        bool ok = false;
        if (origin is not null && target is not null)
        {
            ok = Compute(origin, target, localFactors);
        }
        else if (cartOp is not null)
        {
            ok = Compute(cartOp, localFactors);
        }

        if (!ok)
        {
            tp.AddWarning(mapit, "Mapped Item, case not recognized, location ignored");
        }

        //  La Shape, et la mise en position
        StepRepresentation maprep = mapit.MappingSource().MappedRepresentation();
        Binder binder = tp.Find(maprep);
        if (binder is null)
        {
            binder = tp.Transferring(maprep, progress);
        }

        ShapeBinder shbinder = binder as ShapeBinder;
        if (shbinder is null)
        {
            tp.AddWarning(mapit, "No Shape Produced");
        }
        else
        {
            theResult = shbinder.Result();
            Transform(ref theResult);
        }

        return theResult;
    }

    public Shape TranslateMappedItem(MappedItem mapit, TransientProcess tp)
        => TranslateMappedItem(mapit, tp, new Factors());
}
