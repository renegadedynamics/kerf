// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCartesianTransformationOperator.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCartesianTransformationOperator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Trsf from gp to a STEP cartesian_transformation_operator_3d.</summary>
internal sealed class MakeCartesianTransformationOperator : GeometryRoot
{
    private StepCartesianTransformationOperator3d _trsfOp;

    //=================================================================================================

    /// <summary>
    /// A transformation: its inverse's rows as the three axes, its inverse's scale, and the origin that StepToGeom's
    /// MakeTransformation3d reads back.
    /// </summary>
    public MakeCartesianTransformationOperator(in Transform3D trsf, in Factors localFactors)
    {
        Transform3D invTrsf = trsf;
        invTrsf = invTrsf.Inverted();
        double scale = invTrsf.ScaleFactor;

        Matrix3x3 invMat = invTrsf.RotationPart;

        Matrix3x3 mat = trsf.RotationPart;
        Xyz translation = invTrsf.TranslationXyz().Multiplied(mat);
        translation.Reverse();

        MakeCartesianPoint mkPoint = new MakeCartesianPoint(new Point3D(translation), localFactors.LengthFactor());
        MakeDirection mkDir1 = new MakeDirection(new Direction3D(invMat.Row(1)));
        MakeDirection mkDir2 = new MakeDirection(new Direction3D(invMat.Row(2)));
        MakeDirection mkDir3 = new MakeDirection(new Direction3D(invMat.Row(3)));

        _trsfOp = new StepCartesianTransformationOperator3d();
        _trsfOp.SetName(new TCollection_HAsciiString(""));
        _trsfOp.SetAxis1(mkDir1.Value());
        _trsfOp.SetAxis2(mkDir2.Value());
        _trsfOp.SetAxis3(mkDir3.Value());
        _trsfOp.SetLocalOrigin(mkPoint.Value());
        _trsfOp.SetScale(scale);

        _done = true;
    }

    /// <summary>A transformation (default factors).</summary>
    public MakeCartesianTransformationOperator(in Transform3D trsf) : this(trsf, new Factors())
    {
    }

    /// <summary>The operator made (OCCT's inline accessor, which doesn't check IsDone).</summary>
    public StepCartesianTransformationOperator3d Value()
    {
        return _trsfOp;
    }
}
