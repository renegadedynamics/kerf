// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCartesianTransformationOperator.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCartesianTransformationOperator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of Trsf from gp to a STEP cartesian_transformation_operator_3d.</summary>
public sealed class GeomToStep_MakeCartesianTransformationOperator : GeomToStep_Root
{
    private StepGeom_CartesianTransformationOperator3d myTrsfOp;

    //=================================================================================================

    /// <summary>
    /// A transformation: its inverse's rows as the three axes, its inverse's scale, and the origin that StepToGeom's
    /// MakeTransformation3d reads back.
    /// </summary>
    public GeomToStep_MakeCartesianTransformationOperator(in gp_Trsf theTrsf, in StepData_Factors theLocalFactors)
    {
        gp_Trsf anInvTrsf = theTrsf;
        anInvTrsf.Invert();
        double aScale = anInvTrsf.ScaleFactor();

        gp_Mat anInvMat = anInvTrsf.HVectorialPart();

        gp_Mat aMat = theTrsf.HVectorialPart();
        gp_XYZ aTranslation = anInvTrsf.TranslationPart().Multiplied(aMat);
        aTranslation.Reverse();

        GeomToStep_MakeCartesianPoint aMkPoint = new GeomToStep_MakeCartesianPoint(new gp_Pnt(aTranslation), theLocalFactors.LengthFactor());
        GeomToStep_MakeDirection aMkDir1 = new GeomToStep_MakeDirection(new gp_Dir(anInvMat.Row(1)));
        GeomToStep_MakeDirection aMkDir2 = new GeomToStep_MakeDirection(new gp_Dir(anInvMat.Row(2)));
        GeomToStep_MakeDirection aMkDir3 = new GeomToStep_MakeDirection(new gp_Dir(anInvMat.Row(3)));

        myTrsfOp = new StepGeom_CartesianTransformationOperator3d();
        myTrsfOp.SetName(new TCollection_HAsciiString(""));
        myTrsfOp.SetAxis1(aMkDir1.Value());
        myTrsfOp.SetAxis2(aMkDir2.Value());
        myTrsfOp.SetAxis3(aMkDir3.Value());
        myTrsfOp.SetLocalOrigin(aMkPoint.Value());
        myTrsfOp.SetScale(aScale);

        done = true;
    }

    /// <summary>A transformation (default factors).</summary>
    public GeomToStep_MakeCartesianTransformationOperator(in gp_Trsf theTrsf) : this(theTrsf, new StepData_Factors())
    {
    }

    /// <summary>The operator made (OCCT's inline accessor, which doesn't check IsDone).</summary>
    public StepGeom_CartesianTransformationOperator3d Value()
    {
        return myTrsfOp;
    }
}
