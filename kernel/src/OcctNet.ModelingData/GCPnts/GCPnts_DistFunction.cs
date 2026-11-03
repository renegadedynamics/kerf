// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_DistFunction.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_DistFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The negated square distance between a point on the curve and the chord between the points of parameters U1 and
/// U2, used to find the maximal deflection of the chord.
/// </summary>
/// <remarks>Keeps a reference to the curve, as OCCT does.</remarks>
public class GCPnts_DistFunction : Function
{
    private readonly Adaptor3d_Curve myCurve;
    private readonly Line3D myLin;
    private readonly double myU1;
    private readonly double myU2;

    public GCPnts_DistFunction(Adaptor3d_Curve theCurve, double U1, double U2)
    {
        myCurve = theCurve;
        myU1 = U1;
        myU2 = U2;
        Point3D P1 = theCurve.Value(U1), P2 = theCurve.Value(U2);
        if (P1.SquareDistance(P2) > Constants.Resolution())
        {
            myLin = new Line3D(P1, new Direction3D(P2.XYZ() - P1.XYZ()));
        }
        else
        {
            theCurve.D0(U1 + .01 * (U2 - U1), out P2);
            myLin = new Line3D(P1, new Direction3D(P2.XYZ() - P1.XYZ()));
        }
    }

    /// <summary>A copy, on the same curve.</summary>
    public GCPnts_DistFunction(GCPnts_DistFunction theOther)
    {
        myCurve = theOther.myCurve;
        myLin = theOther.myLin;
        myU1 = theOther.myU1;
        myU2 = theOther.myU2;
    }

    /// <remarks>Out of [U1, U2] it returns false with F = 0, where OCCT leaves F unchanged (decision 32).</remarks>
    public override bool Value(double X, out double F)
    {
        if (X < myU1 || X > myU2)
        {
            F = 0.0;
            return false;
        }
        F = -myLin.SquareDistance(myCurve.Value(X));
        return true;
    }
}

/// <summary><see cref="GCPnts_DistFunction"/> as a function of one variable for the multi-variable minimisers.</summary>
public class GCPnts_DistFunctionMV : MultipleVarFunction
{
    private readonly GCPnts_DistFunction myMaxCurvLinDist;

    public GCPnts_DistFunctionMV(GCPnts_DistFunction theCurvLinDist)
    {
        myMaxCurvLinDist = theCurvLinDist;
    }

    public override bool Value(math_Vector X, out double F)
    {
        bool Ok = myMaxCurvLinDist.Value(X[1], out F);
        return Ok;
    }

    public override int VariableCount
    {
        get
        {
            return 1;
        }
    }
}
