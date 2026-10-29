// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncConeLSDist.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncConeLSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The least-squares criterion for fitting a cone of given axis direction to a set of points: the sum of the
/// squared distances from the points to the cone. The variables are the location (X(1) to X(3)), the semi-angle
/// X(4) and the reference radius X(5).
/// </summary>
public sealed class GeomConvert_FuncConeLSDist : math_MultipleVarFunction
{
    private NCollection_HArray1<gp_XYZ>? myPoints;
    private gp_Dir myDir = new gp_Dir();

    /// <summary>A function without points (set them with <see cref="SetPoints"/>).</summary>
    public GeomConvert_FuncConeLSDist()
    {
    }

    /// <summary>The function for thePoints (kept, not copied) and the axis direction theDir.</summary>
    public GeomConvert_FuncConeLSDist(NCollection_HArray1<gp_XYZ> thePoints, in gp_Dir theDir)
    {
        myPoints = thePoints;
        myDir = theDir;
    }

    /// <summary>Sets the points (kept, not copied).</summary>
    public void SetPoints(NCollection_HArray1<gp_XYZ> thePoints) => myPoints = thePoints;

    /// <summary>Sets the axis direction.</summary>
    public void SetDir(in gp_Dir theDir) => myDir = theDir;

    /// <summary>5: the location, the semi-angle and the radius.</summary>
    public override int NbVariables() => 5;

    /// <summary>The sum of the squared distances from the points to the cone X.</summary>
    public override bool Value(math_Vector X, out double F)
    {
        var aLoc = new gp_Pnt(X[1], X[2], X[3]);
        double aSemiAngle = X[4], anR = X[5];
        var aPos = new gp_Ax3(aLoc, myDir);
        F = 0.0;
        int i;
        for (i = myPoints!.Lower(); i <= myPoints.Upper(); ++i)
        {
            double u, v;
            var aPi = new gp_Pnt(myPoints.Value(i));
            ElSLib.ConeParameters(aPos, anR, aSemiAngle, aPi, out u, out v);
            gp_Pnt aPp;
            ElSLib.ConeD0(u, v, aPos, anR, aSemiAngle, out aPp);
            F += aPi.SquareDistance(aPp);
        }

        return true;
    }
}
