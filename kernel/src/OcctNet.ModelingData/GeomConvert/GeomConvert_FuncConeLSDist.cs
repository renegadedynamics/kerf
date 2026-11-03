// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncConeLSDist.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncConeLSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The least-squares criterion for fitting a cone of given axis direction to a set of points: the sum of the
/// squared distances from the points to the cone. The variables are the location (X(1) to X(3)), the semi-angle
/// X(4) and the reference radius X(5).
/// </summary>
public sealed class GeomConvert_FuncConeLSDist : MultipleVarFunction
{
    private NCollection_HArray1<Xyz>? myPoints;
    private Direction3D myDir = new Direction3D();

    /// <summary>A function without points (set them with <see cref="SetPoints"/>).</summary>
    public GeomConvert_FuncConeLSDist()
    {
    }

    /// <summary>The function for thePoints (kept, not copied) and the axis direction theDir.</summary>
    public GeomConvert_FuncConeLSDist(NCollection_HArray1<Xyz> thePoints, in Direction3D theDir)
    {
        myPoints = thePoints;
        myDir = theDir;
    }

    /// <summary>Sets the points (kept, not copied).</summary>
    public void SetPoints(NCollection_HArray1<Xyz> thePoints) => myPoints = thePoints;

    /// <summary>Sets the axis direction.</summary>
    public void SetDir(in Direction3D theDir) => myDir = theDir;

    /// <summary>5: the location, the semi-angle and the radius.</summary>
    public override int VariableCount => 5;

    /// <summary>The sum of the squared distances from the points to the cone X.</summary>
    public override bool Value(math_Vector X, out double F)
    {
        var aLoc = new Point3D(X[1], X[2], X[3]);
        double aSemiAngle = X[4], anR = X[5];
        var aPos = new CoordinateSystem(aLoc, myDir);
        F = 0.0;
        int i;
        for (i = myPoints!.Lower(); i <= myPoints.Upper(); ++i)
        {
            double u, v;
            var aPi = new Point3D(myPoints.Value(i));
            global::Kerf.Engine.Geometry.ElSLib.ConeParameters(aPos, anR, aSemiAngle, aPi, out u, out v);
            Point3D aPp;
            global::Kerf.Engine.Geometry.ElSLib.ConeD0(u, v, aPos, anR, aSemiAngle, out aPp);
            F += aPi.SquareDistance(aPp);
        }

        return true;
    }
}
