// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncCylinderLSDist.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncCylinderLSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The least-squares criterion for fitting a cylinder of given axis direction dir to points P(i):
/// F(x0, y0, z0, R) = Sum[|(P(i) - Loc)^dir|^2 - R^2]^2, with its gradient, for math_BFGS. The variables are the
/// axis location (X(1) to X(3)) and the radius X(4).
/// </summary>
/// <remarks>
/// OCCT: <see cref="Gradient"/> scales G(6) instead of G(4) (occt-issues.md). OCCT's release build writes past
/// the 4-element vector and returns an unscaled radius derivative; the port throws. <see cref="Values"/> is right.
/// </remarks>
public sealed class GeomConvert_FuncCylinderLSDist : math_MultipleVarFunctionWithGradient
{
    private NCollection_HArray1<gp_XYZ>? myPoints;
    private gp_Dir myDir = new gp_Dir();

    /// <summary>A function without points (set them with <see cref="SetPoints"/>).</summary>
    public GeomConvert_FuncCylinderLSDist()
    {
    }

    /// <summary>The function for thePoints (kept, not copied) and the axis direction theDir.</summary>
    public GeomConvert_FuncCylinderLSDist(NCollection_HArray1<gp_XYZ> thePoints, in gp_Dir theDir)
    {
        myPoints = thePoints;
        myDir = theDir;
    }

    /// <summary>Sets the points (kept, not copied).</summary>
    public void SetPoints(NCollection_HArray1<gp_XYZ> thePoints) => myPoints = thePoints;

    /// <summary>Sets the axis direction.</summary>
    public void SetDir(in gp_Dir theDir) => myDir = theDir;

    /// <summary>4: the axis location and the radius.</summary>
    public override int NbVariables() => 4;

    /// <summary>The criterion at X.</summary>
    public override bool Value(math_Vector X, out double F)
    {
        var aLoc = new gp_XYZ(X[1], X[2], X[3]);
        double anR2 = X[4] * X[4];
        F = 0.0;
        int i;
        for (i = myPoints!.Lower(); i <= myPoints.Upper(); ++i)
        {
            var aV = new gp_Vec(myPoints.Value(i) - aLoc);
            double aD2 = aV.CrossSquareMagnitude(new gp_Vec(myDir));
            double d = aD2 - anR2;
            F += d * d;
        }

        return true;
    }

    /// <summary>The gradient at X.</summary>
    /// <exception cref="Standard_OutOfRange">Always, after OCCT's error described on the class.</exception>
    public override bool Gradient(math_Vector X, math_Vector G)
    {
        var aLoc = new gp_XYZ(X[1], X[2], X[3]);
        double anR = X[4], anR2 = anR * anR;
        double x = myDir.X(), y = myDir.Y(), z = myDir.Z();
        G.Init(0.0);
        int i;
        for (i = myPoints!.Lower(); i <= myPoints.Upper(); ++i)
        {
            var aV = new gp_Vec(myPoints.Value(i) - aLoc);
            double aD2 = aV.CrossSquareMagnitude(new gp_Vec(myDir));
            double d = aD2 - anR2;
            double Dx0 = 2.0 * (aV.Z() * x - aV.X() * z) * z - 2.0 * (aV.X() * y - aV.Y() * x) * y;
            double Dy0 = -2.0 * (aV.Y() * z - aV.Z() * y) * z + 2.0 * (aV.X() * y - aV.Y() * x) * x;
            double Dz0 = 2.0 * (aV.Y() * z - aV.Z() * y) * y - 2.0 * (aV.Z() * x - aV.X() * z) * x;
            G[1] += d * Dx0;
            G[2] += d * Dy0;
            G[3] += d * Dz0;
            //
            G[4] += d;
        }

        G *= 2;
        G[6] *= -2.0 * anR; // OCCT: G(6) instead of G(4), past the end of the vector
        return true;
    }

    /// <summary>The criterion and its gradient at X.</summary>
    public override bool Values(math_Vector X, out double F, math_Vector G)
    {
        var aLoc = new gp_XYZ(X[1], X[2], X[3]);
        double anR = X[4], anR2 = anR * anR;
        double x = myDir.X(), y = myDir.Y(), z = myDir.Z();
        F = 0.0;
        G.Init(0.0);
        int i;
        for (i = myPoints!.Lower(); i <= myPoints.Upper(); ++i)
        {
            var aV = new gp_Vec(myPoints.Value(i) - aLoc);
            double aD2 = aV.CrossSquareMagnitude(new gp_Vec(myDir));
            double d = aD2 - anR2;
            double Dx0 = 2.0 * (aV.Z() * x - aV.X() * z) * z - 2.0 * (aV.X() * y - aV.Y() * x) * y;
            double Dy0 = -2.0 * (aV.Y() * z - aV.Z() * y) * z + 2.0 * (aV.X() * y - aV.Y() * x) * x;
            double Dz0 = 2.0 * (aV.Y() * z - aV.Z() * y) * y - 2.0 * (aV.Z() * x - aV.X() * z) * x;
            G[1] += d * Dx0;
            G[2] += d * Dy0;
            G[3] += d * Dz0;
            //
            G[4] += d;
            //
            F += d * d;
        }

        G *= 2;
        G[4] *= -2.0 * anR;
        return true;
    }
}
