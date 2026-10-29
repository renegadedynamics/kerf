// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncSphereLSDist.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncSphereLSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The least-squares criterion for fitting a sphere to points (x(i), y(i), z(i)):
/// F(x0, y0, z0, R) = Sum[(x(i) - x0)^2 + (y(i) - y0)^2 + (z(i) - z0)^2 - R^2]^2, with its gradient, for
/// math_BFGS. The variables are the centre (X(1) to X(3)) and the radius X(4).
/// </summary>
public sealed class GeomConvert_FuncSphereLSDist : math_MultipleVarFunctionWithGradient
{
    private NCollection_HArray1<gp_XYZ>? myPoints;

    /// <summary>A function without points (set them with <see cref="SetPoints"/>).</summary>
    public GeomConvert_FuncSphereLSDist()
    {
    }

    /// <summary>The function for thePoints (kept, not copied).</summary>
    public GeomConvert_FuncSphereLSDist(NCollection_HArray1<gp_XYZ> thePoints)
    {
        myPoints = thePoints;
    }

    /// <summary>Sets the points (kept, not copied).</summary>
    public void SetPoints(NCollection_HArray1<gp_XYZ> thePoints) => myPoints = thePoints;

    /// <summary>4: the centre and the radius.</summary>
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
            double d = (myPoints.Value(i) - aLoc).SquareModulus() - anR2;
            F += d * d;
        }

        return true;
    }

    /// <summary>The gradient at X.</summary>
    public override bool Gradient(math_Vector X, math_Vector G)
    {
        var aLoc = new gp_XYZ(X[1], X[2], X[3]);
        double anR = X[4], anR2 = anR * anR;
        G.Init(0.0);
        int i;
        for (i = myPoints!.Lower(); i <= myPoints.Upper(); ++i)
        {
            gp_XYZ dLoc = myPoints.Value(i) - aLoc;
            double d = dLoc.SquareModulus() - anR2;
            G[1] += d * dLoc.X();
            G[2] += d * dLoc.Y();
            G[3] += d * dLoc.Z();
            G[4] += d;
        }

        G *= -4;
        G[4] *= anR;
        return true;
    }

    /// <summary>The criterion and its gradient at X.</summary>
    public override bool Values(math_Vector X, out double F, math_Vector G)
    {
        var aLoc = new gp_XYZ(X[1], X[2], X[3]);
        double anR = X[4], anR2 = anR * anR;
        G.Init(0.0);
        F = 0.0;
        int i;
        for (i = myPoints!.Lower(); i <= myPoints.Upper(); ++i)
        {
            gp_XYZ dLoc = myPoints.Value(i) - aLoc;
            double d = dLoc.SquareModulus() - anR2;
            G[1] += d * dLoc.X();
            G[2] += d * dLoc.Y();
            G[3] += d * dLoc.Z();
            G[4] += d;
            F += d * d;
        }

        G *= -4;
        G[4] *= anR;
        return true;
    }
}
