// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_SurfCurvFuncInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_SurfCurvFuncInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function of a blending surface between a surface and a curve, used to find a solution on a restriction of
/// the surface. Its variables are wguide, wcurv, wrst: the parameters on the guide line, on the curve and on the
/// restriction.
/// </summary>
public abstract class Blend_SurfCurvFuncInv : math_FunctionSetWithDerivatives
{
    /// <summary>Returns 3.</summary>
    public override int NbVariables()
    {
        return 3;
    }

    /// <summary>Sets the restriction on which a solution is found.</summary>
    public abstract void Set(Adaptor2d_Curve2d Rst);

    /// <summary>The parametric tolerance of each of the 3 variables, from the 3d tolerance <paramref name="Tol"/>.</summary>
    public abstract void GetTolerance(math_Vector Tolerance, double Tol);

    /// <summary>The lowest and greatest values allowed for each of the 3 variables.</summary>
    public abstract void GetBounds(math_Vector InfBound, math_Vector SupBound);

    /// <summary>True if <paramref name="Sol"/> is a zero of the function.</summary>
    public abstract bool IsSolution(math_Vector Sol, double Tol);
}
