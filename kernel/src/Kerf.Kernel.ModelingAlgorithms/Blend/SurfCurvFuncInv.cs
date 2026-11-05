// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_SurfCurvFuncInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_SurfCurvFuncInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a blending surface between a surface and a curve, used to find a solution on a restriction of
/// the surface. Its variables are wguide, wcurv, wrst: the parameters on the guide line, on the curve and on the
/// restriction.
/// </summary>
internal abstract class SurfCurvFuncInv : FunctionSetWithDerivatives
{
    /// <summary>Returns 3.</summary>
    public override int NbVariables()
    {
        return 3;
    }

    /// <summary>Sets the restriction on which a solution is found.</summary>
    public abstract void Set(Curve2DAdaptorBase rst);

    /// <summary>The parametric tolerance of each of the 3 variables, from the 3d tolerance <paramref name="tol"/>.</summary>
    public abstract void GetTolerance(math_Vector tolerance, double tol);

    /// <summary>The lowest and greatest values allowed for each of the 3 variables.</summary>
    public abstract void GetBounds(math_Vector infBound, math_Vector supBound);

    /// <summary>True if <paramref name="sol"/> is a zero of the function.</summary>
    public abstract bool IsSolution(math_Vector sol, double tol);
}
