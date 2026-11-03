// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_FuncInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_FuncInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>
/// The function of a blending surface between two surfaces, used to find a solution on a restriction of one of
/// them. Its variables are t, w, U, V: the parameter on the curve on surface, the parameter on the guide line, and the
/// parametric coordinates of the point on the partner surface.
/// </summary>
public abstract class Blend_FuncInv : FunctionSetWithDerivatives
{
    /// <summary>Returns 4.</summary>
    public override int NbVariables()
    {
        return 4;
    }

    /// <summary>
    /// Sets the curve on surface on which a solution is found: on the first surface if <paramref name="OnFirst"/>,
    /// otherwise on the second.
    /// </summary>
    public abstract void Set(bool OnFirst, Curve2DAdaptorBase COnSurf);

    /// <summary>The parametric tolerance of each of the 4 variables, from the 3d tolerance <paramref name="Tol"/>.</summary>
    public abstract void GetTolerance(math_Vector Tolerance, double Tol);

    /// <summary>The lowest and greatest values allowed for each of the 4 variables.</summary>
    public abstract void GetBounds(math_Vector InfBound, math_Vector SupBound);

    /// <summary>True if <paramref name="Sol"/> is a zero of the function.</summary>
    public abstract bool IsSolution(math_Vector Sol, double Tol);
}
