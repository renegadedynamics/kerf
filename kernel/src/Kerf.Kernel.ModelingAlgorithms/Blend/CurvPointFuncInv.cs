// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_CurvPointFuncInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/Blend/Blend_CurvPointFuncInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The function of a blending surface between a surface and a curve, used to find a solution at a given point of
/// the curve.
/// </summary>
internal abstract class CurvPointFuncInv : FunctionSetWithDerivatives
{
    /// <summary>Returns 2 (OCCT's documentation says 3; its code returns 2).</summary>
    public override int NbVariables()
    {
        return 2;
    }

    /// <summary>Sets the point on which a solution is found.</summary>
    public abstract void Set(in Point3D p);

    /// <summary>The parametric tolerance of each variable, from the 3d tolerance <paramref name="tol"/>.</summary>
    public abstract void GetTolerance(math_Vector tolerance, double tol);

    /// <summary>The lowest and greatest values allowed for each variable.</summary>
    public abstract void GetBounds(math_Vector infBound, math_Vector supBound);

    /// <summary>True if <paramref name="sol"/> is a zero of the function.</summary>
    public abstract bool IsSolution(math_Vector sol, double tol);
}
