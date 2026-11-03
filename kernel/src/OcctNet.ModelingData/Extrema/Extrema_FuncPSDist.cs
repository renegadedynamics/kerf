// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncPSDist.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncPSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The square distance between a point P and a surface S, F(u, v) = SquareDist(P, S(u, v)), with its gradient
/// ((S - P).Su, (S - P).Sv), for math_BFGS.
/// </summary>
/// <remarks>
/// OCCT keeps references to the surface and the point; the port keeps the surface adaptor (which must outlive the
/// function, as in OCCT) and a copy of the point. Outside the surface's parameter range the functions return false and
/// write 0 to their <c>out</c> value (decision 221).
/// </remarks>
public class Extrema_FuncPSDist : MultipleVarFunctionWithGradient
{
    private readonly Adaptor3d_Surface mySurf;
    private readonly Point3D myP;

    public Extrema_FuncPSDist(Adaptor3d_Surface theS, in Point3D theP)
    {
        mySurf = theS;
        myP = theP;
    }

    public override int VariableCount => 2;

    public override bool Value(math_Vector X, out double F)
    {
        if (!IsInside(X))
        {
            F = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        F = mySurf.Value(X[1], X[2]).SquareDistance(myP);

        return true;
    }

    public override bool Gradient(math_Vector X, math_Vector G)
    {
        if (!IsInside(X))
        {
            return false;
        }

        mySurf.D1(X[1], X[2], out Point3D aP, out Vector3D Du1s, out Vector3D Dv1s);

        Vector3D P1P2 = new(aP, myP);

        G[1] = P1P2.Dot(Du1s);
        G[2] = P1P2.Dot(Dv1s);

        return true;
    }

    public override bool Values(math_Vector X, out double F, math_Vector G)
    {
        if (!IsInside(X))
        {
            F = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        mySurf.D1(X[1], X[2], out Point3D aP, out Vector3D Du1s, out Vector3D Dv1s);

        Vector3D P1P2 = new(aP, myP);

        G[1] = P1P2.Dot(Du1s);
        G[2] = P1P2.Dot(Dv1s);

        F = mySurf.Value(X[1], X[2]).SquareDistance(myP);

        return true;
    }

    // Checks that the point is inside the surface's parameter space.
    private bool IsInside(math_Vector X)
    {
        return X[1] >= mySurf.FirstUParameter() && X[1] <= mySurf.LastUParameter()
               && X[2] >= mySurf.FirstVParameter() && X[2] <= mySurf.LastVParameter();
    }
}
