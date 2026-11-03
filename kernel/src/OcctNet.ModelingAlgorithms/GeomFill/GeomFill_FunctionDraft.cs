// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionDraft.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionDraft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function G(w) - S(u, v) of a draft sweep, G the generatrix and S the stop surface, with its derivatives
/// in the variables (w, u, v) and in the spine parameter t.
/// </summary>
/// <remarks>A working object; it keeps the caller's adaptors and only evaluates them.</remarks>
public sealed class GeomFill_FunctionDraft : FunctionSetWithDerivatives
{
    private readonly Adaptor3d_Curve TheCurve;
    private readonly Adaptor3d_Surface TheSurface;

    /// <summary>Initialises the section (the generatrix) and the stop surface.</summary>
    public GeomFill_FunctionDraft(Adaptor3d_Surface S, Adaptor3d_Curve C)
    {
        TheCurve = C;
        TheSurface = S;
    }

    /// <summary>3: (t, u, v).</summary>
    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector X, math_Vector F)
    {
        TheCurve.D0(X[1], out Point3D P);
        TheSurface.D0(X[2], X[3], out Point3D P1);

        F[1] = P.Coord(1) - P1.Coord(1);
        F[2] = P.Coord(2) - P1.Coord(2);
        F[3] = P.Coord(3) - P1.Coord(3);

        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        int i;
        TheCurve.D1(X[1], out _, out Vector3D DP);
        TheSurface.D1(X[2], X[3], out _, out Vector3D DP1U, out Vector3D DP1V);

        for (i = 1; i <= 3; i++)
        {
            D[i, 1] = DP.Coord(i);
            D[i, 2] = -DP1U.Coord(i);
            D[i, 3] = -DP1V.Coord(i);
        }
        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        int i;
        TheCurve.D1(X[1], out Point3D P, out Vector3D DP);                             // derivee de la generatrice
        TheSurface.D1(X[2], X[3], out Point3D P1, out Vector3D DP1U, out Vector3D DP1V); // derivee de la new surface

        for (i = 1; i <= 3; i++)
        {
            F[i] = P.Coord(i) - P1.Coord(i);

            D[i, 1] = DP.Coord(i);
            D[i, 2] = -DP1U.Coord(i);
            D[i, 3] = -DP1V.Coord(i);
        }

        return true;
    }

    /// <summary>The first derivative in t.</summary>
    public bool DerivT(Adaptor3d_Curve C, double Param, double W, in Vector3D dN, double teta, math_Vector F)
    {
        C.D1(Param, out _, out Vector3D DP); // derivee de la section

        // OCCT: the term W cos(teta) B' (the turning binormal) is missing, so LocationDraft's trace derivatives are
        // wrong wherever the binormal turns.
        F[1] = DP.Coord(1) + W * dN.Coord(1) * Math.Sin(teta);
        F[2] = DP.Coord(2) + W * dN.Coord(2) * Math.Sin(teta);
        F[3] = DP.Coord(3) + W * dN.Coord(3) * Math.Sin(teta);

        return true;
    }

    /// <summary>The second derivative in t.</summary>
    public bool Deriv2T(Adaptor3d_Curve C, double Param, double W, in Vector3D d2N, double teta, math_Vector F)
    {
        C.D2(Param, out _, out _, out Vector3D D2P); // derivee de la section

        F[1] = D2P.Coord(1) + W * d2N.Coord(1) * Math.Sin(teta);
        F[2] = D2P.Coord(2) + W * d2N.Coord(2) * Math.Sin(teta);
        F[3] = D2P.Coord(3) + W * d2N.Coord(3) * Math.Sin(teta);

        return true;
    }

    /// <summary>The second derivative in t and X.</summary>
    public bool DerivTX(in Vector3D dN, double teta, Matrix D)
    {
        int i;
        for (i = 1; i <= 3; i++)
        {
            D[i, 1] = dN.Coord(i) * Math.Sin(teta); // derivee / W
            D[i, 2] = 0.0;                          // derivee / U
            D[i, 3] = 0.0;                          // derivee / V
        }

        return true;
    }

    /// <summary>The second derivative in X.</summary>
    public bool Deriv2X(math_Vector X, GeomFill_Tensor T)
    {
        int i;

        TheSurface.D2(X[2], X[3], out _, out _, out _, out Vector3D D2Pu, out Vector3D D2Pv, out Vector3D D2Puv);

        T.Init(0.0); // tenseur

        for (i = 1; i <= 3; i++)
        {
            T[i, 2, 2] = -D2Pu.Coord(i);
            T[i, 3, 2] = T[i, 2, 3] = -D2Puv.Coord(i);
            T[i, 3, 3] = -D2Pv.Coord(i);
        }

        return true;
    }
}
