// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionDraft.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_FunctionDraft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The function G(w) - S(u, v) of a draft sweep, G the generatrix and S the stop surface, with its derivatives
/// in the variables (w, u, v) and in the spine parameter t.
/// </summary>
/// <remarks>A working object; it keeps the caller's adaptors and only evaluates them.</remarks>
internal sealed class FunctionDraft : FunctionSetWithDerivatives
{
    private readonly CurveAdaptorBase _theCurve;
    private readonly SurfaceAdaptorBase _theSurface;

    /// <summary>Initialises the section (the generatrix) and the stop surface.</summary>
    public FunctionDraft(SurfaceAdaptorBase s, CurveAdaptorBase c)
    {
        _theCurve = c;
        _theSurface = s;
    }

    /// <summary>3: (t, u, v).</summary>
    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector x, math_Vector f)
    {
        _theCurve.D0(x[1], out Point3D p);
        _theSurface.D0(x[2], x[3], out Point3D p1);

        f[1] = p.Coord(1) - p1.Coord(1);
        f[2] = p.Coord(2) - p1.Coord(2);
        f[3] = p.Coord(3) - p1.Coord(3);

        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        int i;
        _theCurve.D1(x[1], out _, out Vector3D dp);
        _theSurface.D1(x[2], x[3], out _, out Vector3D dp1U, out Vector3D dp1V);

        for (i = 1; i <= 3; i++)
        {
            d[i, 1] = dp.Coord(i);
            d[i, 2] = -dp1U.Coord(i);
            d[i, 3] = -dp1V.Coord(i);
        }
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        int i;
        _theCurve.D1(x[1], out Point3D p, out Vector3D dp);                             // derivee de la generatrice
        _theSurface.D1(x[2], x[3], out Point3D p1, out Vector3D dp1U, out Vector3D dp1V); // derivee de la new surface

        for (i = 1; i <= 3; i++)
        {
            f[i] = p.Coord(i) - p1.Coord(i);

            d[i, 1] = dp.Coord(i);
            d[i, 2] = -dp1U.Coord(i);
            d[i, 3] = -dp1V.Coord(i);
        }

        return true;
    }

    /// <summary>The first derivative in t.</summary>
    public bool DerivT(CurveAdaptorBase c, double param, double w, in Vector3D dN, double teta, math_Vector f)
    {
        c.D1(param, out _, out Vector3D dp); // derivee de la section

        // OCCT: the term W cos(teta) B' (the turning binormal) is missing, so LocationDraft's trace derivatives are
        // wrong wherever the binormal turns.
        f[1] = dp.Coord(1) + w * dN.Coord(1) * Math.Sin(teta);
        f[2] = dp.Coord(2) + w * dN.Coord(2) * Math.Sin(teta);
        f[3] = dp.Coord(3) + w * dN.Coord(3) * Math.Sin(teta);

        return true;
    }

    /// <summary>The second derivative in t.</summary>
    public bool Deriv2T(CurveAdaptorBase c, double param, double w, in Vector3D d2N, double teta, math_Vector f)
    {
        c.D2(param, out _, out _, out Vector3D d2P); // derivee de la section

        f[1] = d2P.Coord(1) + w * d2N.Coord(1) * Math.Sin(teta);
        f[2] = d2P.Coord(2) + w * d2N.Coord(2) * Math.Sin(teta);
        f[3] = d2P.Coord(3) + w * d2N.Coord(3) * Math.Sin(teta);

        return true;
    }

    /// <summary>The second derivative in t and X.</summary>
    public bool DerivTX(in Vector3D dN, double teta, Matrix d)
    {
        int i;
        for (i = 1; i <= 3; i++)
        {
            d[i, 1] = dN.Coord(i) * Math.Sin(teta); // derivee / W
            d[i, 2] = 0.0;                          // derivee / U
            d[i, 3] = 0.0;                          // derivee / V
        }

        return true;
    }

    /// <summary>The second derivative in X.</summary>
    public bool Deriv2X(math_Vector x, SurfaceTensor t)
    {
        int i;

        _theSurface.D2(x[2], x[3], out _, out _, out _, out Vector3D d2Pu, out Vector3D d2Pv, out Vector3D d2Puv);

        t.Init(0.0); // tenseur

        for (i = 1; i <= 3; i++)
        {
            t[i, 2, 2] = -d2Pu.Coord(i);
            t[i, 3, 2] = t[i, 2, 3] = -d2Puv.Coord(i);
            t[i, 3, 3] = -d2Pv.Coord(i);
        }

        return true;
    }
}
