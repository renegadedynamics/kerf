// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncPSDist.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncPSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The square distance between a point P and a surface S, F(u, v) = SquareDist(P, S(u, v)), with its gradient
/// ((S - P).Su, (S - P).Sv), for math_BFGS.
/// </summary>
/// <remarks>
/// OCCT keeps references to the surface and the point; the port keeps the surface adaptor (which must outlive the
/// function, as in OCCT) and a copy of the point. Outside the surface's parameter range the functions return false and
/// write 0 to their <c>out</c> value (decision 221).
/// </remarks>
internal class FuncPSDist : MultipleVarFunctionWithGradient
{
    private readonly SurfaceAdaptorBase _surf;
    private readonly Point3D _p;

    public FuncPSDist(SurfaceAdaptorBase s, in Point3D p)
    {
        _surf = s;
        _p = p;
    }

    public override int VariableCount => 2;

    public override bool Value(math_Vector x, out double f)
    {
        if (!IsInside(x))
        {
            f = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        f = _surf.Value(x[1], x[2]).SquareDistanceTo(_p);

        return true;
    }

    public override bool Gradient(math_Vector x, math_Vector g)
    {
        if (!IsInside(x))
        {
            return false;
        }

        _surf.D1(x[1], x[2], out Point3D p, out Vector3D du1s, out Vector3D dv1s);

        Vector3D p1P2 = new(p, _p);

        g[1] = p1P2.Dot(du1s);
        g[2] = p1P2.Dot(dv1s);

        return true;
    }

    public override bool Values(math_Vector x, out double f, math_Vector g)
    {
        if (!IsInside(x))
        {
            f = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        _surf.D1(x[1], x[2], out Point3D p, out Vector3D du1s, out Vector3D dv1s);

        Vector3D p1P2 = new(p, _p);

        g[1] = p1P2.Dot(du1s);
        g[2] = p1P2.Dot(dv1s);

        f = _surf.Value(x[1], x[2]).SquareDistanceTo(_p);

        return true;
    }

    // Checks that the point is inside the surface's parameter space.
    private bool IsInside(math_Vector x)
    {
        return x[1] >= _surf.FirstUParameter() && x[1] <= _surf.LastUParameter()
               && x[2] >= _surf.FirstVParameter() && x[2] <= _surf.LastVParameter();
    }
}
