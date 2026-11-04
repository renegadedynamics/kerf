// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The square distance between a curve and a surface,
/// F(cu, su, sv) = |C(cu) - S(su, sv)|^2, with its gradient and hessian (each halved), for global optimisation.
/// </summary>
/// <remarks>
/// The curve and the surface are kept by reference and must outlive the function, as in OCCT. Outside the parameter
/// ranges the functions return false and write 0 to their <c>out</c> value (decision 221).
/// </remarks>
internal class GlobOptFuncCS : MultipleVarFunctionWithHessian
{
    private readonly CurveAdaptorBase _c;
    private readonly SurfaceAdaptorBase _s;

    // F(cu, su, sv)=(C^{(x)}(cu)-S^{(x)}(su,sv))^{2}+
    //               (C^{(y)}(cu)-S^{(y)}(su,sv))^{2}+
    //               (C^{(z)}(cu)-S^{(z)}(su,sv))^{2}

    private void ValueCore(double cu, double su, double sv, out double f)
    {
        f = _c.Value(cu).SquareDistanceTo(_s.Value(su, sv));
    }

    private void GradientCore(double cu, double su, double sv, math_Vector g)
    {
        _c.D1(cu, out Point3D cd0, out Vector3D cd1);
        _s.D1(su, sv, out Point3D sd0, out Vector3D sd1U, out Vector3D sd1V);

        g[1] = +(cd0.X - sd0.X) * cd1.X + (cd0.Y - sd0.Y) * cd1.Y + (cd0.Z - sd0.Z) * cd1.Z;
        g[2] = -(cd0.X - sd0.X) * sd1U.X - (cd0.Y - sd0.Y) * sd1U.Y
               - (cd0.Z - sd0.Z) * sd1U.Z;
        g[3] = -(cd0.X - sd0.X) * sd1V.X - (cd0.Y - sd0.Y) * sd1V.Y
               - (cd0.Z - sd0.Z) * sd1V.Z;
    }

    private void Hessian(double cu, double su, double sv, Matrix h)
    {
        _c.D2(cu, out Point3D cd0, out Vector3D cd1, out Vector3D cd2);
        _s.D2(su, sv, out Point3D sd0, out Vector3D sd1U, out Vector3D sd1V, out Vector3D sd2UU, out Vector3D sd2VV,
            out Vector3D sd2UV);

        h[1, 1] = +cd1.X * cd1.X + cd1.Y * cd1.Y + cd1.Z * cd1.Z
                  + (cd0.X - sd0.X) * cd2.X + (cd0.Y - sd0.Y) * cd2.Y
                  + (cd0.Z - sd0.Z) * cd2.Z;

        h[1, 2] = -cd1.X * sd1U.X - cd1.Y * sd1U.Y - cd1.Z * sd1U.Z;

        h[1, 3] = -cd1.X * sd1V.X - cd1.Y * sd1V.Y - cd1.Z * sd1V.Z;

        h[2, 1] = h[1, 2];

        h[2, 2] = +sd1U.X * sd1U.X + sd1U.Y * sd1U.Y + sd1U.Z * sd1U.Z
                  - (cd0.X - sd0.X) * sd2UU.X - (cd0.Y - sd0.Y) * sd2UU.Y
                  - (cd0.Z - sd0.Z) * sd2UU.Z;

        h[2, 3] = +sd1U.X * sd1V.X + sd1U.Y * sd1V.Y + sd1U.Z * sd1V.Z
                  - (cd0.X - sd0.X) * sd2UV.X - (cd0.Y - sd0.Y) * sd2UV.Y
                  - (cd0.Z - sd0.Z) * sd2UV.Z;

        h[3, 1] = h[1, 3];

        h[3, 2] = h[2, 3];

        h[3, 3] = +sd1V.X * sd1V.X + sd1V.Y * sd1V.Y + sd1V.Z * sd1V.Z
                  - (cd0.X - sd0.X) * sd2VV.X - (cd0.Y - sd0.Y) * sd2VV.Y
                  - (cd0.Z - sd0.Z) * sd2VV.Z;
    }

    private bool CheckInputData(math_Vector x, out double cu, out double su, out double sv)
    {
        int startIndex = x.Lower();
        cu = x[startIndex];
        su = x[startIndex + 1];
        sv = x[startIndex + 2];

        return cu >= _c.FirstParameter() && cu <= _c.LastParameter() && su >= _s.FirstUParameter()
               && su <= _s.LastUParameter() && sv >= _s.FirstVParameter()
               && sv <= _s.LastVParameter();
    }

    /// <summary>The function of a curve and a surface, which must outlive it.</summary>
    public GlobOptFuncCS(CurveAdaptorBase c, SurfaceAdaptorBase s)
    {
        _c = c;
        _s = s;
    }

    public override int VariableCount => 3;

    public override bool Value(math_Vector x, out double f)
    {
        if (!CheckInputData(x, out double cu, out double su, out double sv))
        {
            f = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        ValueCore(cu, su, sv, out f);
        return true;
    }

    public override bool Gradient(math_Vector x, math_Vector g)
    {
        if (!CheckInputData(x, out double cu, out double su, out double sv))
        {
            return false;
        }

        GradientCore(cu, su, sv, g);
        return true;
    }

    public override bool Values(math_Vector x, out double f, math_Vector g)
    {
        if (!CheckInputData(x, out double cu, out double su, out double sv))
        {
            f = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        ValueCore(cu, su, sv, out f);
        GradientCore(cu, su, sv, g);
        return true;
    }

    public override bool Values(math_Vector x, out double f, math_Vector g, Matrix h)
    {
        if (!CheckInputData(x, out double cu, out double su, out double sv))
        {
            f = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        ValueCore(cu, su, sv, out f);
        GradientCore(cu, su, sv, g);
        Hessian(cu, su, sv, h);
        return true;
    }
}
