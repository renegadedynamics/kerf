// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GtoCConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GtoCConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GtoCConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A G1 (G2, G3) continuity constraint of a plate at a point: the pinpoint constraints on the derivatives that make the
/// deformed surface (of initial derivatives D1S, D2S, D3S) tangent (osculating) to a target surface (D1T, D2T, D3T).
/// With a direction nP, the correction is kept normal to nP.
/// </summary>
/// <remarks>
/// A class: OCCT's by-value class holds an array of nine pinpoint constraints, which a C++ copy copies (the copy
/// constructor here). <c>nb_PPC()</c> is 0 where the normals degenerate or are nearly orthogonal, 2 after the G1
/// constraints, 5 after G2 and 9 after G3.
/// </remarks>
internal class GtoCConstraint
{
    private readonly PinpointConstraint[] _ppc = new PinpointConstraint[9];
    private readonly D1 _d1SurfInit;
    private readonly Xy _pnt2d;
    private readonly int nb_PPConstraints;

    private const double Normin = 1.0e-10;
    private const double Cosmin = 1.0e-2;

    public GtoCConstraint(GtoCConstraint @ref)
    {
        _d1SurfInit = @ref._d1SurfInit;
        _pnt2d            = @ref._pnt2d;
        nb_PPConstraints = @ref.nb_PPConstraints;
        for (int i = 0; i < nb_PPConstraints; i++)
        {
            _ppc[i] = @ref._ppc[i];
        }
    }

    public GtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T)
    {
        _d1SurfInit = d1S;
        _pnt2d            = point2d;
        nb_PPConstraints = 0;

        Xyz normale = d1T._du ^ d1T._dv;
        // alr le 12/11/96
        if (normale.Modulus() < Normin)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints

        Xyz normaleS = d1S._du ^ d1S._dv;
        // alr le 12/11/96
        if (normaleS.Modulus() < Normin)
        {
            return;
        }
        normaleS.Normalize();
        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < Cosmin)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        Xyz du = normaleS * -(normale * d1S._du) * invcos;
        Xyz dv = normaleS * -(normale * d1S._dv) * invcos;

        _ppc[0] = new PinpointConstraint(_pnt2d, du, 1, 0);
        _ppc[1] = new PinpointConstraint(_pnt2d, dv, 0, 1);

        nb_PPConstraints = 2;
    }

    public GtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T, in Xyz nP)
    {
        _d1SurfInit = d1S;
        _pnt2d            = point2d;
        nb_PPConstraints = 0;

        Xyz normale = d1T._du ^ d1T._dv;
        if (normale.Modulus() < Normin)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints

        Xyz normaleS = d1S._du ^ d1S._dv;
        if (normaleS.Modulus() < Normin)
        {
            return;
        }
        normaleS.Normalize();

        Xyz nSP = normaleS - nP * (nP * normaleS);
        if (nSP.Modulus() < Normin)
        {
            return;
        }
        nSP.Normalize();

        double cos_normales = normale * nSP;
        if (Math.Abs(cos_normales) < Cosmin)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        Xyz du = nSP * -(normale * d1S._du) * invcos;
        Xyz dv = nSP * -(normale * d1S._dv) * invcos;

        _ppc[0] = new PinpointConstraint(_pnt2d, du, 1, 0);
        _ppc[1] = new PinpointConstraint(_pnt2d, dv, 0, 1);

        nb_PPConstraints = 2;
    }

    public GtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T, in D2 d2S, in D2 d2T)
    {
        _d1SurfInit = d1S;

        _pnt2d            = point2d;
        nb_PPConstraints = 0;

        Xyz normale = d1T._du ^ d1T._dv;
        // alr le 12/11/96
        if (normale.Modulus() < Normin)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        Xyz normaleS = d1S._du ^ d1S._dv;
        // alr le 12/11/96
        if (normaleS.Modulus() < Normin)
        {
            return;
        }
        normaleS.Normalize();

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < Cosmin)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        Xyz du = normaleS * -(normale * d1S._du) * invcos;
        Xyz dv = normaleS * -(normale * d1S._dv) * invcos;

        _ppc[0] = new PinpointConstraint(_pnt2d, du, 1, 0);
        _ppc[1] = new PinpointConstraint(_pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        Xyz su = d1S._du + du;
        Xyz sv = d1S._dv + dv;

        Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = su * d1T._du;
        mat[0, 1] = su * d1T._dv;
        mat[1, 0] = sv * d1T._du;
        mat[1, 1] = sv * d1T._dv;
        Gauss gauss = new(mat);
        if (!gauss.IsDone)
        {
            return;
        }

        math_Vector vec = new(0, 1);
        vec[0] = su * su;
        vec[1] = su * sv;
        math_Vector sol = new(0, 1);

        gauss.Solve(vec, sol);
        double a = sol[0];
        double b = sol[1];

        vec[0] = sv * su;
        vec[1] = sv * sv;

        gauss.Solve(vec, sol);
        double c = sol[0];
        double d = sol[1];

        Xyz suu = d2T._duu * (a * a) + d2T._duv * (2 * a * b) + d2T._dvv * (b * b);
        Xyz suv = d2T._duu * (a * c) + d2T._duv * (a * d + b * c) + d2T._dvv * (b * d);
        Xyz svv = d2T._duu * (c * c) + d2T._duv * (2 * c * d) + d2T._dvv * (d * d);

        Xyz duu = normaleS * (normale * (suu - d2S._duu)) * invcos;
        Xyz duv = normaleS * (normale * (suv - d2S._duv)) * invcos;
        Xyz dvv = normaleS * (normale * (svv - d2S._dvv)) * invcos;

        _ppc[2] = new PinpointConstraint(_pnt2d, duu, 2, 0);
        _ppc[3] = new PinpointConstraint(_pnt2d, duv, 1, 1);
        _ppc[4] = new PinpointConstraint(_pnt2d, dvv, 0, 2);
        nb_PPConstraints = 5;
    }

    public GtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T, in D2 d2S, in D2 d2T, in Xyz nP)
    {
        _d1SurfInit = d1S;

        _pnt2d            = point2d;
        nb_PPConstraints = 0;

        Xyz normale = d1T._du ^ d1T._dv;
        // alr le 12/11/96
        if (normale.Modulus() < Normin)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        Xyz normaleS = d1S._du ^ d1S._dv;
        // alr le 12/11/96
        if (normaleS.Modulus() < Normin)
        {
            return;
        }
        normaleS.Normalize();

        Xyz nSP = normaleS - nP * (nP * normaleS);
        if (nSP.Modulus() < Normin)
        {
            return;
        }
        nSP.Normalize();

        double cos_normales = normale * nSP;
        if (Math.Abs(cos_normales) < Cosmin)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        Xyz du = nSP * -(normale * d1S._du) * invcos;
        Xyz dv = nSP * -(normale * d1S._dv) * invcos;

        _ppc[0] = new PinpointConstraint(_pnt2d, du, 1, 0);
        _ppc[1] = new PinpointConstraint(_pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        Xyz su = d1S._du + du;
        Xyz sv = d1S._dv + dv;

        Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = su * d1T._du;
        mat[0, 1] = su * d1T._dv;
        mat[1, 0] = sv * d1T._du;
        mat[1, 1] = sv * d1T._dv;
        Gauss gauss = new(mat);
        if (!gauss.IsDone)
        {
            return;
        }

        math_Vector vec = new(0, 1);
        vec[0] = su * su;
        vec[1] = su * sv;
        math_Vector sol = new(0, 1);

        gauss.Solve(vec, sol);
        double a = sol[0];
        double b = sol[1];

        vec[0] = sv * su;
        vec[1] = sv * sv;

        gauss.Solve(vec, sol);
        double c = sol[0];
        double d = sol[1];

        Xyz suu = d2T._duu * (a * a) + d2T._duv * (2 * a * b) + d2T._dvv * (b * b);
        Xyz suv = d2T._duu * (a * c) + d2T._duv * (a * d + b * c) + d2T._dvv * (b * d);
        Xyz svv = d2T._duu * (c * c) + d2T._duv * (2 * c * d) + d2T._dvv * (d * d);

        Xyz duu = nSP * (normale * (suu - d2S._duu)) * invcos;
        Xyz duv = nSP * (normale * (suv - d2S._duv)) * invcos;
        Xyz dvv = nSP * (normale * (svv - d2S._dvv)) * invcos;

        _ppc[2] = new PinpointConstraint(_pnt2d, duu, 2, 0);
        _ppc[3] = new PinpointConstraint(_pnt2d, duv, 1, 1);
        _ppc[4] = new PinpointConstraint(_pnt2d, dvv, 0, 2);
        nb_PPConstraints = 5;
    }

    public GtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T, in D2 d2S, in D2 d2T, in D3 d3S, in D3 d3T)
    {
        _d1SurfInit = d1S;
        _pnt2d            = point2d;
        nb_PPConstraints = 0;

        Xyz normale = d1T._du ^ d1T._dv;
        if (normale.Modulus() < Normin)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        Xyz normaleS = d1S._du ^ d1S._dv;
        if (normaleS.Modulus() < Normin)
        {
            return;
        }
        normaleS.Normalize();

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < Cosmin)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        Xyz du = normaleS * -(normale * d1S._du) * invcos;
        Xyz dv = normaleS * -(normale * d1S._dv) * invcos;

        _ppc[0] = new PinpointConstraint(_pnt2d, du, 1, 0);
        _ppc[1] = new PinpointConstraint(_pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        Xyz su = d1S._du + du;
        Xyz sv = d1S._dv + dv;

        Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = su * d1T._du;
        mat[0, 1] = su * d1T._dv;
        mat[1, 0] = sv * d1T._du;
        mat[1, 1] = sv * d1T._dv;
        Gauss gauss = new(mat);
        if (!gauss.IsDone)
        {
            return;
        }

        math_Vector vec = new(0, 1);
        vec[0] = su * su;
        vec[1] = su * sv;
        math_Vector sol = new(0, 1);

        gauss.Solve(vec, sol);
        double a = sol[0];
        double b = sol[1];

        vec[0] = sv * su;
        vec[1] = sv * sv;

        gauss.Solve(vec, sol);
        double c = sol[0];
        double d = sol[1];

        Xyz suu = d2T._duu * (a * a) + d2T._duv * (2 * a * b) + d2T._dvv * (b * b);
        Xyz suv = d2T._duu * (a * c) + d2T._duv * (a * d + b * c) + d2T._dvv * (b * d);
        Xyz svv = d2T._duu * (c * c) + d2T._duv * (2 * c * d) + d2T._dvv * (d * d);

        Xyz duu = normaleS * (normale * (suu - d2S._duu)) * invcos;
        Xyz duv = normaleS * (normale * (suv - d2S._duv)) * invcos;
        Xyz dvv = normaleS * (normale * (svv - d2S._dvv)) * invcos;

        _ppc[2] = new PinpointConstraint(_pnt2d, duu, 2, 0);
        _ppc[3] = new PinpointConstraint(_pnt2d, duv, 1, 1);
        _ppc[4] = new PinpointConstraint(_pnt2d, dvv, 0, 2);
        nb_PPConstraints = 5;

        // G3 Constraints

        vec[0] = (d2S._duu + duu - suu) * su;
        vec[1] = (d2S._duu + duu - suu) * sv;
        gauss.Solve(vec, sol);
        double b0uu = sol[0];
        double b1uu = sol[1];

        vec[0] = (d2S._duv + duv - suv) * su;
        vec[1] = (d2S._duv + duv - suv) * sv;
        gauss.Solve(vec, sol);
        double b0uv = sol[0];
        double b1uv = sol[1];

        vec[0] = (d2S._dvv + dvv - svv) * su;
        vec[1] = (d2S._dvv + dvv - svv) * sv;
        gauss.Solve(vec, sol);
        double b0vv = sol[0];
        double b1vv = sol[1];

        Xyz suuu = d3T._duuu * (a * a * a) + d3T._duuv * (3 * a * a * b) + d3T._duvv * (3 * a * b * b)
                                    + d3T._dvvv * (b * b * b);
        Xyz suuv = d3T._duuu * (a * a * c) + d3T._duuv * (a * a * d + 2 * a * b * c)
                                    + d3T._duvv * (b * b * c + 2 * a * b * d) + d3T._dvvv * (b * b * d);
        Xyz suvv = d3T._duuu * (a * c * c) + d3T._duuv * (b * c * c + 2 * a * c * d)
                                    + d3T._duvv * (a * d * d + 2 * b * c * d) + d3T._dvvv * (b * d * d);
        Xyz svvv = d3T._duuu * (c * c * c) + d3T._duuv * (3 * c * c * d) + d3T._duvv * (3 * c * d * d)
                                    + d3T._dvvv * (d * d * d);

        ref double a0u = ref a;
        ref double a1u = ref b;
        ref double a0v = ref c;
        ref double a1v = ref d;
        suuu += d2T._duu * (3 * a0u * b0uu) + d2T._duv * (3 * (a0u * b1uu + a1u * b0uu))
                        + d2T._dvv * (3 * a1u * b1uu);
        suuv += d2T._duu * (2 * a0u * b0uv + a0v * b0uu)
                        + d2T._duv * (2 * (a0u * b1uv + a1u * b0uv) + a0v * b1uu + a1v * b0uu)
                        + d2T._dvv * (2 * a1u * b1uv + a1v * b1uu);
        suvv += d2T._duu * (a0u * b0vv + 2 * a0v * b0uv)
                        + d2T._duv * (2 * (a0v * b1uv + a1v * b0uv) + a0u * b1vv + a1u * b0vv)
                        + d2T._dvv * (2 * a1v * b1uv + a1u * b1vv);
        svvv += d2T._duu * (3 * a0v * b0vv) + d2T._duv * (3 * (a0v * b1vv + a1v * b0vv))
                        + d2T._dvv * (3 * a1v * b1vv);

        Xyz duuu = normaleS * (normale * (suuu - d3S._duuu)) * invcos;
        Xyz duuv = normaleS * (normale * (suuv - d3S._duuv)) * invcos;
        Xyz duvv = normaleS * (normale * (suvv - d3S._duvv)) * invcos;
        Xyz dvvv = normaleS * (normale * (svvv - d3S._dvvv)) * invcos;

        _ppc[5] = new PinpointConstraint(_pnt2d, duuu, 3, 0);
        _ppc[6] = new PinpointConstraint(_pnt2d, duuv, 2, 1);
        _ppc[7] = new PinpointConstraint(_pnt2d, duvv, 1, 2);
        _ppc[8] = new PinpointConstraint(_pnt2d, dvvv, 0, 3);
        nb_PPConstraints = 9;
    }

    public GtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T, in D2 d2S, in D2 d2T, in D3 d3S, in D3 d3T, in Xyz nP)
    {
        _ = nP; // OCCT: this variant ignores nP (its name is commented out in OCCT's signature)
        _d1SurfInit = d1S;

        _pnt2d            = point2d;
        nb_PPConstraints = 0;

        Xyz normale = d1T._du ^ d1T._dv;
        if (normale.Modulus() < Normin)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        Xyz normaleS = d1S._du ^ d1S._dv;
        if (normaleS.Modulus() < Normin)
        {
            return;
        }
        normaleS.Normalize();

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < Cosmin)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        Xyz du = normaleS * -(normale * d1S._du) * invcos;
        Xyz dv = normaleS * -(normale * d1S._dv) * invcos;

        _ppc[0] = new PinpointConstraint(_pnt2d, du, 1, 0);
        _ppc[1] = new PinpointConstraint(_pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        Xyz su = d1S._du + du;
        Xyz sv = d1S._dv + dv;

        Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = su * d1T._du;
        mat[0, 1] = su * d1T._dv;
        mat[1, 0] = sv * d1T._du;
        mat[1, 1] = sv * d1T._dv;
        Gauss gauss = new(mat);

        if (!gauss.IsDone)
        {
            return;
        }

        math_Vector vec = new(0, 1);
        vec[0] = su * su;
        vec[1] = su * sv;
        math_Vector sol = new(0, 1);

        gauss.Solve(vec, sol);
        double a = sol[0];
        double b = sol[1];

        vec[0] = sv * su;
        vec[1] = sv * sv;

        gauss.Solve(vec, sol);
        double c = sol[0];
        double d = sol[1];

        Xyz suu = d2T._duu * (a * a) + d2T._duv * (2 * a * b) + d2T._dvv * (b * b);
        Xyz suv = d2T._duu * (a * c) + d2T._duv * (a * d + b * c) + d2T._dvv * (b * d);
        Xyz svv = d2T._duu * (c * c) + d2T._duv * (2 * c * d) + d2T._dvv * (d * d);

        Xyz duu = normaleS * (normale * (suu - d2S._duu)) * invcos;
        Xyz duv = normaleS * (normale * (suv - d2S._duv)) * invcos;
        Xyz dvv = normaleS * (normale * (svv - d2S._dvv)) * invcos;

        _ppc[2] = new PinpointConstraint(_pnt2d, duu, 2, 0);
        _ppc[3] = new PinpointConstraint(_pnt2d, duv, 1, 1);
        _ppc[4] = new PinpointConstraint(_pnt2d, dvv, 0, 2);
        nb_PPConstraints = 5;

        // G3 Constraints
        vec[0] = (d2S._duu + duu - suu) * su;
        vec[1] = (d2S._duu + duu - suu) * sv;
        gauss.Solve(vec, sol);
        double b0uu = sol[0];
        double b1uu = sol[1];

        vec[0] = (d2S._duv + duv - suv) * su;
        vec[1] = (d2S._duv + duv - suv) * sv;
        gauss.Solve(vec, sol);
        double b0uv = sol[0];
        double b1uv = sol[1];

        vec[0] = (d2S._dvv + dvv - svv) * su;
        vec[1] = (d2S._dvv + dvv - svv) * sv;
        gauss.Solve(vec, sol);
        double b0vv = sol[0];
        double b1vv = sol[1];

        Xyz suuu = d3T._duuu * (a * a * a) + d3T._duuv * (3 * a * a * b) + d3T._duvv * (3 * a * b * b)
                                    + d3T._dvvv * (b * b * b);
        Xyz suuv = d3T._duuu * (a * a * c) + d3T._duuv * (a * a * d + 2 * a * b * c)
                                    + d3T._duvv * (b * b * c + 2 * a * b * d) + d3T._dvvv * (b * b * d);
        Xyz suvv = d3T._duuu * (a * c * c) + d3T._duuv * (b * c * c + 2 * a * c * d)
                                    + d3T._duvv * (a * d * d + 2 * b * c * d) + d3T._dvvv * (b * d * d);
        Xyz svvv = d3T._duuu * (c * c * c) + d3T._duuv * (3 * c * c * d) + d3T._duvv * (3 * c * d * d)
                                    + d3T._dvvv * (d * d * d);

        ref double a0u = ref a;
        ref double a1u = ref b;
        ref double a0v = ref c;
        ref double a1v = ref d;
        suuu += d2T._duu * (3 * a0u * b0uu) + d2T._duv * (3 * (a0u * b1uu + a1u * b0uu))
                        + d2T._dvv * (3 * a1u * b1uu);
        suuv += d2T._duu * (2 * a0u * b0uv + a0v * b0uu)
                        + d2T._duv * (2 * (a0u * b1uv + a1u * b0uv) + a0v * b1uu + a1v * b0uu)
                        + d2T._dvv * (2 * a1u * b1uv + a1v * b1uu);
        suvv += d2T._duu * (a0u * b0vv + 2 * a0v * b0uv)
                        + d2T._duv * (2 * (a0v * b1uv + a1v * b0uv) + a0u * b1vv + a1u * b0vv)
                        + d2T._dvv * (2 * a1v * b1uv + a1u * b1vv);
        svvv += d2T._duu * (3 * a0v * b0vv) + d2T._duv * (3 * (a0v * b1vv + a1v * b0vv))
                        + d2T._dvv * (3 * a1v * b1vv);

        Xyz duuu = normaleS * (normale * (suuu - d3S._duuu)) * invcos;
        Xyz duuv = normaleS * (normale * (suuv - d3S._duuv)) * invcos;
        Xyz duvv = normaleS * (normale * (suvv - d3S._duvv)) * invcos;
        Xyz dvvv = normaleS * (normale * (svvv - d3S._dvvv)) * invcos;

        _ppc[5] = new PinpointConstraint(_pnt2d, duuu, 3, 0);
        _ppc[6] = new PinpointConstraint(_pnt2d, duuv, 2, 1);
        _ppc[7] = new PinpointConstraint(_pnt2d, duvv, 1, 2);
        _ppc[8] = new PinpointConstraint(_pnt2d, dvvv, 0, 3);
        nb_PPConstraints = 9;
    }

    public int nb_PPC() => nb_PPConstraints;

    public PinpointConstraint GetPPC(int index) => _ppc[index];

    public D1 D1SurfInit() => _d1SurfInit;
}
