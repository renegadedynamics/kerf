// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_FreeGtoCConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_FreeGtoCConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_FreeGtoCConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A free G1 (G2, G3) continuity constraint of a plate at a point: as <see cref="GtoCConstraint"/>, but only the
/// components of the derivatives along the target's normal are imposed (linear scalar constraints), with an optional
/// incremental load that rotates the target towards the surface (NLPlate's incremental solve).
/// </summary>
/// <remarks>
/// A class: OCCT's by-value class holds arrays of five pinpoint and four linear scalar constraints, which a C++ copy
/// copies (the linear constraints then share their arrays, see <see cref="LinearScalarConstraint"/>).
/// </remarks>
public class FreeGtoCConstraint
{
    // Locals of OCCT's inner blocks that a later declaration reuses are renamed with a 1 (C# scopes): du1, dv1, c1, s1, d1.
    private readonly Xy _pnt2d;
    private readonly int nb_PPConstraints;
    private readonly int nb_LSConstraints;
    private readonly PinpointConstraint[] _ppc = new PinpointConstraint[5];
    private readonly LinearScalarConstraint[] _lsc =
    [
        new LinearScalarConstraint(), new LinearScalarConstraint(),
        new LinearScalarConstraint(), new LinearScalarConstraint(),
    ];

    private const double Normin = 1.0e-10;
    private const double Cosmin = 1.0e-2;

    public FreeGtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T, double incrementalLoad = 1.0, int orientation = 0)
    {
        _pnt2d            = point2d;
        nb_PPConstraints = 0;
        nb_LSConstraints = 0;

        Xyz normale = d1T._du ^ d1T._dv;
        if (normale.Modulus() < Normin)
        {
            return;
        }
        normale.Normalize();

        if (incrementalLoad != 1.0)
        {
            Xyz n0 = d1S._du ^ d1S._dv;
            if (n0.Modulus() < Normin)
            {
                return;
            }
            n0.Normalize();
            Xyz n1 = normale;
            if (orientation != 0)
            {
                n1 *= orientation;
            }
            double c1 = n0 * n1;
            if (orientation == 0)
            {
                if (c1 < 0.0)
                {
                    c1 *= -1.0;
                    n1 *= -1.0;
                }
            }

            double s1 = n0.CrossMagnitude(n1);
            if ((s1 < 1.0e-2) && (c1 < 0.0))
            {
                return;
            }
            double angle = Math.Atan2(c1, s1);
            // if (angle < 0.0) angle += M_PI;

            Xyz d1 = n0 ^ n1;
            d1.Normalize();
            Direction3D dir = new Direction3D(d1);
            Transform3D rota = new();
            Axis3D  axe = new(new Point3D(0, 0, 0), dir);
            rota.SetRotation(axe, angle * (incrementalLoad - 1.0));
            //      gp_Trsf rota = gce_MakeRotation(new gp_Pnt(0,0,0), dir, angle*(IncrementalLoad-1.0));
            rota.Transforms(ref normale);
        }

        Xyz du = d1S._du * (-1.0);
        Xyz dv = d1S._dv * (-1.0);

        _lsc[0] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, du, 1, 0), normale);
        _lsc[1] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, dv, 0, 1), normale);
        nb_LSConstraints = 2;
    }

    public FreeGtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T0, in D2 d2S, in D2 d2T0, double incrementalLoad = 1.0, int orientation = 0)
    {
        _pnt2d            = point2d;
        nb_PPConstraints = 0;
        nb_LSConstraints = 0;
        D1 d1T     = d1T0;
        D2 d2T     = d2T0;

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
            if (incrementalLoad != 1.0)
            {
                return;
            }
            Xyz du1 = d1S._du * (-1.0);
            Xyz dv1 = d1S._dv * (-1.0);

            _lsc[0] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, du1, 1, 0), normale);
            _lsc[1] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, dv1, 0, 1), normale);
            nb_LSConstraints = 2;
            return;
        }
        normaleS.Normalize();

        if (incrementalLoad != 1.0)
        {
            Xyz n0 = normaleS;
            Xyz n1 = normale;
            if (orientation != 0)
            {
                n1 *= orientation;
            }
            double c1 = n0 * n1;
            if (orientation == 0)
            {
                if (c1 < 0.0)
                {
                    c1 *= -1.0;
                    n1 *= -1.0;
                }
            }

            double s1 = n0.CrossMagnitude(n1);
            if ((s1 < 1.0e-2) && (c1 < 0.0))
            {
                return;
            }
            double angle = Math.Atan2(c1, s1);

            Xyz d1 = n0 ^ n1;
            d1.Normalize();
            Direction3D dir = new Direction3D(d1);
            Transform3D rota = new();
            Axis3D  axe = new(new Point3D(0, 0, 0), dir);
            rota.SetRotation(axe, angle * (incrementalLoad - 1.0));
            //      gp_Trsf rota = gce_MakeRotation(new gp_Pnt(0,0,0), dir, angle*(IncrementalLoad-1.0));
            rota.Transforms(ref normale);
            rota.Transforms(ref d1T._du);
            rota.Transforms(ref d1T._dv);
            rota.Transforms(ref d2T._duu);
            rota.Transforms(ref d2T._duv);
            rota.Transforms(ref d2T._dvv);
        }

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < Cosmin)
        {
            Xyz du1 = d1S._du * (-1.0);
            Xyz dv1 = d1S._dv * (-1.0);

            _lsc[0] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, du1, 1, 0), normale);
            _lsc[1] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, dv1, 0, 1), normale);
            nb_LSConstraints = 2;
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

        Xyz duu = suu - d2S._duu;
        Xyz duv = suv - d2S._duv;
        Xyz dvv = svv - d2S._dvv;
        duu *= incrementalLoad;
        duv *= incrementalLoad;
        dvv *= incrementalLoad;

        _lsc[0] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, duu, 2, 0), normale);
        _lsc[1] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, duv, 1, 1), normale);
        _lsc[2] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, dvv, 0, 2), normale);
        nb_LSConstraints = 3;
    }

    public FreeGtoCConstraint(in Xy point2d, in D1 d1S, in D1 d1T0, in D2 d2S, in D2 d2T0, in D3 d3S, in D3 d3T0, double incrementalLoad = 1.0, int orientation = 0)
    {
        _pnt2d            = point2d;
        nb_PPConstraints = 0;
        nb_LSConstraints = 0;
        D1 d1T     = d1T0;
        D2 d2T     = d2T0;
        D3 d3T     = d3T0;

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
            if (incrementalLoad != 1.0)
            {
                return;
            }
            Xyz du1 = d1S._du * (-1.0);
            Xyz dv1 = d1S._dv * (-1.0);

            _lsc[0] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, du1, 1, 0), normale);
            _lsc[1] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, dv1, 0, 1), normale);
            nb_LSConstraints = 2;
            return;
        }
        normaleS.Normalize();

        if (incrementalLoad != 1.0)
        {
            Xyz n0 = normaleS;
            Xyz n1 = normale;
            if (orientation != 0)
            {
                n1 *= orientation;
            }
            double c1 = n0 * n1;
            if (orientation == 0)
            {
                if (c1 < 0.0)
                {
                    c1 *= -1.0;
                    n1 *= -1.0;
                }
            }
            double s1 = n0.CrossMagnitude(n1);
            if ((s1 < 1.0e-2) && (c1 < 0.0))
            {
                return;
            }
            double angle = Math.Atan2(c1, s1);

            Xyz d1 = n0 ^ n1;
            d1.Normalize();
            Direction3D dir = new Direction3D(d1);
            Transform3D rota = new();
            Axis3D  axe = new(new Point3D(0, 0, 0), dir);
            rota.SetRotation(axe, angle * (incrementalLoad - 1.0));
            //      gp_Trsf rota = gce_MakeRotation(new gp_Pnt(0,0,0), dir, angle*(IncrementalLoad-1.0));
            rota.Transforms(ref normale);
            rota.Transforms(ref d1T._du);
            rota.Transforms(ref d1T._dv);
            rota.Transforms(ref d2T._duu);
            rota.Transforms(ref d2T._duv);
            rota.Transforms(ref d2T._dvv);
            rota.Transforms(ref d3T._duuu);
            rota.Transforms(ref d3T._duuv);
            rota.Transforms(ref d3T._duvv);
            rota.Transforms(ref d3T._dvvv);
        }

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < Cosmin)
        {
            Xyz du1 = d1S._du * (-1.0);
            Xyz dv1 = d1S._dv * (-1.0);

            _lsc[0] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, du1, 1, 0), normale);
            _lsc[1] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, dv1, 0, 1), normale);
            nb_LSConstraints = 2;
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

        Xyz duuu = suuu - d3S._duuu;
        Xyz duuv = suuv - d3S._duuv;
        Xyz duvv = suvv - d3S._duvv;
        Xyz dvvv = svvv - d3S._dvvv;
        duuu *= incrementalLoad;
        duuv *= incrementalLoad;
        duvv *= incrementalLoad;
        dvvv *= incrementalLoad;

        _lsc[0] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, duuu, 3, 0), normale);
        _lsc[1] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, duuv, 2, 1), normale);
        _lsc[2] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, duvv, 1, 2), normale);
        _lsc[3] = new LinearScalarConstraint(new PinpointConstraint(_pnt2d, dvvv, 0, 3), normale);
        nb_LSConstraints = 4;
    }

    public int nb_PPC() => nb_PPConstraints;

    public PinpointConstraint GetPPC(int index) => _ppc[index];

    public int nb_LSC() => nb_LSConstraints;

    public LinearScalarConstraint LSC(int index) => _lsc[index];
}
