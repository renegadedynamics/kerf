// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_FreeGtoCConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_FreeGtoCConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_FreeGtoCConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A free G1 (G2, G3) continuity constraint of a plate at a point: as <see cref="Plate_GtoCConstraint"/>, but only the
/// components of the derivatives along the target's normal are imposed (linear scalar constraints), with an optional
/// incremental load that rotates the target towards the surface (NLPlate's incremental solve).
/// </summary>
/// <remarks>
/// A class: OCCT's by-value class holds arrays of five pinpoint and four linear scalar constraints, which a C++ copy
/// copies (the linear constraints then share their arrays, see <see cref="Plate_LinearScalarConstraint"/>).
/// </remarks>
public class Plate_FreeGtoCConstraint
{
    // Locals of OCCT's inner blocks that a later declaration reuses are renamed with a 1 (C# scopes): du1, dv1, c1, s1, d1.
    private readonly Xy pnt2d;
    private readonly int nb_PPConstraints;
    private readonly int nb_LSConstraints;
    private readonly Plate_PinpointConstraint[] myPPC = new Plate_PinpointConstraint[5];
    private readonly Plate_LinearScalarConstraint[] myLSC =
    [
        new Plate_LinearScalarConstraint(), new Plate_LinearScalarConstraint(),
        new Plate_LinearScalarConstraint(), new Plate_LinearScalarConstraint(),
    ];

    private const double NORMIN = 1.0e-10;
    private const double COSMIN = 1.0e-2;

    public Plate_FreeGtoCConstraint(in Xy point2d, in Plate_D1 D1S, in Plate_D1 D1T, double IncrementalLoad = 1.0, int orientation = 0)
    {
        pnt2d            = point2d;
        nb_PPConstraints = 0;
        nb_LSConstraints = 0;

        Xyz normale = D1T.Du ^ D1T.Dv;
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        if (IncrementalLoad != 1.0)
        {
            Xyz N0 = D1S.Du ^ D1S.Dv;
            if (N0.Modulus() < NORMIN)
            {
                return;
            }
            N0.Normalize();
            Xyz N1 = normale;
            if (orientation != 0)
            {
                N1 *= orientation;
            }
            double c1 = N0 * N1;
            if (orientation == 0)
            {
                if (c1 < 0.0)
                {
                    c1 *= -1.0;
                    N1 *= -1.0;
                }
            }

            double s1 = N0.CrossMagnitude(N1);
            if ((s1 < 1.0e-2) && (c1 < 0.0))
            {
                return;
            }
            double angle = Math.Atan2(c1, s1);
            // if (angle < 0.0) angle += M_PI;

            Xyz d1 = N0 ^ N1;
            d1.Normalize();
            Direction3D dir = new Direction3D(d1);
            Transform3D rota = new();
            Axis3D  Axe = new(new Point3D(0, 0, 0), dir);
            rota.SetRotation(Axe, angle * (IncrementalLoad - 1.0));
            //      gp_Trsf rota = gce_MakeRotation(new gp_Pnt(0,0,0), dir, angle*(IncrementalLoad-1.0));
            rota.Transforms(ref normale);
        }

        Xyz du = D1S.Du * (-1.0);
        Xyz dv = D1S.Dv * (-1.0);

        myLSC[0] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, du, 1, 0), normale);
        myLSC[1] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, dv, 0, 1), normale);
        nb_LSConstraints = 2;
    }

    public Plate_FreeGtoCConstraint(in Xy point2d, in Plate_D1 D1S, in Plate_D1 D1T0, in Plate_D2 D2S, in Plate_D2 D2T0, double IncrementalLoad = 1.0, int orientation = 0)
    {
        pnt2d            = point2d;
        nb_PPConstraints = 0;
        nb_LSConstraints = 0;
        Plate_D1 D1T     = D1T0;
        Plate_D2 D2T     = D2T0;

        Xyz normale = D1T.Du ^ D1T.Dv;
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        Xyz normaleS = D1S.Du ^ D1S.Dv;
        if (normaleS.Modulus() < NORMIN)
        {
            if (IncrementalLoad != 1.0)
            {
                return;
            }
            Xyz du1 = D1S.Du * (-1.0);
            Xyz dv1 = D1S.Dv * (-1.0);

            myLSC[0] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, du1, 1, 0), normale);
            myLSC[1] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, dv1, 0, 1), normale);
            nb_LSConstraints = 2;
            return;
        }
        normaleS.Normalize();

        if (IncrementalLoad != 1.0)
        {
            Xyz N0 = normaleS;
            Xyz N1 = normale;
            if (orientation != 0)
            {
                N1 *= orientation;
            }
            double c1 = N0 * N1;
            if (orientation == 0)
            {
                if (c1 < 0.0)
                {
                    c1 *= -1.0;
                    N1 *= -1.0;
                }
            }

            double s1 = N0.CrossMagnitude(N1);
            if ((s1 < 1.0e-2) && (c1 < 0.0))
            {
                return;
            }
            double angle = Math.Atan2(c1, s1);

            Xyz d1 = N0 ^ N1;
            d1.Normalize();
            Direction3D dir = new Direction3D(d1);
            Transform3D rota = new();
            Axis3D  Axe = new(new Point3D(0, 0, 0), dir);
            rota.SetRotation(Axe, angle * (IncrementalLoad - 1.0));
            //      gp_Trsf rota = gce_MakeRotation(new gp_Pnt(0,0,0), dir, angle*(IncrementalLoad-1.0));
            rota.Transforms(ref normale);
            rota.Transforms(ref D1T.Du);
            rota.Transforms(ref D1T.Dv);
            rota.Transforms(ref D2T.Duu);
            rota.Transforms(ref D2T.Duv);
            rota.Transforms(ref D2T.Dvv);
        }

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < COSMIN)
        {
            Xyz du1 = D1S.Du * (-1.0);
            Xyz dv1 = D1S.Dv * (-1.0);

            myLSC[0] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, du1, 1, 0), normale);
            myLSC[1] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, dv1, 0, 1), normale);
            nb_LSConstraints = 2;
            return;
        }

        double invcos = 1.0 / cos_normales;

        Xyz du = normaleS * -(normale * D1S.Du) * invcos;
        Xyz dv = normaleS * -(normale * D1S.Dv) * invcos;

        myPPC[0] = new Plate_PinpointConstraint(pnt2d, du, 1, 0);
        myPPC[1] = new Plate_PinpointConstraint(pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        Xyz Su = D1S.Du + du;
        Xyz Sv = D1S.Dv + dv;

        Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = Su * D1T.Du;
        mat[0, 1] = Su * D1T.Dv;
        mat[1, 0] = Sv * D1T.Du;
        mat[1, 1] = Sv * D1T.Dv;
        Gauss gauss = new(mat);
        if (!gauss.IsDone)
        {
            return;
        }

        math_Vector vec = new(0, 1);
        vec[0] = Su * Su;
        vec[1] = Su * Sv;
        math_Vector sol = new(0, 1);

        gauss.Solve(vec, sol);
        double a = sol[0];
        double b = sol[1];

        vec[0] = Sv * Su;
        vec[1] = Sv * Sv;

        gauss.Solve(vec, sol);
        double c = sol[0];
        double d = sol[1];

        Xyz Suu = D2T.Duu * (a * a) + D2T.Duv * (2 * a * b) + D2T.Dvv * (b * b);
        Xyz Suv = D2T.Duu * (a * c) + D2T.Duv * (a * d + b * c) + D2T.Dvv * (b * d);
        Xyz Svv = D2T.Duu * (c * c) + D2T.Duv * (2 * c * d) + D2T.Dvv * (d * d);

        Xyz duu = Suu - D2S.Duu;
        Xyz duv = Suv - D2S.Duv;
        Xyz dvv = Svv - D2S.Dvv;
        duu *= IncrementalLoad;
        duv *= IncrementalLoad;
        dvv *= IncrementalLoad;

        myLSC[0] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, duu, 2, 0), normale);
        myLSC[1] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, duv, 1, 1), normale);
        myLSC[2] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, dvv, 0, 2), normale);
        nb_LSConstraints = 3;
    }

    public Plate_FreeGtoCConstraint(in Xy point2d, in Plate_D1 D1S, in Plate_D1 D1T0, in Plate_D2 D2S, in Plate_D2 D2T0, in Plate_D3 D3S, in Plate_D3 D3T0, double IncrementalLoad = 1.0, int orientation = 0)
    {
        pnt2d            = point2d;
        nb_PPConstraints = 0;
        nb_LSConstraints = 0;
        Plate_D1 D1T     = D1T0;
        Plate_D2 D2T     = D2T0;
        Plate_D3 D3T     = D3T0;

        Xyz normale = D1T.Du ^ D1T.Dv;
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        Xyz normaleS = D1S.Du ^ D1S.Dv;
        if (normaleS.Modulus() < NORMIN)
        {
            if (IncrementalLoad != 1.0)
            {
                return;
            }
            Xyz du1 = D1S.Du * (-1.0);
            Xyz dv1 = D1S.Dv * (-1.0);

            myLSC[0] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, du1, 1, 0), normale);
            myLSC[1] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, dv1, 0, 1), normale);
            nb_LSConstraints = 2;
            return;
        }
        normaleS.Normalize();

        if (IncrementalLoad != 1.0)
        {
            Xyz N0 = normaleS;
            Xyz N1 = normale;
            if (orientation != 0)
            {
                N1 *= orientation;
            }
            double c1 = N0 * N1;
            if (orientation == 0)
            {
                if (c1 < 0.0)
                {
                    c1 *= -1.0;
                    N1 *= -1.0;
                }
            }
            double s1 = N0.CrossMagnitude(N1);
            if ((s1 < 1.0e-2) && (c1 < 0.0))
            {
                return;
            }
            double angle = Math.Atan2(c1, s1);

            Xyz d1 = N0 ^ N1;
            d1.Normalize();
            Direction3D dir = new Direction3D(d1);
            Transform3D rota = new();
            Axis3D  Axe = new(new Point3D(0, 0, 0), dir);
            rota.SetRotation(Axe, angle * (IncrementalLoad - 1.0));
            //      gp_Trsf rota = gce_MakeRotation(new gp_Pnt(0,0,0), dir, angle*(IncrementalLoad-1.0));
            rota.Transforms(ref normale);
            rota.Transforms(ref D1T.Du);
            rota.Transforms(ref D1T.Dv);
            rota.Transforms(ref D2T.Duu);
            rota.Transforms(ref D2T.Duv);
            rota.Transforms(ref D2T.Dvv);
            rota.Transforms(ref D3T.Duuu);
            rota.Transforms(ref D3T.Duuv);
            rota.Transforms(ref D3T.Duvv);
            rota.Transforms(ref D3T.Dvvv);
        }

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < COSMIN)
        {
            Xyz du1 = D1S.Du * (-1.0);
            Xyz dv1 = D1S.Dv * (-1.0);

            myLSC[0] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, du1, 1, 0), normale);
            myLSC[1] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, dv1, 0, 1), normale);
            nb_LSConstraints = 2;
            return;
        }

        double invcos = 1.0 / cos_normales;

        Xyz du = normaleS * -(normale * D1S.Du) * invcos;
        Xyz dv = normaleS * -(normale * D1S.Dv) * invcos;

        myPPC[0] = new Plate_PinpointConstraint(pnt2d, du, 1, 0);
        myPPC[1] = new Plate_PinpointConstraint(pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        Xyz Su = D1S.Du + du;
        Xyz Sv = D1S.Dv + dv;

        Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = Su * D1T.Du;
        mat[0, 1] = Su * D1T.Dv;
        mat[1, 0] = Sv * D1T.Du;
        mat[1, 1] = Sv * D1T.Dv;
        Gauss gauss = new(mat);
        if (!gauss.IsDone)
        {
            return;
        }

        math_Vector vec = new(0, 1);
        vec[0] = Su * Su;
        vec[1] = Su * Sv;
        math_Vector sol = new(0, 1);

        gauss.Solve(vec, sol);
        double a = sol[0];
        double b = sol[1];

        vec[0] = Sv * Su;
        vec[1] = Sv * Sv;

        gauss.Solve(vec, sol);
        double c = sol[0];
        double d = sol[1];

        Xyz Suu = D2T.Duu * (a * a) + D2T.Duv * (2 * a * b) + D2T.Dvv * (b * b);
        Xyz Suv = D2T.Duu * (a * c) + D2T.Duv * (a * d + b * c) + D2T.Dvv * (b * d);
        Xyz Svv = D2T.Duu * (c * c) + D2T.Duv * (2 * c * d) + D2T.Dvv * (d * d);

        Xyz duu = normaleS * (normale * (Suu - D2S.Duu)) * invcos;
        Xyz duv = normaleS * (normale * (Suv - D2S.Duv)) * invcos;
        Xyz dvv = normaleS * (normale * (Svv - D2S.Dvv)) * invcos;

        myPPC[2] = new Plate_PinpointConstraint(pnt2d, duu, 2, 0);
        myPPC[3] = new Plate_PinpointConstraint(pnt2d, duv, 1, 1);
        myPPC[4] = new Plate_PinpointConstraint(pnt2d, dvv, 0, 2);
        nb_PPConstraints = 5;

        // G3 Constraints

        vec[0] = (D2S.Duu + duu - Suu) * Su;
        vec[1] = (D2S.Duu + duu - Suu) * Sv;
        gauss.Solve(vec, sol);
        double B0uu = sol[0];
        double B1uu = sol[1];

        vec[0] = (D2S.Duv + duv - Suv) * Su;
        vec[1] = (D2S.Duv + duv - Suv) * Sv;
        gauss.Solve(vec, sol);
        double B0uv = sol[0];
        double B1uv = sol[1];

        vec[0] = (D2S.Dvv + dvv - Svv) * Su;
        vec[1] = (D2S.Dvv + dvv - Svv) * Sv;
        gauss.Solve(vec, sol);
        double B0vv = sol[0];
        double B1vv = sol[1];

        Xyz Suuu = D3T.Duuu * (a * a * a) + D3T.Duuv * (3 * a * a * b) + D3T.Duvv * (3 * a * b * b)
                                    + D3T.Dvvv * (b * b * b);
        Xyz Suuv = D3T.Duuu * (a * a * c) + D3T.Duuv * (a * a * d + 2 * a * b * c)
                                    + D3T.Duvv * (b * b * c + 2 * a * b * d) + D3T.Dvvv * (b * b * d);
        Xyz Suvv = D3T.Duuu * (a * c * c) + D3T.Duuv * (b * c * c + 2 * a * c * d)
                                    + D3T.Duvv * (a * d * d + 2 * b * c * d) + D3T.Dvvv * (b * d * d);
        Xyz Svvv = D3T.Duuu * (c * c * c) + D3T.Duuv * (3 * c * c * d) + D3T.Duvv * (3 * c * d * d)
                                    + D3T.Dvvv * (d * d * d);

        ref double A0u = ref a;
        ref double A1u = ref b;
        ref double A0v = ref c;
        ref double A1v = ref d;
        Suuu += D2T.Duu * (3 * A0u * B0uu) + D2T.Duv * (3 * (A0u * B1uu + A1u * B0uu))
                        + D2T.Dvv * (3 * A1u * B1uu);
        Suuv += D2T.Duu * (2 * A0u * B0uv + A0v * B0uu)
                        + D2T.Duv * (2 * (A0u * B1uv + A1u * B0uv) + A0v * B1uu + A1v * B0uu)
                        + D2T.Dvv * (2 * A1u * B1uv + A1v * B1uu);
        Suvv += D2T.Duu * (A0u * B0vv + 2 * A0v * B0uv)
                        + D2T.Duv * (2 * (A0v * B1uv + A1v * B0uv) + A0u * B1vv + A1u * B0vv)
                        + D2T.Dvv * (2 * A1v * B1uv + A1u * B1vv);
        Svvv += D2T.Duu * (3 * A0v * B0vv) + D2T.Duv * (3 * (A0v * B1vv + A1v * B0vv))
                        + D2T.Dvv * (3 * A1v * B1vv);

        Xyz duuu = Suuu - D3S.Duuu;
        Xyz duuv = Suuv - D3S.Duuv;
        Xyz duvv = Suvv - D3S.Duvv;
        Xyz dvvv = Svvv - D3S.Dvvv;
        duuu *= IncrementalLoad;
        duuv *= IncrementalLoad;
        duvv *= IncrementalLoad;
        dvvv *= IncrementalLoad;

        myLSC[0] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, duuu, 3, 0), normale);
        myLSC[1] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, duuv, 2, 1), normale);
        myLSC[2] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, duvv, 1, 2), normale);
        myLSC[3] = new Plate_LinearScalarConstraint(new Plate_PinpointConstraint(pnt2d, dvvv, 0, 3), normale);
        nb_LSConstraints = 4;
    }

    public int nb_PPC() => nb_PPConstraints;

    public Plate_PinpointConstraint GetPPC(int Index) => myPPC[Index];

    public int nb_LSC() => nb_LSConstraints;

    public Plate_LinearScalarConstraint LSC(int Index) => myLSC[Index];
}
