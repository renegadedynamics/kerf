// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GtoCConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GtoCConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GtoCConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class Plate_GtoCConstraint
{
    private readonly Plate_PinpointConstraint[] myPPC = new Plate_PinpointConstraint[9];
    private readonly Plate_D1 myD1SurfInit;
    private readonly gp_XY pnt2d;
    private readonly int nb_PPConstraints;

    private const double NORMIN = 1.0e-10;
    private const double COSMIN = 1.0e-2;

    public Plate_GtoCConstraint(Plate_GtoCConstraint @ref)
    {
        myD1SurfInit = @ref.myD1SurfInit;
        pnt2d            = @ref.pnt2d;
        nb_PPConstraints = @ref.nb_PPConstraints;
        for (int i = 0; i < nb_PPConstraints; i++)
        {
            myPPC[i] = @ref.myPPC[i];
        }
    }

    public Plate_GtoCConstraint(in gp_XY point2d, in Plate_D1 D1S, in Plate_D1 D1T)
    {
        myD1SurfInit = D1S;
        pnt2d            = point2d;
        nb_PPConstraints = 0;

        gp_XYZ normale = D1T.Du ^ D1T.Dv;
        // alr le 12/11/96
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints

        gp_XYZ normaleS = D1S.Du ^ D1S.Dv;
        // alr le 12/11/96
        if (normaleS.Modulus() < NORMIN)
        {
            return;
        }
        normaleS.Normalize();
        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < COSMIN)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        gp_XYZ du = normaleS * -(normale * D1S.Du) * invcos;
        gp_XYZ dv = normaleS * -(normale * D1S.Dv) * invcos;

        myPPC[0] = new Plate_PinpointConstraint(pnt2d, du, 1, 0);
        myPPC[1] = new Plate_PinpointConstraint(pnt2d, dv, 0, 1);

        nb_PPConstraints = 2;
    }

    public Plate_GtoCConstraint(in gp_XY point2d, in Plate_D1 D1S, in Plate_D1 D1T, in gp_XYZ nP)
    {
        myD1SurfInit = D1S;
        pnt2d            = point2d;
        nb_PPConstraints = 0;

        gp_XYZ normale = D1T.Du ^ D1T.Dv;
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints

        gp_XYZ normaleS = D1S.Du ^ D1S.Dv;
        if (normaleS.Modulus() < NORMIN)
        {
            return;
        }
        normaleS.Normalize();

        gp_XYZ nSP = normaleS - nP * (nP * normaleS);
        if (nSP.Modulus() < NORMIN)
        {
            return;
        }
        nSP.Normalize();

        double cos_normales = normale * nSP;
        if (Math.Abs(cos_normales) < COSMIN)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        gp_XYZ du = nSP * -(normale * D1S.Du) * invcos;
        gp_XYZ dv = nSP * -(normale * D1S.Dv) * invcos;

        myPPC[0] = new Plate_PinpointConstraint(pnt2d, du, 1, 0);
        myPPC[1] = new Plate_PinpointConstraint(pnt2d, dv, 0, 1);

        nb_PPConstraints = 2;
    }

    public Plate_GtoCConstraint(in gp_XY point2d, in Plate_D1 D1S, in Plate_D1 D1T, in Plate_D2 D2S, in Plate_D2 D2T)
    {
        myD1SurfInit = D1S;

        pnt2d            = point2d;
        nb_PPConstraints = 0;

        gp_XYZ normale = D1T.Du ^ D1T.Dv;
        // alr le 12/11/96
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        gp_XYZ normaleS = D1S.Du ^ D1S.Dv;
        // alr le 12/11/96
        if (normaleS.Modulus() < NORMIN)
        {
            return;
        }
        normaleS.Normalize();

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < COSMIN)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        gp_XYZ du = normaleS * -(normale * D1S.Du) * invcos;
        gp_XYZ dv = normaleS * -(normale * D1S.Dv) * invcos;

        myPPC[0] = new Plate_PinpointConstraint(pnt2d, du, 1, 0);
        myPPC[1] = new Plate_PinpointConstraint(pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        gp_XYZ Su = D1S.Du + du;
        gp_XYZ Sv = D1S.Dv + dv;

        math_Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = Su * D1T.Du;
        mat[0, 1] = Su * D1T.Dv;
        mat[1, 0] = Sv * D1T.Du;
        mat[1, 1] = Sv * D1T.Dv;
        math_Gauss gauss = new(mat);
        if (!gauss.IsDone())
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

        gp_XYZ Suu = D2T.Duu * (a * a) + D2T.Duv * (2 * a * b) + D2T.Dvv * (b * b);
        gp_XYZ Suv = D2T.Duu * (a * c) + D2T.Duv * (a * d + b * c) + D2T.Dvv * (b * d);
        gp_XYZ Svv = D2T.Duu * (c * c) + D2T.Duv * (2 * c * d) + D2T.Dvv * (d * d);

        gp_XYZ duu = normaleS * (normale * (Suu - D2S.Duu)) * invcos;
        gp_XYZ duv = normaleS * (normale * (Suv - D2S.Duv)) * invcos;
        gp_XYZ dvv = normaleS * (normale * (Svv - D2S.Dvv)) * invcos;

        myPPC[2] = new Plate_PinpointConstraint(pnt2d, duu, 2, 0);
        myPPC[3] = new Plate_PinpointConstraint(pnt2d, duv, 1, 1);
        myPPC[4] = new Plate_PinpointConstraint(pnt2d, dvv, 0, 2);
        nb_PPConstraints = 5;
    }

    public Plate_GtoCConstraint(in gp_XY point2d, in Plate_D1 D1S, in Plate_D1 D1T, in Plate_D2 D2S, in Plate_D2 D2T, in gp_XYZ nP)
    {
        myD1SurfInit = D1S;

        pnt2d            = point2d;
        nb_PPConstraints = 0;

        gp_XYZ normale = D1T.Du ^ D1T.Dv;
        // alr le 12/11/96
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        gp_XYZ normaleS = D1S.Du ^ D1S.Dv;
        // alr le 12/11/96
        if (normaleS.Modulus() < NORMIN)
        {
            return;
        }
        normaleS.Normalize();

        gp_XYZ nSP = normaleS - nP * (nP * normaleS);
        if (nSP.Modulus() < NORMIN)
        {
            return;
        }
        nSP.Normalize();

        double cos_normales = normale * nSP;
        if (Math.Abs(cos_normales) < COSMIN)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        gp_XYZ du = nSP * -(normale * D1S.Du) * invcos;
        gp_XYZ dv = nSP * -(normale * D1S.Dv) * invcos;

        myPPC[0] = new Plate_PinpointConstraint(pnt2d, du, 1, 0);
        myPPC[1] = new Plate_PinpointConstraint(pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        gp_XYZ Su = D1S.Du + du;
        gp_XYZ Sv = D1S.Dv + dv;

        math_Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = Su * D1T.Du;
        mat[0, 1] = Su * D1T.Dv;
        mat[1, 0] = Sv * D1T.Du;
        mat[1, 1] = Sv * D1T.Dv;
        math_Gauss gauss = new(mat);
        if (!gauss.IsDone())
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

        gp_XYZ Suu = D2T.Duu * (a * a) + D2T.Duv * (2 * a * b) + D2T.Dvv * (b * b);
        gp_XYZ Suv = D2T.Duu * (a * c) + D2T.Duv * (a * d + b * c) + D2T.Dvv * (b * d);
        gp_XYZ Svv = D2T.Duu * (c * c) + D2T.Duv * (2 * c * d) + D2T.Dvv * (d * d);

        gp_XYZ duu = nSP * (normale * (Suu - D2S.Duu)) * invcos;
        gp_XYZ duv = nSP * (normale * (Suv - D2S.Duv)) * invcos;
        gp_XYZ dvv = nSP * (normale * (Svv - D2S.Dvv)) * invcos;

        myPPC[2] = new Plate_PinpointConstraint(pnt2d, duu, 2, 0);
        myPPC[3] = new Plate_PinpointConstraint(pnt2d, duv, 1, 1);
        myPPC[4] = new Plate_PinpointConstraint(pnt2d, dvv, 0, 2);
        nb_PPConstraints = 5;
    }

    public Plate_GtoCConstraint(in gp_XY point2d, in Plate_D1 D1S, in Plate_D1 D1T, in Plate_D2 D2S, in Plate_D2 D2T, in Plate_D3 D3S, in Plate_D3 D3T)
    {
        myD1SurfInit = D1S;
        pnt2d            = point2d;
        nb_PPConstraints = 0;

        gp_XYZ normale = D1T.Du ^ D1T.Dv;
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        gp_XYZ normaleS = D1S.Du ^ D1S.Dv;
        if (normaleS.Modulus() < NORMIN)
        {
            return;
        }
        normaleS.Normalize();

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < COSMIN)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        gp_XYZ du = normaleS * -(normale * D1S.Du) * invcos;
        gp_XYZ dv = normaleS * -(normale * D1S.Dv) * invcos;

        myPPC[0] = new Plate_PinpointConstraint(pnt2d, du, 1, 0);
        myPPC[1] = new Plate_PinpointConstraint(pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        gp_XYZ Su = D1S.Du + du;
        gp_XYZ Sv = D1S.Dv + dv;

        math_Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = Su * D1T.Du;
        mat[0, 1] = Su * D1T.Dv;
        mat[1, 0] = Sv * D1T.Du;
        mat[1, 1] = Sv * D1T.Dv;
        math_Gauss gauss = new(mat);
        if (!gauss.IsDone())
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

        gp_XYZ Suu = D2T.Duu * (a * a) + D2T.Duv * (2 * a * b) + D2T.Dvv * (b * b);
        gp_XYZ Suv = D2T.Duu * (a * c) + D2T.Duv * (a * d + b * c) + D2T.Dvv * (b * d);
        gp_XYZ Svv = D2T.Duu * (c * c) + D2T.Duv * (2 * c * d) + D2T.Dvv * (d * d);

        gp_XYZ duu = normaleS * (normale * (Suu - D2S.Duu)) * invcos;
        gp_XYZ duv = normaleS * (normale * (Suv - D2S.Duv)) * invcos;
        gp_XYZ dvv = normaleS * (normale * (Svv - D2S.Dvv)) * invcos;

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

        gp_XYZ Suuu = D3T.Duuu * (a * a * a) + D3T.Duuv * (3 * a * a * b) + D3T.Duvv * (3 * a * b * b)
                                    + D3T.Dvvv * (b * b * b);
        gp_XYZ Suuv = D3T.Duuu * (a * a * c) + D3T.Duuv * (a * a * d + 2 * a * b * c)
                                    + D3T.Duvv * (b * b * c + 2 * a * b * d) + D3T.Dvvv * (b * b * d);
        gp_XYZ Suvv = D3T.Duuu * (a * c * c) + D3T.Duuv * (b * c * c + 2 * a * c * d)
                                    + D3T.Duvv * (a * d * d + 2 * b * c * d) + D3T.Dvvv * (b * d * d);
        gp_XYZ Svvv = D3T.Duuu * (c * c * c) + D3T.Duuv * (3 * c * c * d) + D3T.Duvv * (3 * c * d * d)
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

        gp_XYZ duuu = normaleS * (normale * (Suuu - D3S.Duuu)) * invcos;
        gp_XYZ duuv = normaleS * (normale * (Suuv - D3S.Duuv)) * invcos;
        gp_XYZ duvv = normaleS * (normale * (Suvv - D3S.Duvv)) * invcos;
        gp_XYZ dvvv = normaleS * (normale * (Svvv - D3S.Dvvv)) * invcos;

        myPPC[5] = new Plate_PinpointConstraint(pnt2d, duuu, 3, 0);
        myPPC[6] = new Plate_PinpointConstraint(pnt2d, duuv, 2, 1);
        myPPC[7] = new Plate_PinpointConstraint(pnt2d, duvv, 1, 2);
        myPPC[8] = new Plate_PinpointConstraint(pnt2d, dvvv, 0, 3);
        nb_PPConstraints = 9;
    }

    public Plate_GtoCConstraint(in gp_XY point2d, in Plate_D1 D1S, in Plate_D1 D1T, in Plate_D2 D2S, in Plate_D2 D2T, in Plate_D3 D3S, in Plate_D3 D3T, in gp_XYZ nP)
    {
        _ = nP; // OCCT: this variant ignores nP (its name is commented out in OCCT's signature)
        myD1SurfInit = D1S;

        pnt2d            = point2d;
        nb_PPConstraints = 0;

        gp_XYZ normale = D1T.Du ^ D1T.Dv;
        if (normale.Modulus() < NORMIN)
        {
            return;
        }
        normale.Normalize();

        // G1 Constraints
        gp_XYZ normaleS = D1S.Du ^ D1S.Dv;
        if (normaleS.Modulus() < NORMIN)
        {
            return;
        }
        normaleS.Normalize();

        double cos_normales = normale * normaleS;
        if (Math.Abs(cos_normales) < COSMIN)
        {
            return;
        }
        double invcos = 1.0 / cos_normales;

        gp_XYZ du = normaleS * -(normale * D1S.Du) * invcos;
        gp_XYZ dv = normaleS * -(normale * D1S.Dv) * invcos;

        myPPC[0] = new Plate_PinpointConstraint(pnt2d, du, 1, 0);
        myPPC[1] = new Plate_PinpointConstraint(pnt2d, dv, 0, 1);
        nb_PPConstraints = 2;

        // G2 Constraints
        gp_XYZ Su = D1S.Du + du;
        gp_XYZ Sv = D1S.Dv + dv;

        math_Matrix mat = new(0, 1, 0, 1);
        mat[0, 0] = Su * D1T.Du;
        mat[0, 1] = Su * D1T.Dv;
        mat[1, 0] = Sv * D1T.Du;
        mat[1, 1] = Sv * D1T.Dv;
        math_Gauss gauss = new(mat);

        if (!gauss.IsDone())
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

        gp_XYZ Suu = D2T.Duu * (a * a) + D2T.Duv * (2 * a * b) + D2T.Dvv * (b * b);
        gp_XYZ Suv = D2T.Duu * (a * c) + D2T.Duv * (a * d + b * c) + D2T.Dvv * (b * d);
        gp_XYZ Svv = D2T.Duu * (c * c) + D2T.Duv * (2 * c * d) + D2T.Dvv * (d * d);

        gp_XYZ duu = normaleS * (normale * (Suu - D2S.Duu)) * invcos;
        gp_XYZ duv = normaleS * (normale * (Suv - D2S.Duv)) * invcos;
        gp_XYZ dvv = normaleS * (normale * (Svv - D2S.Dvv)) * invcos;

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

        gp_XYZ Suuu = D3T.Duuu * (a * a * a) + D3T.Duuv * (3 * a * a * b) + D3T.Duvv * (3 * a * b * b)
                                    + D3T.Dvvv * (b * b * b);
        gp_XYZ Suuv = D3T.Duuu * (a * a * c) + D3T.Duuv * (a * a * d + 2 * a * b * c)
                                    + D3T.Duvv * (b * b * c + 2 * a * b * d) + D3T.Dvvv * (b * b * d);
        gp_XYZ Suvv = D3T.Duuu * (a * c * c) + D3T.Duuv * (b * c * c + 2 * a * c * d)
                                    + D3T.Duvv * (a * d * d + 2 * b * c * d) + D3T.Dvvv * (b * d * d);
        gp_XYZ Svvv = D3T.Duuu * (c * c * c) + D3T.Duuv * (3 * c * c * d) + D3T.Duvv * (3 * c * d * d)
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

        gp_XYZ duuu = normaleS * (normale * (Suuu - D3S.Duuu)) * invcos;
        gp_XYZ duuv = normaleS * (normale * (Suuv - D3S.Duuv)) * invcos;
        gp_XYZ duvv = normaleS * (normale * (Suvv - D3S.Duvv)) * invcos;
        gp_XYZ dvvv = normaleS * (normale * (Svvv - D3S.Dvvv)) * invcos;

        myPPC[5] = new Plate_PinpointConstraint(pnt2d, duuu, 3, 0);
        myPPC[6] = new Plate_PinpointConstraint(pnt2d, duuv, 2, 1);
        myPPC[7] = new Plate_PinpointConstraint(pnt2d, duvv, 1, 2);
        myPPC[8] = new Plate_PinpointConstraint(pnt2d, dvvv, 0, 3);
        nb_PPConstraints = 9;
    }

    public int nb_PPC() => nb_PPConstraints;

    public Plate_PinpointConstraint GetPPC(int Index) => myPPC[Index];

    public Plate_D1 D1SurfInit() => myD1SurfInit;
}
