// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_QuasiAngularConvertor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_QuasiAngularConvertor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts an arc of circle, and its derivatives, into the poles and weights of a rational B-spline of degree 6
/// (one span) with the "quasi-angular" parametrisation.
/// </summary>
/// <remarks>
/// <para>
/// QuasiAngular is a rational definition of cos(theta(t)) and sin(theta(t)) on [-alpha, +alpha]:
/// cos = (U² - V²) / (U² + V²), sin = 2UV / (U² + V²), with V(t) = t + c t³, U(t) = 1 + b t², c = 1/3 + b,
/// b = -1/gamma² + gamma / (3 (tan gamma - gamma)) and gamma = alpha / 2.
/// </para>
/// <para>A working object; <see cref="Init"/> must be called before <c>Section</c>.</para>
/// </remarks>
public sealed class GeomFill_QuasiAngularConvertor
{
    private const double NullAngle = 1.0e-6;

    // B is the monomial-to-BSpline conversion matrix on [-1,1], degree 6.
    // It is a mathematical constant computed once.
    private static readonly math_Matrix THE_BASIS = ComputeBasis();

    private bool myinit;
    private readonly math_Matrix B;
    private math_Vector Px;
    private math_Vector Py;
    private readonly math_Vector W;
    private readonly math_Vector Vx;
    private readonly math_Vector Vy;
    private readonly math_Vector Vw;

    public GeomFill_QuasiAngularConvertor()
    {
        myinit = false;
        B = new math_Matrix(1, 7, 1, 7);
        Px = new math_Vector(1, 7);
        Py = new math_Vector(1, 7);
        W = new math_Vector(1, 7);
        Vx = new math_Vector(1, 7);
        Vy = new math_Vector(1, 7);
        Vw = new math_Vector(1, 7);
    }

    /// <summary>Whether <see cref="Init"/> has been called.</summary>
    public bool Initialized() => myinit;

    /// <summary>Computes the conversion matrix (once).</summary>
    public void Init()
    {
        if (myinit)
        {
            return;
        }

        B.Initialized(THE_BASIS);

        Vx.Init(0);
        Vx[1] = 1;
        Vy.Init(0);
        Vy[2] = 2;
        Vw.Init(0);
        Vw[1] = 1;
        myinit = true;
    }

    private static math_Matrix ComputeBasis()
    {
        const int anOrdre = 7;
        NCollection_Array1<double> aCoeffs = new(1, anOrdre * anOrdre);
        NCollection_Array1<double> anInter = new(1, 2), aTrueInter = new(1, 2);
        anInter.SetValue(1, -1);
        anInter.SetValue(2, 1);
        aTrueInter.SetValue(1, -1);
        aTrueInter.SetValue(2, 1);
        aCoeffs.Init(0);
        for (int ii = 1; ii <= anOrdre; ii++)
        {
            aCoeffs.SetValue(ii + (ii - 1) * anOrdre, 1);
        }

        Convert_CompPolynomialToPoles aConverter = new Convert_CompPolynomialToPoles(anOrdre,
                                                                                    anOrdre - 1,
                                                                                    anOrdre - 1,
                                                                                    aCoeffs,
                                                                                    anInter,
                                                                                    aTrueInter);
        NCollection_Array2<double> aPoles = aConverter.Poles();
        math_Matrix aResult = new math_Matrix(1, anOrdre, 1, anOrdre);
        for (int jj = 1; jj <= anOrdre; jj++)
        {
            for (int ii = 1; ii <= anOrdre; ii++)
            {
                double aTerm = aPoles.Value(ii, jj);
                if (Math.Abs(aTerm - 1) < 1.0e-9)
                {
                    aTerm = 1;
                }
                if (Math.Abs(aTerm + 1) < 1.0e-9)
                {
                    aTerm = -1;
                }
                aResult[ii, jj] = aTerm;
            }
        }
        return aResult;
    }

    /// <summary>The poles and weights of the arc of Angle from FirstPnt about Center, around the axis Dir.</summary>
    public void Section(in gp_Pnt FirstPnt, in gp_Pnt Center, in gp_Vec Dir, double Angle,
                        NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> Weights)
    {
        double b, b2, c, c2, tan_b;
        int ii;
        double beta, beta2, beta3, beta4, beta5, beta6, wi;
        gp_XYZ aux;
        gp_Mat Rot = new gp_Mat();
        // Calcul de la transformation
        gp_Vec V1 = new gp_Vec(Center, FirstPnt), V2;
        Rot.SetRotation(Dir.XYZ(), Angle / 2);
        aux = V1.XYZ();
        aux *= Rot;
        V1.SetXYZ(aux);
        V2 = Dir ^ V1;

        gp_Mat M = new gp_Mat(V1.X(), V2.X(), 0, V1.Y(), V2.Y(), 0, V1.Z(), V2.Z(), 0);

        // Calcul des coeffs  -----------
        beta = Angle / 4;
        beta2 = beta * beta;
        beta3 = beta * beta2;
        beta4 = beta2 * beta2;
        beta5 = beta3 * beta2;
        beta6 = beta3 * beta3;

        if ((M_PI / 2 - beta) > NullAngle)
        {
            if (Math.Abs(beta) < NullAngle)
            {
                double cf = 2.0 / (3 * 5 * 7);
                b = -(0.2 + cf * beta2) / (1 + 0.2 * beta2);
                //     b = beta5 / cf;
            }
            else
            {
                tan_b = Math.Tan(beta);
                b = -1.0e0 / beta2;
                b += beta / (3 * (tan_b - beta));
            }
        }
        else
        {
            b = ((double)-1) / beta2;
        }
        c = ((double)1) / 3 + b;
        b2 = b * b;
        c2 = c * c;

        // X = U*U - V*V
        Vx[3] = beta2 * (2 * b - 1);
        Vx[5] = beta4 * (b2 - 2 * c);
        Vx[7] = -beta6 * c2;

        // Y = 2*U*V
        Vy[2] = 2 * beta;
        Vy[4] = beta3 * 2 * (c + b);
        Vy[6] = 2 * beta5 * b * c;

        // W = U*U + V*V
        Vw[3] = beta2 * (1 + 2 * b);
        Vw[5] = beta4 * (2 * c + b2);
        Vw[7] = beta6 * c2;

        // Calculs des poles
        Px.Multiply(B, Vx);
        Py.Multiply(B, Vy);
        W.Multiply(B, Vw);

        // Transfo
        gp_XYZ pnt = new gp_XYZ();
        for (ii = 1; ii <= 7; ii++)
        {
            wi = W[ii];
            pnt.SetCoord(Px[ii] / wi, Py[ii] / wi, 0);
            pnt *= M;
            pnt += Center.XYZ();
            Poles[ii].ChangeCoord() = pnt;
            Weights[ii] = wi;
        }
    }

    /// <summary>The poles and weights of the arc and their first derivatives.</summary>
    public void Section(in gp_Pnt FirstPnt, in gp_Vec DFirstPnt, in gp_Pnt Center, in gp_Vec DCenter, in gp_Vec Dir,
                        in gp_Vec DDir, double Angle, double DAngle, NCollection_Array1<gp_Pnt> Poles,
                        NCollection_Array1<gp_Vec> DPoles, NCollection_Array1<double> Weights,
                        NCollection_Array1<double> DWeights)
    {
        int Ordre = 7;
        math_Vector DVx = new(1, Ordre), DVy = new(1, Ordre), DVw = new(1, Ordre), DPx = new(1, Ordre),
                    DPy = new(1, Ordre), DW = new(1, Ordre);
        double b, b2, c, c2, tan_b;
        double bpr, dtan_b;
        int ii;
        double beta, beta2, beta3, beta4, beta5, beta6, betaprim;
        gp_Vec V1 = new gp_Vec(Center, FirstPnt), V1Prim = new gp_Vec(), V2;

        // Calcul des  transformations
        gp_XYZ aux;
        double Sina, Cosa;
        gp_Mat Rot = new gp_Mat(), RotPrim, D = new gp_Mat(), DPrim = new gp_Mat();
        // La rotation s'ecrit I +  sin(Ang) * D + (1. - cos(Ang)) * D*D
        // ou D est l'application x -> Dir ^ x
        Rot.SetRotation(Dir.XYZ(), Angle / 2);
        // La derive s'ecrit donc :
        // AngPrim * (sin(Ang)*D*D + cos(Ang)*D)
        // + sin(Ang)*DPrim  + (1. - cos(Ang)) *(DPrim*D + D*DPrim)
        Sina = Math.Sin(Angle / 2);
        Cosa = Math.Cos(Angle / 2);
        D.SetCross(Dir.XYZ());
        DPrim.SetCross(DDir.XYZ());

        RotPrim = (D.Powered(2)).Multiplied(Sina);
        RotPrim += D.Multiplied(Cosa);
        RotPrim *= DAngle / 2;
        RotPrim += DPrim.Multiplied(Sina);
        RotPrim += ((DPrim.Multiplied(D)).Added(D.Multiplied(DPrim))).Multiplied(1 - Cosa);

        aux = (DFirstPnt - DCenter).XYZ().Multiplied(Rot);
        aux += V1.XYZ().Multiplied(RotPrim);
        V1Prim.SetXYZ(aux);
        aux = V1.XYZ();
        aux.Multiply(Rot);
        V1.SetXYZ(aux);
        V2 = Dir ^ V1;
        gp_Mat M = new gp_Mat(V1.X(), V2.X(), 0, V1.Y(), V2.Y(), 0, V1.Z(), V2.Z(), 0);
        V2 = (DDir.Crossed(V1)).Added(Dir.Crossed(V1Prim));
        gp_Mat MPrim = new gp_Mat(V1Prim.X(), V2.X(), 0, V1Prim.Y(), V2.Y(), 0, V1Prim.Z(), V2.Z(), 0);

        // Calcul des constante  -----------
        beta = Angle / 4;
        betaprim = DAngle / 4;
        beta2 = beta * beta;
        beta3 = beta * beta2;
        beta4 = beta2 * beta2;
        beta5 = beta3 * beta2;
        beta6 = beta3 * beta3;

        if (Math.Abs(beta) < NullAngle)
        {
            // On calcul b par D.L
            double cf = 2.0 / (3 * 5 * 7);
            double Num, Denom;
            Num = 0.2 + cf * beta2;
            Denom = 1 + 0.2 * beta2;
            b = -Num / Denom;
            bpr = -2 * beta * betaprim * (cf * Denom - 0.2 * Num) / (Denom * Denom);
        }
        else
        {
            b = ((double)-1) / beta2;
            bpr = (2 * betaprim) / beta3;
            if ((M_PI / 2 - beta) > NullAngle)
            {
                tan_b = Math.Tan(beta);
                dtan_b = betaprim * (1 + tan_b * tan_b);
                b2 = tan_b - beta;
                b += beta / (3 * b2);
                bpr += (betaprim * tan_b - beta * dtan_b) / (3 * b2 * b2);
            }
        }

        c = ((double)1) / 3 + b;
        b2 = b * b;
        c2 = c * c;

        // X = U*U - V*V
        Vx[3] = beta2 * (2 * b - 1);
        Vx[5] = beta4 * (b2 - 2 * c);
        Vx[7] = -beta6 * c2;
        DVx.Init(0);
        DVx[3] = 2 * (beta * betaprim * (2 * b - 1) + bpr * beta2);
        DVx[5] = 4 * beta3 * betaprim * (b2 - 2 * c) + 2 * beta4 * bpr * (b - 1);
        DVx[7] = -6 * beta5 * betaprim * c2 - 2 * beta6 * bpr * c;

        // Y = 2*U*V
        Vy[2] = 2 * beta;
        Vy[4] = beta3 * 2 * (c + b);
        Vy[6] = 2 * beta5 * b * c;
        DVy.Init(0);
        DVy[2] = 2 * betaprim;
        DVy[4] = 6 * beta2 * betaprim * (b + c) + 4 * beta3 * bpr;
        DVy[6] = 10 * beta4 * betaprim * b * c + 2 * beta5 * bpr * (b + c);

        // W = U*U + V*V
        Vw[3] = beta2 * (1 + 2 * b);
        Vw[5] = beta4 * (2 * c + b2);
        Vw[7] = beta6 * c2;
        DVw.Init(0);
        //  DVw(3) = 2*(beta*betaprim*(1 + 2*b) + beta2*bpr);
        DVw[3] = 2 * beta * (betaprim * (1 + 2 * b) + beta * bpr);
        //  DVw(5) = 4*beta3*betaprim*(2*c + b2) + 2*beta4*bpr*(b+1);
        DVw[5] = 2 * beta3 * (2 * betaprim * (2 * c + b2) + beta * bpr * (b + 1));
        //  DVw(7) = 6*beta5*betaprim*c2 + 2*beta6*bpr*c;
        DVw[7] = 2 * beta5 * c * (3 * betaprim * c + beta * bpr);

        // Calcul des poles
        Px.Multiply(B, Vx);
        Py.Multiply(B, Vy);
        W.Multiply(B, Vw);
        DPx.Multiply(B, DVx);
        DPy.Multiply(B, DVy);
        DW.Multiply(B, DVw);

        gp_XYZ P = new gp_XYZ(), DP = new gp_XYZ();
        double wi;

        for (ii = 1; ii <= Ordre; ii++)
        {
            wi = W[ii];
            P.SetCoord(Px[ii] / wi, Py[ii] / wi, 0);
            DP.SetCoord(DPx[ii] / wi, DPy[ii] / wi, 0);
            DP -= (DW[ii] / wi) * P;

            Poles[ii].ChangeCoord() = M * P + Center.XYZ();
            P *= MPrim;
            DP *= M;
            aux.SetLinearForm(1, P, 1, DP, DCenter.XYZ());
            DPoles[ii].SetXYZ(aux);
            Weights[ii] = wi;
            DWeights[ii] = DW[ii];
        }
    }

    /// <summary>The poles and weights of the arc and their first and second derivatives.</summary>
    public void Section(in gp_Pnt FirstPnt, in gp_Vec DFirstPnt, in gp_Vec D2FirstPnt, in gp_Pnt Center,
                        in gp_Vec DCenter, in gp_Vec D2Center, in gp_Vec Dir, in gp_Vec DDir, in gp_Vec D2Dir,
                        double Angle, double DAngle, double D2Angle, NCollection_Array1<gp_Pnt> Poles,
                        NCollection_Array1<gp_Vec> DPoles, NCollection_Array1<gp_Vec> D2Poles,
                        NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights,
                        NCollection_Array1<double> D2Weights)
    {
        int Ordre = 7;
        math_Vector DVx = new(1, Ordre), DVy = new(1, Ordre), DVw = new(1, Ordre), D2Vx = new(1, Ordre),
                    D2Vy = new(1, Ordre), D2Vw = new(1, Ordre);
        math_Vector DPx, DPy, DW = new(1, Ordre), D2Px, D2Py, D2W = new(1, Ordre);

        int ii;
        double daux, b, b2, c, c2, bpr, bsc;
        gp_Vec V1 = new gp_Vec(Center, FirstPnt), V1Prim, V1Secn = new gp_Vec(), V2;

        // Calcul des  transformations
        gp_XYZ auxyz;
        double Sina, Cosa;
        gp_Mat Rot = new gp_Mat(), RotPrim, RotSecn, D = new gp_Mat(), DPrim = new gp_Mat(), DSecn = new gp_Mat(),
               DDP, Maux;
        // La rotation s'ecrit I +  sin(Ang) * D + (1. - cos(Ang)) * D*D
        // ou D est l'application x -> Dir ^ x
        Rot.SetRotation(Dir.XYZ(), Angle / 2);
        // La derive s'ecrit donc :
        // AngPrim * (sin(Ang)*D*D + cos(Ang)*D)
        // + sin(Ang)*DPrim  + (1. - cos(Ang)) *(DPrim*D + D*DPrim)
        Sina = Math.Sin(Angle / 2);
        Cosa = Math.Cos(Angle / 2);
        D.SetCross(Dir.XYZ());
        DPrim.SetCross(DDir.XYZ());
        DSecn.SetCross(D2Dir.XYZ());

        DDP = (DPrim.Multiplied(D)).Added(D.Multiplied(DPrim));
        RotPrim = (D.Powered(2)).Multiplied(Sina);
        RotPrim += D.Multiplied(Cosa);
        RotPrim *= DAngle / 2;
        RotPrim += DPrim.Multiplied(Sina);
        RotPrim += DDP.Multiplied(1 - Cosa);

        RotSecn = (D.Powered(2)).Multiplied(Sina);
        RotSecn += D.Multiplied(Cosa);
        RotSecn *= D2Angle / 2;
        Maux = (D.Powered(2)).Multiplied(Cosa);
        Maux -= D.Multiplied(Sina);
        Maux *= DAngle / 2;
        Maux += DDP.Multiplied(2 * Sina);
        Maux += DPrim.Multiplied(2 * Cosa);
        Maux *= DAngle / 2;
        RotSecn += Maux;
        Maux = (DSecn.Multiplied(D)).Added(D.Multiplied(DSecn));
        Maux += (DPrim.Powered(2)).Multiplied(2);
        Maux *= 1 - Cosa;
        Maux += DSecn.Multiplied(Sina);
        RotSecn += Maux;

        V1Prim = DFirstPnt - DCenter;
        auxyz = (D2FirstPnt - D2Center).XYZ().Multiplied(Rot);
        auxyz += 2 * (V1Prim.XYZ().Multiplied(RotPrim));
        auxyz += V1.XYZ().Multiplied(RotSecn);
        V1Secn.SetXYZ(auxyz);
        auxyz = V1Prim.XYZ().Multiplied(Rot);
        auxyz += V1.XYZ().Multiplied(RotPrim);
        V1Prim.SetXYZ(auxyz);
        auxyz = V1.XYZ();
        auxyz.Multiply(Rot);
        V1.SetXYZ(auxyz);
        V2 = Dir ^ V1;

        gp_Mat M = new gp_Mat(V1.X(), V2.X(), 0, V1.Y(), V2.Y(), 0, V1.Z(), V2.Z(), 0);
        V2 = (DDir.Crossed(V1)).Added(Dir.Crossed(V1Prim));
        gp_Mat MPrim = new gp_Mat(V1Prim.X(), V2.X(), 0, V1Prim.Y(), V2.Y(), 0, V1Prim.Z(), V2.Z(), 0);

        V2 = DDir ^ V1Prim;
        V2 *= 2;
        V2 += (D2Dir.Crossed(V1)).Added(Dir.Crossed(V1Secn));
        gp_Mat MSecn = new gp_Mat(V1Secn.X(), V2.X(), 0, V1Secn.Y(), V2.Y(), 0, V1Secn.Z(), V2.Z(), 0);

        // Calcul des coeff  -----------
        double tan_b, dtan_b, d2tan_b;
        double beta, beta2, beta3, beta4, beta5, beta6, betaprim, betasecn;
        double betaprim2, bpr2;
        beta = Angle / 4;
        betaprim = DAngle / 4;
        betasecn = D2Angle / 4;
        beta2 = beta * beta;
        beta3 = beta * beta2;
        beta4 = beta2 * beta2;
        beta5 = beta3 * beta2;
        beta6 = beta3 * beta3;
        betaprim2 = betaprim * betaprim;

        if (Math.Abs(beta) < NullAngle)
        {
            // On calcul b par D.L
            double cf = -2.0 / 21;
            double Num, Denom, aux;
            Num = 0.2 + cf * beta2;
            Denom = 1 + 0.2 * beta2;
            aux = (cf * Denom - 0.2 * Num) / (Denom * Denom);
            b = -Num / Denom;
            bpr = -2 * beta * betaprim * aux;
            bsc = 2 * aux * (betaprim2 + beta * betasecn - 2 * beta * betaprim2);
        }
        else
        {
            b = ((double)-1) / beta2;
            bpr = (2 * betaprim) / beta3;
            bsc = (2 * betasecn - 6 * betaprim * (betaprim / beta)) / beta3;
            if ((M_PI / 2 - beta) > NullAngle)
            {
                tan_b = Math.Tan(beta);
                dtan_b = betaprim * (1 + tan_b * tan_b);
                d2tan_b = betasecn * (1 + tan_b * tan_b) + 2 * betaprim * tan_b * dtan_b;
                b2 = tan_b - beta;
                b += beta / (3 * b2);
                double aux = betaprim * tan_b - beta * dtan_b;
                bpr += aux / (3 * b2 * b2);
                daux = betasecn * tan_b - beta * d2tan_b;
                bsc += (daux - 2 * aux * betaprim * tan_b * tan_b / b2) / (3 * b2 * b2);
            }
        }

        c = ((double)1) / 3 + b;
        b2 = b * b;
        c2 = c * c;
        bpr2 = bpr * bpr;

        // X = U*U - V*V
        Vx[3] = beta2 * (2 * b - 1);
        Vx[5] = beta4 * (b2 - 2 * c);
        Vx[7] = -beta6 * c2;
        DVx.Init(0);
        DVx[3] = 2 * (beta * betaprim * (2 * b - 1) + bpr * beta2);
        DVx[5] = 4 * beta3 * betaprim * (b2 - 2 * c) + 2 * beta4 * bpr * (b - 1);
        DVx[7] = -6 * beta5 * betaprim * c2 - 2 * beta6 * bpr * c;
        D2Vx.Init(0);
        D2Vx[3] = 2 * ((betaprim2 + beta * betasecn) * (2 * b - 1) + 8 * beta * betaprim * bpr + bsc * beta2);
        D2Vx[5] = 4 * (b2 - 2 * c) * (3 * beta2 * betaprim2 + beta3 * betasecn)
                  + 16 * beta3 * betaprim * bpr * (b - 1) + 2 * beta4 * (bsc * (b - 1) + bpr2);
        D2Vx[7] = -6 * c2 * (5 * beta4 * betaprim2 + beta5 * betasecn) - 24 * beta5 * betaprim * bpr * c
                  - 2 * beta6 * (bsc * c + bpr2);

        // Y = 2*U*V
        Vy[2] = 2 * beta;
        Vy[4] = beta3 * 2 * (c + b);
        Vy[6] = 2 * beta5 * b * c;
        DVy.Init(0);
        DVy[2] = 2 * betaprim;
        DVy[4] = 6 * beta2 * betaprim * (b + c) + 4 * beta3 * bpr;
        DVy[6] = 10 * beta4 * betaprim * b * c + 2 * beta5 * bpr * (b + c);
        D2Vy.Init(0);
        D2Vy[2] = 2 * betasecn;
        D2Vy[4] = 6 * (b + c) * (2 * beta * betaprim2 + beta2 * betasecn)
                  + 24 * beta2 * betaprim * bpr * (b + c) + 4 * beta3 * bsc;
        D2Vy[6] = 10 * b * c * (4 * beta3 * betaprim2 + beta4 * betasecn)
                  + 40 * beta4 * betaprim * bpr * (b + c) + 2 * beta5 * (bsc * (b + c) + 2 * bpr2);

        // W = U*U + V*V
        Vw[3] = beta2 * (1 + 2 * b);
        Vw[5] = beta4 * (2 * c + b2);
        Vw[7] = beta6 * c2;
        DVw.Init(0);
        DVw[3] = 2 * (beta * betaprim * (1 + 2 * b) + beta2 * bpr);
        DVw[5] = 4 * beta3 * betaprim * (2 * c + b2) + 2 * beta4 * bpr * (b + 1);
        DVw[7] = 6 * beta5 * betaprim * c2 + 2 * beta6 * bpr * c;
        D2Vw.Init(0);
        D2Vw[3] = 2 * ((betaprim2 + beta * betasecn) * (2 * b + 1) + 8 * beta * betaprim * bpr + bsc * beta2);
        // OCCT: "(b + 11)" where the derivative of beta4 * (2c + b²) needs (b + 1) (occt-issues.md).
        D2Vw[5] = 4 * (b2 + 2 * c) * (3 * beta2 * betaprim2 + beta3 * betasecn)
                  + 16 * beta3 * betaprim * bpr * (b + 11) + 2 * beta4 * (bsc * (b + 1) + bpr2);
        D2Vw[7] = 6 * c2 * (5 * beta4 * betaprim2 + beta5 * betasecn) + 24 * beta5 * betaprim * bpr * c
                  + 2 * beta6 * (bsc * c + bpr2);

        // Calcul des poles
        Px = B * Vx;
        Py = B * Vy;
        W.Multiply(B, Vw);
        DPx = B * DVx;
        DPy = B * DVy;
        DW.Multiply(B, DVw);
        D2Px = B * D2Vx;
        D2Py = B * D2Vy;
        D2W.Multiply(B, D2Vw);

        gp_XYZ P = new gp_XYZ(), DP = new gp_XYZ(), D2P = new gp_XYZ();
        double wi, dwi;
        for (ii = 1; ii <= Ordre; ii++)
        {
            wi = W[ii];
            dwi = DW[ii];
            P.SetCoord(Px[ii] / wi, Py[ii] / wi, 0);
            DP.SetCoord(DPx[ii] / wi, DPy[ii] / wi, 0);

            D2P.SetCoord(D2Px[ii] / wi, D2Py[ii] / wi, 0);
            D2P -= 2 * (dwi / wi) * DP;
            D2P += (2 * Math.Pow(dwi / wi, 2) - D2W[ii] / wi) * P;
            DP -= (DW[ii] / wi) * P;

            Poles[ii].ChangeCoord() = M * P + Center.XYZ();
            auxyz.SetLinearForm(1, MPrim * P, 1, M * DP, DCenter.XYZ());
            DPoles[ii].SetXYZ(auxyz);
            P *= MSecn;
            DP *= MPrim;
            D2P *= M;
            auxyz.SetLinearForm(1, P, 2, DP, 1, D2P, D2Center.XYZ());
            D2Poles[ii].SetXYZ(auxyz);
            Weights[ii] = wi;
            DWeights[ii] = dwi;
            D2Weights[ii] = D2W[ii];
        }
    }
}
