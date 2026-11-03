// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_QuasiAngularConvertor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_QuasiAngularConvertor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Conversion;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

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
public sealed class QuasiAngularConvertor
{
    private const double NullAngle = 1.0e-6;

    // B is the monomial-to-BSpline conversion matrix on [-1,1], degree 6.
    // It is a mathematical constant computed once.
    private static readonly Matrix THE_BASIS = ComputeBasis();

    private bool _myinit;
    private readonly Matrix _b;
    private math_Vector _px;
    private math_Vector _py;
    private readonly math_Vector _w;
    private readonly math_Vector _vx;
    private readonly math_Vector _vy;
    private readonly math_Vector _vw;

    public QuasiAngularConvertor()
    {
        _myinit = false;
        _b = new Matrix(1, 7, 1, 7);
        _px = new math_Vector(1, 7);
        _py = new math_Vector(1, 7);
        _w = new math_Vector(1, 7);
        _vx = new math_Vector(1, 7);
        _vy = new math_Vector(1, 7);
        _vw = new math_Vector(1, 7);
    }

    /// <summary>Whether <see cref="Init"/> has been called.</summary>
    public bool Initialized() => _myinit;

    /// <summary>Computes the conversion matrix (once).</summary>
    public void Init()
    {
        if (_myinit)
        {
            return;
        }

        _b.Initialized(THE_BASIS);

        _vx.Init(0);
        _vx[1] = 1;
        _vy.Init(0);
        _vy[2] = 2;
        _vw.Init(0);
        _vw[1] = 1;
        _myinit = true;
    }

    private static Matrix ComputeBasis()
    {
        const int ordre = 7;
        NCollection_Array1<double> coeffs = new(1, ordre * ordre);
        NCollection_Array1<double> inter = new(1, 2), trueInter = new(1, 2);
        inter.SetValue(1, -1);
        inter.SetValue(2, 1);
        trueInter.SetValue(1, -1);
        trueInter.SetValue(2, 1);
        coeffs.Init(0);
        for (int ii = 1; ii <= ordre; ii++)
        {
            coeffs.SetValue(ii + (ii - 1) * ordre, 1);
        }

        CompPolynomialToPoles converter = new CompPolynomialToPoles(ordre,
                                                                                    ordre - 1,
                                                                                    ordre - 1,
                                                                                    coeffs,
                                                                                    inter,
                                                                                    trueInter);
        NCollection_Array2<double> poles = converter.Poles();
        Matrix result = new Matrix(1, ordre, 1, ordre);
        for (int jj = 1; jj <= ordre; jj++)
        {
            for (int ii = 1; ii <= ordre; ii++)
            {
                double term = poles.Value(ii, jj);
                if (Math.Abs(term - 1) < 1.0e-9)
                {
                    term = 1;
                }
                if (Math.Abs(term + 1) < 1.0e-9)
                {
                    term = -1;
                }
                result[ii, jj] = term;
            }
        }
        return result;
    }

    /// <summary>The poles and weights of the arc of Angle from FirstPnt about Center, around the axis Dir.</summary>
    public void Section(in Point3D firstPnt, in Point3D center, in Vector3D dir, double angle,
                        NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights)
    {
        double b, b2, c, c2, tan_b;
        int ii;
        double beta, beta2, beta3, beta4, beta5, beta6, wi;
        Xyz aux;
        Matrix3x3 rot = new Matrix3x3();
        // Calcul de la transformation
        Vector3D v1 = new Vector3D(center, firstPnt), v2;
        rot.SetRotation(dir.XYZ(), angle / 2);
        aux = v1.XYZ();
        aux *= rot;
        v1.SetXYZ(aux);
        v2 = dir ^ v1;

        Matrix3x3 m = new Matrix3x3(v1.X(), v2.X(), 0, v1.Y(), v2.Y(), 0, v1.Z(), v2.Z(), 0);

        // Calcul des coeffs  -----------
        beta = angle / 4;
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
        _vx[3] = beta2 * (2 * b - 1);
        _vx[5] = beta4 * (b2 - 2 * c);
        _vx[7] = -beta6 * c2;

        // Y = 2*U*V
        _vy[2] = 2 * beta;
        _vy[4] = beta3 * 2 * (c + b);
        _vy[6] = 2 * beta5 * b * c;

        // W = U*U + V*V
        _vw[3] = beta2 * (1 + 2 * b);
        _vw[5] = beta4 * (2 * c + b2);
        _vw[7] = beta6 * c2;

        // Calculs des poles
        _px.Multiply(_b, _vx);
        _py.Multiply(_b, _vy);
        _w.Multiply(_b, _vw);

        // Transfo
        Xyz pnt = new Xyz();
        for (ii = 1; ii <= 7; ii++)
        {
            wi = _w[ii];
            pnt.SetCoord(_px[ii] / wi, _py[ii] / wi, 0);
            pnt *= m;
            pnt += center.XYZ();
            poles[ii].ChangeCoord() = pnt;
            weights[ii] = wi;
        }
    }

    /// <summary>The poles and weights of the arc and their first derivatives.</summary>
    public void Section(in Point3D firstPnt, in Vector3D dFirstPnt, in Point3D center, in Vector3D dCenter, in Vector3D dir,
                        in Vector3D dDir, double angle, double dAngle, NCollection_Array1<Point3D> poles,
                        NCollection_Array1<Vector3D> dPoles, NCollection_Array1<double> weights,
                        NCollection_Array1<double> dWeights)
    {
        int ordre = 7;
        math_Vector dVx = new(1, ordre), dVy = new(1, ordre), dVw = new(1, ordre), dPx = new(1, ordre),
                    dPy = new(1, ordre), dw = new(1, ordre);
        double b, b2, c, c2, tan_b;
        double bpr, dtan_b;
        int ii;
        double beta, beta2, beta3, beta4, beta5, beta6, betaprim;
        Vector3D v1 = new Vector3D(center, firstPnt), v1Prim = new Vector3D(), v2;

        // Calcul des  transformations
        Xyz aux;
        double sina, cosa;
        Matrix3x3 rot = new Matrix3x3(), rotPrim, d = new Matrix3x3(), dPrim = new Matrix3x3();
        // La rotation s'ecrit I +  sin(Ang) * D + (1. - cos(Ang)) * D*D
        // ou D est l'application x -> Dir ^ x
        rot.SetRotation(dir.XYZ(), angle / 2);
        // La derive s'ecrit donc :
        // AngPrim * (sin(Ang)*D*D + cos(Ang)*D)
        // + sin(Ang)*DPrim  + (1. - cos(Ang)) *(DPrim*D + D*DPrim)
        sina = Math.Sin(angle / 2);
        cosa = Math.Cos(angle / 2);
        d.SetCross(dir.XYZ());
        dPrim.SetCross(dDir.XYZ());

        rotPrim = (d.Powered(2)).Multiplied(sina);
        rotPrim += d.Multiplied(cosa);
        rotPrim *= dAngle / 2;
        rotPrim += dPrim.Multiplied(sina);
        rotPrim += ((dPrim.Multiplied(d)).Added(d.Multiplied(dPrim))).Multiplied(1 - cosa);

        aux = (dFirstPnt - dCenter).XYZ().Multiplied(rot);
        aux += v1.XYZ().Multiplied(rotPrim);
        v1Prim.SetXYZ(aux);
        aux = v1.XYZ();
        aux.Multiply(rot);
        v1.SetXYZ(aux);
        v2 = dir ^ v1;
        Matrix3x3 m = new Matrix3x3(v1.X(), v2.X(), 0, v1.Y(), v2.Y(), 0, v1.Z(), v2.Z(), 0);
        v2 = (dDir.Crossed(v1)).Added(dir.Crossed(v1Prim));
        Matrix3x3 mPrim = new Matrix3x3(v1Prim.X(), v2.X(), 0, v1Prim.Y(), v2.Y(), 0, v1Prim.Z(), v2.Z(), 0);

        // Calcul des constante  -----------
        beta = angle / 4;
        betaprim = dAngle / 4;
        beta2 = beta * beta;
        beta3 = beta * beta2;
        beta4 = beta2 * beta2;
        beta5 = beta3 * beta2;
        beta6 = beta3 * beta3;

        if (Math.Abs(beta) < NullAngle)
        {
            // On calcul b par D.L
            double cf = 2.0 / (3 * 5 * 7);
            double num, denom;
            num = 0.2 + cf * beta2;
            denom = 1 + 0.2 * beta2;
            b = -num / denom;
            bpr = -2 * beta * betaprim * (cf * denom - 0.2 * num) / (denom * denom);
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
        _vx[3] = beta2 * (2 * b - 1);
        _vx[5] = beta4 * (b2 - 2 * c);
        _vx[7] = -beta6 * c2;
        dVx.Init(0);
        dVx[3] = 2 * (beta * betaprim * (2 * b - 1) + bpr * beta2);
        dVx[5] = 4 * beta3 * betaprim * (b2 - 2 * c) + 2 * beta4 * bpr * (b - 1);
        dVx[7] = -6 * beta5 * betaprim * c2 - 2 * beta6 * bpr * c;

        // Y = 2*U*V
        _vy[2] = 2 * beta;
        _vy[4] = beta3 * 2 * (c + b);
        _vy[6] = 2 * beta5 * b * c;
        dVy.Init(0);
        dVy[2] = 2 * betaprim;
        dVy[4] = 6 * beta2 * betaprim * (b + c) + 4 * beta3 * bpr;
        dVy[6] = 10 * beta4 * betaprim * b * c + 2 * beta5 * bpr * (b + c);

        // W = U*U + V*V
        _vw[3] = beta2 * (1 + 2 * b);
        _vw[5] = beta4 * (2 * c + b2);
        _vw[7] = beta6 * c2;
        dVw.Init(0);
        //  DVw(3) = 2*(beta*betaprim*(1 + 2*b) + beta2*bpr);
        dVw[3] = 2 * beta * (betaprim * (1 + 2 * b) + beta * bpr);
        //  DVw(5) = 4*beta3*betaprim*(2*c + b2) + 2*beta4*bpr*(b+1);
        dVw[5] = 2 * beta3 * (2 * betaprim * (2 * c + b2) + beta * bpr * (b + 1));
        //  DVw(7) = 6*beta5*betaprim*c2 + 2*beta6*bpr*c;
        dVw[7] = 2 * beta5 * c * (3 * betaprim * c + beta * bpr);

        // Calcul des poles
        _px.Multiply(_b, _vx);
        _py.Multiply(_b, _vy);
        _w.Multiply(_b, _vw);
        dPx.Multiply(_b, dVx);
        dPy.Multiply(_b, dVy);
        dw.Multiply(_b, dVw);

        Xyz p = new Xyz(), dp = new Xyz();
        double wi;

        for (ii = 1; ii <= ordre; ii++)
        {
            wi = _w[ii];
            p.SetCoord(_px[ii] / wi, _py[ii] / wi, 0);
            dp.SetCoord(dPx[ii] / wi, dPy[ii] / wi, 0);
            dp -= (dw[ii] / wi) * p;

            poles[ii].ChangeCoord() = m * p + center.XYZ();
            p *= mPrim;
            dp *= m;
            aux.SetLinearForm(1, p, 1, dp, dCenter.XYZ());
            dPoles[ii].SetXYZ(aux);
            weights[ii] = wi;
            dWeights[ii] = dw[ii];
        }
    }

    /// <summary>The poles and weights of the arc and their first and second derivatives.</summary>
    public void Section(in Point3D firstPnt, in Vector3D dFirstPnt, in Vector3D d2FirstPnt, in Point3D center,
                        in Vector3D dCenter, in Vector3D d2Center, in Vector3D dir, in Vector3D dDir, in Vector3D d2Dir,
                        double angle, double dAngle, double d2Angle, NCollection_Array1<Point3D> poles,
                        NCollection_Array1<Vector3D> dPoles, NCollection_Array1<Vector3D> d2Poles,
                        NCollection_Array1<double> weights, NCollection_Array1<double> dWeights,
                        NCollection_Array1<double> d2Weights)
    {
        int ordre = 7;
        math_Vector dVx = new(1, ordre), dVy = new(1, ordre), dVw = new(1, ordre), d2Vx = new(1, ordre),
                    d2Vy = new(1, ordre), d2Vw = new(1, ordre);
        math_Vector dPx, dPy, dw = new(1, ordre), d2Px, d2Py, d2W = new(1, ordre);

        int ii;
        double daux, b, b2, c, c2, bpr, bsc;
        Vector3D v1 = new Vector3D(center, firstPnt), v1Prim, v1Secn = new Vector3D(), v2;

        // Calcul des  transformations
        Xyz auxyz;
        double sina, cosa;
        Matrix3x3 rot = new Matrix3x3(), rotPrim, rotSecn, d = new Matrix3x3(), dPrim = new Matrix3x3(), dSecn = new Matrix3x3(),
               ddp, maux;
        // La rotation s'ecrit I +  sin(Ang) * D + (1. - cos(Ang)) * D*D
        // ou D est l'application x -> Dir ^ x
        rot.SetRotation(dir.XYZ(), angle / 2);
        // La derive s'ecrit donc :
        // AngPrim * (sin(Ang)*D*D + cos(Ang)*D)
        // + sin(Ang)*DPrim  + (1. - cos(Ang)) *(DPrim*D + D*DPrim)
        sina = Math.Sin(angle / 2);
        cosa = Math.Cos(angle / 2);
        d.SetCross(dir.XYZ());
        dPrim.SetCross(dDir.XYZ());
        dSecn.SetCross(d2Dir.XYZ());

        ddp = (dPrim.Multiplied(d)).Added(d.Multiplied(dPrim));
        rotPrim = (d.Powered(2)).Multiplied(sina);
        rotPrim += d.Multiplied(cosa);
        rotPrim *= dAngle / 2;
        rotPrim += dPrim.Multiplied(sina);
        rotPrim += ddp.Multiplied(1 - cosa);

        rotSecn = (d.Powered(2)).Multiplied(sina);
        rotSecn += d.Multiplied(cosa);
        rotSecn *= d2Angle / 2;
        maux = (d.Powered(2)).Multiplied(cosa);
        maux -= d.Multiplied(sina);
        maux *= dAngle / 2;
        maux += ddp.Multiplied(2 * sina);
        maux += dPrim.Multiplied(2 * cosa);
        maux *= dAngle / 2;
        rotSecn += maux;
        maux = (dSecn.Multiplied(d)).Added(d.Multiplied(dSecn));
        maux += (dPrim.Powered(2)).Multiplied(2);
        maux *= 1 - cosa;
        maux += dSecn.Multiplied(sina);
        rotSecn += maux;

        v1Prim = dFirstPnt - dCenter;
        auxyz = (d2FirstPnt - d2Center).XYZ().Multiplied(rot);
        auxyz += 2 * (v1Prim.XYZ().Multiplied(rotPrim));
        auxyz += v1.XYZ().Multiplied(rotSecn);
        v1Secn.SetXYZ(auxyz);
        auxyz = v1Prim.XYZ().Multiplied(rot);
        auxyz += v1.XYZ().Multiplied(rotPrim);
        v1Prim.SetXYZ(auxyz);
        auxyz = v1.XYZ();
        auxyz.Multiply(rot);
        v1.SetXYZ(auxyz);
        v2 = dir ^ v1;

        Matrix3x3 m = new Matrix3x3(v1.X(), v2.X(), 0, v1.Y(), v2.Y(), 0, v1.Z(), v2.Z(), 0);
        v2 = (dDir.Crossed(v1)).Added(dir.Crossed(v1Prim));
        Matrix3x3 mPrim = new Matrix3x3(v1Prim.X(), v2.X(), 0, v1Prim.Y(), v2.Y(), 0, v1Prim.Z(), v2.Z(), 0);

        v2 = dDir ^ v1Prim;
        v2 *= 2;
        v2 += (d2Dir.Crossed(v1)).Added(dir.Crossed(v1Secn));
        Matrix3x3 mSecn = new Matrix3x3(v1Secn.X(), v2.X(), 0, v1Secn.Y(), v2.Y(), 0, v1Secn.Z(), v2.Z(), 0);

        // Calcul des coeff  -----------
        double tan_b, dtan_b, d2tan_b;
        double beta, beta2, beta3, beta4, beta5, beta6, betaprim, betasecn;
        double betaprim2, bpr2;
        beta = angle / 4;
        betaprim = dAngle / 4;
        betasecn = d2Angle / 4;
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
            double num, denom, aux;
            num = 0.2 + cf * beta2;
            denom = 1 + 0.2 * beta2;
            aux = (cf * denom - 0.2 * num) / (denom * denom);
            b = -num / denom;
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
        _vx[3] = beta2 * (2 * b - 1);
        _vx[5] = beta4 * (b2 - 2 * c);
        _vx[7] = -beta6 * c2;
        dVx.Init(0);
        dVx[3] = 2 * (beta * betaprim * (2 * b - 1) + bpr * beta2);
        dVx[5] = 4 * beta3 * betaprim * (b2 - 2 * c) + 2 * beta4 * bpr * (b - 1);
        dVx[7] = -6 * beta5 * betaprim * c2 - 2 * beta6 * bpr * c;
        d2Vx.Init(0);
        d2Vx[3] = 2 * ((betaprim2 + beta * betasecn) * (2 * b - 1) + 8 * beta * betaprim * bpr + bsc * beta2);
        d2Vx[5] = 4 * (b2 - 2 * c) * (3 * beta2 * betaprim2 + beta3 * betasecn)
                  + 16 * beta3 * betaprim * bpr * (b - 1) + 2 * beta4 * (bsc * (b - 1) + bpr2);
        d2Vx[7] = -6 * c2 * (5 * beta4 * betaprim2 + beta5 * betasecn) - 24 * beta5 * betaprim * bpr * c
                  - 2 * beta6 * (bsc * c + bpr2);

        // Y = 2*U*V
        _vy[2] = 2 * beta;
        _vy[4] = beta3 * 2 * (c + b);
        _vy[6] = 2 * beta5 * b * c;
        dVy.Init(0);
        dVy[2] = 2 * betaprim;
        dVy[4] = 6 * beta2 * betaprim * (b + c) + 4 * beta3 * bpr;
        dVy[6] = 10 * beta4 * betaprim * b * c + 2 * beta5 * bpr * (b + c);
        d2Vy.Init(0);
        d2Vy[2] = 2 * betasecn;
        d2Vy[4] = 6 * (b + c) * (2 * beta * betaprim2 + beta2 * betasecn)
                  + 24 * beta2 * betaprim * bpr * (b + c) + 4 * beta3 * bsc;
        d2Vy[6] = 10 * b * c * (4 * beta3 * betaprim2 + beta4 * betasecn)
                  + 40 * beta4 * betaprim * bpr * (b + c) + 2 * beta5 * (bsc * (b + c) + 2 * bpr2);

        // W = U*U + V*V
        _vw[3] = beta2 * (1 + 2 * b);
        _vw[5] = beta4 * (2 * c + b2);
        _vw[7] = beta6 * c2;
        dVw.Init(0);
        dVw[3] = 2 * (beta * betaprim * (1 + 2 * b) + beta2 * bpr);
        dVw[5] = 4 * beta3 * betaprim * (2 * c + b2) + 2 * beta4 * bpr * (b + 1);
        dVw[7] = 6 * beta5 * betaprim * c2 + 2 * beta6 * bpr * c;
        d2Vw.Init(0);
        d2Vw[3] = 2 * ((betaprim2 + beta * betasecn) * (2 * b + 1) + 8 * beta * betaprim * bpr + bsc * beta2);
        // OCCT: "(b + 11)" where the derivative of beta4 * (2c + b²) needs (b + 1) (occt-issues.md).
        d2Vw[5] = 4 * (b2 + 2 * c) * (3 * beta2 * betaprim2 + beta3 * betasecn)
                  + 16 * beta3 * betaprim * bpr * (b + 11) + 2 * beta4 * (bsc * (b + 1) + bpr2);
        d2Vw[7] = 6 * c2 * (5 * beta4 * betaprim2 + beta5 * betasecn) + 24 * beta5 * betaprim * bpr * c
                  + 2 * beta6 * (bsc * c + bpr2);

        // Calcul des poles
        _px = _b * _vx;
        _py = _b * _vy;
        _w.Multiply(_b, _vw);
        dPx = _b * dVx;
        dPy = _b * dVy;
        dw.Multiply(_b, dVw);
        d2Px = _b * d2Vx;
        d2Py = _b * d2Vy;
        d2W.Multiply(_b, d2Vw);

        Xyz p = new Xyz(), dp = new Xyz(), d2P = new Xyz();
        double wi, dwi;
        for (ii = 1; ii <= ordre; ii++)
        {
            wi = _w[ii];
            dwi = dw[ii];
            p.SetCoord(_px[ii] / wi, _py[ii] / wi, 0);
            dp.SetCoord(dPx[ii] / wi, dPy[ii] / wi, 0);

            d2P.SetCoord(d2Px[ii] / wi, d2Py[ii] / wi, 0);
            d2P -= 2 * (dwi / wi) * dp;
            d2P += (2 * Math.Pow(dwi / wi, 2) - d2W[ii] / wi) * p;
            dp -= (dw[ii] / wi) * p;

            poles[ii].ChangeCoord() = m * p + center.XYZ();
            auxyz.SetLinearForm(1, mPrim * p, 1, m * dp, dCenter.XYZ());
            dPoles[ii].SetXYZ(auxyz);
            p *= mSecn;
            dp *= mPrim;
            d2P *= m;
            auxyz.SetLinearForm(1, p, 2, dp, 1, d2P, d2Center.XYZ());
            d2Poles[ii].SetXYZ(auxyz);
            weights[ii] = wi;
            dWeights[ii] = dwi;
            d2Weights[ii] = d2W[ii];
        }
    }
}
