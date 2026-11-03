// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_EvolRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_EvolRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).


using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.BlendFunc_SectionShape;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;

namespace Occt;

/// <summary>
/// The function of a fillet of evolving radius between two surfaces along a guide curve: as BlendFunc_ConstRad,
/// with the radius given by a law of the guide's parameter.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="ComputeValues"/> caches its last positioning, and the walking reads
/// the cached points and tangents. OCCT's <c>thread_local</c> scratch values of ComputeValues are read across calls
/// (when the cache skips a positioning), so they are <c>[ThreadStatic]</c> (plan 2.3, decision 1037).
/// </remarks>
public class BlendFunc_EvolRad : Blend_Function
{
    private const double Eps = 1.0e-15;

    // Per-thread scratch, reused across calls via the t_OK / myX_OK guards of ComputeValues (OCCT's thread_local).
    [ThreadStatic] private static Vector3D d3u1, d3v1, d3uuv1, d3uvv1, d3u2, d3v2, d3uuv2, d3uvv2;
    [ThreadStatic] private static Vector3D d1gui, d2gui, d3gui;
    [ThreadStatic] private static Point3D ptgui;
    [ThreadStatic] private static double invnormtg, dinvnormtg;

    private readonly SurfaceAdaptorBase surf1;
    private readonly SurfaceAdaptorBase surf2;
    private readonly CurveAdaptorBase curv;
    private CurveAdaptorBase tcurv;
    private readonly Law_Function fevol;
    private Law_Function tevol;
    private Point3D pts1;
    private Point3D pts2;
    private bool istangent;
    private Vector3D tg1;
    private Vector2D tg12d;
    private Vector3D tg2;
    private Vector2D tg22d;
    private double param;
    private double sg1;
    private double sg2;
    private double ray;
    private double dray;
    private double d2ray;
    private int choix;
    private int myXOrder;
    private int myTOrder;
    private readonly math_Vector xval;
    private double tval;
    private Vector3D d1u1;
    private Vector3D d1u2;
    private Vector3D d1v1;
    private Vector3D d1v2;
    private Vector3D d2u1;
    private Vector3D d2v1;
    private Vector3D d2uv1;
    private Vector3D d2u2;
    private Vector3D d2v2;
    private Vector3D d2uv2;
    private Vector3D dn1w;
    private Vector3D dn2w;
    private Vector3D d2n1w;
    private Vector3D d2n2w;
    private Vector3D nplan;
    private Vector3D nsurf1;
    private Vector3D nsurf2;
    private Vector3D dns1u1;
    private Vector3D dns1u2;
    private Vector3D dns1v1;
    private Vector3D dns1v2;
    private Vector3D dnplan;
    private Vector3D d2nplan;
    // OCCT's dnsurf1 and dnsurf2 are never used: left out.
    private Vector3D dndu1;
    private Vector3D dndu2;
    private Vector3D dndv1;
    private Vector3D dndv2;
    private Vector3D d2ndu1;
    private Vector3D d2ndu2;
    private Vector3D d2ndv1;
    private Vector3D d2ndv2;
    private Vector3D d2nduv1;
    private Vector3D d2nduv2;
    private Vector3D d2ndtu1;
    private Vector3D d2ndtu2;
    private Vector3D d2ndtv1;
    private Vector3D d2ndtv2;
    private readonly math_Vector E;
    private readonly Matrix DEDX;
    private readonly math_Vector DEDT;
    private readonly BlendFunc_Tensor D2EDX2;
    private readonly Matrix D2EDXDT;
    private readonly math_Vector D2EDT2;
    private double minang;
    private double maxang;
    private double lengthmin;
    private double lengthmax;
    private double distmin;
    private BlendFunc_SectionShape mySShape;
    private ParameterisationType myTConv;

    /// <remarks>OCCT leaves param, the radius and the "quadrant" uninitialised; here they are zero.</remarks>
    public BlendFunc_EvolRad(SurfaceAdaptorBase S1, SurfaceAdaptorBase S2, CurveAdaptorBase C, Law_Function Law)
    {
        surf1 = S1;
        surf2 = S2;
        curv = C;
        tcurv = C;
        istangent = true;
        xval = new math_Vector(1, 4);
        E = new math_Vector(1, 4);
        DEDX = new Matrix(1, 4, 1, 4);
        DEDT = new math_Vector(1, 4);
        D2EDX2 = new BlendFunc_Tensor(4, 4, 4);
        D2EDXDT = new Matrix(1, 4, 1, 4);
        D2EDT2 = new math_Vector(1, 4);
        minang = RealLast();
        maxang = RealFirst();
        lengthmin = RealLast();
        lengthmax = RealFirst();
        distmin = RealLast();
        mySShape = BlendFunc_Rational;

        fevol = Law;
        tevol = Law;

        // Initialisaton of cash control variables.
        tval = -9.876e100;
        xval.Init(-9.876e100);
        myXOrder = -1;
        myTOrder = -1;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    /// <summary>Sets the "quadrant" <paramref name="Choix"/> (1 to 8) in which the fillet lies.</summary>
    public void Set(int Choix)
    {
        choix = Choix;
        switch (choix)
        {
            case 1:
            case 2:
                {
                    sg1 = -1.0;
                    sg2 = -1.0;
                }
                break;
            case 3:
            case 4:
                {
                    sg1 = 1.0;
                    sg2 = -1.0;
                }
                break;
            case 5:
            case 6:
                {
                    sg1 = 1.0;
                    sg2 = 1.0;
                }
                break;
            case 7:
            case 8:
                {
                    sg1 = -1.0;
                    sg2 = 1.0;
                }
                break;
            default:
                sg1 = sg2 = -1.0;
                break;
        }
    }

    /// <summary>Sets the type of the sections the approximation makes.</summary>
    public void Set(BlendFunc_SectionShape TypeSection)
    {
        mySShape = TypeSection;
    }

    //=======================================================================
    // function : ComputeValues
    // purpose  : OBLIGATORY passage for all computations
    //           This method manages the positioning on Surfaces and Curves
    //           Partial calculation of equations and their derivatives
    //           Storage of some intermediary results in fields to be
    //           used in other methods.
    //=======================================================================
    private bool ComputeValues(math_Vector X, int Order, bool byParam = false, double Param = 0)
    {
        double T = Param, aux;

        // Case of implicite parameter
        if (!byParam)
        {
            T = param;
        }

        // Is the work already done ?
        bool myX_OK = (Order <= myXOrder);
        for (int ii = 1; ((ii <= X.Length()) && myX_OK); ii++)
        {
            myX_OK = (X[ii] == xval[ii]);
        }

        bool t_OK = ((T == tval) && ((Order <= myTOrder) || (!byParam)));

        if (myX_OK && (t_OK))
        {
            return true;
        }

        // Processing of t
        if (!t_OK)
        {
            tval = T;
            if (byParam)
            {
                myTOrder = Order;
            }
            else
            {
                myTOrder = 0;
            }

            //----- Positioning on the curve ----------------
            switch (myTOrder)
            {
                case 0:
                    {
                        tcurv.D1(T, out ptgui, out d1gui);
                        nplan = d1gui.Normalized();
                        ray = tevol.Value(T);
                    }
                    break;

                case 1:
                    {
                        tcurv.D2(T, out ptgui, out d1gui, out d2gui);
                        nplan = d1gui.Normalized();
                        invnormtg = ((double)1) / d1gui.Magnitude();
                        dnplan.SetLinearForm(invnormtg, d2gui, -invnormtg * (nplan.Dot(d2gui)), nplan);
                        tevol.D1(T, out ray, out dray);
                    }
                    break;
                case 2:
                    {
                        tcurv.D3(T, out ptgui, out d1gui, out d2gui, out d3gui);
                        nplan = d1gui.Normalized();
                        invnormtg = ((double)1) / d1gui.Magnitude();
                        dnplan.SetLinearForm(invnormtg, d2gui, -invnormtg * (nplan.Dot(d2gui)), nplan);
                        dinvnormtg = -nplan.Dot(d2gui) * invnormtg * invnormtg;
                        d2nplan.SetLinearForm(invnormtg, d3gui, dinvnormtg, d2gui);
                        aux = dinvnormtg * (nplan.Dot(d2gui)) + invnormtg * (dnplan.Dot(d2gui) + nplan.Dot(d3gui));
                        d2nplan.SetLinearForm(-invnormtg * (nplan.Dot(d2gui)), dnplan, -aux, nplan, d2nplan);
                        tevol.D2(T, out ray, out dray, out d2ray);
                        break;
                    }
                default:
                    return false;
            }
        }

        // Processing of X
        if (!myX_OK)
        {
            xval.Initialized(X);
            myXOrder = Order;
            //-------------- Positioning on surfaces -----------------
            switch (myXOrder)
            {
                case 0:
                    {
                        surf1.D1(X[1], X[2], out pts1, out d1u1, out d1v1);
                        nsurf1 = d1u1.Crossed(d1v1);
                        surf2.D1(X[3], X[4], out pts2, out d1u2, out d1v2);
                        nsurf2 = d1u2.Crossed(d1v2);
                        break;
                    }
                case 1:
                    {
                        surf1.D2(X[1], X[2], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
                        nsurf1 = d1u1.Crossed(d1v1);
                        dns1u1 = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
                        dns1v1 = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
                        surf2.D2(X[3], X[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
                        nsurf2 = d1u2.Crossed(d1v2);
                        dns1u2 = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
                        dns1v2 = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
                        break;
                    }
                case 2:
                    {
                        // OCCT: unlike BlendFunc_ConstRad, order 2 does not update dns1u1, dns1v1, dns1u2 and
                        // dns1v2, which keep the values of the last order-1 positioning (decision 1039).
                        surf1.D3(X[1], X[2], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1, out d3u1,
                                 out d3v1, out d3uuv1, out d3uvv1);
                        nsurf1 = d1u1.Crossed(d1v1);
                        surf2.D3(X[3], X[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2, out d3u2,
                                 out d3v2, out d3uuv2, out d3uvv2);
                        nsurf2 = d1u2.Crossed(d1v2);
                        break;
                    }
                default:
                    return false;
            }

            // Case of degenerated surfaces
            if (nsurf1.Magnitude() < Eps)
            {
                Point2D P = new Point2D(X[1], X[2]);
                if (Order == 0)
                {
                    BlendFunc.ComputeNormal(surf1, P, ref nsurf1);
                }
                else
                {
                    BlendFunc.ComputeDNormal(surf1, P, ref nsurf1, ref dns1u1, ref dns1v1);
                }
            }

            if (nsurf2.Magnitude() < Eps)
            {
                Point2D P = new Point2D(X[3], X[4]);
                if (Order == 0)
                {
                    BlendFunc.ComputeNormal(surf2, P, ref nsurf2);
                }
                else
                {
                    BlendFunc.ComputeDNormal(surf2, P, ref nsurf2, ref dns1u2, ref dns1v2);
                }
            }
        }

        // -------------------- Positioning of order 0 ---------------------
        double invnorm1, invnorm2, ndotns1, ndotns2, theD;
        double ray1 = sg1 * ray;
        double ray2 = sg2 * ray;
        Vector3D ncrossns1, ncrossns2, resul = new Vector3D(), temp = new Vector3D(), n1 = new Vector3D(), n2 = new Vector3D();

        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));

        E[1] = (nplan.X() * (pts1.X() + pts2.X()) + nplan.Y() * (pts1.Y() + pts2.Y())
                + nplan.Z() * (pts1.Z() + pts2.Z()))
                 / 2
               + theD;

        ncrossns1 = nplan.Crossed(nsurf1);
        ncrossns2 = nplan.Crossed(nsurf2);

        invnorm1 = ncrossns1.Magnitude();
        invnorm2 = ncrossns2.Magnitude();

        if (invnorm1 > Eps)
        {
            invnorm1 = ((double)1) / invnorm1;
        }
        else
        {
            invnorm1 = 1; // Unsatisfactory, but it is not necessary to crash
            // OCCT_DEBUG trace left out.
        }

        if (invnorm2 > Eps)
        {
            invnorm2 = ((double)1) / invnorm2;
        }
        else
        {
            invnorm2 = 1; //  Unsatisfactory, but it is not necessary to crash
            // OCCT_DEBUG trace left out.
        }

        ndotns1 = nplan.Dot(nsurf1);
        ndotns2 = nplan.Dot(nsurf2);

        n1.SetLinearForm(ndotns1, nplan, -1.0, nsurf1);
        n1.Multiply(invnorm1);
        n2.SetLinearForm(ndotns2, nplan, -1.0, nsurf2);
        n2.Multiply(invnorm2);

        resul.SetLinearForm(ray1, n1, -ray2, n2, new Vector3D(pts2, pts1));

        E[2] = resul.X();
        E[3] = resul.Y();
        E[4] = resul.Z();

        // -------------------- Positioning of order 1 ---------------------
        if (Order >= 1)
        {
            double grosterme, cube, carre;

            DEDX[1, 1] = nplan.Dot(d1u1) / 2;
            DEDX[1, 2] = nplan.Dot(d1v1) / 2;
            DEDX[1, 3] = nplan.Dot(d1u2) / 2;
            DEDX[1, 4] = nplan.Dot(d1v2) / 2;

            cube = invnorm1 * invnorm1 * invnorm1;
            // Derived in relation to u1
            grosterme = -ncrossns1.Dot(nplan.Crossed(dns1u1)) * cube;
            dndu1.SetLinearForm(grosterme * ndotns1 + invnorm1 * nplan.Dot(dns1u1), nplan, -grosterme, nsurf1,
                                -invnorm1, dns1u1);

            resul.SetLinearForm(ray1, dndu1, d1u1);
            DEDX[2, 1] = resul.X();
            DEDX[3, 1] = resul.Y();
            DEDX[4, 1] = resul.Z();

            // Derived in relation to v1

            grosterme = -ncrossns1.Dot(nplan.Crossed(dns1v1)) * cube;
            dndv1.SetLinearForm(grosterme * ndotns1 + invnorm1 * nplan.Dot(dns1v1), nplan, -grosterme, nsurf1,
                                -invnorm1, dns1v1);

            resul.SetLinearForm(ray1, dndv1, d1v1);
            DEDX[2, 2] = resul.X();
            DEDX[3, 2] = resul.Y();
            DEDX[4, 2] = resul.Z();

            cube = invnorm2 * invnorm2 * invnorm2;
            // Derived in relation to u2
            grosterme = -ncrossns2.Dot(nplan.Crossed(dns1u2)) * cube;
            dndu2.SetLinearForm(grosterme * ndotns2 + invnorm2 * nplan.Dot(dns1u2), nplan, -grosterme, nsurf2,
                                -invnorm2, dns1u2);

            resul.SetLinearForm(-ray2, dndu2, -1, d1u2);
            DEDX[2, 3] = resul.X();
            DEDX[3, 3] = resul.Y();
            DEDX[4, 3] = resul.Z();

            // Derived in relation to v2
            grosterme = -ncrossns2.Dot(nplan.Crossed(dns1v2)) * cube;
            dndv2.SetLinearForm(grosterme * ndotns2 + invnorm2 * nplan.Dot(dns1v2), nplan, -grosterme, nsurf2,
                                -invnorm2, dns1v2);

            resul.SetLinearForm(-ray2, dndv2, -1, d1v2);
            DEDX[2, 4] = resul.X();
            DEDX[3, 4] = resul.Y();
            DEDX[4, 4] = resul.Z();

            if (byParam)
            {
                temp.SetXYZ((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ());
                // Derived from n1 in relation to w
                grosterme = ncrossns1.Dot(dnplan.Crossed(nsurf1)) * invnorm1 * invnorm1;
                dn1w.SetLinearForm((dnplan.Dot(nsurf1) - grosterme * ndotns1) * invnorm1, nplan,
                                   ndotns1 * invnorm1, dnplan, grosterme * invnorm1, nsurf1);

                // Derivee from n2 in relation to w
                grosterme = ncrossns2.Dot(dnplan.Crossed(nsurf2)) * invnorm2 * invnorm2;
                dn2w.SetLinearForm((dnplan.Dot(nsurf2) - grosterme * ndotns2) * invnorm2, nplan,
                                   ndotns2 * invnorm2, dnplan, grosterme * invnorm2, nsurf2);

                DEDT[1] = dnplan.Dot(temp) - 1.0 / invnormtg;
                temp.SetLinearForm(ray2, dn2w, sg2 * dray, n2);
                resul.SetLinearForm(ray1, dn1w, sg1 * dray, n1, -1, temp);
                DEDT[2] = resul.X();
                DEDT[3] = resul.Y();
                DEDT[4] = resul.Z();
            }

            // ------   Positioning of order 2  -----------------------------
            if (Order == 2)
            {
                Vector3D d2ns1u1 = new Vector3D(), d2ns1u2 = new Vector3D(), d2ns1v1 = new Vector3D(),
                       d2ns1v2 = new Vector3D(), d2ns1uv1, d2ns1uv2;
                double uterm, vterm, smallterm, p1, p2, p12;
                double DPrim, DSecn;
                D2EDX2.Init(0);

                D2EDX2[1, 1, 1] = nplan.Dot(d2u1) / 2;
                D2EDX2[1, 2, 1] = D2EDX2[1, 1, 2] = nplan.Dot(d2uv1) / 2;
                D2EDX2[1, 2, 2] = nplan.Dot(d2v1) / 2;

                D2EDX2[1, 3, 3] = nplan.Dot(d2u2) / 2;
                D2EDX2[1, 4, 3] = D2EDX2[1, 3, 4] = nplan.Dot(d2uv2) / 2;
                D2EDX2[1, 4, 4] = nplan.Dot(d2v2) / 2;
                // ================
                // ==  Surface 1 ==
                // ================
                carre = invnorm1 * invnorm1;
                cube = carre * invnorm1;
                // Derived double compared to u1
                // Derived from the norm
                d2ns1u1.SetLinearForm(1, d3u1.Crossed(d1v1), 2, d2u1.Crossed(d2uv1), 1, d1u1.Crossed(d3uuv1));
                DPrim = ncrossns1.Dot(nplan.Crossed(dns1u1));
                smallterm = -2 * DPrim * cube;
                DSecn = ncrossns1.Dot(nplan.Crossed(d2ns1u1)) + (nplan.Crossed(dns1u1)).SquareMagnitude();
                grosterme = (3 * DPrim * DPrim * carre - DSecn) * cube;

                temp.SetLinearForm(grosterme * ndotns1, nplan, -grosterme, nsurf1);
                p1 = nplan.Dot(dns1u1);
                p2 = nplan.Dot(d2ns1u1);
                d2ndu1.SetLinearForm(invnorm1 * p2 + smallterm * p1, nplan, -smallterm, dns1u1, -invnorm1, d2ns1u1);
                d2ndu1 += temp;
                resul.SetLinearForm(ray1, d2ndu1, d2u1);
                D2EDX2[2, 1, 1] = resul.X();
                D2EDX2[3, 1, 1] = resul.Y();
                D2EDX2[4, 1, 1] = resul.Z();

                // Derived double compared to u1, v1
                // Derived from the norm
                d2ns1uv1 = (d3uuv1.Crossed(d1v1)) + (d2u1.Crossed(d2v1)) + (d1u1.Crossed(d3uvv1));
                uterm = ncrossns1.Dot(nplan.Crossed(dns1u1));
                vterm = ncrossns1.Dot(nplan.Crossed(dns1v1));
                DSecn = (nplan.Crossed(dns1v1)).Dot(nplan.Crossed(dns1u1)) + ncrossns1.Dot(nplan.Crossed(d2ns1uv1));
                grosterme = (3 * uterm * vterm * carre - DSecn) * cube;
                uterm *= -cube; // and only now
                vterm *= -cube;

                p1 = nplan.Dot(dns1u1);
                p2 = nplan.Dot(dns1v1);
                temp.SetLinearForm(grosterme * ndotns1, nplan, -grosterme, nsurf1, -invnorm1, d2ns1uv1);
                d2nduv1.SetLinearForm(invnorm1 * nplan.Dot(d2ns1uv1) + uterm * p2 + vterm * p1, nplan, -uterm,
                                      dns1v1, -vterm, dns1u1);

                d2nduv1 += temp;
                resul.SetLinearForm(ray1, d2nduv1, d2uv1);

                D2EDX2[2, 2, 1] = D2EDX2[2, 1, 2] = resul.X();
                D2EDX2[3, 2, 1] = D2EDX2[3, 1, 2] = resul.Y();
                D2EDX2[4, 2, 1] = D2EDX2[4, 1, 2] = resul.Z();

                // Derived double compared to v1
                // Derived from the norm
                d2ns1v1.SetLinearForm(1, d1u1.Crossed(d3v1), 2, d2uv1.Crossed(d2v1), 1, d3uvv1.Crossed(d1v1));
                DPrim = ncrossns1.Dot(nplan.Crossed(dns1v1));
                smallterm = -2 * DPrim * cube;
                DSecn = ncrossns1.Dot(nplan.Crossed(d2ns1v1)) + (nplan.Crossed(dns1v1)).SquareMagnitude();
                grosterme = (3 * DPrim * DPrim * carre - DSecn) * cube;

                p1 = nplan.Dot(dns1v1);
                p2 = nplan.Dot(d2ns1v1);
                temp.SetLinearForm(grosterme * ndotns1, nplan, -grosterme, nsurf1);
                d2ndv1.SetLinearForm(invnorm1 * p2 + smallterm * p1, nplan, -smallterm, dns1v1, -invnorm1, d2ns1v1);
                d2ndv1 += temp;
                resul.SetLinearForm(ray1, d2ndv1, d2v1);

                D2EDX2[2, 2, 2] = resul.X();
                D2EDX2[3, 2, 2] = resul.Y();
                D2EDX2[4, 2, 2] = resul.Z();
                // ================
                // ==  Surface 2 ==
                // ================
                carre = invnorm2 * invnorm2;
                cube = carre * invnorm2;
                // Derived double compared to u2
                // Derived from the norm
                d2ns1u2.SetLinearForm(1, d3u2.Crossed(d1v2), 2, d2u2.Crossed(d2uv2), 1, d1u2.Crossed(d3uuv2));
                DPrim = ncrossns2.Dot(nplan.Crossed(dns1u2));
                smallterm = -2 * DPrim * cube;
                DSecn = ncrossns2.Dot(nplan.Crossed(d2ns1u2)) + (nplan.Crossed(dns1u2)).SquareMagnitude();
                grosterme = (3 * DPrim * DPrim * carre - DSecn) * cube;

                temp.SetLinearForm(grosterme * ndotns2, nplan, -grosterme, nsurf2);
                p1 = nplan.Dot(dns1u2);
                p2 = nplan.Dot(d2ns1u2);
                d2ndu2.SetLinearForm(invnorm2 * p2 + smallterm * p1, nplan, -smallterm, dns1u2, -invnorm2, d2ns1u2);
                d2ndu2 += temp;
                resul.SetLinearForm(-ray2, d2ndu2, -1, d2u2);
                D2EDX2[2, 3, 3] = resul.X();
                D2EDX2[3, 3, 3] = resul.Y();
                D2EDX2[4, 3, 3] = resul.Z();

                // Derived double compared to u2, v2
                // Derived from the norm
                d2ns1uv2 = (d3uuv2.Crossed(d1v2)) + (d2u2.Crossed(d2v2)) + (d1u2.Crossed(d3uvv2));
                uterm = ncrossns2.Dot(nplan.Crossed(dns1u2));
                vterm = ncrossns2.Dot(nplan.Crossed(dns1v2));
                DSecn = (nplan.Crossed(dns1v2)).Dot(nplan.Crossed(dns1u2)) + ncrossns2.Dot(nplan.Crossed(d2ns1uv2));
                grosterme = (3 * uterm * vterm * carre - DSecn) * cube;
                uterm *= -cube; // and only now
                vterm *= -cube;

                p1 = nplan.Dot(dns1u2);
                p2 = nplan.Dot(dns1v2);
                temp.SetLinearForm(grosterme * ndotns2, nplan, -grosterme, nsurf2, -invnorm2, d2ns1uv2);
                d2nduv2.SetLinearForm(invnorm2 * nplan.Dot(d2ns1uv2) + uterm * p2 + vterm * p1, nplan, -uterm,
                                      dns1v2, -vterm, dns1u2);

                d2nduv2 += temp;
                resul.SetLinearForm(-ray2, d2nduv2, -1, d2uv2);

                D2EDX2[2, 4, 3] = D2EDX2[2, 3, 4] = resul.X();
                D2EDX2[3, 4, 3] = D2EDX2[3, 3, 4] = resul.Y();
                D2EDX2[4, 4, 3] = D2EDX2[4, 3, 4] = resul.Z();

                // Derived double compared to v2
                // Derived from the norm
                d2ns1v2.SetLinearForm(1, d1u2.Crossed(d3v2), 2, d2uv2.Crossed(d2v2), 1, d3uvv2.Crossed(d1v2));
                DPrim = ncrossns2.Dot(nplan.Crossed(dns1v2));
                smallterm = -2 * DPrim * cube;
                DSecn = ncrossns2.Dot(nplan.Crossed(d2ns1v2)) + (nplan.Crossed(dns1v2)).SquareMagnitude();
                grosterme = (3 * DPrim * DPrim * carre - DSecn) * cube;

                p1 = nplan.Dot(dns1v2);
                p2 = nplan.Dot(d2ns1v2);
                temp.SetLinearForm(grosterme * ndotns2, nplan, -grosterme, nsurf2);
                d2ndv2.SetLinearForm(invnorm2 * p2 + smallterm * p1, nplan, -smallterm, dns1v2, -invnorm2, d2ns1v2);
                d2ndv2 += temp;
                resul.SetLinearForm(-ray2, d2ndv2, -1, d2v2);

                D2EDX2[2, 4, 4] = resul.X();
                D2EDX2[3, 4, 4] = resul.Y();
                D2EDX2[4, 4, 4] = resul.Z();

                if (byParam)
                {
                    double tterm;
                    //  ---------- Derivation double in t, X --------------------------
                    D2EDXDT[1, 1] = dnplan.Dot(d1u1) / 2;
                    D2EDXDT[1, 2] = dnplan.Dot(d1v1) / 2;
                    D2EDXDT[1, 3] = dnplan.Dot(d1u2) / 2;
                    D2EDXDT[1, 4] = dnplan.Dot(d1v2) / 2;

                    carre = invnorm1 * invnorm1;
                    cube = carre * invnorm1;
                    //--> Derived compared to u1 and t
                    tterm = ncrossns1.Dot(dnplan.Crossed(nsurf1));
                    smallterm = -tterm * cube;
                    // Derived from the norm
                    uterm = ncrossns1.Dot(nplan.Crossed(dns1u1));
                    DSecn = (nplan.Crossed(dns1u1)).Dot(dnplan.Crossed(nsurf1))
                            + ncrossns1.Dot(dnplan.Crossed(dns1u1));
                    grosterme = (3 * uterm * tterm * carre - DSecn) * cube;
                    uterm *= -cube;

                    p1 = dnplan.Dot(nsurf1);
                    p2 = nplan.Dot(dns1u1);
                    p12 = dnplan.Dot(dns1u1);

                    d2ndtu1.SetLinearForm(invnorm1 * p12 + smallterm * p2 + uterm * p1 + grosterme * ndotns1, nplan,
                                          invnorm1 * p2 + uterm * ndotns1, dnplan, -smallterm, dns1u1);
                    d2ndtu1 -= grosterme * nsurf1;

                    resul.SetLinearForm(ray1, d2ndtu1, sg1 * dray, dndu1);
                    D2EDXDT[2, 1] = resul.X();
                    D2EDXDT[3, 1] = resul.Y();
                    D2EDXDT[4, 1] = resul.Z();

                    //--> Derived compared to v1 and t
                    // Derived from the norm
                    uterm = ncrossns1.Dot(nplan.Crossed(dns1v1));
                    DSecn = (nplan.Crossed(dns1v1)).Dot(dnplan.Crossed(nsurf1))
                            + ncrossns1.Dot(dnplan.Crossed(dns1v1));
                    grosterme = (3 * uterm * tterm * carre - DSecn) * cube;
                    uterm *= -cube;

                    p1 = dnplan.Dot(nsurf1);
                    p2 = nplan.Dot(dns1v1);
                    p12 = dnplan.Dot(dns1v1);
                    d2ndtv1.SetLinearForm(invnorm1 * p12 + uterm * p1 + smallterm * p2 + grosterme * ndotns1, nplan,
                                          invnorm1 * p2 + uterm * ndotns1, dnplan, -smallterm, dns1v1);
                    d2ndtv1 -= grosterme * nsurf1;

                    resul.SetLinearForm(ray1, d2ndtv1, sg1 * dray, dndv1);
                    D2EDXDT[2, 2] = resul.X();
                    D2EDXDT[3, 2] = resul.Y();
                    D2EDXDT[4, 2] = resul.Z();

                    carre = invnorm2 * invnorm2;
                    cube = carre * invnorm2;
                    //--> Derived compared to u2 and t
                    tterm = ncrossns2.Dot(dnplan.Crossed(nsurf2));
                    smallterm = -tterm * cube;
                    // Derived from the norm
                    uterm = ncrossns2.Dot(nplan.Crossed(dns1u2));
                    DSecn = (nplan.Crossed(dns1u2)).Dot(dnplan.Crossed(nsurf2))
                            + ncrossns2.Dot(dnplan.Crossed(dns1u2));
                    grosterme = (3 * uterm * tterm * carre - DSecn) * cube;
                    uterm *= -cube;

                    p1 = dnplan.Dot(nsurf2);
                    p2 = nplan.Dot(dns1u2);
                    p12 = dnplan.Dot(dns1u2);

                    d2ndtu2.SetLinearForm(invnorm2 * p12 + smallterm * p2 + uterm * p1 + grosterme * ndotns2, nplan,
                                          invnorm2 * p2 + uterm * ndotns2, dnplan, -smallterm, dns1u2);
                    d2ndtu2 -= grosterme * nsurf2;

                    resul.SetLinearForm(-ray2, d2ndtu2, -sg2 * dray, dndu2);
                    D2EDXDT[2, 3] = resul.X();
                    D2EDXDT[3, 3] = resul.Y();
                    D2EDXDT[4, 3] = resul.Z();

                    //--> Derived compared to v2 and t
                    // Derived from the norm
                    uterm = ncrossns2.Dot(nplan.Crossed(dns1v2));
                    DSecn = (nplan.Crossed(dns1v2)).Dot(dnplan.Crossed(nsurf2))
                            + ncrossns2.Dot(dnplan.Crossed(dns1v2));
                    grosterme = (3 * uterm * tterm * carre - DSecn) * cube;
                    uterm *= -cube;

                    p1 = dnplan.Dot(nsurf2);
                    p2 = nplan.Dot(dns1v2);
                    p12 = dnplan.Dot(dns1v2);

                    d2ndtv2.SetLinearForm(invnorm2 * p12 + smallterm * p2 + uterm * p1 + grosterme * ndotns2, nplan,
                                          invnorm2 * p2 + uterm * ndotns2, dnplan, -smallterm, dns1v2);
                    d2ndtv2 -= grosterme * nsurf2;

                    resul.SetLinearForm(-ray2, d2ndtv2, -sg2 * dray, dndv2);
                    D2EDXDT[2, 4] = resul.X();
                    D2EDXDT[3, 4] = resul.Y();
                    D2EDXDT[4, 4] = resul.Z();

                    //  ---------- Derivation double in t -----------------------------
                    // Derived from n1 compared to w
                    carre = invnorm1 * invnorm1;
                    cube = carre * invnorm1;
                    // Derived from the norm
                    DPrim = ncrossns1.Dot(dnplan.Crossed(nsurf1));
                    smallterm = -2 * DPrim * cube;
                    DSecn = (dnplan.Crossed(nsurf1)).SquareMagnitude() + ncrossns1.Dot(d2nplan.Crossed(nsurf1));
                    grosterme = (3 * DPrim * DPrim * carre - DSecn) * cube;

                    p1 = dnplan.Dot(nsurf1);
                    p2 = d2nplan.Dot(nsurf1);

                    temp.SetLinearForm(grosterme * ndotns1, nplan, -grosterme, nsurf1);
                    d2n1w.SetLinearForm(smallterm * p1 + invnorm1 * p2, nplan, smallterm * ndotns1 + 2 * invnorm1 * p1,
                                        dnplan, ndotns1 * invnorm1, d2nplan);
                    d2n1w += temp;

                    // Derived from n2 compared to w
                    carre = invnorm2 * invnorm2;
                    cube = carre * invnorm2;
                    // Derived from the norm
                    DPrim = ncrossns2.Dot(dnplan.Crossed(nsurf2));
                    smallterm = -2 * DPrim * cube;
                    DSecn = (dnplan.Crossed(nsurf2)).SquareMagnitude() + ncrossns2.Dot(d2nplan.Crossed(nsurf2));
                    grosterme = (3 * DPrim * DPrim * carre - DSecn) * cube;

                    p1 = dnplan.Dot(nsurf2);
                    p2 = d2nplan.Dot(nsurf2);

                    temp.SetLinearForm(grosterme * ndotns2, nplan, -grosterme, nsurf2);
                    d2n2w.SetLinearForm(smallterm * p1 + invnorm2 * p2, nplan, smallterm * ndotns2 + 2 * invnorm2 * p1,
                                        dnplan, ndotns2 * invnorm2, d2nplan);
                    d2n2w += temp;

                    temp.SetXYZ((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ());
                    D2EDT2[1] = d2nplan.Dot(temp) - 2 * dnplan.Dot(d1gui) - nplan.Dot(d2gui);
                    resul.SetLinearForm(ray1, d2n1w, 2 * sg1 * dray, dn1w, sg1 * d2ray, n1);
                    temp.SetLinearForm(ray2, d2n2w, 2 * sg2 * dray, dn2w, sg2 * d2ray, n2);
                    resul -= temp;
                    D2EDT2[2] = resul.X();
                    D2EDT2[3] = resul.Y();
                    D2EDT2[4] = resul.Z();
                }
            }
        }

        return true;
    }

    public override void Set(double Param)
    {
        param = Param;
    }

    //=======================================================================
    // function : Set
    // purpose  : Segments curve in its useful part.
    //           Small precision is taken at random
    //=======================================================================
    public override void Set(double First, double Last)
    {
        tcurv = curv.Trim(First, Last, 1.0e-12);
        tevol = fevol.Trim(First, Last, 1.0e-12);
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = surf1.UResolution(Tol);
        Tolerance[2] = surf1.VResolution(Tol);
        Tolerance[3] = surf2.UResolution(Tol);
        Tolerance[4] = surf2.VResolution(Tol);
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = surf1.FirstUParameter();
        InfBound[2] = surf1.FirstVParameter();
        InfBound[3] = surf2.FirstUParameter();
        InfBound[4] = surf2.FirstVParameter();
        SupBound[1] = surf1.LastUParameter();
        SupBound[2] = surf1.LastVParameter();
        SupBound[3] = surf2.LastUParameter();
        SupBound[4] = surf2.LastVParameter();

        for (int i = 1; i <= 4; i++)
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[i]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[i]))
            {
                double range = (SupBound[i] - InfBound[i]);
                InfBound[i] -= range;
                SupBound[i] += range;
            }
        }
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        double norm, Cosa, Sina, Angle;
        bool Ok = true;

        Ok = ComputeValues(Sol, 1, true, param);

        if (Math.Abs(E[1]) <= Tol && E[2] * E[2] + E[3] * E[3] + E[4] * E[4] <= Tol * Tol)
        {
            // ns1, ns2, np are copied locally to avoid crushing the fields !
            Vector3D ns1, ns2, np;
            ns1 = nsurf1;
            ns2 = nsurf2;
            np = nplan;

            norm = nplan.Crossed(ns1).Magnitude();
            if (norm < Eps)
            {
                norm = 1; // Unsatisfactory, but it is not necessary to stop
            }

            ns1.SetLinearForm(nplan.Dot(ns1) / norm, nplan, -1.0 / norm, ns1);

            norm = nplan.Crossed(ns2).Magnitude();
            if (norm < Eps)
            {
                norm = 1; // Unsatisfactory, but it is not necessary to stop
            }

            ns2.SetLinearForm(nplan.Dot(ns2) / norm, nplan, -1.0 / norm, ns2);

            double maxpiv = 1.0e-14;
            Gauss Resol = new(DEDX, maxpiv);
            istangent = false;
            if (Resol.IsDone)
            {
                math_Vector controle = new(1, 4), solution = new(1, 4), tolerances = new(1, 4);
                GetTolerance(tolerances, Tol);
                Resol.Solve(-DEDT, solution);
                controle.Initialized(DEDT.Added(DEDX.Multiplied(solution)));
                if (Math.Abs(controle[1]) > tolerances[1] || Math.Abs(controle[2]) > tolerances[2]
                    || Math.Abs(controle[3]) > tolerances[3] || Math.Abs(controle[4]) > tolerances[4])
                {
                    // OCCT_DEBUG trace left out ("Cheminement : echec calcul des derivees").
                    istangent = true;
                }

                if (!istangent)
                {
                    tg1.SetLinearForm(solution[1], d1u1, solution[2], d1v1);
                    tg2.SetLinearForm(solution[3], d1u2, solution[4], d1v2);
                    tg12d.SetCoord(solution[1], solution[2]);
                    tg22d.SetCoord(solution[3], solution[4]);
                }
            }
            else
            {
                istangent = true;
            }

            // update of maxang
            if (sg1 > 0.0)
            { // sg1*ray
                ns1.Reverse();
            }

            if (sg2 > 0.0)
            { // sg2*ray
                ns2.Reverse();
            }

            Cosa = ns1.Dot(ns2);
            Sina = nplan.Dot(ns1.Crossed(ns2));
            if (choix % 2 != 0)
            {
                Sina = -Sina; // nplan is changed into -nplan
            }

            if (Cosa > 1.0)
            {
                Cosa = 1.0;
                Sina = 0.0;
            }

            Angle = Math.Acos(Cosa);

            // Reframing on  ]-pi/2, 3pi/2]
            if (Sina < 0.0)
            {
                if (Cosa > 0.0)
                {
                    Angle = -Angle;
                }
                else
                {
                    Angle = 2.0 * M_PI - Angle;
                }
            }

            if (Math.Abs(Angle) > maxang)
            {
                maxang = Math.Abs(Angle);
            }

            if (Math.Abs(Angle) < minang)
            {
                minang = Math.Abs(Angle);
            }

            if (Math.Abs(Angle * ray) < lengthmin)
            {
                lengthmin = Math.Abs(Angle * ray);
            }

            if (Math.Abs(Angle * ray) > lengthmax)
            {
                lengthmax = Math.Abs(Angle * ray);
            }

            distmin = Min(distmin, pts1.Distance(pts2));

            return Ok;
        }

        istangent = true;
        return false;
    }

    /// <summary>The minimal distance between the two extremities of the computed sections.</summary>
    public override double GetMinimalDistance()
    {
        return distmin;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        bool Ok = ComputeValues(X, 0);
        F.Initialized(E);
        return Ok;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        bool Ok = ComputeValues(X, 1);
        D.Initialized(DEDX);
        return Ok;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        bool Ok = ComputeValues(X, 1);
        F.Initialized(E);
        D.Initialized(DEDX);
        return Ok;
    }

    public override Point3D PointOnS1()
    {
        return pts1;
    }

    public override Point3D PointOnS2()
    {
        return pts2;
    }

    public override bool IsTangencyPoint()
    {
        return istangent;
    }

    public override Vector3D TangentOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }

        return tg1;
    }

    public override Vector3D TangentOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }

        return tg2;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }

        return tg12d;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }

        return tg22d;
    }

    public override void Tangent(double U1, double V1, double U2, double V2, out Vector3D TgF, out Vector3D TgL,
                                 out Vector3D NmF, out Vector3D NmL)
    {
        Point3D Center = new Point3D();
        Vector3D ns1;
        double invnorm1;

        if ((U1 != xval[1]) || (V1 != xval[2]) || (U2 != xval[3]) || (V2 != xval[4]))
        {
            surf1.D1(U1, V1, out _, out Vector3D d1u, out Vector3D d1v);
            NmF = ns1 = d1u.Crossed(d1v);
            surf2.D1(U2, V2, out _, out d1u, out d1v);
            NmL = d1u.Crossed(d1v);
        }
        else
        {
            NmF = ns1 = nsurf1;
            NmL = nsurf2;
        }

        invnorm1 = nplan.Crossed(ns1).Magnitude();
        if (invnorm1 < Eps)
        {
            invnorm1 = 1;
        }
        else
        {
            invnorm1 = 1.0 / invnorm1;
        }

        ns1.SetLinearForm(nplan.Dot(ns1) * invnorm1, nplan, -invnorm1, ns1);
        Center.SetXYZ(pts1.XYZ() + sg1 * ray * ns1.XYZ());

        TgF = nplan.Crossed(new Vector3D(Center, pts1));
        TgL = nplan.Crossed(new Vector3D(Center, pts2));
        if (choix % 2 == 1)
        {
            TgF.Reverse();
            TgL.Reverse();
        }
    }

    public override bool TwistOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }

        return tg1.Dot(nplan) < 0.0;
    }

    public override bool TwistOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException();
        }

        return tg2.Dot(nplan) < 0.0;
    }

    /// <summary>
    /// The circular section at <paramref name="Param"/> through the points (U1, V1) and (U2, V2), with its parameter
    /// range: for a quick, approximate view of the surface.
    /// </summary>
    public void Section(double Param, double U1, double V1, double U2, double V2, out double Pdeb, out double Pfin,
                        ref Circle3D C)
    {
        Point3D Center = new Point3D();
        Vector3D ns1, np;

        math_Vector X = new(1, 4);
        X[1] = U1;
        X[2] = V1;
        X[3] = U2;
        X[4] = V2;
        double prm = Param;

        ComputeValues(X, 0, true, prm);

        ns1 = nsurf1;
        np = nplan;

        double norm1;
        norm1 = nplan.Crossed(ns1).Magnitude();
        if (norm1 < Eps)
        {
            norm1 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        ns1.SetLinearForm(nplan.Dot(ns1) / norm1, nplan, -1.0 / norm1, ns1);
        Center.SetXYZ(pts1.XYZ() + sg1 * ray * ns1.XYZ());

        // ns1 is oriented from the center to pts1

        if (sg1 > 0.0)
        {
            ns1.Reverse();
        }

        if (choix % 2 != 0)
        {
            np.Reverse();
        }

        C.SetRadius(Math.Abs(ray));
        C.SetPosition(new Frame3D(Center, new Direction3D(np), new Direction3D(ns1)));
        Pdeb = 0.0;
        Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, pts2);
        // Test negative and almost null angles : Singular Case
        if (Pfin > 1.5 * M_PI)
        {
            np.Reverse();
            C.SetPosition(new Frame3D(Center, new Direction3D(np), new Direction3D(ns1)));
            Pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(C, pts2);
        }

        if (Pfin < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            Pfin += global::Kerf.Engine.Foundation.Precision.PConfusion();
        }
    }

    public override bool IsRational()
    {
        return (mySShape == BlendFunc_Rational || mySShape == BlendFunc_QuasiAngular);
    }

    public override double GetSectionSize()
    {
        return lengthmax;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> Weigths)
    {
        BlendFunc.GetMinimalWeights(mySShape, myTConv, minang, maxang, Weigths);
    }

    public override int NbIntervals(Continuity S)
    {
        int Nb_Int_Courbe, Nb_Int_Loi;
        Nb_Int_Courbe = curv.NbIntervals(BlendFunc.NextShape(S));
        Nb_Int_Loi = fevol.NbIntervals(S);

        if (Nb_Int_Loi == 1)
        {
            return Nb_Int_Courbe;
        }

        NCollection_Array1<double> IntC = new(1, Nb_Int_Courbe + 1);
        NCollection_Array1<double> IntL = new(1, Nb_Int_Loi + 1);
        NCollection_Sequence<double> Inter = new();
        curv.Intervals(IntC, BlendFunc.NextShape(S));
        fevol.Intervals(IntL, S);

        FusionneIntervalles(IntC, IntL, Inter);
        return Inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        int Nb_Int_Courbe, Nb_Int_Loi;
        Nb_Int_Courbe = curv.NbIntervals(BlendFunc.NextShape(S));
        Nb_Int_Loi = fevol.NbIntervals(S);

        if (Nb_Int_Loi == 1)
        {
            curv.Intervals(T, BlendFunc.NextShape(S));
        }
        else
        {
            NCollection_Array1<double> IntC = new(1, Nb_Int_Courbe + 1);
            NCollection_Array1<double> IntL = new(1, Nb_Int_Loi + 1);
            NCollection_Sequence<double> Inter = new();
            curv.Intervals(IntC, BlendFunc.NextShape(S));
            fevol.Intervals(IntL, S);

            FusionneIntervalles(IntC, IntL, Inter);
            for (int ii = 1; ii <= Inter.Length(); ii++)
            {
                T[ii] = Inter[ii];
            }
        }
    }

    private static void FusionneIntervalles(NCollection_Array1<double> I1, NCollection_Array1<double> I2,
                                            NCollection_Sequence<double> Seq)
    {
        int ind1 = 1, ind2 = 1;
        double Epspar = global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99;
        // supposed that positioning works with PConfusion()/2
        double v1, v2;
        // Initialisation : IND1 and IND2 point at the 1st element
        // of each of 2 tables to be processed. INDS points at the last
        // element of TABSOR

        //--- TABSOR is filled by parsing TABLE1 and TABLE2 simultaneously ---
        //------------------ by removing multiple occurrences ------------

        while ((ind1 <= I1.Upper()) && (ind2 <= I2.Upper()))
        {
            v1 = I1[ind1];
            v2 = I2[ind2];
            if (Math.Abs(v1 - v2) <= Epspar)
            {
                // Here elements of I1 and I2 are suitable.
                Seq.Append((v1 + v2) / 2);
                ind1++;
                ind2++;
            }
            else if (v1 < v2)
            {
                // Here the element of I1 is suitable.
                Seq.Append(v1);
                ind1++;
            }
            else
            {
                // Here the element of TABLE2 is suitable.
                Seq.Append(v2);
                ind2++;
            }
        }

        if (ind1 > I1.Upper())
        {
            //----- Here I1 is empty, to be completed with the end of TABLE2 -------
            for (; ind2 <= I2.Upper(); ind2++)
            {
                Seq.Append(I2[ind2]);
            }
        }

        if (ind2 > I2.Upper())
        {
            //----- Here I2 is empty, to be completed with the end of I1 -------
            for (; ind1 <= I1.Upper(); ind1++)
            {
                Seq.Append(I1[ind1]);
            }
        }
    }

    public override void GetShape(out int NbPoles, out int NbKnots, out int Degree, out int NbPoles2d)
    {
        NbPoles2d = 2;
        BlendFunc.GetShape(mySShape, maxang, out NbPoles, out NbKnots, out Degree, ref myTConv);
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determine the Tolerance to be used in approximations.
    //=======================================================================
    public override void GetTolerance(double BoundTol, double SurfTol, double AngleTol, math_Vector Tol3d,
                                      math_Vector Tol1d)
    {
        int low = Tol3d.Lower(), up = Tol3d.Upper();
        double rayon = lengthmin / maxang; // a radius is subtracted
        double Tol;
        Tol = GeomFill.GetTolerance(myTConv, maxang, rayon, AngleTol, SurfTol);
        Tol1d.Init(SurfTol);
        Tol3d.Init(SurfTol);
        Tol3d[low + 1] = Tol3d[up - 1] = Min(Tol, SurfTol);
        Tol3d[low] = Tol3d[up] = Min(Tol, BoundTol);
    }

    public override void Knots(NCollection_Array1<double> TKnots)
    {
        GeomFill.Knots(myTConv, TKnots);
    }

    public override void Mults(NCollection_Array1<int> TMults)
    {
        GeomFill.Mults(myTConv, TMults);
    }

    public override void Section(in Blend_Point P, NCollection_Array1<Point3D> Poles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<double> Weights)
    {
        Point3D Center = new Point3D();
        Vector3D ns1, ns2, np;

        math_Vector X = new(1, 4);
        double prm = P.Parameter();

        int low = Poles.Lower();
        int upp = Poles.Upper();

        P.ParametersOnS1(out X[1], out X[2]);
        P.ParametersOnS2(out X[3], out X[4]);

        // Calculation and storage of distmin
        ComputeValues(X, 0, true, prm);
        distmin = Min(distmin, pts1.Distance(pts2));

        // ns1, ns2, np are copied locally to avoid crashing the fields !
        ns1 = nsurf1;
        ns2 = nsurf2;
        np = nplan;

        Poles2d[Poles2d.Lower()].SetCoord(X[1], X[2]);
        Poles2d[Poles2d.Upper()].SetCoord(X[3], X[4]);

        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = pts1;
            Poles[upp] = pts2;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            return;
        }

        double norm1, norm2;
        norm1 = nplan.Crossed(ns1).Magnitude();
        norm2 = nplan.Crossed(ns2).Magnitude();
        if (norm1 < Eps)
        {
            norm1 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        if (norm2 < Eps)
        {
            norm2 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        ns1.SetLinearForm(nplan.Dot(ns1) / norm1, nplan, -1.0 / norm1, ns1);
        ns2.SetLinearForm(nplan.Dot(ns2) / norm2, nplan, -1.0 / norm2, ns2);

        Center.SetXYZ(pts1.XYZ() + sg1 * ray * ns1.XYZ());

        // ns1 (resp. ns2) is oriented from center to pts1 (resp. pts2),
        // and the trihedron ns1,ns2,nplan is made direct.

        if (sg1 > 0.0)
        {
            ns1.Reverse();
        }

        if (sg2 > 0.0)
        {
            ns2.Reverse();
        }

        if (choix % 2 != 0)
        {
            np.Reverse();
        }

        GeomFill.GetCircle(myTConv, ns1, ns2, np, pts1, pts2, Math.Abs(ray), Center, Poles, Weights);
    }

    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights)
    {
        Vector3D ns1, ns2, np, dnp, dnorm1w = new Vector3D(), dnorm2w = new Vector3D(), tgc = new Vector3D();
        double norm1, norm2, rayprim;

        Point3D Center = new Point3D();
        math_Vector sol = new(1, 4), secmember = new(1, 4);

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();
        bool istgt = true;

        P.ParametersOnS1(out sol[1], out sol[2]);
        P.ParametersOnS2(out sol[3], out sol[4]);

        // Calculation of equations
        ComputeValues(sol, 1, true, prm);
        distmin = Min(distmin, pts1.Distance(pts2));

        // ns1, ns2, np are copied locally to avoid crushing the fields !
        ns1 = nsurf1;
        ns2 = nsurf2;
        np = nplan;
        dnp = dnplan;
        rayprim = dray;

        if (!pts1.IsEqual(pts2, 1.0e-4))
        {
            // Calculation of derived  Normal processing
            Gauss Resol = new(DEDX, 1.0e-9);

            if (Resol.IsDone)
            {
                Resol.Solve(-DEDT, secmember);
                istgt = false;
            }
        }

        if (istgt)
        {
            Svd SingRS = new(DEDX);
            if (SingRS.IsDone)
            {
                SingRS.Solve(-DEDT, secmember, 1.0e-6);
                istgt = false;
            }
        }

        if (!istgt)
        {
            tg1.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
            tg2.SetLinearForm(secmember[3], d1u2, secmember[4], d1v2);

            dnorm1w.SetLinearForm(secmember[1], dndu1, secmember[2], dndv1, dn1w);
            dnorm2w.SetLinearForm(secmember[3], dndu2, secmember[4], dndv2, dn2w);
            istgt = false;
        }

        // Tops 2d
        Poles2d[Poles2d.Lower()].SetCoord(sol[1], sol[2]);
        Poles2d[Poles2d.Upper()].SetCoord(sol[3], sol[4]);
        if (!istgt)
        {
            DPoles2d[Poles2d.Lower()].SetCoord(secmember[1], secmember[2]);
            DPoles2d[Poles2d.Upper()].SetCoord(secmember[3], secmember[4]);
        }

        // the linear case is processed...
        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = pts1;
            Poles[upp] = pts2;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            if (!istgt)
            {
                DPoles[low] = tg1;
                DPoles[upp] = tg2;
                DWeights[low] = 0.0;
                DWeights[upp] = 0.0;
            }

            return (!istgt);
        }

        // Case of the circle
        norm1 = nplan.Crossed(ns1).Magnitude();
        norm2 = nplan.Crossed(ns2).Magnitude();
        if (norm1 < Eps)
        {
            norm1 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        if (norm2 < Eps)
        {
            norm2 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        ns1.SetLinearForm(nplan.Dot(ns1) / norm1, nplan, -1.0 / norm1, ns1);
        ns2.SetLinearForm(nplan.Dot(ns2) / norm2, nplan, -1.0 / norm2, ns2);

        Center.SetXYZ(pts1.XYZ() + sg1 * ray * ns1.XYZ());
        if (!istgt)
        {
            tgc.SetLinearForm(sg1 * ray, dnorm1w, sg1 * dray, ns1, tg1);
        }

        // ns1 is oriented from center to pts1, and  ns2 from center to pts2
        // and the trihedron ns1,ns2,nplan is made direct

        if (sg1 > 0.0)
        {
            ns1.Reverse();
            if (!istgt)
            {
                dnorm1w.Reverse();
            }
        }

        if (sg2 > 0.0)
        {
            ns2.Reverse();
            if (!istgt)
            {
                dnorm2w.Reverse();
            }
        }

        if (choix % 2 != 0)
        {
            np.Reverse();
            dnp.Reverse();
        }

        if (ray < 0.0)
        { // to avoid std::abs(dray) some lines below
            rayprim = -rayprim;
        }

        if (!istgt)
        {
            return GeomFill.GetCircle(myTConv, ns1, ns2, dnorm1w, dnorm2w, np, dnp, pts1, pts2, tg1, tg2,
                                      Math.Abs(ray), rayprim, Center, tgc, Poles, DPoles, Weights, DWeights);
        }
        else
        {
            GeomFill.GetCircle(myTConv, ns1, ns2, np, pts1, pts2, Math.Abs(ray), Center, Poles, Weights);
            return false;
        }
    }

    public override bool Section(in Blend_Point P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                                 NCollection_Array1<Vector3D> D2Poles, NCollection_Array1<Point2D> Poles2d,
                                 NCollection_Array1<Vector2D> DPoles2d, NCollection_Array1<Vector2D> D2Poles2d,
                                 NCollection_Array1<double> Weights, NCollection_Array1<double> DWeights,
                                 NCollection_Array1<double> D2Weights)
    {
        Vector3D ns1, ns2, np, dnp, d2np, dnorm1w = new Vector3D(), dnorm2w = new Vector3D(), d2norm1w = new Vector3D(),
               d2norm2w = new Vector3D();
        Vector3D tgc = new Vector3D(), dtgc = new Vector3D(), dtg1 = new Vector3D(), dtg2 = new Vector3D(),
               temp = new Vector3D(), tempbis = new Vector3D();
        double norm1, norm2, rayprim, raysecn;

        Point3D Center = new Point3D();
        math_Vector X = new(1, 4), sol = new(1, 4), secmember = new(1, 4);
        Matrix D2DXdSdt = new(1, 4, 1, 4);

        double prm = P.Parameter();
        int low = Poles.Lower();
        int upp = Poles.Upper();
        bool istgt = true;

        P.ParametersOnS1(out X[1], out X[2]);
        P.ParametersOnS2(out X[3], out X[4]);

        // A commented-out OCCT_DEBUG check of the derivatives by finite differences is left out.

        // Calculation of equations
        ComputeValues(X, 2, true, prm);
        distmin = Min(distmin, pts1.Distance(pts2));

        // ns1, ns2, np are copied locally to avois crushing the fields !
        ns1 = nsurf1;
        ns2 = nsurf2;
        np = nplan;
        dnp = dnplan;
        d2np = d2nplan;
        rayprim = dray;
        raysecn = d2ray;

        // Calculation of derivatives

        if (!pts1.IsEqual(pts2, 1.0e-4))
        {
            Gauss Resol = new(DEDX, 1.0e-9); // Precise tolerance !!!!!
            // Calculation of derivatives Processing Normal
            if (Resol.IsDone)
            {
                Resol.Solve(-DEDT, sol);
                D2EDX2.Multiply(sol, D2DXdSdt);
                secmember.Initialized(-(D2EDT2 + (2 * D2EDXDT + D2DXdSdt) * sol));
                Resol.Solve(secmember);
                istgt = false;
            }
        }

        if (istgt)
        {
            Svd SingRS = new(DEDX);
            math_Vector Vbis = new(1, 4);
            if (SingRS.IsDone)
            {
                SingRS.Solve(-DEDT, sol, 1.0e-6);
                D2EDX2.Multiply(sol, D2DXdSdt);
                Vbis.Initialized(-(D2EDT2 + (2 * D2EDXDT + D2DXdSdt) * sol));
                SingRS.Solve(Vbis, secmember, 1.0e-6);
                istgt = false;
            }
        }

        if (!istgt)
        {
            tg1.SetLinearForm(sol[1], d1u1, sol[2], d1v1);
            tg2.SetLinearForm(sol[3], d1u2, sol[4], d1v2);

            dnorm1w.SetLinearForm(sol[1], dndu1, sol[2], dndv1, dn1w);
            dnorm2w.SetLinearForm(sol[3], dndu2, sol[4], dndv2, dn2w);
            temp.SetLinearForm(sol[1] * sol[1], d2u1, 2 * sol[1] * sol[2], d2uv1, sol[2] * sol[2], d2v1);

            dtg1.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1, temp);

            temp.SetLinearForm(sol[3] * sol[3], d2u2, 2 * sol[3] * sol[4], d2uv2, sol[4] * sol[4], d2v2);
            dtg2.SetLinearForm(secmember[3], d1u2, secmember[4], d1v2, temp);

            temp.SetLinearForm(sol[1] * sol[1], d2ndu1, 2 * sol[1] * sol[2], d2nduv1, sol[2] * sol[2], d2ndv1);

            tempbis.SetLinearForm(2 * sol[1], d2ndtu1, 2 * sol[2], d2ndtv1, d2n1w);
            temp += tempbis;
            d2norm1w.SetLinearForm(secmember[1], dndu1, secmember[2], dndv1, temp);

            temp.SetLinearForm(sol[3] * sol[3], d2ndu2, 2 * sol[3] * sol[4], d2nduv2, sol[4] * sol[4], d2ndv2);
            tempbis.SetLinearForm(2 * sol[3], d2ndtu2, 2 * sol[4], d2ndtv2, d2n2w);
            temp += tempbis;
            d2norm2w.SetLinearForm(secmember[3], dndu2, secmember[4], dndv2, temp);
        }

        // Tops 2d
        Poles2d[Poles2d.Lower()].SetCoord(X[1], X[2]);
        Poles2d[Poles2d.Upper()].SetCoord(X[3], X[4]);
        if (!istgt)
        {
            DPoles2d[Poles2d.Lower()].SetCoord(sol[1], sol[2]);
            DPoles2d[Poles2d.Upper()].SetCoord(sol[3], sol[4]);
            D2Poles2d[Poles2d.Lower()].SetCoord(secmember[1], secmember[2]);
            D2Poles2d[Poles2d.Upper()].SetCoord(secmember[3], secmember[4]);
        }

        // linear case is processed...
        if (mySShape == BlendFunc_Linear)
        {
            Poles[low] = pts1;
            Poles[upp] = pts2;
            Weights[low] = 1.0;
            Weights[upp] = 1.0;
            if (!istgt)
            {
                DPoles[low] = tg1;
                DPoles[upp] = tg2;
                DPoles[low] = dtg1; // OCCT writes DPoles twice and never D2Poles
                DPoles[upp] = dtg2;
                DWeights[low] = 0.0;
                DWeights[upp] = 0.0;
                D2Weights[low] = 0.0;
                D2Weights[upp] = 0.0;
            }

            return (!istgt);
        }

        // Case of circle
        norm1 = nplan.Crossed(ns1).Magnitude();
        norm2 = nplan.Crossed(ns2).Magnitude();
        if (norm1 < Eps)
        {
            norm1 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        if (norm2 < Eps)
        {
            norm2 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        ns1.SetLinearForm(nplan.Dot(ns1) / norm1, nplan, -1.0 / norm1, ns1);
        ns2.SetLinearForm(nplan.Dot(ns2) / norm2, nplan, -1.0 / norm2, ns2);

        Center.SetXYZ(pts1.XYZ() + sg1 * ray * ns1.XYZ());
        if (!istgt)
        {
            tgc.SetLinearForm(sg1 * ray, dnorm1w, sg1 * dray, ns1, tg1);
            dtgc.SetLinearForm(sg1 * ray, d2norm1w, 2 * sg1 * dray, dnorm1w, sg1 * d2ray, ns1);
            dtgc += dtg1;
        }

        // ns1 is oriented from the center to pts1, and ns2 from the center to pts2
        // and the trihedron ns1,ns2,nplan is made direct

        if (sg1 > 0.0)
        {
            ns1.Reverse();
            if (!istgt)
            {
                dnorm1w.Reverse();
                d2norm1w.Reverse();
            }
        }

        if (sg2 > 0.0)
        {
            ns2.Reverse();
            if (!istgt)
            {
                dnorm2w.Reverse();
                d2norm2w.Reverse();
            }
        }

        if (choix % 2 != 0)
        {
            np.Reverse();
            dnp.Reverse();
            d2np.Reverse();
        }

        if (ray < 0.0)
        { // to avoid std::abs(dray) several lines below
            rayprim = -rayprim;
            raysecn = -raysecn;
        }

        if (!istgt)
        {
            return GeomFill.GetCircle(myTConv, ns1, ns2, dnorm1w, dnorm2w, d2norm1w, d2norm2w, np, dnp, d2np, pts1,
                                      pts2, tg1, tg2, dtg1, dtg2, Math.Abs(ray), rayprim, raysecn, Center, tgc,
                                      dtgc, Poles, DPoles, D2Poles, Weights, DWeights, D2Weights);
        }
        else
        {
            GeomFill.GetCircle(myTConv, ns1, ns2, nplan, pts1, pts2, Math.Abs(ray), Center, Poles, Weights);
            return false;
        }
    }


    public override void Resolution(int IC2d, double Tol, out double TolU, out double TolV)
    {
        if (IC2d == 1)
        {
            TolU = surf1.UResolution(Tol);
            TolV = surf1.VResolution(Tol);
        }
        else
        {
            TolU = surf2.UResolution(Tol);
            TolV = surf2.VResolution(Tol);
        }
    }
}
