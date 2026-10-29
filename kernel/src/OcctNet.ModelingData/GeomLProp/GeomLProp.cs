// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The continuity at the junction of two 3D curves (or portions of curves) built by concatenation.
/// </summary>
public static class GeomLProp
{
    private static int GeomAbsToInteger(GeomAbs_Shape gcont)
    {
        int cont = 0;
        switch (gcont)
        {
            case GeomAbs_C0:
                cont = 0;
                break;
            case GeomAbs_G1:
                cont = 1;
                break;
            case GeomAbs_C1:
                cont = 2;
                break;
            case GeomAbs_G2:
                cont = 3;
                break;
            case GeomAbs_C2:
                cont = 4;
                break;
            case GeomAbs_C3:
                cont = 5;
                break;
            case GeomAbs_CN:
                cont = 6;
                break;
        }

        return cont;
    }

    /// <summary>
    /// The regularity at the junction of C1 at u1 and C2 at u2 (r1, r2: whether each curve is taken reversed), with
    /// the linear and angular tolerances tl and ta used to compare the derivatives: C0, G1, C1 or C2. Throws
    /// Standard_Failure if the points at u1 and u2 are not confused.
    /// </summary>
    public static GeomAbs_Shape Continuity(Geom_Curve C1,
                                           Geom_Curve C2,
                                           double u1,
                                           double u2,
                                           bool r1,
                                           bool r2,
                                           double tl,
                                           double ta)
    {
        GeomAbs_Shape cont = GeomAbs_C0;
        int index1, index2;
        double tolerance;
        bool fini = false;
        gp_Vec d1, d2;
        gp_Dir dir1, dir2;
        int cont1, cont2;
        GeomAbs_Shape gcont1 = C1.Continuity(), gcont2 = C2.Continuity();
        cont1 = GeomAbsToInteger(gcont1);
        cont2 = GeomAbsToInteger(gcont2);

        Geom_Curve aCurve1 = C1;
        Geom_Curve aCurve2 = C2;
        if (C1.IsKind(typeof(Geom_TrimmedCurve)))
        {
            Geom_TrimmedCurve aTrimmed = (Geom_TrimmedCurve)aCurve1;
            aCurve1 = aTrimmed.BasisCurve();
        }

        if (C2.IsKind(typeof(Geom_TrimmedCurve)))
        {
            Geom_TrimmedCurve aTrimmed = (Geom_TrimmedCurve)aCurve2;
            aCurve2 = aTrimmed.BasisCurve();
        }

        // OCCT: Degree() - Multiplicity() is the order of continuity at the knot (0 for C0, 1 for C1, 2 for C2), but
        // it is compared below with GeomAbsToInteger's scale (2 for C1, 4 for C2): a C1 knot gives n = 0 and a C2
        // knot n = 1, so a junction at an inner knot is reported one or two levels lower than it is
        if (aCurve1.IsKind(typeof(Geom_BSplineCurve)))
        {
            Geom_BSplineCurve BSplineCurve = (Geom_BSplineCurve)aCurve1;
            BSplineCurve.Resolution(tl, out tolerance);
            BSplineCurve.LocateU(u1, tolerance, out index1, out index2);

            if (index1 > 1 && index2 < BSplineCurve.NbKnots() && index1 == index2)
            {
                cont1 = BSplineCurve.Degree() - BSplineCurve.Multiplicity(index1);
            }
            else
            {
                cont1 = 5;
            }
        }

        if (aCurve2.IsKind(typeof(Geom_BSplineCurve)))
        {
            Geom_BSplineCurve BSplineCurve = (Geom_BSplineCurve)aCurve2;
            BSplineCurve.Resolution(tl, out tolerance);
            BSplineCurve.LocateU(u2, tolerance, out index1, out index2);

            if (index1 > 1 && index2 < BSplineCurve.NbKnots() && index1 == index2)
            {
                cont2 = BSplineCurve.Degree() - BSplineCurve.Multiplicity(index1);
            }
            else
            {
                cont2 = 5;
            }
        }

        int n1 = 0, n2 = 0;
        if (cont1 >= 5)
        {
            n1 = 3;
        }
        else if (cont1 == 4)
        {
            n1 = 2;
        }
        else if (cont1 == 2)
        {
            n1 = 1;
        }

        if (cont2 >= 5)
        {
            n2 = 3;
        }
        else if (cont2 == 4)
        {
            n2 = 2;
        }
        else if (cont2 == 2)
        {
            n2 = 1;
        }

        var clp1 = new GeomLProp_CLProps(C1, u1, n1, tl);
        var clp2 = new GeomLProp_CLProps(C2, u2, n2, tl);
        if (!(clp1.Value().IsEqual(clp2.Value(), tl)))
        {
            throw new Standard_Failure("Courbes non jointives");
        }

        int min = Min(n1, n2);
        if (min >= 1)
        {
            d1 = clp1.D1();
            d2 = clp2.D1();
            if (r1)
            {
                d1.Reverse();
            }

            if (r2)
            {
                d2.Reverse();
            }

            if (d1.IsEqual(d2, tl, ta))
            {
                cont = GeomAbs_C1;
            }
            else if (clp1.IsTangentDefined() && clp2.IsTangentDefined())
            {
                clp1.Tangent(out dir1);
                clp2.Tangent(out dir2);
                if (r1)
                {
                    dir1.Reverse();
                }

                if (r2)
                {
                    dir2.Reverse();
                }

                if (dir1.IsEqual(dir2, ta))
                {
                    cont = GeomAbs_G1;
                }

                fini = true;
            }
            else
            {
                fini = true;
            }
        }

        if (min >= 2 && !fini)
        {
            d1 = clp1.D2();
            d2 = clp2.D2();
            if (d1.IsEqual(d2, tl, ta))
            {
                cont = GeomAbs_C2;
            }
        }

        return cont;
    }

    /// <summary>
    /// <see cref="Continuity(Geom_Curve, Geom_Curve, double, double, bool, bool, double, double)"/> with the
    /// tolerances Precision::Confusion() and Precision::Angular().
    /// </summary>
    public static GeomAbs_Shape Continuity(Geom_Curve C1, Geom_Curve C2, double u1, double u2, bool r1, bool r2)
    {
        return Continuity(C1, C2, u1, u2, r1, r2, Precision.Confusion(), Precision.Angular());
    }
}
