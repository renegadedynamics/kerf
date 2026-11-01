// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BlendFunc_SectionShape;
using static Occt.Convert_ParameterisationType;
using static Occt.CSLib_NormalStatus;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The functions that compute blends between two surfaces (constant radius, evolving radius, ruled, chamfers), and
/// their shared tools.
/// </summary>
/// <remarks>
/// OCCT declares <c>BlendFunc::Knots</c> and <c>BlendFunc::Mults</c> but never defines them; they are left out.
/// </remarks>
public static class BlendFunc
{
    // 09/09/1996 PMN Ajout des methodes GetCircle et GetTolerance.
    // 30/12/1996 PMN Ajout de GetMinimalWeight
    // 23/09/1997 PMN Supprimme GetCircle et GetTol (passe dans GeomFill)

    /// <summary>
    /// The shape of the B-spline sections of type <paramref name="SShape"/> for arcs up to <paramref name="MaxAng"/>,
    /// and the conversion that makes them (left unchanged for <see cref="BlendFunc_Linear"/>).
    /// </summary>
    public static void GetShape(BlendFunc_SectionShape SShape, double MaxAng, out int NbPoles, out int NbKnots,
                                out int Degree, ref Convert_ParameterisationType TConv)
    {
        NbPoles = 0;
        NbKnots = 0;
        Degree = 0;
        switch (SShape)
        {
            case BlendFunc_Rational:
                {
                    int NbSpan = (int)(Math.Ceiling(3.0 * Math.Abs(MaxAng) / 2.0 / M_PI));
                    NbPoles = 2 * NbSpan + 1;
                    NbKnots = NbSpan + 1;
                    Degree = 2;
                    if (NbSpan == 1)
                    {
                        TConv = Convert_TgtThetaOver2_1;
                    }
                    else
                    { // QuasiAngular affin d'etre C1 (et meme beaucoup plus)
                        NbPoles = 7;
                        NbKnots = 2;
                        Degree = 6;
                        TConv = Convert_QuasiAngular;
                    }
                }
                break;
            case BlendFunc_QuasiAngular:
                {
                    NbPoles = 7;
                    NbKnots = 2;
                    Degree = 6;
                    TConv = Convert_QuasiAngular;
                }
                break;
            case BlendFunc_Polynomial:
                {
                    NbPoles = 8;
                    NbKnots = 2;
                    Degree = 7;
                    TConv = Convert_Polynomial;
                }
                break;
            case BlendFunc_Linear:
                {
                    NbPoles = 2;
                    NbKnots = 2;
                    Degree = 1;
                }
                break;
        }
    }

    //=======================================================================
    // function : GetMinimalWeights
    // purpose  : On suppose les extremum de poids sont obtenus pour les
    //           extremums d'angles (A verifier eventuelement pour Quasi-Angular)
    //=======================================================================

    /// <summary>The smallest weights of the sections between the angles <paramref name="MinAng"/> and <paramref name="MaxAng"/>.</summary>
    public static void GetMinimalWeights(BlendFunc_SectionShape SShape, Convert_ParameterisationType TConv,
                                         double MinAng, double MaxAng, NCollection_Array1<double> Weights)
    {
        switch (SShape)
        {
            case BlendFunc_Polynomial:
            case BlendFunc_Linear:
                {
                    Weights.Init(1);
                }
                break;
            case BlendFunc_Rational:
            case BlendFunc_QuasiAngular:
                {
                    gp_Ax2 popAx2 = new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z));
                    gp_Circ C = new gp_Circ(popAx2, 1);
                    Geom_TrimmedCurve Sect1 = new Geom_TrimmedCurve(new Geom_Circle(C), 0.0, MaxAng);
                    Geom_BSplineCurve CtoBspl = GeomConvert.CurveToBSplineCurve(Sect1, TConv);
                    Weights.Assign(CtoBspl.InternalWeightsArray());
                    double angle_min = Max(Precision.PConfusion(), MinAng);
                    Geom_TrimmedCurve Sect2 = new Geom_TrimmedCurve(new Geom_Circle(C), 0.0, angle_min);
                    CtoBspl = GeomConvert.CurveToBSplineCurve(Sect2, TConv);
                    NCollection_Array1<double> poids = CtoBspl.InternalWeightsArray();
                    for (int ii = Weights.Lower(); ii <= Weights.Upper(); ii++)
                    {
                        if (poids[ii] < Weights[ii])
                        {
                            Weights[ii] = poids[ii];
                        }
                    }
                }
                break;
        }
    }

    /// <summary>The next level of continuity.</summary>
    public static GeomAbs_Shape NextShape(GeomAbs_Shape S)
    {
        switch (S)
        {
            case GeomAbs_C0:
                return GeomAbs_C1;
            case GeomAbs_C1:
                return GeomAbs_C2;
            case GeomAbs_C2:
                return GeomAbs_C3;
            default:
                break;
        }

        return GeomAbs_CN;
    }

    /// <summary>
    /// The normal of <paramref name="Surf"/> at <paramref name="p2d"/> from its higher derivatives, where the first
    /// ones are parallel (a singular point); false when it is not defined.
    /// </summary>
    public static bool ComputeNormal(Adaptor3d_Surface Surf, in gp_Pnt2d p2d, ref gp_Vec Normal)
    {
        const int MaxOrder = 3;
        double U = p2d.X();
        double V = p2d.Y();

        int i, j;

        NCollection_Array2<gp_Vec> DerSurf = new(0, MaxOrder + 1, 0, MaxOrder + 1);
        for (i = 1; i <= MaxOrder + 1; i++)
        {
            DerSurf.SetValue(i, 0, Surf.DN(U, V, i, 0));
        }

        for (i = 0; i <= MaxOrder + 1; i++)
        {
            for (j = 1; j <= MaxOrder + 1; j++)
            {
                DerSurf.SetValue(i, j, Surf.DN(U, V, i, j));
            }
        }

        NCollection_Array2<gp_Vec> DerNUV = new(0, MaxOrder, 0, MaxOrder);
        for (i = 0; i <= MaxOrder; i++)
        {
            for (j = 0; j <= MaxOrder; j++)
            {
                DerNUV.SetValue(i, j, CSLib.DNNUV(i, j, DerSurf));
            }
        }

        gp_Dir thenormal = new gp_Dir();
        double Umin = Surf.FirstUParameter();
        double Umax = Surf.LastUParameter();
        double Vmin = Surf.FirstVParameter();
        double Vmax = Surf.LastVParameter(); // szv: was FirstVParameter!
        CSLib.Normal(MaxOrder, DerNUV, 1.0e-9, U, V, Umin, Umax, Vmin, Vmax, out CSLib_NormalStatus stat,
                     ref thenormal, out _, out _);
        if (stat == CSLib_Defined)
        {
            Normal.SetXYZ(thenormal.XYZ());
            return true;
        }

        return false;
    }

    /// <summary>As <see cref="ComputeNormal"/>, with the normal's derivatives in U and V.</summary>
    public static bool ComputeDNormal(Adaptor3d_Surface Surf, in gp_Pnt2d p2d, ref gp_Vec Normal, ref gp_Vec DNu,
                                      ref gp_Vec DNv)
    {
        const int MaxOrder = 3;
        double U = p2d.X();
        double V = p2d.Y();

        int i, j;

        NCollection_Array2<gp_Vec> DerSurf = new(0, MaxOrder + 1, 0, MaxOrder + 1);
        for (i = 1; i <= MaxOrder + 1; i++)
        {
            DerSurf.SetValue(i, 0, Surf.DN(U, V, i, 0));
        }

        for (i = 0; i <= MaxOrder + 1; i++)
        {
            for (j = 1; j <= MaxOrder + 1; j++)
            {
                DerSurf.SetValue(i, j, Surf.DN(U, V, i, j));
            }
        }

        NCollection_Array2<gp_Vec> DerNUV = new(0, MaxOrder, 0, MaxOrder);
        for (i = 0; i <= MaxOrder; i++)
        {
            for (j = 0; j <= MaxOrder; j++)
            {
                DerNUV.SetValue(i, j, CSLib.DNNUV(i, j, DerSurf));
            }
        }

        gp_Dir thenormal = new gp_Dir();
        double Umin = Surf.FirstUParameter();
        double Umax = Surf.LastUParameter();
        double Vmin = Surf.FirstVParameter();
        double Vmax = Surf.LastVParameter(); // szv: was FirstVParameter!
        CSLib.Normal(MaxOrder, DerNUV, 1.0e-9, U, V, Umin, Umax, Vmin, Vmax, out CSLib_NormalStatus stat,
                     ref thenormal, out int OrderU, out int OrderV);
        if (stat == CSLib_Defined)
        {
            Normal.SetXYZ(thenormal.XYZ());
            DNu = CSLib.DNNormal(1, 0, DerNUV, OrderU, OrderV);
            DNv = CSLib.DNNormal(0, 1, DerNUV, OrderU, OrderV);
            return true;
        }

        return false;
    }
}
