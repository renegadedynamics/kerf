// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Occt.BlendFunc_SectionShape;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Engine.Geometry.NormalStatus;
using static Kerf.Geometry.Continuity;

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
                                out int Degree, ref ParameterisationType TConv)
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
                        TConv = TgtThetaOver2_1;
                    }
                    else
                    { // QuasiAngular affin d'etre C1 (et meme beaucoup plus)
                        NbPoles = 7;
                        NbKnots = 2;
                        Degree = 6;
                        TConv = QuasiAngular;
                    }
                }
                break;
            case BlendFunc_QuasiAngular:
                {
                    NbPoles = 7;
                    NbKnots = 2;
                    Degree = 6;
                    TConv = QuasiAngular;
                }
                break;
            case BlendFunc_Polynomial:
                {
                    NbPoles = 8;
                    NbKnots = 2;
                    Degree = 7;
                    TConv = Polynomial;
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
    public static void GetMinimalWeights(BlendFunc_SectionShape SShape, ParameterisationType TConv,
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
                    Frame3D popAx2 = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
                    Circle3D C = new Circle3D(popAx2, 1);
                    Geom_TrimmedCurve Sect1 = new Geom_TrimmedCurve(new Geom_Circle(C), 0.0, MaxAng);
                    Geom_BSplineCurve CtoBspl = GeomConvert.CurveToBSplineCurve(Sect1, TConv);
                    Weights.Assign(CtoBspl.InternalWeightsArray());
                    double angle_min = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), MinAng);
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
    public static Continuity NextShape(Continuity S)
    {
        switch (S)
        {
            case C0:
                return C1;
            case C1:
                return C2;
            case C2:
                return C3;
            default:
                break;
        }

        return CN;
    }

    /// <summary>
    /// The normal of <paramref name="Surf"/> at <paramref name="p2d"/> from its higher derivatives, where the first
    /// ones are parallel (a singular point); false when it is not defined.
    /// </summary>
    public static bool ComputeNormal(Adaptor3d_Surface Surf, in Point2D p2d, ref Vector3D Normal)
    {
        const int MaxOrder = 3;
        double U = p2d.X();
        double V = p2d.Y();

        int i, j;

        NCollection_Array2<Vector3D> DerSurf = new(0, MaxOrder + 1, 0, MaxOrder + 1);
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

        NCollection_Array2<Vector3D> DerNUV = new(0, MaxOrder, 0, MaxOrder);
        for (i = 0; i <= MaxOrder; i++)
        {
            for (j = 0; j <= MaxOrder; j++)
            {
                DerNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, DerSurf));
            }
        }

        Direction3D thenormal = new Direction3D();
        double Umin = Surf.FirstUParameter();
        double Umax = Surf.LastUParameter();
        double Vmin = Surf.FirstVParameter();
        double Vmax = Surf.LastVParameter(); // szv: was FirstVParameter!
        global::Kerf.Engine.Geometry.CSLib.Normal(MaxOrder, DerNUV, 1.0e-9, U, V, Umin, Umax, Vmin, Vmax, out NormalStatus stat,
                     ref thenormal, out _, out _);
        if (stat == Defined)
        {
            Normal.SetXYZ(thenormal.XYZ());
            return true;
        }

        return false;
    }

    /// <summary>As <see cref="ComputeNormal"/>, with the normal's derivatives in U and V.</summary>
    public static bool ComputeDNormal(Adaptor3d_Surface Surf, in Point2D p2d, ref Vector3D Normal, ref Vector3D DNu,
                                      ref Vector3D DNv)
    {
        const int MaxOrder = 3;
        double U = p2d.X();
        double V = p2d.Y();

        int i, j;

        NCollection_Array2<Vector3D> DerSurf = new(0, MaxOrder + 1, 0, MaxOrder + 1);
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

        NCollection_Array2<Vector3D> DerNUV = new(0, MaxOrder, 0, MaxOrder);
        for (i = 0; i <= MaxOrder; i++)
        {
            for (j = 0; j <= MaxOrder; j++)
            {
                DerNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, DerSurf));
            }
        }

        Direction3D thenormal = new Direction3D();
        double Umin = Surf.FirstUParameter();
        double Umax = Surf.LastUParameter();
        double Vmin = Surf.FirstVParameter();
        double Vmax = Surf.LastVParameter(); // szv: was FirstVParameter!
        global::Kerf.Engine.Geometry.CSLib.Normal(MaxOrder, DerNUV, 1.0e-9, U, V, Umin, Umax, Vmin, Vmax, out NormalStatus stat,
                     ref thenormal, out int OrderU, out int OrderV);
        if (stat == Defined)
        {
            Normal.SetXYZ(thenormal.XYZ());
            DNu = global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 0, DerNUV, OrderU, OrderV);
            DNv = global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 1, DerNUV, OrderU, OrderV);
            return true;
        }

        return false;
    }
}
