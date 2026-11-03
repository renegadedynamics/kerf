// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Fillets.BlendFunctions.SectionShape;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Engine.Geometry.NormalStatus;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Fillets.BlendFunctions;

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
    /// The shape of the B-spline sections of type <paramref name="sShape"/> for arcs up to <paramref name="maxAng"/>,
    /// and the conversion that makes them (left unchanged for <see cref="Linear"/>).
    /// </summary>
    public static void GetShape(SectionShape sShape, double maxAng, out int nbPoles, out int nbKnots,
                                out int degree, ref ParameterisationType tConv)
    {
        nbPoles = 0;
        nbKnots = 0;
        degree = 0;
        switch (sShape)
        {
            case SectionShape.Rational:
                {
                    int nbSpan = (int)(Math.Ceiling(3.0 * Math.Abs(maxAng) / 2.0 / M_PI));
                    nbPoles = 2 * nbSpan + 1;
                    nbKnots = nbSpan + 1;
                    degree = 2;
                    if (nbSpan == 1)
                    {
                        tConv = TgtThetaOver2_1;
                    }
                    else
                    { // QuasiAngular affin d'etre C1 (et meme beaucoup plus)
                        nbPoles = 7;
                        nbKnots = 2;
                        degree = 6;
                        tConv = global::Kerf.Engine.Geometry.Conversion.ParameterisationType.QuasiAngular;
                    }
                }
                break;
            case SectionShape.QuasiAngular:
                {
                    nbPoles = 7;
                    nbKnots = 2;
                    degree = 6;
                    tConv = global::Kerf.Engine.Geometry.Conversion.ParameterisationType.QuasiAngular;
                }
                break;
            case SectionShape.Polynomial:
                {
                    nbPoles = 8;
                    nbKnots = 2;
                    degree = 7;
                    tConv = global::Kerf.Engine.Geometry.Conversion.ParameterisationType.Polynomial;
                }
                break;
            case SectionShape.Linear:
                {
                    nbPoles = 2;
                    nbKnots = 2;
                    degree = 1;
                }
                break;
        }
    }

    //=======================================================================
    // function : GetMinimalWeights
    // purpose  : On suppose les extremum de poids sont obtenus pour les
    //           extremums d'angles (A verifier eventuelement pour Quasi-Angular)
    //=======================================================================

    /// <summary>The smallest weights of the sections between the angles <paramref name="minAng"/> and <paramref name="maxAng"/>.</summary>
    public static void GetMinimalWeights(SectionShape sShape, ParameterisationType tConv,
                                         double minAng, double maxAng, NCollection_Array1<double> weights)
    {
        switch (sShape)
        {
            case SectionShape.Polynomial:
            case SectionShape.Linear:
                {
                    weights.Init(1);
                }
                break;
            case SectionShape.Rational:
            case SectionShape.QuasiAngular:
                {
                    Frame3D popAx2 = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
                    Circle3D c = new Circle3D(popAx2, 1);
                    TrimmedCurve3D sect1 = new TrimmedCurve3D(new CircleCurve3D(c), 0.0, maxAng);
                    BSplineCurve3D ctoBspl = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(sect1, tConv);
                    weights.Assign(ctoBspl.InternalWeightsArray());
                    double angle_min = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), minAng);
                    TrimmedCurve3D sect2 = new TrimmedCurve3D(new CircleCurve3D(c), 0.0, angle_min);
                    ctoBspl = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(sect2, tConv);
                    NCollection_Array1<double> poids = ctoBspl.InternalWeightsArray();
                    for (int ii = weights.Lower(); ii <= weights.Upper(); ii++)
                    {
                        if (poids[ii] < weights[ii])
                        {
                            weights[ii] = poids[ii];
                        }
                    }
                }
                break;
        }
    }

    /// <summary>The next level of continuity.</summary>
    public static Continuity NextShape(Continuity s)
    {
        switch (s)
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
    /// The normal of <paramref name="surf"/> at <paramref name="p2d"/> from its higher derivatives, where the first
    /// ones are parallel (a singular point); false when it is not defined.
    /// </summary>
    public static bool ComputeNormal(SurfaceAdaptorBase surf, in Point2D p2d, ref Vector3D normal)
    {
        const int maxOrder = 3;
        double u = p2d.X();
        double v = p2d.Y();

        int i, j;

        NCollection_Array2<Vector3D> derSurf = new(0, maxOrder + 1, 0, maxOrder + 1);
        for (i = 1; i <= maxOrder + 1; i++)
        {
            derSurf.SetValue(i, 0, surf.DN(u, v, i, 0));
        }

        for (i = 0; i <= maxOrder + 1; i++)
        {
            for (j = 1; j <= maxOrder + 1; j++)
            {
                derSurf.SetValue(i, j, surf.DN(u, v, i, j));
            }
        }

        NCollection_Array2<Vector3D> derNUV = new(0, maxOrder, 0, maxOrder);
        for (i = 0; i <= maxOrder; i++)
        {
            for (j = 0; j <= maxOrder; j++)
            {
                derNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, derSurf));
            }
        }

        Direction3D thenormal = new Direction3D();
        double umin = surf.FirstUParameter();
        double umax = surf.LastUParameter();
        double vmin = surf.FirstVParameter();
        double vmax = surf.LastVParameter(); // szv: was FirstVParameter!
        global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder, derNUV, 1.0e-9, u, v, umin, umax, vmin, vmax, out NormalStatus stat,
                     ref thenormal, out _, out _);
        if (stat == Defined)
        {
            normal.SetXYZ(thenormal.XYZ());
            return true;
        }

        return false;
    }

    /// <summary>As <see cref="ComputeNormal"/>, with the normal's derivatives in U and V.</summary>
    public static bool ComputeDNormal(SurfaceAdaptorBase surf, in Point2D p2d, ref Vector3D normal, ref Vector3D dNu,
                                      ref Vector3D dNv)
    {
        const int maxOrder = 3;
        double u = p2d.X();
        double v = p2d.Y();

        int i, j;

        NCollection_Array2<Vector3D> derSurf = new(0, maxOrder + 1, 0, maxOrder + 1);
        for (i = 1; i <= maxOrder + 1; i++)
        {
            derSurf.SetValue(i, 0, surf.DN(u, v, i, 0));
        }

        for (i = 0; i <= maxOrder + 1; i++)
        {
            for (j = 1; j <= maxOrder + 1; j++)
            {
                derSurf.SetValue(i, j, surf.DN(u, v, i, j));
            }
        }

        NCollection_Array2<Vector3D> derNUV = new(0, maxOrder, 0, maxOrder);
        for (i = 0; i <= maxOrder; i++)
        {
            for (j = 0; j <= maxOrder; j++)
            {
                derNUV.SetValue(i, j, global::Kerf.Engine.Geometry.CSLib.DNNUV(i, j, derSurf));
            }
        }

        Direction3D thenormal = new Direction3D();
        double umin = surf.FirstUParameter();
        double umax = surf.LastUParameter();
        double vmin = surf.FirstVParameter();
        double vmax = surf.LastVParameter(); // szv: was FirstVParameter!
        global::Kerf.Engine.Geometry.CSLib.Normal(maxOrder, derNUV, 1.0e-9, u, v, umin, umax, vmin, vmax, out NormalStatus stat,
                     ref thenormal, out int orderU, out int orderV);
        if (stat == Defined)
        {
            normal.SetXYZ(thenormal.XYZ());
            dNu = global::Kerf.Engine.Geometry.CSLib.DNNormal(1, 0, derNUV, orderU, orderV);
            dNv = global::Kerf.Engine.Geometry.CSLib.DNNormal(0, 1, derNUV, orderU, orderV);
            return true;
        }

        return false;
    }
}
