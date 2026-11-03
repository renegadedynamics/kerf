// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ComputeTangence.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ComputeTangence.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Implicit.ConstIsoparametric;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// OCCT's free functions <c>ChoixRef</c> and <c>IntImp_ComputeTangence</c> (the second is
/// <see cref="ComputeTangence"/> here: C# can't name a member as its class).
/// </summary>
public static class Tangence
{
    private static readonly ConstIsoparametric[] StaticChoixRef =
    [
        ConstIsoparametric.UIsoparametricOnCaro1,
        ConstIsoparametric.VIsoparametricOnCaro1,
        ConstIsoparametric.UIsoparametricOnCaro2,
        ConstIsoparametric.VIsoparametricOnCaro2,
    ];

    /// <summary>The isoparametric of rank <paramref name="index"/> (0 to 3) in the default order.</summary>
    public static ConstIsoparametric ChoixRef(int index)
    {
        Standard_OutOfRange_Raise_if(index < 0 || index > 3, "ChoixRef() in IntImp_ComputeTangence.cxx");
        return StaticChoixRef[index];
    }

    /// <summary>
    /// Computes the tangent to the intersection of two surfaces from their first derivatives, and sorts the
    /// isoparametrics best suited to march along it.
    /// </summary>
    /// <param name="dPuv">The derivatives in u and v on surface 1, then in u and v on surface 2 (four vectors).</param>
    /// <param name="epsUV">The tolerances in u and v on surface 1, then on surface 2.</param>
    /// <param name="tgduv">The components of the tangent to the intersection along dp/du and dp/dv of surface 1,
    /// then of surface 2.</param>
    /// <param name="tabIso">The isoparametrics, best first (written only when the surfaces are not tangent).</param>
    /// <returns>True if the surfaces are tangent (or a derivative vanishes).</returns>
    public static bool ComputeTangence(ReadOnlySpan<Vector3D> dPuv, ReadOnlySpan<double> epsUV, Span<double> tgduv,
                                       Span<ConstIsoparametric> tabIso)
    //                 input arguments:
    // DPuv [0] = derivative in u on caro 1
    // DPuv [1] = derivative in v on caro 1
    // DPuv [2] = derivative in u on caro 2
    // DPuv [3] = derivative in v on caro 2
    // EpsUV[0] tolerance in u on caro1
    // EpsUV[1] tolerance in v on caro1
    // EpsUV[2] tolerance in u on caro2
    // EpsUV[3] tolerance in v on caro2
    //                 output arguments:
    // Tgduv[0] component along dp/du of caro1 of the tangent to the intersection
    // Tgduv[1] component along dp/dv of caro1 of the tangent to the intersection
    // Tgduv[2] component along dp/du of caro2 of the tangent to the intersection
    // Tgduv[3] component along dp/dv of caro2 of the tangent to the intersection
    // TabIso[0...3] best isos, in decreasing order, as candidates for the intersection
    //         algorithm
    // compute the tangent to the intersection, using the following property of
    // the scalar product a^(b^c)=b(ac)-c(ab): we obtain the components of the
    // tangent to the intersection in the 2 tangent planes (t=n1^n2 where n1 is the normal
    // to the first patch, n2 to the 2nd)
    // we make sure that the tangent planes of the 2 patches are not parallel
    // the components of the intersection in the tangent planes give the angle
    // between the isoparametrics of one patch and the other patch
    // the cosines are sorted in increasing order: the smallest cosine gives the
    // best angle, so the best iso to choose to find the intersection
    {
        Span<double> normDuv = stackalloc double[4];
        double m2, tol2;
        int i;
        //
        tol2 = 1.0e-32;
        //
        for (i = 0; i < 4; ++i)
        {
            normDuv[i] = dPuv[i].SquareMagnitude();
            if (normDuv[i] <= tol2)
            {
                return true;
            }
        }
        //
        //-------------------------------------------------
        Vector3D n1 = dPuv[0];
        n1 = n1.Crossed(dPuv[1]);
        //
        m2 = n1.SquareMagnitude();
        if (m2 < tol2)
        {
            return true;
        }
        n1 = n1.Normalized();
        //-------------------------------------------------
        Vector3D n2 = dPuv[2];
        n2 = n2.Crossed(dPuv[3]);
        m2 = n2.SquareMagnitude();
        if (m2 < tol2)
        {
            return true;
        }
        n2 = n2.Normalized();
        //
        for (i = 0; i < 4; ++i)
        {
            normDuv[i] = Math.Sqrt(normDuv[i]);
        }
        tgduv[0] = -dPuv[1].Dot(n2);
        tgduv[1] = dPuv[0].Dot(n2);
        tgduv[2] = dPuv[3].Dot(n1);
        tgduv[3] = -dPuv[2].Dot(n1);

        bool tangent =
            (Math.Abs(tgduv[0]) <= epsUV[0] * normDuv[1] && Math.Abs(tgduv[1]) <= epsUV[1] * normDuv[0]
             && Math.Abs(tgduv[2]) <= epsUV[2] * normDuv[3] && Math.Abs(tgduv[3]) <= epsUV[3] * normDuv[2]);
        if (!tangent)
        {
            double t = n1.Dot(n2);
            if (t < 0.0)
            {
                t = -t;
            }
            if (t > 0.999999999)
            {
                tangent = true;
            }
        }

        if (!tangent)
        {
            normDuv[0] = Math.Abs(tgduv[1]) / normDuv[0]; // iso u on caro1
            normDuv[1] = Math.Abs(tgduv[0]) / normDuv[1]; // iso v on caro1
            normDuv[2] = Math.Abs(tgduv[3]) / normDuv[2]; // iso u on caro2
            normDuv[3] = Math.Abs(tgduv[2]) / normDuv[3]; // iso v on caro2

            //-- Sort NormDuv (in parallel with ChoixRef)
            bool triOk;
            double t;
            ConstIsoparametric ti;
            for (i = 0; i <= 3; i++)
            {
                tabIso[i] = StaticChoixRef[i];
            }
            do
            {
                triOk = true;
                for (i = 1; i <= 3; i++)
                {
                    if (normDuv[i - 1] > normDuv[i])
                    {
                        triOk = false;
                        t = normDuv[i];
                        normDuv[i] = normDuv[i - 1];
                        normDuv[i - 1] = t;

                        ti = tabIso[i];
                        tabIso[i] = tabIso[i - 1];
                        tabIso[i - 1] = ti;
                    }
                }
            } while (!triOk);
        }
        return tangent;
    }
}
