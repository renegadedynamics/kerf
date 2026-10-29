// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ComputeTangence.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ComputeTangence.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntImp_ConstIsoparametric;

namespace Occt;

/// <summary>
/// OCCT's free functions <c>ChoixRef</c> and <c>IntImp_ComputeTangence</c> (the second is
/// <see cref="ComputeTangence"/> here: C# can't name a member as its class).
/// </summary>
public static class IntImp_ComputeTangence
{
    private static readonly IntImp_ConstIsoparametric[] staticChoixRef =
    [
        IntImp_UIsoparametricOnCaro1,
        IntImp_VIsoparametricOnCaro1,
        IntImp_UIsoparametricOnCaro2,
        IntImp_VIsoparametricOnCaro2,
    ];

    /// <summary>The isoparametric of rank <paramref name="theIndex"/> (0 to 3) in the default order.</summary>
    public static IntImp_ConstIsoparametric ChoixRef(int theIndex)
    {
        Standard_OutOfRange_Raise_if(theIndex < 0 || theIndex > 3, "ChoixRef() in IntImp_ComputeTangence.cxx");
        return staticChoixRef[theIndex];
    }

    /// <summary>
    /// Computes the tangent to the intersection of two surfaces from their first derivatives, and sorts the
    /// isoparametrics best suited to march along it.
    /// </summary>
    /// <param name="DPuv">The derivatives in u and v on surface 1, then in u and v on surface 2 (four vectors).</param>
    /// <param name="EpsUV">The tolerances in u and v on surface 1, then on surface 2.</param>
    /// <param name="Tgduv">The components of the tangent to the intersection along dp/du and dp/dv of surface 1,
    /// then of surface 2.</param>
    /// <param name="TabIso">The isoparametrics, best first (written only when the surfaces are not tangent).</param>
    /// <returns>True if the surfaces are tangent (or a derivative vanishes).</returns>
    public static bool ComputeTangence(ReadOnlySpan<gp_Vec> DPuv, ReadOnlySpan<double> EpsUV, Span<double> Tgduv,
                                       Span<IntImp_ConstIsoparametric> TabIso)
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
        Span<double> NormDuv = stackalloc double[4];
        double aM2, aTol2;
        int i;
        //
        aTol2 = 1.0e-32;
        //
        for (i = 0; i < 4; ++i)
        {
            NormDuv[i] = DPuv[i].SquareMagnitude();
            if (NormDuv[i] <= aTol2)
            {
                return true;
            }
        }
        //
        //-------------------------------------------------
        gp_Vec N1 = DPuv[0];
        N1.Cross(DPuv[1]);
        //
        aM2 = N1.SquareMagnitude();
        if (aM2 < aTol2)
        {
            return true;
        }
        N1.Normalize();
        //-------------------------------------------------
        gp_Vec N2 = DPuv[2];
        N2.Cross(DPuv[3]);
        aM2 = N2.SquareMagnitude();
        if (aM2 < aTol2)
        {
            return true;
        }
        N2.Normalize();
        //
        for (i = 0; i < 4; ++i)
        {
            NormDuv[i] = Math.Sqrt(NormDuv[i]);
        }
        Tgduv[0] = -DPuv[1].Dot(N2);
        Tgduv[1] = DPuv[0].Dot(N2);
        Tgduv[2] = DPuv[3].Dot(N1);
        Tgduv[3] = -DPuv[2].Dot(N1);

        bool tangent =
            (Math.Abs(Tgduv[0]) <= EpsUV[0] * NormDuv[1] && Math.Abs(Tgduv[1]) <= EpsUV[1] * NormDuv[0]
             && Math.Abs(Tgduv[2]) <= EpsUV[2] * NormDuv[3] && Math.Abs(Tgduv[3]) <= EpsUV[3] * NormDuv[2]);
        if (!tangent)
        {
            double t = N1.Dot(N2);
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
            NormDuv[0] = Math.Abs(Tgduv[1]) / NormDuv[0]; // iso u on caro1
            NormDuv[1] = Math.Abs(Tgduv[0]) / NormDuv[1]; // iso v on caro1
            NormDuv[2] = Math.Abs(Tgduv[3]) / NormDuv[2]; // iso u on caro2
            NormDuv[3] = Math.Abs(Tgduv[2]) / NormDuv[3]; // iso v on caro2

            //-- Sort NormDuv (in parallel with ChoixRef)
            bool triOk;
            double t;
            IntImp_ConstIsoparametric ti;
            for (i = 0; i <= 3; i++)
            {
                TabIso[i] = staticChoixRef[i];
            }
            do
            {
                triOk = true;
                for (i = 1; i <= 3; i++)
                {
                    if (NormDuv[i - 1] > NormDuv[i])
                    {
                        triOk = false;
                        t = NormDuv[i];
                        NormDuv[i] = NormDuv[i - 1];
                        NormDuv[i - 1] = t;

                        ti = TabIso[i];
                        TabIso[i] = TabIso[i - 1];
                        TabIso[i - 1] = ti;
                    }
                }
            } while (!triOk);
        }
        return tangent;
    }
}
