// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib.hxx
// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib.lxx
// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib.cxx
// Port of: src/FoundationClasses/TKMath/BSplSLib/BSplSLib_BzSyntaxes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified RLE Aug 93 - Complete rewrite
// xab  21-Mar-95  implemented cache mechanism
// pmn  25-09-96   Interpolation
// jct  25-09-96 : Correction de l'alloc de LocalArray dans RationalDerivative.
// pmn  07-10-96 : Correction de DN dans le cas rationnal.
// pmn  06-02-97 : Correction des poids dans RationalDerivative. (PRO700)
//  pmn  16-10-96 : Correction de PolesCoefficient (PRO5782)
//                  ColLength et RowLength avaient encore frappes !!

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Geometry.Splines;

/// <summary>
/// B-spline surface library: geometric functions for rational and non-rational, periodic and non-periodic B-spline
/// surfaces, built on the multi-dimensional spline methods of <see cref="BSplCLib"/>.
/// </summary>
/// <remarks>
/// A surface is given by its poles (an array of points), weights, knots and multiplicities in U and V, degrees,
/// rationality flags (weights not constant in U or V) and periodicity flags. The bounds of UKnots and UMults should
/// be the same, as should those of VKnots and VMults; the bounds of Poles and Weights should be the same. The
/// rows of Poles are the U direction, the columns the V direction. Optional arguments are nullable: no weights
/// (<see cref="NoWeights"/>) is a non-rational surface, no multiplicities means flat knots.
/// </remarks>
internal static partial class SurfaceBSplSLib
{
    // for null derivatives
    private static ReadOnlySpan<double> BSplSLib_zero => [0.0, 0.0, 0.0];

    //! Maximum supported degree for B-spline surfaces.
    private static readonly int THE_MAX_DEGREE = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    //! Validates that the given degrees do not exceed the maximum supported degree.
    private static void ValidateBSplineDegree(int uDegree, int vDegree)
    {
        Standard_OutOfRange_Raise_if(uDegree > THE_MAX_DEGREE || vDegree > THE_MAX_DEGREE,
                                     "BSplSLib: bspline degree is greater than maximum supported");
    }

    //=======================================================================
    // struct : BSplSLib_DataContainer
    // purpose: Auxiliary structure providing buffers for poles and knots used in
    //         evaluation of bspline (allocated in the stack)
    //=======================================================================
    // Note: OCCT's buffers are sized for the maximum degree (4 * 26 * 26 poles, 2 * 25 knots each, 48 derivatives);
    // the port's are stack buffers sized for the actual degrees (see newDataContainer's callers).
    private readonly ref struct BSplSLib_DataContainer(Span<double> thePoles, Span<double> theKnots1,
                                                       Span<double> theKnots2, Span<double> theDers)
    {
        public readonly Span<double> poles = thePoles;
        public readonly Span<double> knots1 = theKnots1;
        public readonly Span<double> knots2 = theKnots2;
        public readonly Span<double> ders = theDers;
    }

    // Sizes of the data container's buffers for the given degrees (each buffer holds what the evaluation of one
    // patch uses: (UDegree + 1) * (VDegree + 1) homogeneous poles, 2 * Degree knots, the derivatives up to (3, 3)).
    private static int DcPolesSize(int uDegree, int vDegree) => 4 * (uDegree + 1) * (vDegree + 1);

    private static int DcKnotsSize(int uDegree, int vDegree) => 2 * Max(uDegree, vDegree);

    private const int DcDersSize = 48;

    // A span of the coordinates of the points of a 2D array from the point (theRow, theCol) on: OCCT's
    // `(double*)&Poles(theRow, theCol)`.
    private static ReadOnlySpan<double> CoordsFrom(NCollection_Array2<Point3D> array, int row, int col)
    {
        int offset = (row - array.LowerRow()) * array.RowLength() + (col - array.LowerCol());
        return MemoryMarshal.Cast<Point3D, double>(array.AsSpan()).Slice(offset * 3);
    }

    // Stands for OCCT's `(double*)&Weights(theRow, theCol)`.
    private static ReadOnlySpan<double> ValuesFrom(NCollection_Array2<double> array, int row, int col)
    {
        int offset = (row - array.LowerRow()) * array.RowLength() + (col - array.LowerCol());
        return array.AsSpan().Slice(offset);
    }

    //**************************************************************************
    //                     Evaluation methods
    //**************************************************************************

    //=======================================================================
    // function : RationalDerivative
    // purpose  : computes the rational derivatives when we have the
    //           the derivatives of the homogeneous numerator and the
    //           the derivatives of the denominator
    //=======================================================================

    /// <summary>
    /// Computes the derivatives of a rational surface from the derivatives of its homogeneous numerator and
    /// denominator. <paramref name="hDerivatives"/> holds, for p = 0..UDeg and q = 0..VDeg, the (p, q) derivative
    /// of the numerator (3 values) followed by that of the denominator; <paramref name="rDerivatives"/> receives,
    /// for p = 0..N and q = 0..M, the (p, q) derivative of the rational function (3 values each) if
    /// <paramref name="all"/> is true, or only the (N, M) derivative otherwise.
    /// </summary>
    public static void RationalDerivative(int uDeg, int vDeg, int n, int m, ReadOnlySpan<double> hDerivatives,
                                          Span<double> rDerivatives, bool all = true)
    {
        //
        //  if All is True all derivatives are computed. if Not only
        //  the requested N, M is computed
        //
        //                                           Numerator(u,v)
        //   let f(u,v) be a rational function  = ------------------
        //                                         Denominator(u,v)
        //
        //
        //   Let (N,M) the order of the derivatives we want : then since
        //   we have :
        //
        //         Numerator = f * Denominator
        //
        //   we derive :
        //
        //   (N,M)         1           (         (N M)                  (p q)            (N -p M-q) )
        //  f       =  ------------    (  Numerator   -  SUM SUM  a   * f    * Denominator          )
        //                       (0,0) (                 p<N q<M   p q                              )
        //             Denominator
        //
        //   with :
        //
        //              ( N )  ( M )
        //    a      =  (   )  (   )
        //     p q      ( p )  ( q )
        //
        //
        //   HDerivatives is an array where derivatives are stored in the following form
        //   Numerator is assumee to have 3 functions that is a vector of dimension
        //   3
        //
        //             (0,0)           (0,0)                       (0, DegV)           (0, DegV)
        //     Numerator     Denominator      ...         Numerator          Denominator
        //
        //             (1,0)           (1,0)                       (1, DegV)           (1, DegV)
        //     Numerator     Denominator      ...         Numerator          Denominator
        //
        //         ...........................................................
        //
        //
        //            (DegU,0)        (DegU,0)                  (DegU, DegV)           (DegU, DegV)
        //     Numerator     Denominator      ...         Numerator          Denominator
        //
        //
        int ii, jj, pp, qq, index, index1, index2;
        int m1, m3, m4, n1, iiM1, iiM3, jjM1, ppM1, ppM3;
        int minN, minN1, minM, minM1;
        int index_u, index_u1, index_v, index_v1, index_w;

        m1 = m + 1;
        n1 = n + 1;
        ii = n1 * m1;
        m3 = (m1 << 1) + m1;
        m4 = (vDeg + 1) << 2;

        int storeSize = all ? 0 : ii * 3;
        Span<double> storeDerivatives = storeSize <= 128 ? stackalloc double[storeSize] : new double[storeSize];
        Span<double> rArray = all ? rDerivatives : storeDerivatives;
        Span<double> storeW = ii <= 128 ? stackalloc double[ii] : new double[ii];
        ReadOnlySpan<double> homogeneousArray = hDerivatives;
        double denominator, pii, pip, pjq;

        denominator = 1.0e0 / homogeneousArray[3];
        index_u = 0;
        index_u1 = 0;
        if (uDeg < n)
        {
            minN = uDeg;
        }
        else
        {
            minN = n;
        }
        if (vDeg < m)
        {
            minM = vDeg;
        }
        else
        {
            minM = m;
        }
        minN1 = minN + 1;
        minM1 = minM + 1;
        iiM1 = -m1;

        for (ii = 0; ii < minN1; ii++)
        {
            iiM1 += m1;
            index_v = index_u;
            index_v1 = index_u1;
            index_w = iiM1;

            for (jj = 0; jj < minM1; jj++)
            {
                rArray[index_v++] = homogeneousArray[index_v1++];
                rArray[index_v++] = homogeneousArray[index_v1++];
                rArray[index_v++] = homogeneousArray[index_v1++];
                storeW[index_w++] = homogeneousArray[index_v1++];
            }

            for (jj = minM1; jj < m1; jj++)
            {
                rArray[index_v++] = 0.0;
                rArray[index_v++] = 0.0;
                rArray[index_v++] = 0.0;
                storeW[index_w++] = 0.0;
            }
            index_u1 += m4;
            index_u += m3;
        }
        index_v = minN1 * m3;
        index_w = minN1 * m1;

        for (ii = minN1; ii < n1; ii++)
        {

            for (jj = 0; jj < m1; jj++)
            {
                rArray[index_v++] = 0.0e0;
                rArray[index_v++] = 0.0e0;
                rArray[index_v++] = 0.0e0;
                storeW[index_w++] = 0.0e0;
            }
        }

        // ---------------  Calculation ----------------

        iiM1 = -m1;
        iiM3 = -m3;

        for (ii = 0; ii <= n; ii++)
        {
            iiM1 += m1;
            iiM3 += m3;
            index1 = iiM3 - 3;
            jjM1 = iiM1;

            for (jj = 0; jj <= m; jj++)
            {
                jjM1++;
                ppM1 = -m1;
                ppM3 = -m3;
                index1 += 3;

                for (pp = 0; pp < ii; pp++)
                {
                    ppM1 += m1;
                    ppM3 += m3;
                    index = ppM3;
                    index2 = jjM1 - ppM1;
                    pip = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(ii, pp);

                    for (qq = 0; qq <= jj; qq++)
                    {
                        index2--;
                        pjq = pip * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(jj, qq) * storeW[index2];
                        rArray[index1] -= pjq * rArray[index];
                        index++;
                        index1++;
                        rArray[index1] -= pjq * rArray[index];
                        index++;
                        index1++;
                        rArray[index1] -= pjq * rArray[index];
                        index++;
                        index1 -= 2;
                    }
                }
                index = iiM3;
                index2 = jj + 1;
                pii = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(ii, ii);

                for (qq = 0; qq < jj; qq++)
                {
                    index2--;
                    pjq = pii * global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(jj, qq) * storeW[index2];
                    rArray[index1] -= pjq * rArray[index];
                    index++;
                    index1++;
                    rArray[index1] -= pjq * rArray[index];
                    index++;
                    index1++;
                    rArray[index1] -= pjq * rArray[index];
                    index++;
                    index1 -= 2;
                }
                rArray[index1] *= denominator;
                index1++;
                rArray[index1] *= denominator;
                index1++;
                rArray[index1] *= denominator;
                index1 -= 2;
            }
        }
        if (!all)
        {
            rArray = rDerivatives;
            index = n * m1 + m;
            index = (index << 1) + index;
            rArray[0] = storeDerivatives[index];
            index++;
            rArray[1] = storeDerivatives[index];
            index++;
            rArray[2] = storeDerivatives[index];
        }
    }

    //=================================================================================================

    //
    // PrepareEval :
    //
    // Prepare all data for computing points :
    //  local arrays of knots
    //  local array  of poles (multiplied by the weights if rational)
    //
    //  The first direction to compute (smaller degree) is returned
    //  and the poles are stored according to this direction.

    private static bool PrepareEval(double u,
                                    double v,
                                    int Uindex,
                                    int Vindex,
                                    int uDegree,
                                    int vDegree,
                                    bool uRat,
                                    bool vRat,
                                    bool uPer,
                                    bool vPer,
                                    NCollection_Array2<Point3D> Poles,
                                    NCollection_Array2<double>? weights,
                                    NCollection_Array1<double> uKnots,
                                    NCollection_Array1<double> vKnots,
                                    NCollection_Array1<int>? uMults,
                                    NCollection_Array1<int>? vMults,
                                    out double u1, // first  parameter to use
                                    out double u2, // second parameter to use
                                    out int d1,    // first degree
                                    out int d2,    // second degree
                                    out bool rational,
                                    BSplSLib_DataContainer dc)
    {
        rational = uRat || vRat;
        int uindex = Uindex;
        int vindex = Vindex;
        int ukLower = uKnots.Lower();
        int ukUpper = uKnots.Upper();
        int vkLower = vKnots.Lower();
        int vkUpper = vKnots.Upper();

        if (uDegree <= vDegree)
        {
            // compute the indices
            if (uindex < ukLower || uindex > ukUpper)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(uDegree, uKnots, uMults, u, uPer, ref uindex, out u1);
            }
            else
            {
                u1 = u;
            }

            if (vindex < vkLower || vindex > vkUpper)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(vDegree, vKnots, vMults, v, vPer, ref vindex, out u2);
            }
            else
            {
                u2 = v;
            }

            // get the knots
            d1 = uDegree;
            d2 = vDegree;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(uDegree, uindex, uPer, uKnots, uMults, dc.knots1);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(vDegree, vindex, vPer, vKnots, vMults, dc.knots2);

            if (uMults == null)
            {
                uindex -= ukLower + uDegree;
            }
            else
            {
                uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(uDegree, uindex, uPer, uMults);
            }

            if (vMults == null)
            {
                vindex -= vkLower + vDegree;
            }
            else
            {
                vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(vDegree, vindex, vPer, vMults);
            }

            // get the poles
            int i, j, ip, jp;
            double w;
            Span<double> poles = dc.poles;
            int pole = 0;
            d1 = uDegree;
            d2 = vDegree;
            int pLowerRow = Poles.LowerRow();
            int pUpperRow = Poles.UpperRow();
            int pLowerCol = Poles.LowerCol();
            int pUpperCol = Poles.UpperCol();

            // verify if locally non rational
            if (rational)
            {
                rational = false;
                ip = pLowerRow + uindex;
                jp = pLowerCol + vindex;

                if (ip < pLowerRow)
                {
                    ip = pUpperRow;
                }
                if (jp < pLowerCol)
                {
                    jp = pUpperCol;
                }

                w = weights!.Value(ip, jp);
                double eps = Epsilon(w);
                double dw;

                for (i = 0; i <= uDegree && !rational; i++)
                {
                    jp = pLowerCol + vindex;

                    if (jp < pLowerCol)
                    {
                        jp = pUpperCol;
                    }

                    for (j = 0; j <= vDegree && !rational; j++)
                    {
                        dw = weights.Value(ip, jp) - w;
                        if (dw < 0)
                        {
                            dw = -dw;
                        }

                        rational = (dw > eps);

                        jp++;

                        if (jp > pUpperCol)
                        {
                            jp = pLowerCol;
                        }
                    }

                    ip++;

                    if (ip > pUpperRow)
                    {
                        ip = pLowerRow;
                    }
                }
            }

            // copy the poles
            ip = pLowerRow + uindex;

            if (ip < pLowerRow)
            {
                ip = pUpperRow;
            }

            if (rational)
            {
                for (i = 0; i <= d1; i++)
                {
                    jp = pLowerCol + vindex;

                    if (jp < pLowerCol)
                    {
                        jp = pUpperCol;
                    }

                    for (j = 0; j <= d2; j++)
                    {
                        Point3D p = Poles.Value(ip, jp);
                        poles[pole + 3] = w = weights!.Value(ip, jp);
                        poles[pole + 0] = p.X * w;
                        poles[pole + 1] = p.Y * w;
                        poles[pole + 2] = p.Z * w;
                        pole += 4;
                        jp++;

                        if (jp > pUpperCol)
                        {
                            jp = pLowerCol;
                        }
                    }

                    ip++;

                    if (ip > pUpperRow)
                    {
                        ip = pLowerRow;
                    }
                }
            }
            else
            {
                for (i = 0; i <= d1; i++)
                {
                    jp = pLowerCol + vindex;

                    if (jp < pLowerCol)
                    {
                        jp = pUpperCol;
                    }

                    for (j = 0; j <= d2; j++)
                    {
                        Point3D p = Poles.Value(ip, jp);
                        poles[pole + 0] = p.X;
                        poles[pole + 1] = p.Y;
                        poles[pole + 2] = p.Z;
                        pole += 3;
                        jp++;

                        if (jp > pUpperCol)
                        {
                            jp = pLowerCol;
                        }
                    }

                    ip++;

                    if (ip > pUpperRow)
                    {
                        ip = pLowerRow;
                    }
                }
            }

            return true;
        }
        else
        {
            // compute the indices
            if (uindex < ukLower || uindex > ukUpper)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(uDegree, uKnots, uMults, u, uPer, ref uindex, out u2);
            }
            else
            {
                u2 = u;
            }

            if (vindex < vkLower || vindex > vkUpper)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(vDegree, vKnots, vMults, v, vPer, ref vindex, out u1);
            }
            else
            {
                u1 = v;
            }

            // get the knots

            d2 = uDegree;
            d1 = vDegree;

            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(uDegree, uindex, uPer, uKnots, uMults, dc.knots2);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(vDegree, vindex, vPer, vKnots, vMults, dc.knots1);

            if (uMults == null)
            {
                uindex -= ukLower + uDegree;
            }
            else
            {
                uindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(uDegree, uindex, uPer, uMults);
            }

            if (vMults == null)
            {
                vindex -= vkLower + vDegree;
            }
            else
            {
                vindex = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(vDegree, vindex, vPer, vMults);
            }

            // get the poles
            int i, j, ip, jp;
            double w;
            Span<double> poles = dc.poles;
            int pole = 0;
            d1 = vDegree;
            d2 = uDegree;
            int pLowerRow = Poles.LowerRow();
            int pUpperRow = Poles.UpperRow();
            int pLowerCol = Poles.LowerCol();
            int pUpperCol = Poles.UpperCol();

            // verify if locally non rational
            if (rational)
            {
                rational = false;
                ip = pLowerRow + uindex;
                jp = pLowerCol + vindex;

                if (ip < pLowerRow)
                {
                    ip = pUpperRow;
                }

                if (jp < pLowerCol)
                {
                    jp = pUpperCol;
                }

                w = weights!.Value(ip, jp);
                double eps = Epsilon(w);
                double dw;

                for (i = 0; i <= uDegree && !rational; i++)
                {
                    jp = pLowerCol + vindex;

                    if (jp < pLowerCol)
                    {
                        jp = pUpperCol;
                    }

                    for (j = 0; j <= vDegree && !rational; j++)
                    {
                        dw = weights.Value(ip, jp) - w;
                        if (dw < 0)
                        {
                            dw = -dw;
                        }
                        rational = dw > eps;

                        jp++;

                        if (jp > pUpperCol)
                        {
                            jp = pLowerCol;
                        }
                    }

                    ip++;

                    if (ip > pUpperRow)
                    {
                        ip = pLowerRow;
                    }
                }
            }

            // copy the poles
            jp = pLowerCol + vindex;

            if (jp < pLowerCol)
            {
                jp = pUpperCol;
            }

            if (rational)
            {
                for (i = 0; i <= d1; i++)
                {
                    ip = pLowerRow + uindex;

                    if (ip < pLowerRow)
                    {
                        ip = pUpperRow;
                    }

                    for (j = 0; j <= d2; j++)
                    {
                        Point3D p = Poles.Value(ip, jp);
                        poles[pole + 3] = w = weights!.Value(ip, jp);
                        poles[pole + 0] = p.X * w;
                        poles[pole + 1] = p.Y * w;
                        poles[pole + 2] = p.Z * w;
                        pole += 4;
                        ip++;

                        if (ip > pUpperRow)
                        {
                            ip = pLowerRow;
                        }
                    }

                    jp++;

                    if (jp > pUpperCol)
                    {
                        jp = pLowerCol;
                    }
                }
            }
            else
            {
                for (i = 0; i <= d1; i++)
                {
                    ip = pLowerRow + uindex;

                    if (ip < pLowerRow)
                    {
                        ip = pUpperRow;
                    }

                    if (ip > pUpperRow)
                    {
                        ip = pLowerRow;
                    }

                    for (j = 0; j <= d2; j++)
                    {
                        Point3D p = Poles.Value(ip, jp);
                        poles[pole + 0] = p.X;
                        poles[pole + 1] = p.Y;
                        poles[pole + 2] = p.Z;
                        pole += 3;
                        ip++;

                        if (ip > pUpperRow)
                        {
                            ip = pLowerRow;
                        }
                    }

                    jp++;

                    if (jp > pUpperCol)
                    {
                        jp = pLowerCol;
                    }
                }
            }

            return false;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Computes the point of the surface at (U, V). UIndex and VIndex are the knot spans of U and V if known
    /// (outside the knots' range, they are located). Without multiplicities, the knots are flat knots.
    /// </summary>
    public static void D0(double u,
                          double v,
                          int uIndex,
                          int vIndex,
                          NCollection_Array2<Point3D> poles,
                          NCollection_Array2<double>? weights,
                          NCollection_Array1<double> uKnots,
                          NCollection_Array1<double> vKnots,
                          NCollection_Array1<int>? uMults,
                          NCollection_Array1<int>? vMults,
                          int uDegree,
                          int vDegree,
                          bool uRat,
                          bool vRat,
                          bool uPer,
                          bool vPer,
                          out Point3D p)
    {
        //  int k ;
        HomogeneousD0(u,
                      v,
                      uIndex,
                      vIndex,
                      poles,
                      weights,
                      uKnots,
                      vKnots,
                      uMults,
                      vMults,
                      uDegree,
                      vDegree,
                      uRat,
                      vRat,
                      uPer,
                      vPer,
                      out double w,
                      out p);
        p = p.WithX(p.X / w);
        p = p.WithY(p.Y / w);
        p = p.WithZ(p.Z / w);
    }

    //=================================================================================================

    /// <summary>
    /// Homogeneous evaluation of the poles and weights at (U, V): P is the numerator and W the denominator if
    /// there are weights, otherwise 1.
    /// </summary>
    public static void HomogeneousD0(double u,
                                     double v,
                                     int uIndex,
                                     int vIndex,
                                     NCollection_Array2<Point3D> poles,
                                     NCollection_Array2<double>? weights,
                                     NCollection_Array1<double> uKnots,
                                     NCollection_Array1<double> vKnots,
                                     NCollection_Array1<int>? uMults,
                                     NCollection_Array1<int>? vMults,
                                     int uDegree,
                                     int vDegree,
                                     bool uRat,
                                     bool vRat,
                                     bool uPer,
                                     bool vPer,
                                     out double w,
                                     out Point3D p)
    {
        bool rational;
        //  int k,dim;
        int dim;
        double u1, u2;
        int d1, d2;
        w = 1.0e0;

        ValidateBSplineDegree(uDegree, vDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[DcPolesSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcDersSize]);
        PrepareEval(u,
                    v,
                    uIndex,
                    vIndex,
                    uDegree,
                    vDegree,
                    uRat,
                    vRat,
                    uPer,
                    vPer,
                    poles,
                    weights,
                    uKnots,
                    vKnots,
                    uMults,
                    vMults,
                    out u1,
                    out u2,
                    out d1,
                    out d2,
                    out rational,
                    dc);
        if (rational)
        {
            dim = 4;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u1, d1, dc.knots1, dim * (d2 + 1), dc.poles);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles);
            w = dc.poles[3];
            p = new Point3D(dc.poles[0], dc.poles[1], dc.poles[2]);
        }
        else
        {
            dim = 3;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u1, d1, dc.knots1, dim * (d2 + 1), dc.poles);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles);
            p = new Point3D(dc.poles[0], dc.poles[1], dc.poles[2]);
        }
    }

    //=================================================================================================

    /// <summary>Computes the point and the first derivatives of the surface at (U, V) (see <see cref="D0"/>).</summary>
    public static void D1(double u,
                          double v,
                          int uIndex,
                          int vIndex,
                          NCollection_Array2<Point3D> poles,
                          NCollection_Array2<double>? weights,
                          NCollection_Array1<double> uKnots,
                          NCollection_Array1<double> vKnots,
                          NCollection_Array1<int>? uMults,
                          NCollection_Array1<int>? vMults,
                          int uDegree,
                          int vDegree,
                          bool uRat,
                          bool vRat,
                          bool uPer,
                          bool vPer,
                          out Point3D p,
                          out Vector3D vu,
                          out Vector3D vv)
    {
        bool rational;
        //  int k,dim,dim2;
        int dim, dim2;
        double u1, u2;
        int d1, d2;
        scoped ReadOnlySpan<double> result, resVu, resVv;
        ValidateBSplineDegree(uDegree, vDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[DcPolesSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcDersSize]);
        if (PrepareEval(u,
                        v,
                        uIndex,
                        vIndex,
                        uDegree,
                        vDegree,
                        uRat,
                        vRat,
                        uPer,
                        vPer,
                        poles,
                        weights,
                        uKnots,
                        vKnots,
                        uMults,
                        vMults,
                        out u1,
                        out u2,
                        out d1,
                        out d2,
                        out rational,
                        dc))
        {
            if (rational)
            {
                dim = 4;
                dim2 = (d2 + 1) << 2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[dim2..]);
                SurfaceBSplSLib.RationalDerivative(d1, d2, 1, 1, dc.poles, dc.ders);
                result = dc.ders;
                resVu = result[6..];
                resVv = result[3..];
            }
            else
            {
                dim = 3;
                dim2 = d2 + 1;
                dim2 = (dim2 << 1) + dim2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[dim2..]);
                result = dc.poles;
                resVu = result[dim2..];
                resVv = result[3..];
            }
        }
        else
        {
            if (rational)
            {
                dim = 4;
                dim2 = (d2 + 1) << 2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[dim2..]);
                SurfaceBSplSLib.RationalDerivative(d1, d2, 1, 1, dc.poles, dc.ders);
                result = dc.ders;
                resVu = result[3..];
                resVv = result[6..];
            }
            else
            {
                dim = 3;
                dim2 = d2 + 1;
                dim2 = (dim2 << 1) + dim2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[dim2..]);
                result = dc.poles;
                resVu = result[3..];
                resVv = result[dim2..];
            }
        }

        p = new Point3D(result[0], result[1], result[2]);
        vu = new Vector3D(resVu[0], resVu[1], resVu[2]);
        vv = new Vector3D(resVv[0], resVv[1], resVv[2]);
    }

    //=================================================================================================

    /// <summary>
    /// Homogeneous evaluation of the point and first derivatives at (U, V): N, Nu, Nv for the numerator and
    /// D, Du, Dv for the denominator (1, 0, 0 if there are no weights).
    /// </summary>
    public static void HomogeneousD1(double u,
                                     double v,
                                     int uIndex,
                                     int vIndex,
                                     NCollection_Array2<Point3D> poles,
                                     NCollection_Array2<double>? weights,
                                     NCollection_Array1<double> uKnots,
                                     NCollection_Array1<double> vKnots,
                                     NCollection_Array1<int>? uMults,
                                     NCollection_Array1<int>? vMults,
                                     int uDegree,
                                     int vDegree,
                                     bool uRat,
                                     bool vRat,
                                     bool uPer,
                                     bool vPer,
                                     out Point3D n,
                                     out Vector3D nu,
                                     out Vector3D nv,
                                     out double d,
                                     out double du,
                                     out double dv)
    {
        bool rational;
        //  int k,dim;
        int dim;
        double u1, u2;
        int d1, d2;

        d = 1.0e0;
        du = 0.0e0;
        dv = 0.0e0;
        ValidateBSplineDegree(uDegree, vDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[DcPolesSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcDersSize]);
        bool ufirst = PrepareEval(u,
                                  v,
                                  uIndex,
                                  vIndex,
                                  uDegree,
                                  vDegree,
                                  uRat,
                                  vRat,
                                  uPer,
                                  vPer,
                                  poles,
                                  weights,
                                  uKnots,
                                  vKnots,
                                  uMults,
                                  vMults,
                                  out u1,
                                  out u2,
                                  out d1,
                                  out d2,
                                  out rational,
                                  dc);
        dim = rational ? 4 : 3;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim * (d2 + 1), dc.poles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim * (d2 + 1))..]);

        scoped ReadOnlySpan<double> result, resVu, resVv;
        result = dc.poles;
        resVu = result[(ufirst ? dim * (d2 + 1) : dim)..];
        resVv = result[(ufirst ? dim : dim * (d2 + 1))..];

        n = new Point3D(result[0], result[1], result[2]);
        nu = new Vector3D(resVu[0], resVu[1], resVu[2]);
        nv = new Vector3D(resVv[0], resVv[1], resVv[2]);

        if (rational)
        {
            d = result[3];
            du = resVu[3];
            dv = resVv[3];
        }
    }

    //=================================================================================================

    /// <summary>Computes the point and the derivatives up to order 2 at (U, V) (see <see cref="D0"/>).</summary>
    public static void D2(double u,
                          double v,
                          int uIndex,
                          int vIndex,
                          NCollection_Array2<Point3D> poles,
                          NCollection_Array2<double>? weights,
                          NCollection_Array1<double> uKnots,
                          NCollection_Array1<double> vKnots,
                          NCollection_Array1<int>? uMults,
                          NCollection_Array1<int>? vMults,
                          int uDegree,
                          int vDegree,
                          bool uRat,
                          bool vRat,
                          bool uPer,
                          bool vPer,
                          out Point3D p,
                          out Vector3D vu,
                          out Vector3D vv,
                          out Vector3D vuu,
                          out Vector3D vvv,
                          out Vector3D vuv)
    {
        bool rational;
        //  int k,dim,dim2;
        int dim, dim2;
        double u1, u2;
        int d1, d2;
        scoped ReadOnlySpan<double> result;
        scoped ReadOnlySpan<double> resVu, resVv, resVuu, resVvv, resVuv;
        ValidateBSplineDegree(uDegree, vDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[DcPolesSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcDersSize]);
        if (PrepareEval(u,
                        v,
                        uIndex,
                        vIndex,
                        uDegree,
                        vDegree,
                        uRat,
                        vRat,
                        uPer,
                        vPer,
                        poles,
                        weights,
                        uKnots,
                        vKnots,
                        uMults,
                        vMults,
                        out u1,
                        out u2,
                        out d1,
                        out d2,
                        out rational,
                        dc))
        {
            if (rational)
            {
                dim = 4;
                dim2 = (d2 + 1) << 2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 2, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                SurfaceBSplSLib.RationalDerivative(d1, d2, 2, 2, dc.poles, dc.ders);
                result = dc.ders;
                resVu = result[9..];
                resVv = result[3..];
                resVuu = result[18..];
                resVvv = result[6..];
                resVuv = result[12..];
            }
            else
            {
                dim = 3;
                dim2 = d2 + 1;
                dim2 = (dim2 << 1) + dim2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 2, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                result = dc.poles;
                resVu = result[dim2..];
                resVv = result[3..];
                if (uDegree <= 1)
                {
                    resVuu = BSplSLib_zero;
                }
                else
                {
                    resVuu = result[(dim2 << 1)..];
                }
                if (vDegree <= 1)
                {
                    resVvv = BSplSLib_zero;
                }
                else
                {
                    resVvv = result[6..];
                }
                resVuv = result[((d2 << 1) + d2 + 6)..];
            }
        }
        else
        {
            if (rational)
            {
                dim = 4;
                dim2 = (d2 + 1) << 2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 2, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                SurfaceBSplSLib.RationalDerivative(d1, d2, 2, 2, dc.poles, dc.ders);
                result = dc.ders;
                resVu = result[3..];
                resVv = result[9..];
                resVuu = result[6..];
                resVvv = result[18..];
                resVuv = result[12..];
            }
            else
            {
                dim = 3;
                dim2 = d2 + 1;
                dim2 = (dim2 << 1) + dim2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 2, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                result = dc.poles;
                resVu = result[3..];
                resVv = result[dim2..];
                if (uDegree <= 1)
                {
                    resVuu = BSplSLib_zero;
                }
                else
                {
                    resVuu = result[6..];
                }
                if (vDegree <= 1)
                {
                    resVvv = BSplSLib_zero;
                }
                else
                {
                    resVvv = result[(dim2 << 1)..];
                }
                resVuv = result[((d2 << 1) + d2 + 6)..];
            }
        }

        p = new Point3D(result[0], result[1], result[2]);
        vu = new Vector3D(resVu[0], resVu[1], resVu[2]);
        vv = new Vector3D(resVv[0], resVv[1], resVv[2]);
        vuu = new Vector3D(resVuu[0], resVuu[1], resVuu[2]);
        vvv = new Vector3D(resVvv[0], resVvv[1], resVvv[2]);
        vuv = new Vector3D(resVuv[0], resVuv[1], resVuv[2]);
    }

    //=================================================================================================

    /// <summary>Computes the point and the derivatives up to order 3 at (U, V) (see <see cref="D0"/>).</summary>
    public static void D3(double u,
                          double v,
                          int uIndex,
                          int vIndex,
                          NCollection_Array2<Point3D> poles,
                          NCollection_Array2<double>? weights,
                          NCollection_Array1<double> uKnots,
                          NCollection_Array1<double> vKnots,
                          NCollection_Array1<int>? uMults,
                          NCollection_Array1<int>? vMults,
                          int uDegree,
                          int vDegree,
                          bool uRat,
                          bool vRat,
                          bool uPer,
                          bool vPer,
                          out Point3D p,
                          out Vector3D vu,
                          out Vector3D vv,
                          out Vector3D vuu,
                          out Vector3D vvv,
                          out Vector3D vuv,
                          out Vector3D vuuu,
                          out Vector3D vvvv,
                          out Vector3D vuuv,
                          out Vector3D vuvv)
    {
        bool rational;
        //  int k,dim,dim2;
        int dim, dim2;
        double u1, u2;
        int d1, d2;
        scoped ReadOnlySpan<double> result;
        scoped ReadOnlySpan<double> resVu, resVv, resVuu, resVvv, resVuv, resVuuu, resVvvv, resVuuv, resVuvv;
        ValidateBSplineDegree(uDegree, vDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[DcPolesSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcDersSize]);
        if (PrepareEval(u,
                        v,
                        uIndex,
                        vIndex,
                        uDegree,
                        vDegree,
                        uRat,
                        vRat,
                        uPer,
                        vPer,
                        poles,
                        weights,
                        uKnots,
                        vKnots,
                        uMults,
                        vMults,
                        out u1,
                        out u2,
                        out d1,
                        out d2,
                        out rational,
                        dc))
        {
            if (rational)
            {
                dim = 4;
                dim2 = (d2 + 1) << 2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 3, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 3, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                if (d1 > 2)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[((dim2 << 1) + dim2)..]);
                }
                SurfaceBSplSLib.RationalDerivative(d1, d2, 3, 3, dc.poles, dc.ders);
                result = dc.ders;
                resVu = result[12..];
                resVv = result[3..];
                resVuu = result[24..];
                resVvv = result[6..];
                resVuv = result[15..];
                resVuuu = result[36..];
                resVvvv = result[9..];
                resVuuv = result[27..];
                resVuvv = result[18..];
            }
            else
            {
                dim = 3;
                dim2 = (d2 + 1);
                dim2 = (dim2 << 1) + dim2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 3, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 3, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                if (d1 > 2)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[((dim2 << 1) + dim2)..]);
                }
                result = dc.poles;
                resVu = result[dim2..];
                resVv = result[3..];
                if (uDegree <= 1)
                {
                    resVuu = BSplSLib_zero;
                    resVuuv = BSplSLib_zero;
                }
                else
                {
                    resVuu = result[(dim2 << 1)..];
                    resVuuv = result[((dim2 << 1) + 3)..];
                }
                if (vDegree <= 1)
                {
                    resVvv = BSplSLib_zero;
                    resVuvv = BSplSLib_zero;
                }
                else
                {
                    resVvv = result[6..];
                    resVuvv = result[(dim2 + 6)..];
                }
                resVuv = result[((d2 << 1) + d2 + 6)..];
                if (uDegree <= 2)
                {
                    resVuuu = BSplSLib_zero;
                }
                else
                {
                    resVuuu = result[((dim2 << 1) + dim2)..];
                }
                if (vDegree <= 2)
                {
                    resVvvv = BSplSLib_zero;
                }
                else
                {
                    resVvvv = result[9..];
                }
            }
        }
        else
        {
            if (rational)
            {
                dim = 4;
                dim2 = (d2 + 1) << 2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 3, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 3, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                if (d1 > 2)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[((dim2 << 1) + dim2)..]);
                }
                SurfaceBSplSLib.RationalDerivative(d1, d2, 3, 3, dc.poles, dc.ders);
                result = dc.ders;
                resVu = result[3..];
                resVv = result[12..];
                resVuu = result[6..];
                resVvv = result[24..];
                resVuv = result[15..];
                resVuuu = result[9..];
                resVvvv = result[36..];
                resVuuv = result[18..];
                resVuvv = result[27..];
            }
            else
            {
                dim = 3;
                dim2 = (d2 + 1);
                dim2 = (dim2 << 1) + dim2;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, 3, dc.knots1, dim2, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 3, dc.knots2, dim, dc.poles);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                if (d1 > 2)
                {
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[((dim2 << 1) + dim2)..]);
                }
                result = dc.poles;
                resVu = result[3..];
                resVv = result[dim2..];
                if (uDegree <= 1)
                {
                    resVuu = BSplSLib_zero;
                    resVuuv = BSplSLib_zero;
                }
                else
                {
                    resVuu = result[6..];
                    resVuuv = result[(dim2 + 6)..];
                }
                if (vDegree <= 1)
                {
                    resVvv = BSplSLib_zero;
                    resVuvv = BSplSLib_zero;
                }
                else
                {
                    resVvv = result[(dim2 << 1)..];
                    resVuvv = result[((dim2 << 1) + 3)..];
                }
                resVuv = result[((d2 << 1) + d2 + 6)..];
                if (uDegree <= 2)
                {
                    resVuuu = BSplSLib_zero;
                }
                else
                {
                    resVuuu = result[9..];
                }
                if (vDegree <= 2)
                {
                    resVvvv = BSplSLib_zero;
                }
                else
                {
                    resVvvv = result[((dim2 << 1) + dim2)..];
                }
            }
        }

        p = new Point3D(result[0], result[1], result[2]);
        vu = new Vector3D(resVu[0], resVu[1], resVu[2]);
        vv = new Vector3D(resVv[0], resVv[1], resVv[2]);
        vuu = new Vector3D(resVuu[0], resVuu[1], resVuu[2]);
        vvv = new Vector3D(resVvv[0], resVvv[1], resVvv[2]);
        vuv = new Vector3D(resVuv[0], resVuv[1], resVuv[2]);
        vuuu = new Vector3D(resVuuu[0], resVuuu[1], resVuuu[2]);
        vvvv = new Vector3D(resVvvv[0], resVvvv[1], resVvvv[2]);
        vuuv = new Vector3D(resVuuv[0], resVuuv[1], resVuuv[2]);
        vuvv = new Vector3D(resVuvv[0], resVuvv[1], resVuvv[2]);
    }

    //=================================================================================================

    /// <summary>Computes the derivative of order (Nu, Nv) at (U, V) (see <see cref="D0"/>).</summary>
    public static void DN(double u,
                          double v,
                          int nu,
                          int nv,
                          int uIndex,
                          int vIndex,
                          NCollection_Array2<Point3D> poles,
                          NCollection_Array2<double>? weights,
                          NCollection_Array1<double> uKnots,
                          NCollection_Array1<double> vKnots,
                          NCollection_Array1<int>? uMults,
                          NCollection_Array1<int>? vMults,
                          int uDegree,
                          int vDegree,
                          bool uRat,
                          bool vRat,
                          bool uPer,
                          bool vPer,
                          out Vector3D vn)
    {
        bool rational;
        int k, dim;
        double u1, u2;
        int d1, d2;

        ValidateBSplineDegree(uDegree, vDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[DcPolesSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcDersSize]);
        bool ufirst = PrepareEval(u,
                                  v,
                                  uIndex,
                                  vIndex,
                                  uDegree,
                                  vDegree,
                                  uRat,
                                  vRat,
                                  uPer,
                                  vPer,
                                  poles,
                                  weights,
                                  uKnots,
                                  vKnots,
                                  uMults,
                                  vMults,
                                  out u1,
                                  out u2,
                                  out d1,
                                  out d2,
                                  out rational,
                                  dc);
        dim = rational ? 4 : 3;

        if (!rational)
        {
            if ((nu > uDegree) || (nv > vDegree))
            {
                vn = new Vector3D(0.0, 0.0, 0.0);
                return;
            }
        }

        int n1 = ufirst ? nu : nv;
        int n2 = ufirst ? nv : nu;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, n1, dc.knots1, dim * (d2 + 1), dc.poles);

        for (k = 0; k <= Min(n1, d1); k++)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, n2, dc.knots2, dim, dc.poles[(k * dim * (d2 + 1))..]);
        }

        scoped ReadOnlySpan<double> result;
        if (rational)
        {
            SurfaceBSplSLib.RationalDerivative(d1, d2, n1, n2, dc.poles, dc.ders, false);
            result = dc.ders; // because false ci-dessus.
        }
        else
        {
            result = dc.poles[((n1 * (d2 + 1) + n2) * dim)..];
        }

        vn = new Vector3D(result[0], result[1], result[2]);
    }

    //
    // Surface modifications
    //
    // a surface is processed as a curve of curves.
    // i.e : a curve of parameter u where the current point is the set of poles
    //       of the iso.
    //

    //=================================================================================================

    /// <summary>
    /// Computes the poles and weights of an isoparametric curve at <paramref name="param"/> (a U iso if
    /// <paramref name="isU"/>, a V iso otherwise). CWeights is required for a rational surface; for a non-rational
    /// one it is filled with 1 if given.
    /// </summary>
    public static void Iso(double param,
                           bool isU,
                           NCollection_Array2<Point3D> poles,
                           NCollection_Array2<double>? weights,
                           NCollection_Array1<double> knots,
                           NCollection_Array1<int>? mults,
                           int degree,
                           bool periodic,
                           NCollection_Array1<Point3D> cPoles,
                           NCollection_Array1<double>? cWeights)
    {
        int index = 0;
        double u = param;
        bool rational = weights != null;
        int dim = rational ? 4 : 3;

        // compute local knots

        int knotsSize = 2 * degree;
        Span<double> locknots1 = knotsSize <= 128 ? stackalloc double[knotsSize] : new double[knotsSize];
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree, knots, mults, u, periodic, ref index, out u);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(degree, index, periodic, knots, mults, locknots1);
        if (mults == null)
        {
            index -= knots.Lower() + degree;
        }
        else
        {
            index = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(degree, index, periodic, mults);
        }

        // copy the local poles

        //  int f1,l1,f2,l2,i,j,k;
        int f1, l1, f2, l2, i, j;

        if (isU)
        {
            f1 = poles.LowerRow();
            l1 = poles.UpperRow();
            f2 = poles.LowerCol();
            l2 = poles.UpperCol();
        }
        else
        {
            f1 = poles.LowerCol();
            l1 = poles.UpperCol();
            f2 = poles.LowerRow();
            l2 = poles.UpperRow();
        }

        int polesSize = (degree + 1) * (l2 - f2 + 1) * dim;
        Span<double> locpoles = polesSize <= 128 ? stackalloc double[polesSize] : new double[polesSize];

        double w;
        int pole = 0;
        index += f1;

        for (i = 0; i <= degree; i++)
        {

            for (j = f2; j <= l2; j++)
            {

                Point3D p = isU ? poles[index, j] : poles[j, index];
                if (rational)
                {
                    locpoles[pole + 3] = w = isU ? weights![index, j] : weights![j, index];
                    locpoles[pole + 0] = p.X * w;
                    locpoles[pole + 1] = p.Y * w;
                    locpoles[pole + 2] = p.Z * w;
                }
                else
                {
                    locpoles[pole + 0] = p.X;
                    locpoles[pole + 1] = p.Y;
                    locpoles[pole + 2] = p.Z;
                }
                pole += dim;
            }
            index++;
            if (index > l1)
            {
                index = f1;
            }
        }

        // compute the iso
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u, degree, locknots1, (l2 - f2 + 1) * dim, locpoles);

        // get the result
        pole = 0;

        for (i = cPoles.Lower(); i <= cPoles.Upper(); i++)
        {
            if (rational)
            {
                cWeights![i] = w = locpoles[pole + 3];
                cPoles[i] = new Point3D(locpoles[pole + 0] / w, locpoles[pole + 1] / w, locpoles[pole + 2] / w);
            }
            else
            {
                cPoles[i] = new Point3D(locpoles[pole + 0], locpoles[pole + 1], locpoles[pole + 2]);
            }
            pole += dim;
        }

        // if the input is not rational but weights are wanted
        if (!rational && (cWeights != null))
        {

            for (i = cWeights.Lower(); i <= cWeights.Upper(); i++)
            {
                cWeights[i] = 1.0;
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Reverses the array of poles in one direction. <paramref name="last"/> is the index of the new first row
    /// (column): Poles.Upper() on a non-periodic surface, the number of flat knots - degree - 1 (or the sum of the
    /// multiplicities but the last + degree - 1) on a periodic one.
    /// </summary>
    public static void Reverse(NCollection_Array2<Point3D> poles, int last, bool uDirection)
    {
        int i, j, l = last;
        if (uDirection)
        {
            l = poles.LowerRow() + (l - poles.LowerRow()) % (poles.ColLength());
            NCollection_Array2<Point3D> temp = new(0, poles.ColLength() - 1, poles.LowerCol(), poles.UpperCol());

            for (i = poles.LowerRow(); i <= l; i++)
            {

                for (j = poles.LowerCol(); j <= poles.UpperCol(); j++)
                {
                    temp[l - i, j] = poles[i, j];
                }
            }

            for (i = l + 1; i <= poles.UpperRow(); i++)
            {

                for (j = poles.LowerCol(); j <= poles.UpperCol(); j++)
                {
                    temp[l + poles.ColLength() - i, j] = poles[i, j];
                }
            }

            for (i = poles.LowerRow(); i <= poles.UpperRow(); i++)
            {

                for (j = poles.LowerCol(); j <= poles.UpperCol(); j++)
                {
                    poles[i, j] = temp[i - poles.LowerRow(), j];
                }
            }
        }
        else
        {
            l = poles.LowerCol() + (l - poles.LowerCol()) % (poles.RowLength());
            NCollection_Array2<Point3D> temp = new(poles.LowerRow(), poles.UpperRow(), 0, poles.RowLength() - 1);

            for (j = poles.LowerCol(); j <= l; j++)
            {

                for (i = poles.LowerRow(); i <= poles.UpperRow(); i++)
                {
                    temp[i, l - j] = poles[i, j];
                }
            }

            for (j = l + 1; j <= poles.UpperCol(); j++)
            {

                for (i = poles.LowerRow(); i <= poles.UpperRow(); i++)
                {
                    temp[i, l + poles.RowLength() - j] = poles[i, j];
                }
            }

            for (i = poles.LowerRow(); i <= poles.UpperRow(); i++)
            {

                for (j = poles.LowerCol(); j <= poles.UpperCol(); j++)
                {
                    poles[i, j] = temp[i, j - poles.LowerCol()];
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Reverses the array of weights in one direction (see the poles' <see cref="Reverse(NCollection_Array2{Point3D}, int, bool)"/>).</summary>
    public static void Reverse(NCollection_Array2<double> weights, int last, bool uDirection)
    {
        int i, j, l = last;
        if (uDirection)
        {
            l = weights.LowerRow() + (l - weights.LowerRow()) % (weights.ColLength());
            NCollection_Array2<double> temp = new(0,
                                                  weights.ColLength() - 1,
                                                  weights.LowerCol(),
                                                  weights.UpperCol());

            for (i = weights.LowerRow(); i <= l; i++)
            {

                for (j = weights.LowerCol(); j <= weights.UpperCol(); j++)
                {
                    temp[l - i, j] = weights[i, j];
                }
            }

            for (i = l + 1; i <= weights.UpperRow(); i++)
            {

                for (j = weights.LowerCol(); j <= weights.UpperCol(); j++)
                {
                    temp[l + weights.ColLength() - i, j] = weights[i, j];
                }
            }

            for (i = weights.LowerRow(); i <= weights.UpperRow(); i++)
            {

                for (j = weights.LowerCol(); j <= weights.UpperCol(); j++)
                {
                    weights[i, j] = temp[i - weights.LowerRow(), j];
                }
            }
        }
        else
        {
            l = weights.LowerCol() + (l - weights.LowerCol()) % (weights.RowLength());
            NCollection_Array2<double> temp = new(weights.LowerRow(),
                                                  weights.UpperRow(),
                                                  0,
                                                  weights.RowLength() - 1);

            for (j = weights.LowerCol(); j <= l; j++)
            {

                for (i = weights.LowerRow(); i <= weights.UpperRow(); i++)
                {
                    temp[i, l - j] = weights[i, j];
                }
            }

            for (j = l + 1; j <= weights.UpperCol(); j++)
            {

                for (i = weights.LowerRow(); i <= weights.UpperRow(); i++)
                {
                    temp[i, l + weights.RowLength() - j] = weights[i, j];
                }
            }

            for (i = weights.LowerRow(); i <= weights.UpperRow(); i++)
            {

                for (j = weights.LowerCol(); j <= weights.UpperCol(); j++)
                {
                    weights[i, j] = temp[i, j - weights.LowerCol()];
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// False if all the weights in the area [I1, I2] x [J1, J2] are identical, compared with
    /// <paramref name="epsi"/>, or with the Epsilon of the first weight if Epsi is 0.
    /// </summary>
    public static bool IsRational(NCollection_Array2<double> weights,
                                  int i1,
                                  int i2,
                                  int j1,
                                  int j2,
                                  double epsi = 0.0)
    {
        // OCCT: the default tolerance is taken from Weights(I1, I2) (a row index used as a column index), not
        // Weights(I1, J1). OCCT's release build reads that address without a range check; the port reads the same
        // element of the storage (row I1, column I2 counted along the rows) rather than throwing.
        double eps = (epsi > 0.0)
                         ? epsi
                         : Epsilon(weights.AsSpan()[(i1 - weights.LowerRow()) * weights.RowLength() + (i2 - weights.LowerCol())]);
        int i, j;
        int fi = weights.LowerRow(), li = weights.ColLength();
        int fj = weights.LowerCol(), lj = weights.RowLength();

        for (i = i1 - fi; i < i2 - fi; i++)
        {

            for (j = j1 - fj; j < j2 - fj; j++)
            {
                if (Math.Abs(weights[fi + i % li, fj + j % lj] - weights[fi + (i + 1) % li, fj + j % lj])
                    > eps)
                {
                    return true;
                }
            }
        }
        return false;
    }

    //=================================================================================================

    /// <summary>
    /// Copies the coordinates of the poles into FP, row by row if <paramref name="uDirection"/>, column by column
    /// otherwise.
    /// </summary>
    public static void SetPoles(NCollection_Array2<Point3D> poles,
                                NCollection_Array1<double> fp,
                                bool uDirection)
    {
        int i, j, l = fp.Lower();
        int pLowerRow = poles.LowerRow();
        int pUpperRow = poles.UpperRow();
        int pLowerCol = poles.LowerCol();
        int pUpperCol = poles.UpperCol();
        if (uDirection)
        {

            for (i = pLowerRow; i <= pUpperRow; i++)
            {

                for (j = pLowerCol; j <= pUpperCol; j++)
                {
                    Point3D p = poles.Value(i, j);
                    fp[l] = p.X;
                    l++;
                    fp[l] = p.Y;
                    l++;
                    fp[l] = p.Z;
                    l++;
                }
            }
        }
        else
        {

            for (j = pLowerCol; j <= pUpperCol; j++)
            {

                for (i = pLowerRow; i <= pUpperRow; i++)
                {
                    Point3D p = poles.Value(i, j);
                    fp[l] = p.X;
                    l++;
                    fp[l] = p.Y;
                    l++;
                    fp[l] = p.Z;
                    l++;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Copies the homogeneous coordinates of the poles (x * w, y * w, z * w, w) into FP, row by row if
    /// <paramref name="uDirection"/>, column by column otherwise.
    /// </summary>
    public static void SetPoles(NCollection_Array2<Point3D> poles,
                                NCollection_Array2<double> weights,
                                NCollection_Array1<double> fp,
                                bool uDirection)
    {
        int i, j, l = fp.Lower();
        int pLowerRow = poles.LowerRow();
        int pUpperRow = poles.UpperRow();
        int pLowerCol = poles.LowerCol();
        int pUpperCol = poles.UpperCol();
        if (uDirection)
        {

            for (i = pLowerRow; i <= pUpperRow; i++)
            {

                for (j = pLowerCol; j <= pUpperCol; j++)
                {
                    Point3D p = poles.Value(i, j);
                    double w = weights.Value(i, j);
                    fp[l] = p.X * w;
                    l++;
                    fp[l] = p.Y * w;
                    l++;
                    fp[l] = p.Z * w;
                    l++;
                    fp[l] = w;
                    l++;
                }
            }
        }
        else
        {

            for (j = pLowerCol; j <= pUpperCol; j++)
            {

                for (i = pLowerRow; i <= pUpperRow; i++)
                {
                    Point3D p = poles.Value(i, j);
                    double w = weights.Value(i, j);
                    fp[l] = p.X * w;
                    l++;
                    fp[l] = p.Y * w;
                    l++;
                    fp[l] = p.Z * w;
                    l++;
                    fp[l] = w;
                    l++;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Gets the coordinates of the poles from FP (the inverse of <see cref="SetPoles(NCollection_Array2{Point3D}, NCollection_Array1{double}, bool)"/>).</summary>
    public static void GetPoles(NCollection_Array1<double> fp,
                                NCollection_Array2<Point3D> poles,
                                bool uDirection)
    {
        int i, j, l = fp.Lower();
        int pLowerRow = poles.LowerRow();
        int pUpperRow = poles.UpperRow();
        int pLowerCol = poles.LowerCol();
        int pUpperCol = poles.UpperCol();
        if (uDirection)
        {

            for (i = pLowerRow; i <= pUpperRow; i++)
            {

                for (j = pLowerCol; j <= pUpperCol; j++)
                {
                    ref Point3D p = ref poles.ChangeValue(i, j);
                    p = p.WithX(fp[l]);
                    l++;
                    p = p.WithY(fp[l]);
                    l++;
                    p = p.WithZ(fp[l]);
                    l++;
                }
            }
        }
        else
        {

            for (j = pLowerCol; j <= pUpperCol; j++)
            {

                for (i = pLowerRow; i <= pUpperRow; i++)
                {
                    ref Point3D p = ref poles.ChangeValue(i, j);
                    p = p.WithX(fp[l]);
                    l++;
                    p = p.WithY(fp[l]);
                    l++;
                    p = p.WithZ(fp[l]);
                    l++;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Gets the poles and weights from the homogeneous coordinates in FP (the inverse of
    /// <see cref="SetPoles(NCollection_Array2{Point3D}, NCollection_Array2{double}, NCollection_Array1{double}, bool)"/>).
    /// </summary>
    public static void GetPoles(NCollection_Array1<double> fp,
                                NCollection_Array2<Point3D> poles,
                                NCollection_Array2<double> weights,
                                bool uDirection)
    {
        int i, j, l = fp.Lower();
        int pLowerRow = poles.LowerRow();
        int pUpperRow = poles.UpperRow();
        int pLowerCol = poles.LowerCol();
        int pUpperCol = poles.UpperCol();
        if (uDirection)
        {

            for (i = pLowerRow; i <= pUpperRow; i++)
            {

                for (j = pLowerCol; j <= pUpperCol; j++)
                {
                    double w = fp[l + 3];
                    weights[i, j] = w;
                    ref Point3D p = ref poles.ChangeValue(i, j);
                    p = p.WithX(fp[l] / w);
                    l++;
                    p = p.WithY(fp[l] / w);
                    l++;
                    p = p.WithZ(fp[l] / w);
                    l++;
                    l++;
                }
            }
        }
        else
        {

            for (j = pLowerCol; j <= pUpperCol; j++)
            {

                for (i = pLowerRow; i <= pUpperRow; i++)
                {
                    double w = fp[l + 3];
                    weights[i, j] = w;
                    ref Point3D p = ref poles.ChangeValue(i, j);
                    p = p.WithX(fp[l] / w);
                    l++;
                    p = p.WithY(fp[l] / w);
                    l++;
                    p = p.WithZ(fp[l] / w);
                    l++;
                    l++;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Inserts knots in one direction of the surface (see <see cref="BSplCLib.InsertKnots(int, bool, int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, NCollection_Array1{double}, NCollection_Array1{int}?, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array1{int}, double, bool)"/>):
    /// the surface is processed as a curve whose points are the rows (U) or columns (V) of poles.
    /// </summary>
    public static void InsertKnots(bool uDirection,
                                   int degree,
                                   bool periodic,
                                   NCollection_Array2<Point3D> Poles,
                                   NCollection_Array2<double>? weights,
                                   NCollection_Array1<double> knots,
                                   NCollection_Array1<int> mults,
                                   NCollection_Array1<double> addKnots,
                                   NCollection_Array1<int>? addMults,
                                   NCollection_Array2<Point3D> newPoles,
                                   NCollection_Array2<double>? newWeights,
                                   NCollection_Array1<double> newKnots,
                                   NCollection_Array1<int> newMults,
                                   double epsilon,
                                   bool add = true)
    {
        bool rational = weights != null;
        int dim = 3;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.RowLength() * Poles.ColLength());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.RowLength() * newPoles.ColLength());

        if (rational)
        {
            SetPoles(Poles, weights!, poles, uDirection);
        }
        else
        {
            SetPoles(Poles, poles, uDirection);
        }

        if (uDirection)
        {
            dim *= Poles.RowLength();
        }
        else
        {
            dim *= Poles.ColLength();
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(degree,
                             periodic,
                             dim,
                             poles,
                             knots,
                             mults,
                             addKnots,
                             addMults,
                             newpoles,
                             newKnots,
                             newMults,
                             epsilon,
                             add);

        if (rational)
        {
            GetPoles(newpoles, newPoles, newWeights!, uDirection);
        }
        else
        {
            GetPoles(newpoles, newPoles, uDirection);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Removes a knot in one direction of the surface, if the result stays within <paramref name="tolerance"/>
    /// (see <see cref="BSplCLib"/>'s RemoveKnot). Returns false, leaving the outputs unchanged, otherwise.
    /// </summary>
    public static bool RemoveKnot(bool uDirection,
                                  int index,
                                  int mult,
                                  int degree,
                                  bool periodic,
                                  NCollection_Array2<Point3D> Poles,
                                  NCollection_Array2<double>? weights,
                                  NCollection_Array1<double> knots,
                                  NCollection_Array1<int> mults,
                                  NCollection_Array2<Point3D> newPoles,
                                  NCollection_Array2<double>? newWeights,
                                  NCollection_Array1<double> newKnots,
                                  NCollection_Array1<int> newMults,
                                  double tolerance)
    {
        bool rational = weights != null;
        int dim = 3;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.RowLength() * Poles.ColLength());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.RowLength() * newPoles.ColLength());

        if (rational)
        {
            SetPoles(Poles, weights!, poles, uDirection);
        }
        else
        {
            SetPoles(Poles, poles, uDirection);
        }

        if (uDirection)
        {
            dim *= Poles.RowLength();
        }
        else
        {
            dim *= Poles.ColLength();
        }

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.RemoveKnot(index,
                                 mult,
                                 degree,
                                 periodic,
                                 dim,
                                 poles,
                                 knots,
                                 mults,
                                 newpoles,
                                 newKnots,
                                 newMults,
                                 tolerance))
        {
            return false;
        }

        if (rational)
        {
            GetPoles(newpoles, newPoles, newWeights!, uDirection);
        }
        else
        {
            GetPoles(newpoles, newPoles, uDirection);
        }
        return true;
    }

    //=================================================================================================

    /// <summary>Raises the degree in one direction of the surface (see <see cref="BSplCLib"/>'s IncreaseDegree).</summary>
    public static void IncreaseDegree(bool uDirection,
                                      int degree,
                                      int newDegree,
                                      bool periodic,
                                      NCollection_Array2<Point3D> Poles,
                                      NCollection_Array2<double>? weights,
                                      NCollection_Array1<double> knots,
                                      NCollection_Array1<int> mults,
                                      NCollection_Array2<Point3D> newPoles,
                                      NCollection_Array2<double>? newWeights,
                                      NCollection_Array1<double> newKnots,
                                      NCollection_Array1<int> newMults)
    {
        bool rational = weights != null;
        int dim = 3;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.RowLength() * Poles.ColLength());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.RowLength() * newPoles.ColLength());

        if (rational)
        {
            SetPoles(Poles, weights!, poles, uDirection);
        }
        else
        {
            SetPoles(Poles, poles, uDirection);
        }

        if (uDirection)
        {
            dim *= Poles.RowLength();
        }
        else
        {
            dim *= Poles.ColLength();
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(degree,
                                newDegree,
                                periodic,
                                dim,
                                poles,
                                knots,
                                mults,
                                newpoles,
                                newKnots,
                                newMults);

        if (rational)
        {
            GetPoles(newpoles, newPoles, newWeights!, uDirection);
        }
        else
        {
            GetPoles(newpoles, newPoles, uDirection);
        }
    }

    //=================================================================================================

    /// <summary>Makes one direction of a periodic surface non-periodic (see <see cref="BSplCLib"/>'s Unperiodize).</summary>
    public static void Unperiodize(bool uDirection,
                                   int degree,
                                   NCollection_Array1<int> mults,
                                   NCollection_Array1<double> knots,
                                   NCollection_Array2<Point3D> Poles,
                                   NCollection_Array2<double>? weights,
                                   NCollection_Array1<int> newMults,
                                   NCollection_Array1<double> newKnots,
                                   NCollection_Array2<Point3D> newPoles,
                                   NCollection_Array2<double>? newWeights)
    {
        bool rational = weights != null;
        int dim = 3;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.RowLength() * Poles.ColLength());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.RowLength() * newPoles.ColLength());

        if (rational)
        {
            SetPoles(Poles, weights!, poles, uDirection);
        }
        else
        {
            SetPoles(Poles, poles, uDirection);
        }

        if (uDirection)
        {
            dim *= Poles.RowLength();
        }
        else
        {
            dim *= Poles.ColLength();
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Unperiodize(degree, dim, mults, knots, poles, newMults, newKnots, newpoles);

        if (rational)
        {
            GetPoles(newpoles, newPoles, newWeights!, uDirection);
        }
        else
        {
            GetPoles(newpoles, newPoles, uDirection);
        }
    }

    /// <summary>No weights: the argument for a non-rational surface (null).</summary>
    public static NCollection_Array2<double>? NoWeights() => null;

    //=======================================================================
    // function : BuildCache
    // purpose  : Stores theTaylor expansion normalized between 0,1 in the
    //           Cache : in case of  a rational function the Poles are
    //           stored in homogeneous form
    //=======================================================================

    /// <summary>
    /// Computes the Taylor expansion of the span of (U, V), normalized to [0, 1]: the coefficients go into
    /// CachePoles (homogeneous for a rational surface) and CacheWeights (required when Weights is given).
    /// </summary>
    public static void BuildCache(double u,
                                  double v,
                                  double uSpanDomain,
                                  double vSpanDomain,
                                  bool uPeriodic,
                                  bool vPeriodic,
                                  int uDegree,
                                  int vDegree,
                                  int uIndex,
                                  int vIndex,
                                  NCollection_Array1<double> uFlatKnots,
                                  NCollection_Array1<double> vFlatKnots,
                                  NCollection_Array2<Point3D> poles,
                                  NCollection_Array2<double>? weights,
                                  NCollection_Array2<Point3D> cachePoles,
                                  NCollection_Array2<double>? cacheWeights)
    {
        bool rational, rational_u, rational_v, flag_u_or_v;
        int kk, d1, d1p1, d2, d2p1, ii, jj, iii, jjj, index;
        double u1, min_degree_domain, max_degree_domain, f, u2;
        Span<double> factor = stackalloc double[2];
        if (weights != null)
        {
            rational_u = rational_v = true;
        }
        else
        {
            rational_u = rational_v = false;
        }
        ValidateBSplineDegree(uDegree, vDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[DcPolesSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcDersSize]);
        flag_u_or_v = PrepareEval(u,
                                  v,
                                  uIndex,
                                  vIndex,
                                  uDegree,
                                  vDegree,
                                  rational_u,
                                  rational_v,
                                  uPeriodic,
                                  vPeriodic,
                                  poles,
                                  weights,
                                  uFlatKnots,
                                  vFlatKnots,
                                  (global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults()),
                                  (global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults()),
                                  out u1,
                                  out u2,
                                  out d1,
                                  out d2,
                                  out rational,
                                  dc);
        d1p1 = d1 + 1;
        d2p1 = d2 + 1;
        if (rational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, d1, dc.knots1, 4 * d2p1, dc.poles);

            for (kk = 0; kk <= d1; kk++)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, d2, dc.knots2, 4, dc.poles[(kk * 4 * d2p1)..]);
            }
            if (flag_u_or_v)
            {
                min_degree_domain = uSpanDomain;
                max_degree_domain = vSpanDomain;
            }
            else
            {
                min_degree_domain = vSpanDomain;
                max_degree_domain = uSpanDomain;
            }
            factor[0] = 1.0e0;

            for (ii = 0; ii <= d2; ii++)
            {
                iii = ii + 1;
                factor[1] = 1.0e0;

                for (jj = 0; jj <= d1; jj++)
                {
                    jjj = jj + 1;
                    index = jj * d2p1 + ii;
                    index = index << 2;
                    f = factor[0] * factor[1];
                    double x = f * dc.poles[index];
                    index++;
                    double y = f * dc.poles[index];
                    index++;
                    double z = f * dc.poles[index];
                    cachePoles[iii, jjj] = new Point3D(x, y, z);
                    index++;
                    cacheWeights![iii, jjj] = f * dc.poles[index];
                    factor[1] *= min_degree_domain / (double)(jjj);
                }
                factor[0] *= max_degree_domain / (double)(iii);
            }
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, d1, dc.knots1, 3 * d2p1, dc.poles);

            for (kk = 0; kk <= d1; kk++)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, d2, dc.knots2, 3, dc.poles[(kk * 3 * d2p1)..]);
            }
            if (flag_u_or_v)
            {
                min_degree_domain = uSpanDomain;
                max_degree_domain = vSpanDomain;
            }
            else
            {
                min_degree_domain = vSpanDomain;
                max_degree_domain = uSpanDomain;
            }
            factor[0] = 1.0e0;

            for (ii = 0; ii <= d2; ii++)
            {
                iii = ii + 1;
                factor[1] = 1.0e0;

                for (jj = 0; jj <= d1; jj++)
                {
                    jjj = jj + 1;
                    index = jj * d2p1 + ii;
                    index = (index << 1) + index;
                    f = factor[0] * factor[1];
                    double x = f * dc.poles[index];
                    index++;
                    double y = f * dc.poles[index];
                    index++;
                    double z = f * dc.poles[index];
                    cachePoles[iii, jjj] = new Point3D(x, y, z);
                    factor[1] *= min_degree_domain / (double)(jjj);
                }
                factor[0] *= max_degree_domain / (double)(iii);
            }
            if (weights != null)
            {
                //
                // means that PrepareEval did found out that the surface was
                // locally polynomial but since the surface is constructed
                // with some weights we need to set the weight polynomial to constant
                //

                for (ii = 1; ii <= d2p1; ii++)
                {

                    for (jj = 1; jj <= d1p1; jj++)
                    {
                        cacheWeights![ii, jj] = 0.0e0;
                    }
                }
                cacheWeights![1, 1] = 1.0e0;
            }
        }
    }

    /// <summary>
    /// Computes the Taylor expansion of the span of (theU, theV), normalized to [0, 1], into theCacheArray in the
    /// layout of <c>BSplSLib_Cache</c>: (max degree + 1) rows of (min degree + 1) coefficients of x, y, z (and w for a
    /// rational surface).
    /// </summary>
    public static void BuildCache(double u,
                                  double v,
                                  double uSpanDomain,
                                  double vSpanDomain,
                                  bool uPeriodic,
                                  bool vPeriodic,
                                  int uDegree,
                                  int vDegree,
                                  int uIndex,
                                  int vIndex,
                                  NCollection_Array1<double> uFlatKnots,
                                  NCollection_Array1<double> vFlatKnots,
                                  NCollection_Array2<Point3D> poles,
                                  NCollection_Array2<double>? weights,
                                  NCollection_Array2<double> cacheArray)
    {
        bool flag_u_or_v;
        int d1, d2;
        double u1, u2;
        bool isRationalOnParam = (weights != null);
        bool isRational;

        ValidateBSplineDegree(uDegree, vDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[DcPolesSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcKnotsSize(uDegree, vDegree)],
                                        stackalloc double[DcDersSize]);
        flag_u_or_v = PrepareEval(u,
                                  v,
                                  uIndex,
                                  vIndex,
                                  uDegree,
                                  vDegree,
                                  isRationalOnParam,
                                  isRationalOnParam,
                                  uPeriodic,
                                  vPeriodic,
                                  poles,
                                  weights,
                                  uFlatKnots,
                                  vFlatKnots,
                                  (global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults()),
                                  (global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults()),
                                  out u1,
                                  out u2,
                                  out d1,
                                  out d2,
                                  out isRational,
                                  dc);

        int d2p1 = d2 + 1;
        int dimension = isRational ? 4 : 3;
        int cacheShift = // helps to store weights when PrepareEval did not found that the
                          // surface is locally polynomial
            (isRationalOnParam && !isRational) ? dimension + 1 : dimension;

        Span<double> domains = stackalloc double[2];
        // aDomains[0] corresponds to variable with minimal degree
        // aDomains[1] corresponds to variable with maximal degree
        if (flag_u_or_v)
        {
            domains[0] = uSpanDomain;
            domains[1] = vSpanDomain;
        }
        else
        {
            domains[0] = vSpanDomain;
            domains[1] = uSpanDomain;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u1, d1, d1, dc.knots1, dimension * d2p1, dc.poles);
        for (int kk = 0; kk <= d1; kk++)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u2, d2, d2, dc.knots2, dimension, dc.poles[(kk * dimension * d2p1)..]);
        }

        // theCacheArray(theCacheArray.LowerRow(), theCacheArray.LowerCol()) on
        Span<double> aCacheArray = cacheArray.AsSpan();
        int cache = 0;

        Span<double> factors = stackalloc double[2];
        // aFactors[0] corresponds to variable with minimal degree
        // aFactors[1] corresponds to variable with maximal degree
        factors[1] = 1.0;
        int row, col, i;
        double coeff;
        for (row = 0; row <= d2; row++)
        {
            factors[0] = 1.0;
            for (col = 0; col <= d1; col++)
            {
                ReadOnlySpan<double> polyCoeffs = dc.poles[((col * d2p1 + row) * dimension)..];
                coeff = factors[0] * factors[1];
                for (i = 0; i < dimension; i++)
                {
                    aCacheArray[cache + i] = polyCoeffs[i] * coeff;
                }
                cache += cacheShift;
                factors[0] *= domains[0] / (col + 1);
            }
            factors[1] *= domains[1] / (row + 1);
        }

        // Fill the weights for the surface which is not locally polynomial
        if (cacheShift > dimension)
        {
            cache = 0;
            cache += cacheShift - 1;
            for (row = 0; row <= d2; row++)
            {
                for (col = 0; col <= d1; col++)
                {
                    aCacheArray[cache] = 0.0;
                    cache += cacheShift;
                }
            }
            cacheArray.SetValue(cacheArray.LowerRow(),
                                   cacheArray.LowerCol() + cacheShift - 1,
                                   1.0);
        }
    }

    //=======================================================================
    // function : CacheD0
    // purpose  : Evaluates the polynomial cache of the Bspline Curve
    //
    //=======================================================================

    /// <summary>
    /// Evaluates the point at (U, V) from the cache built by <see cref="BuildCache(double, double, double, double, bool, bool, int, int, int, int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array2{Point3D}, NCollection_Array2{double}?, NCollection_Array2{Point3D}, NCollection_Array2{double}?)"/>:
    /// UCacheParameter and VCacheParameter are where the cache was built, USpanLenght and VSpanLength normalize
    /// the polynomials. The cache must be valid for the point.
    /// </summary>
    public static void CacheD0(double uParameter,
                               double vParameter,
                               int uDegree,
                               int vDegree,
                               double uCacheParameter,
                               double vCacheParameter,
                               double uSpanLenght,
                               double vSpanLenght,
                               NCollection_Array2<Point3D> polesArray,
                               NCollection_Array2<double>? weightsArray,
                               out Point3D aPoint)
    {
        //
        // the CacheParameter is where the cache polynomial was evaluated in homogeneous
        // form
        // the SpanLenght     is the normalizing factor so that the polynomial is between
        // 0 and 1
        int
          //    ii,
          dimension,
          min_degree, max_degree;

        Span<double> new_parameter = stackalloc double[2];
        Span<double> inverse = stackalloc double[1];

        // OCCT: the cache's start is taken as PolesArray(LowerCol, LowerRow) (row and column swapped); the same
        // element as long as the lower bounds are equal, which they are for every cache OCCT builds.
        ReadOnlySpan<double> pArray = CoordsFrom(polesArray, polesArray.LowerCol(), polesArray.LowerRow());
        Span<double> myPoint = stackalloc double[3];
        if (uDegree <= vDegree)
        {
            min_degree = uDegree;
            max_degree = vDegree;
            new_parameter[1] = (uParameter - uCacheParameter) / uSpanLenght;
            new_parameter[0] = (vParameter - vCacheParameter) / vSpanLenght;
            dimension = 3 * (uDegree + 1);
        }
        else
        {
            min_degree = vDegree;
            max_degree = uDegree;
            new_parameter[0] = (uParameter - uCacheParameter) / uSpanLenght;
            new_parameter[1] = (vParameter - vCacheParameter) / vSpanLenght;
            dimension = 3 * (vDegree + 1);
        }
        Span<double> locpoles = dimension <= 128 ? stackalloc double[dimension] : new double[dimension];

        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(new_parameter[0],
                                        max_degree,
                                        dimension,
                                        max_degree * dimension,
                                        pArray,
                                        locpoles);

        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                        min_degree,
                                        3,
                                        (min_degree << 1) + min_degree,
                                        locpoles,
                                        myPoint);
        if (weightsArray != null)
        {
            dimension = min_degree + 1;
            NCollection_Array2<double> refWeights = weightsArray;
            // OCCT: row and column swapped, as for the poles
            ReadOnlySpan<double> wArray = ValuesFrom(refWeights, refWeights.LowerCol(), refWeights.LowerRow());
            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(new_parameter[0],
                                            max_degree,
                                            dimension,
                                            max_degree * dimension,
                                            wArray,
                                            locpoles);

            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                            min_degree,
                                            1,
                                            min_degree,
                                            locpoles,
                                            inverse);
            inverse[0] = 1.0e0 / inverse[0];

            myPoint[0] *= inverse[0];
            myPoint[1] *= inverse[0];
            myPoint[2] *= inverse[0];
        }
        aPoint = new Point3D(myPoint[0], myPoint[1], myPoint[2]);
    }

    /// <summary>Evaluates a Bezier surface at (U, V) from the coefficients computed by <see cref="PolesCoefficients(NCollection_Array2{Point3D}, NCollection_Array2{double}?, NCollection_Array2{Point3D}, NCollection_Array2{double}?)"/> (Bezier surfaces only).</summary>
    public static void CoefsD0(double u,
                               double v,
                               NCollection_Array2<Point3D> poles,
                               NCollection_Array2<double>? weights,
                               out Point3D point)
    {
        // OCCT: the V degree (RowLength - 1) is passed as UDegree and the U degree as VDegree, while
        // PolesCoefficients returns the coefficients indexed (U, V): with at least as many U poles as V poles
        // this evaluates the surface at (V, U) with the U and V derivatives exchanged, with fewer it is
        // meaningless. No OCCT code calls the Coefs functions.
        SurfaceBSplSLib.CacheD0(u,
                         v,
                         poles.RowLength() - 1,
                         poles.ColLength() - 1,
                         0.0,
                         0.0,
                         1.0,
                         1.0,
                         poles,
                         weights,
                         out point);
    }

    //=======================================================================
    // function : CacheD1
    // purpose  : Evaluates the polynomial cache of the Bspline Curve
    //
    //=======================================================================

    /// <summary>Evaluates the point and first derivatives at (U, V) from the cache (see <see cref="CacheD0"/>).</summary>
    public static void CacheD1(double uParameter,
                               double vParameter,
                               int uDegree,
                               int vDegree,
                               double uCacheParameter,
                               double vCacheParameter,
                               double uSpanLenght,
                               double vSpanLenght,
                               NCollection_Array2<Point3D> polesArray,
                               NCollection_Array2<double>? weightsArray,
                               out Point3D aPoint,
                               out Vector3D aVecU,
                               out Vector3D aVecV)
    {
        //
        // the CacheParameter is where the cache polynomial was evaluated in homogeneous
        // form
        // the SpanLenght     is the normalizing factor so that the polynomial is between
        // 0 and 1
        int
          //    ii,
          //  jj,
          //  kk,
          dimension,
          min_degree, max_degree;

        double inverse_min, inverse_max;
        Span<double> new_parameter = stackalloc double[2];

        // OCCT: row and column swapped (see CacheD0)
        ReadOnlySpan<double> pArray = CoordsFrom(polesArray, polesArray.LowerCol(), polesArray.LowerRow());
        // local_poles_array[2][2][3]: [i][j][k] at i * 6 + j * 3 + k; local_poles_and_weights_array[2][2][4]: at
        // i * 8 + j * 4 + k; local_weights_array[2][2]: at i * 2 + j
        Span<double> local_poles_array = stackalloc double[12];
        Span<double> local_poles_and_weights_array = stackalloc double[16];
        Span<double> local_weights_array = stackalloc double[4];
        Span<double> my_vec_min = stackalloc double[3];
        Span<double> my_vec_max = stackalloc double[3];
        Span<double> my_point = stackalloc double[3];
        //
        // initialize in case of rational evaluation
        // because RationalDerivative will use all
        // the coefficients
        //
        // (stackalloc zeroes the three local arrays, as OCCT does here when WeightsArray is given)

        bool uMin;
        if (uDegree <= vDegree)
        {
            min_degree = uDegree;
            max_degree = vDegree;
            inverse_min = 1.0e0 / uSpanLenght;
            inverse_max = 1.0e0 / vSpanLenght;
            new_parameter[0] = (vParameter - vCacheParameter) * inverse_max;
            new_parameter[1] = (uParameter - uCacheParameter) * inverse_min;

            dimension = 3 * (uDegree + 1);
            uMin = true; // my_vec_min is aVecU, my_vec_max is aVecV
        }
        else
        {
            min_degree = vDegree;
            max_degree = uDegree;
            inverse_min = 1.0e0 / vSpanLenght;
            inverse_max = 1.0e0 / uSpanLenght;
            new_parameter[0] = (uParameter - uCacheParameter) * inverse_max;
            new_parameter[1] = (vParameter - vCacheParameter) * inverse_min;
            dimension = 3 * (vDegree + 1);
            uMin = false; // my_vec_min is aVecV, my_vec_max is aVecU
        }

        int locSize = 2 * dimension;
        Span<double> locpoles = locSize <= 128 ? stackalloc double[locSize] : new double[locSize];

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[0], 1, max_degree, dimension, pArray, locpoles);

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[1], 1, min_degree, 3, locpoles, local_poles_array);
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                        min_degree,
                                        3,
                                        (min_degree << 1) + min_degree,
                                        locpoles[dimension..],
                                        local_poles_array[6..]);

        if (weightsArray != null)
        {
            dimension = min_degree + 1;
            NCollection_Array2<double> refWeights = weightsArray;
            // OCCT: row and column swapped (see CacheD0)
            ReadOnlySpan<double> wArray = ValuesFrom(refWeights, refWeights.LowerCol(), refWeights.LowerRow());
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[0], 1, max_degree, dimension, wArray, locpoles);

            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[1],
                                1,
                                min_degree,
                                1,
                                locpoles,
                                local_weights_array);
            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                            min_degree,
                                            1,
                                            min_degree,
                                            locpoles[dimension..],
                                            local_weights_array[2..]);

            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    local_poles_and_weights_array[i * 8 + j * 4 + 0] = local_poles_array[i * 6 + j * 3 + 0];
                    local_poles_and_weights_array[i * 8 + j * 4 + 1] = local_poles_array[i * 6 + j * 3 + 1];
                    local_poles_and_weights_array[i * 8 + j * 4 + 2] = local_poles_array[i * 6 + j * 3 + 2];
                    local_poles_and_weights_array[i * 8 + j * 4 + 3] = local_weights_array[i * 2 + j];
                }
            }

            SurfaceBSplSLib.RationalDerivative(1,
                                        1,
                                        1,
                                        1,
                                        local_poles_and_weights_array,
                                        local_poles_array);
        }

        my_point[0] = local_poles_array[0];
        my_vec_min[0] = inverse_min * local_poles_array[3];
        my_vec_max[0] = inverse_max * local_poles_array[6];

        my_point[1] = local_poles_array[1];
        my_vec_min[1] = inverse_min * local_poles_array[4];
        my_vec_max[1] = inverse_max * local_poles_array[7];

        my_point[2] = local_poles_array[2];
        my_vec_min[2] = inverse_min * local_poles_array[5];
        my_vec_max[2] = inverse_max * local_poles_array[8];

        aPoint = new Point3D(my_point[0], my_point[1], my_point[2]);
        Vector3D vecMin = new(my_vec_min[0], my_vec_min[1], my_vec_min[2]);
        Vector3D vecMax = new(my_vec_max[0], my_vec_max[1], my_vec_max[2]);
        aVecU = uMin ? vecMin : vecMax;
        aVecV = uMin ? vecMax : vecMin;
    }

    /// <summary>Evaluates a Bezier surface and its first derivatives at (U, V) from its coefficients (see <see cref="CoefsD0"/>).</summary>
    public static void CoefsD1(double u,
                               double v,
                               NCollection_Array2<Point3D> poles,
                               NCollection_Array2<double>? weights,
                               out Point3D point,
                               out Vector3D vecU,
                               out Vector3D vecV)
    {
        // OCCT: the V degree (RowLength - 1) is passed as UDegree and the U degree as VDegree, while
        // PolesCoefficients returns the coefficients indexed (U, V): with at least as many U poles as V poles
        // this evaluates the surface at (V, U) with the U and V derivatives exchanged, with fewer it is
        // meaningless. No OCCT code calls the Coefs functions.
        SurfaceBSplSLib.CacheD1(u,
                         v,
                         poles.RowLength() - 1,
                         poles.ColLength() - 1,
                         0.0,
                         0.0,
                         1.0,
                         1.0,
                         poles,
                         weights,
                         out point,
                         out vecU,
                         out vecV);
    }

    //=======================================================================
    // function : CacheD2
    // purpose  : Evaluates the polynomial cache of the Bspline Curve
    //
    //=======================================================================

    /// <summary>Evaluates the point and the derivatives up to order 2 at (U, V) from the cache (see <see cref="CacheD0"/>).</summary>
    public static void CacheD2(double uParameter,
                               double vParameter,
                               int uDegree,
                               int vDegree,
                               double uCacheParameter,
                               double vCacheParameter,
                               double uSpanLenght,
                               double vSpanLenght,
                               NCollection_Array2<Point3D> polesArray,
                               NCollection_Array2<double>? weightsArray,
                               out Point3D aPoint,
                               out Vector3D aVecU,
                               out Vector3D aVecV,
                               out Vector3D aVecUU,
                               out Vector3D aVecUV,
                               out Vector3D aVecVV)
    {
        //
        // the CacheParameter is where the cache polynomial was evaluated in homogeneous
        // form
        // the SpanLenght     is the normalizing factor so that the polynomial is between
        // 0 and 1
        int ii,
          //  jj,
          kk, index, dimension, min_degree, max_degree;

        double inverse_min, inverse_max;
        Span<double> new_parameter = stackalloc double[2];

        // OCCT: row and column swapped (see CacheD0)
        ReadOnlySpan<double> pArray = CoordsFrom(polesArray, polesArray.LowerCol(), polesArray.LowerRow());
        // local_poles_array[3][3][3]: [i][j][k] at i * 9 + j * 3 + k; local_poles_and_weights_array[3][3][4]: at
        // i * 12 + j * 4 + k; local_weights_array[3][3]: at i * 3 + j
        Span<double> local_poles_array = stackalloc double[27];
        Span<double> local_poles_and_weights_array = stackalloc double[36];
        Span<double> local_weights_array = stackalloc double[9];
        Span<double> my_point = stackalloc double[3];
        Span<double> my_vec_min = stackalloc double[3];
        Span<double> my_vec_max = stackalloc double[3];
        Span<double> my_vec_min_min = stackalloc double[3];
        Span<double> my_vec_max_max = stackalloc double[3];
        Span<double> my_vec_min_max = stackalloc double[3];

        //
        // initialize in case the min and max degree are less than 2
        //
        // (stackalloc zeroes local_poles_array, as OCCT does here)
        //
        // initialize in case of rational evaluation
        // because RationalDerivative will use all
        // the coefficients
        //
        // (stackalloc zeroes local_poles_and_weights_array and local_weights_array, as OCCT does here when
        // WeightsArray is given)

        bool uMin;
        if (uDegree <= vDegree)
        {
            min_degree = uDegree;
            max_degree = vDegree;
            inverse_min = 1.0e0 / uSpanLenght;
            inverse_max = 1.0e0 / vSpanLenght;
            new_parameter[0] = (vParameter - vCacheParameter) * inverse_max;
            new_parameter[1] = (uParameter - uCacheParameter) * inverse_min;

            dimension = 3 * (uDegree + 1);
            uMin = true; // min is U (aVecU, aVecUU), max is V (aVecV, aVecVV)
        }
        else
        {
            min_degree = vDegree;
            max_degree = uDegree;
            inverse_min = 1.0e0 / vSpanLenght;
            inverse_max = 1.0e0 / uSpanLenght;
            new_parameter[0] = (uParameter - uCacheParameter) * inverse_max;
            new_parameter[1] = (vParameter - vCacheParameter) * inverse_min;
            dimension = 3 * (vDegree + 1);
            uMin = false; // min is V (aVecV, aVecVV), max is U (aVecU, aVecUU)
        }

        int locSize = 3 * dimension;
        Span<double> locpoles = locSize <= 128 ? stackalloc double[locSize] : new double[locSize];

        //
        // initialize in case min or max degree are less than 2
        //
        int minIndMax = 2;
        if (max_degree < 2)
        {
            minIndMax = max_degree;
        }
        int minIndMin = 2;
        if (min_degree < 2)
        {
            minIndMin = min_degree;
        }

        index = minIndMax * dimension;

        for (ii = minIndMax; ii < 3; ii++)
        {

            for (kk = 0; kk < dimension; kk++)
            {
                locpoles[index] = 0.0e0;
                index += 1;
            }
        }

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[0], minIndMax, max_degree, dimension, pArray, locpoles);

        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[1],
                            minIndMin,
                            min_degree,
                            3,
                            locpoles,
                            local_poles_array);
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[1],
                            1,
                            min_degree,
                            3,
                            locpoles[dimension..],
                            local_poles_array[9..]);

        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                        min_degree,
                                        3,
                                        (min_degree << 1) + min_degree,
                                        locpoles[(dimension + dimension)..],
                                        local_poles_array[18..]);

        if (weightsArray != null)
        {
            // OCCT: locpoles is zeroed above row MinIndMax for the poles' dimension only; with the weights' smaller
            // dimension, a max degree below 2 leaves row 2 holding pole values, which are then taken as the second
            // derivative of the weights: the second derivatives of a rational surface of degrees (1, 1) are wrong
            dimension = min_degree + 1;
            NCollection_Array2<double> refWeights = weightsArray;
            // OCCT: row and column swapped (see CacheD0)
            ReadOnlySpan<double> wArray = ValuesFrom(refWeights, refWeights.LowerCol(), refWeights.LowerRow());
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[0],
                                minIndMax,
                                max_degree,
                                dimension,
                                wArray,
                                locpoles);

            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[1],
                                minIndMin,
                                min_degree,
                                1,
                                locpoles,
                                local_weights_array);
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(new_parameter[1],
                                1,
                                min_degree,
                                1,
                                locpoles[dimension..],
                                local_weights_array[3..]);
            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                            min_degree,
                                            1,
                                            min_degree,
                                            locpoles[(dimension + dimension)..],
                                            local_weights_array[6..]);

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    local_poles_and_weights_array[i * 12 + j * 4 + 0] = local_poles_array[i * 9 + j * 3 + 0];
                    local_poles_and_weights_array[i * 12 + j * 4 + 1] = local_poles_array[i * 9 + j * 3 + 1];
                    local_poles_and_weights_array[i * 12 + j * 4 + 2] = local_poles_array[i * 9 + j * 3 + 2];
                }
            }
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    local_poles_and_weights_array[i * 12 + j * 4 + 3] = local_weights_array[i * 3 + j];
                }
            }

            SurfaceBSplSLib.RationalDerivative(2,
                                        2,
                                        2,
                                        2,
                                        local_poles_and_weights_array,
                                        local_poles_array);
        }

        double minmin = inverse_min * inverse_min;
        double minmax = inverse_min * inverse_max;
        double maxmax = inverse_max * inverse_max;

        for (int k = 0; k < 3; k++)
        {
            my_point[k] = local_poles_array[k];                         // [0][0][k]
            my_vec_min[k] = inverse_min * local_poles_array[3 + k];     // [0][1][k]
            my_vec_max[k] = inverse_max * local_poles_array[9 + k];     // [1][0][k]
            my_vec_min_min[k] = minmin * local_poles_array[6 + k];      // [0][2][k]
            my_vec_min_max[k] = minmax * local_poles_array[12 + k];     // [1][1][k]
            my_vec_max_max[k] = maxmax * local_poles_array[18 + k];     // [2][0][k]
        }

        aPoint = new Point3D(my_point[0], my_point[1], my_point[2]);
        Vector3D vecMin = new(my_vec_min[0], my_vec_min[1], my_vec_min[2]);
        Vector3D vecMax = new(my_vec_max[0], my_vec_max[1], my_vec_max[2]);
        Vector3D vecMinMin = new(my_vec_min_min[0], my_vec_min_min[1], my_vec_min_min[2]);
        Vector3D vecMaxMax = new(my_vec_max_max[0], my_vec_max_max[1], my_vec_max_max[2]);
        aVecUV = new Vector3D(my_vec_min_max[0], my_vec_min_max[1], my_vec_min_max[2]);
        aVecU = uMin ? vecMin : vecMax;
        aVecV = uMin ? vecMax : vecMin;
        aVecUU = uMin ? vecMinMin : vecMaxMax;
        aVecVV = uMin ? vecMaxMax : vecMinMin;
    }

    /// <summary>Evaluates a Bezier surface and its derivatives up to order 2 at (U, V) from its coefficients (see <see cref="CoefsD0"/>).</summary>
    public static void CoefsD2(double u,
                               double v,
                               NCollection_Array2<Point3D> poles,
                               NCollection_Array2<double>? weights,
                               out Point3D point,
                               out Vector3D vecU,
                               out Vector3D vecV,
                               out Vector3D vecUU,
                               out Vector3D vecUV,
                               out Vector3D vecVV)
    {
        // OCCT: the V degree (RowLength - 1) is passed as UDegree and the U degree as VDegree, while
        // PolesCoefficients returns the coefficients indexed (U, V): with at least as many U poles as V poles
        // this evaluates the surface at (V, U) with the U and V derivatives exchanged, with fewer it is
        // meaningless. No OCCT code calls the Coefs functions.
        SurfaceBSplSLib.CacheD2(u,
                         v,
                         poles.RowLength() - 1,
                         poles.ColLength() - 1,
                         0.0,
                         0.0,
                         1.0,
                         1.0,
                         poles,
                         weights,
                         out point,
                         out vecU,
                         out vecV,
                         out vecUU,
                         out vecUV,
                         out vecVV);
    }

    /// <summary>
    /// The Taylor expansion of a non-rational Bezier surface at (0, 0) (Bezier surfaces only), for
    /// <see cref="CoefsD0"/>.
    /// </summary>
    public static void PolesCoefficients(NCollection_Array2<Point3D> poles,
                                         NCollection_Array2<Point3D> cachePoles)
    {
        SurfaceBSplSLib.PolesCoefficients(poles, SurfaceBSplSLib.NoWeights(), cachePoles, SurfaceBSplSLib.NoWeights());
    }

    //=================================================================================================

    /// <summary>
    /// The Taylor expansion of a Bezier surface at (0, 0), through <see cref="BuildCache(double, double, double, double, bool, bool, int, int, int, int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array2{Point3D}, NCollection_Array2{double}?, NCollection_Array2{Point3D}, NCollection_Array2{double}?)"/>
    /// (Bezier surfaces only); CacheWeights is required when Weights is given.
    /// </summary>
    public static void PolesCoefficients(NCollection_Array2<Point3D> poles,
                                         NCollection_Array2<double>? weights,
                                         NCollection_Array2<Point3D> cachePoles,
                                         NCollection_Array2<double>? cacheWeights)
    {
        int i;
        int uclas = poles.ColLength();
        int vclas = poles.RowLength();
        NCollection_Array1<double> biduflatknots = new(1, uclas << 1);
        NCollection_Array1<double> bidvflatknots = new(1, vclas << 1);

        for (i = 1; i <= uclas; i++)
        {
            biduflatknots[i] = 0.0;
            biduflatknots[i + uclas] = 1.0;
        }

        for (i = 1; i <= vclas; i++)
        {
            bidvflatknots[i] = 0.0;
            bidvflatknots[i + vclas] = 1.0;
        }
        if (uclas > vclas)
        {
            SurfaceBSplSLib.BuildCache(0.0,
                                0.0,
                                1.0,
                                1.0,
                                false,
                                false,
                                uclas - 1,
                                vclas - 1,
                                0,
                                0,
                                biduflatknots,
                                bidvflatknots,
                                poles,
                                weights,
                                cachePoles,
                                cacheWeights);
        }
        else
        {
            // BuilCache exige que les resultats soient formates en [MaxCoeff,MinCoeff]
            NCollection_Array2<Point3D> cPoles = new(1, vclas, 1, uclas);
            NCollection_Array2<double> cWeights = new(1, vclas, 1, uclas);
            int ii, jj;
            SurfaceBSplSLib.BuildCache(0.0,
                                0.0,
                                1.0,
                                1.0,
                                false,
                                false,
                                uclas - 1,
                                vclas - 1,
                                0,
                                0,
                                biduflatknots,
                                bidvflatknots,
                                poles,
                                weights,
                                cPoles,
                                cWeights);
            if (weights == null)
            {

                for (ii = 1; ii <= uclas; ii++)
                {

                    for (jj = 1; jj <= vclas; jj++)
                    {
                        cachePoles[ii, jj] = cPoles[jj, ii];
                    }
                }
            }
            else
            {

                for (ii = 1; ii <= uclas; ii++)
                {

                    for (jj = 1; jj <= vclas; jj++)
                    {
                        cachePoles[ii, jj] = cPoles[jj, ii];
                        cacheWeights![ii, jj] = cWeights[jj, ii];
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : MovePoint
    // purpose  : Find the new poles which allows  an old point (with a
    //           given  u and v as parameters) to reach a new position
    //=======================================================================

    /// <summary>
    /// Finds the new poles that move the point at (U, V) by <paramref name="displ"/>. UIndex1..UIndex2 and
    /// VIndex1..VIndex2 are the ranges of poles that may move ((1, NbPoles - 1) or (2, NbPoles): no constraint on
    /// one side; (2, NbPoles - 1): the ends stay; (1, NbPoles): a rigid move). If the B-spline basis can't be
    /// computed, the indices are 0 and NewPoles is unchanged.
    /// </summary>
    public static void MovePoint(double u,
                                 double v,
                                 in Vector3D displ,
                                 int uIndex1,
                                 int uIndex2,
                                 int vIndex1,
                                 int vIndex2,
                                 int uDegree,
                                 int vDegree,
                                 bool rational,
                                 NCollection_Array2<Point3D> poles,
                                 NCollection_Array2<double> weights,
                                 NCollection_Array1<double> uFlatKnots,
                                 NCollection_Array1<double> vFlatKnots,
                                 out int uFirstIndex,
                                 out int uLastIndex,
                                 out int vFirstIndex,
                                 out int vLastIndex,
                                 NCollection_Array2<Point3D> newPoles)
    {
        // calculate the UBSplineBasis in the parameter U
        Matrix ubSplineBasis = new(1, 1, 1, uDegree + 1);
        int errorCod1 = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(0,
                                                  uDegree + 1,
                                                  uFlatKnots,
                                                  u,
                                                  out int uFirstNonZeroBsplineIndex,
                                                  ubSplineBasis);
        // calculate the VBSplineBasis in the parameter V
        Matrix vbSplineBasis = new(1, 1, 1, vDegree + 1);
        int errorCod2 = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(0,
                                                  vDegree + 1,
                                                  vFlatKnots,
                                                  v,
                                                  out int vFirstNonZeroBsplineIndex,
                                                  vbSplineBasis);
        if (errorCod1 != 0 || errorCod2 != 0)
        {
            uFirstIndex = 0;
            uLastIndex = 0;
            vFirstIndex = 0;
            vLastIndex = 0;
            return;
        }

        // find the span which is predominant for parameter U
        uFirstIndex = uFirstNonZeroBsplineIndex;
        uLastIndex = uFirstNonZeroBsplineIndex + uDegree;
        if (uFirstIndex < uIndex1)
        {
            uFirstIndex = uIndex1;
        }
        if (uLastIndex > uIndex2)
        {
            uLastIndex = uIndex2;
        }

        double maxValue = 0.0;
        int ukk1 = 0, ukk2;

        for (int i = uFirstIndex - uFirstNonZeroBsplineIndex + 1;
             i <= uLastIndex - uFirstNonZeroBsplineIndex + 1;
             i++)
        {
            if (ubSplineBasis[1, i] > maxValue)
            {
                ukk1 = i + uFirstNonZeroBsplineIndex - 1;
                maxValue = ubSplineBasis[1, i];
            }
        }

        // find a ukk2 if symmetry
        ukk2 = ukk1;
        // OCCT: when [UFirstIndex, ULastIndex] holds none of the poles acting at U, ukk1 stays 0 and the basis
        // matrix is read out of range below (unchecked in OCCT's release build; the port throws Standard_OutOfRange)
        if ((ukk1 + 1) <= uLastIndex)
        {
            if (Math.Abs(ubSplineBasis[1, ukk1 - uFirstNonZeroBsplineIndex + 2] - maxValue) < 1.0e-10)
            {
                ukk2 = ukk1 + 1;
            }
        }

        // find the span which is predominant for parameter V
        vFirstIndex = vFirstNonZeroBsplineIndex;
        vLastIndex = vFirstNonZeroBsplineIndex + vDegree;

        if (vFirstIndex < vIndex1)
        {
            vFirstIndex = vIndex1;
        }
        if (vLastIndex > vIndex2)
        {
            vLastIndex = vIndex2;
        }

        maxValue = 0.0;
        int vkk1 = 0, vkk2;

        for (int j = vFirstIndex - vFirstNonZeroBsplineIndex + 1;
             j <= vLastIndex - vFirstNonZeroBsplineIndex + 1;
             j++)
        {
            if (vbSplineBasis[1, j] > maxValue)
            {
                vkk1 = j + vFirstNonZeroBsplineIndex - 1;
                maxValue = vbSplineBasis[1, j];
            }
        }

        // find a vkk2 if symmetry
        vkk2 = vkk1;
        // OCCT: the same for V (see ukk1 above)
        if ((vkk1 + 1) <= vLastIndex)
        {
            if (Math.Abs(vbSplineBasis[1, vkk1 - vFirstNonZeroBsplineIndex + 2] - maxValue) < 1.0e-10)
            {
                vkk2 = vkk1 + 1;
            }
        }

        // compute the vector of displacement
        double d1 = 0.0;
        double d2 = 0.0;
        double hN, coef, dvalU, dvalV;

        for (int i = 1; i <= uDegree + 1; i++)
        {
            int ii = i + uFirstNonZeroBsplineIndex - 1;
            if (ii < ukk1)
            {
                dvalU = ukk1 - ii;
            }
            else if (ii > ukk2)
            {
                dvalU = ii - ukk2;
            }
            else
            {
                dvalU = 0.0;
            }

            for (int j = 1; j <= vDegree + 1; j++)
            {
                int jj = j + vFirstNonZeroBsplineIndex - 1;
                if (rational)
                {
                    hN = weights[ii, jj] * ubSplineBasis[1, i] * vbSplineBasis[1, j];
                    d2 += hN;
                }
                else
                {
                    hN = ubSplineBasis[1, i] * vbSplineBasis[1, j];
                }
                if (ii >= uFirstIndex && ii <= uLastIndex && jj >= vFirstIndex && jj <= vLastIndex)
                {
                    if (jj < vkk1)
                    {
                        dvalV = vkk1 - jj;
                    }
                    else if (jj > vkk2)
                    {
                        dvalV = jj - vkk2;
                    }
                    else
                    {
                        dvalV = 0.0;
                    }
                    d1 += 1.0 / (dvalU + dvalV + 1.0) * hN;
                }
            }
        }

        if (rational)
        {
            coef = d2 / d1;
        }
        else
        {
            coef = 1.0 / d1;
        }

        // compute the new poles

        for (int i = poles.LowerRow(); i <= poles.UpperRow(); i++)
        {
            if (i < ukk1)
            {
                dvalU = ukk1 - i;
            }
            else if (i > ukk2)
            {
                dvalU = i - ukk2;
            }
            else
            {
                dvalU = 0.0;
            }

            for (int j = poles.LowerCol(); j <= poles.UpperCol(); j++)
            {
                if (i >= uFirstIndex && i <= uLastIndex && j >= vFirstIndex && j <= vLastIndex)
                {
                    if (j < vkk1)
                    {
                        dvalV = vkk1 - j;
                    }
                    else if (j > vkk2)
                    {
                        dvalV = j - vkk2;
                    }
                    else
                    {
                        dvalV = 0.0;
                    }
                    newPoles[i, j] = poles[i, j].Translated((coef / (dvalU + dvalV + 1.0)) * displ);
                }
                else
                {
                    newPoles[i, j] = poles[i, j];
                }
            }
        }
    }

    //=======================================================================
    // function : Resolution
    // purpose  : this computes an estimate for the maximum of the
    // partial derivatives both in U and in V
    //
    //
    // The calculation resembles at the calculation of curves with
    // additional index for the control point. Let Si,j be the
    // control points for ls surface  and  Di,j  the weights.
    // The checking of upper bounds for the partial derivatives
    // will be omitted and Su is the next upper bound in the polynomial case :
    //
    //
    //
    //                        |  Si,j - Si-1,j  |
    //          d *   Max     |  -------------  |
    //                i = 2,n |     ti+d - ti   |
    //                i=1.m
    //
    //
    // and in the rational case :
    //
    //
    //
    //                         Di,j * (Si,j - Sk,j) - Di-1,j * (Si-1,j - Sk,j)
    //   Max   Max       d  *  -----------------------------------------------
    // k=1,n  i dans Rj                   ti+d  - ti
    // j=1,m
    //  ----------------------------------------------------------------------
    //
    //               Min    Di,j
    //              i=1,n
    //              j=1,m
    //
    //
    //
    // with Rj = {j-d, ....,  j+d+d+1}.
    //
    //
    //=======================================================================

    /// <summary>
    /// Given a 3D tolerance, computes tolerances in U and V such that |u1 - u0| &lt; UTolerance and
    /// |v1 - v0| &lt; VTolerance imply |f(u1, v1) - f(u0, v0)| &lt; Tolerance3D (0 if a derivative bound is 0).
    /// </summary>
    public static void Resolution(NCollection_Array2<Point3D> poles,
                                  NCollection_Array2<double>? weights,
                                  NCollection_Array1<double> uKnots,
                                  NCollection_Array1<double> vKnots,
                                  NCollection_Array1<int> uMults,
                                  NCollection_Array1<int> vMults,
                                  int uDegree,
                                  int vDegree,
                                  bool uRational,
                                  bool vRational,
                                  bool uPeriodic,
                                  bool vPeriodic,
                                  double tolerance3D,
                                  out double uTolerance,
                                  out double vTolerance)
    {
        double wij, wmj, wji, wjm;
        double xij, xmj, xji, xjm, xpq, xqp;
        double yij, ymj, yji, yjm, ypq, yqp;
        double zij, zmj, zji, zjm, zpq, zqp;
        double factor, value, min, min_weights = 0, inverse;
        Span<double> max_derivative = stackalloc double[2];

        max_derivative[0] = max_derivative[1] = 0.0e0;

        int pRowLength, pColLength;
        int ii, jj, pp, qq, ii_index, jj_index, pp_index, qq_index;
        int ii_minus;
        Span<int> upper = stackalloc int[2];
        Span<int> lower = stackalloc int[2];
        Span<int> poles_length = stackalloc int[2];
        Span<int> num_poles = stackalloc int[2];
        Span<int> num_flat_knots = stackalloc int[2];

        num_flat_knots[0] = global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(uMults, uDegree, uPeriodic);
        num_flat_knots[1] = global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(vMults, vDegree, vPeriodic);
        NCollection_Array1<double> flat_knots_in_u = new(1, num_flat_knots[0]);
        NCollection_Array1<double> flat_knots_in_v = new(1, num_flat_knots[1]);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(uKnots, uMults, uDegree, uPeriodic, flat_knots_in_u);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(vKnots, vMults, vDegree, vPeriodic, flat_knots_in_v);
        pRowLength = poles.RowLength();
        pColLength = poles.ColLength();
        if (uRational || vRational)
        {
            int wsize = pRowLength * pColLength;
            NCollection_Array2<double> refWights = weights!;
            ReadOnlySpan<double> wg = ValuesFrom(refWights, refWights.LowerRow(), refWights.LowerCol());
            min_weights = wg[0];

            for (ii = 1; ii < wsize; ii++)
            {
                min = wg[ii];
                if (min_weights > min)
                {
                    min_weights = min;
                }
            }
        }
        int ud1 = uDegree + 1;
        int vd1 = vDegree + 1;
        num_poles[0] = num_flat_knots[0] - ud1;
        num_poles[1] = num_flat_knots[1] - vd1;
        poles_length[0] = pColLength;
        poles_length[1] = pRowLength;
        if (uRational)
        {
            int ud2 = uDegree << 1;
            int vd2 = vDegree << 1;

            for (ii = 2; ii <= num_poles[0]; ii++)
            {
                ii_index = (ii - 1) % poles_length[0] + 1;
                ii_minus = (ii - 2) % poles_length[0] + 1;
                inverse = flat_knots_in_u[ii + uDegree] - flat_knots_in_u[ii];
                inverse = 1.0e0 / inverse;
                lower[0] = ii - ud1;
                if (lower[0] < 1)
                {
                    lower[0] = 1;
                }
                upper[0] = ii + ud2 + 1;
                if (upper[0] > num_poles[0])
                {
                    upper[0] = num_poles[0];
                }

                for (jj = 1; jj <= num_poles[1]; jj++)
                {
                    jj_index = (jj - 1) % poles_length[1] + 1;
                    lower[1] = jj - vd1;
                    if (lower[1] < 1)
                    {
                        lower[1] = 1;
                    }
                    upper[1] = jj + vd2 + 1;
                    if (upper[1] > num_poles[1])
                    {
                        upper[1] = num_poles[1];
                    }
                    Point3D pij = poles.Value(ii_index, jj_index);
                    wij = weights!.Value(ii_index, jj_index);
                    Point3D pmj = poles.Value(ii_minus, jj_index);
                    wmj = weights.Value(ii_minus, jj_index);
                    xij = pij.X;
                    yij = pij.Y;
                    zij = pij.Z;
                    xmj = pmj.X;
                    ymj = pmj.Y;
                    zmj = pmj.Z;

                    for (pp = lower[0]; pp <= upper[0]; pp++)
                    {
                        pp_index = (pp - 1) % poles_length[0] + 1;

                        for (qq = lower[1]; qq <= upper[1]; qq++)
                        {
                            value = 0.0e0;
                            qq_index = (qq - 1) % poles_length[1] + 1;
                            Point3D ppq = poles.Value(pp_index, qq_index);
                            xpq = ppq.X;
                            ypq = ppq.Y;
                            zpq = ppq.Z;
                            factor = (xpq - xij) * wij;
                            factor -= (xpq - xmj) * wmj;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            factor = (ypq - yij) * wij;
                            factor -= (ypq - ymj) * wmj;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            factor = (zpq - zij) * wij;
                            factor -= (zpq - zmj) * wmj;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            value *= inverse;
                            if (max_derivative[0] < value)
                            {
                                max_derivative[0] = value;
                            }
                        }
                    }
                }
            }
            max_derivative[0] /= min_weights;
        }
        else
        {

            for (ii = 2; ii <= num_poles[0]; ii++)
            {
                ii_index = (ii - 1) % poles_length[0] + 1;
                ii_minus = (ii - 2) % poles_length[0] + 1;
                inverse = flat_knots_in_u[ii + uDegree] - flat_knots_in_u[ii];
                inverse = 1.0e0 / inverse;

                for (jj = 1; jj <= num_poles[1]; jj++)
                {
                    jj_index = (jj - 1) % poles_length[1] + 1;
                    value = 0.0e0;
                    Point3D pij = poles.Value(ii_index, jj_index);
                    Point3D pmj = poles.Value(ii_minus, jj_index);
                    factor = pij.X - pmj.X;
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    factor = pij.Y - pmj.Y;
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    factor = pij.Z - pmj.Z;
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    value *= inverse;
                    if (max_derivative[0] < value)
                    {
                        max_derivative[0] = value;
                    }
                }
            }
        }
        max_derivative[0] *= uDegree;
        if (vRational)
        {
            int ud2 = uDegree << 1;
            int vd2 = vDegree << 1;

            for (ii = 2; ii <= num_poles[1]; ii++)
            {
                ii_index = (ii - 1) % poles_length[1] + 1;
                ii_minus = (ii - 2) % poles_length[1] + 1;
                inverse = flat_knots_in_v[ii + vDegree] - flat_knots_in_v[ii];
                inverse = 1.0e0 / inverse;
                lower[0] = ii - vd1;
                if (lower[0] < 1)
                {
                    lower[0] = 1;
                }
                upper[0] = ii + vd2 + 1;
                if (upper[0] > num_poles[1])
                {
                    upper[0] = num_poles[1];
                }

                for (jj = 1; jj <= num_poles[0]; jj++)
                {
                    jj_index = (jj - 1) % poles_length[0] + 1;
                    lower[1] = jj - ud1;
                    if (lower[1] < 1)
                    {
                        lower[1] = 1;
                    }
                    upper[1] = jj + ud2 + 1;
                    if (upper[1] > num_poles[0])
                    {
                        upper[1] = num_poles[0];
                    }
                    Point3D pji = poles.Value(jj_index, ii_index);
                    wji = weights!.Value(jj_index, ii_index);
                    Point3D pjm = poles.Value(jj_index, ii_minus);
                    wjm = weights.Value(jj_index, ii_minus);
                    xji = pji.X;
                    yji = pji.Y;
                    zji = pji.Z;
                    xjm = pjm.X;
                    yjm = pjm.Y;
                    zjm = pjm.Z;

                    for (pp = lower[1]; pp <= upper[1]; pp++)
                    {
                        pp_index = (pp - 1) % poles_length[1] + 1;

                        for (qq = lower[0]; qq <= upper[0]; qq++)
                        {
                            value = 0.0e0;
                            qq_index = (qq - 1) % poles_length[0] + 1;
                            Point3D pqp = poles.Value(qq_index, pp_index);
                            xqp = pqp.X;
                            yqp = pqp.Y;
                            zqp = pqp.Z;
                            factor = (xqp - xji) * wji;
                            factor -= (xqp - xjm) * wjm;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            factor = (yqp - yji) * wji;
                            factor -= (yqp - yjm) * wjm;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            factor = (zqp - zji) * wji;
                            factor -= (zqp - zjm) * wjm;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            value *= inverse;
                            if (max_derivative[1] < value)
                            {
                                max_derivative[1] = value;
                            }
                        }
                    }
                }
            }
            max_derivative[1] /= min_weights;
        }
        else
        {

            for (ii = 2; ii <= num_poles[1]; ii++)
            {
                ii_index = (ii - 1) % poles_length[1] + 1;
                ii_minus = (ii - 2) % poles_length[1] + 1;
                inverse = flat_knots_in_v[ii + vDegree] - flat_knots_in_v[ii];
                inverse = 1.0e0 / inverse;

                for (jj = 1; jj <= num_poles[0]; jj++)
                {
                    jj_index = (jj - 1) % poles_length[0] + 1;
                    value = 0.0e0;
                    Point3D pji = poles.Value(jj_index, ii_index);
                    Point3D pjm = poles.Value(jj_index, ii_minus);
                    factor = pji.X - pjm.X;
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    factor = pji.Y - pjm.Y;
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    factor = pji.Z - pjm.Z;
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    value *= inverse;
                    if (max_derivative[1] < value)
                    {
                        max_derivative[1] = value;
                    }
                }
            }
        }
        max_derivative[1] *= vDegree;
        max_derivative[0] *= M_SQRT2;
        max_derivative[1] *= M_SQRT2;
        if (max_derivative[0] != 0.0 && max_derivative[1] != 0.0)
        {
            uTolerance = tolerance3D / max_derivative[0];
            vTolerance = tolerance3D / max_derivative[1];
        }
        else
        {
            uTolerance = vTolerance = 0.0;
        }
    }

    private const double M_SQRT2 = 1.41421356237309504880168872420969808;

    //=================================================================================================

    /// <summary>
    /// Interpolates the rational data points given in Poles and Weights (indexed from 1): ColLength = length of
    /// UParameters, RowLength = length of VParameters; the flat knots' lengths are Degree + length + 1. The poles
    /// and weights are replaced by the interpolating surface's. Gauss elimination without pivoting: guaranteed
    /// for cubic interpolation at knots or at Schoenberg points. InversionProblem is 0, or the faulty pivot.
    /// </summary>
    public static void Interpolate(int uDegree,
                                   int vDegree,
                                   NCollection_Array1<double> uFlatKnots,
                                   NCollection_Array1<double> vFlatKnots,
                                   NCollection_Array1<double> uParameters,
                                   NCollection_Array1<double> vParameters,
                                   NCollection_Array2<Point3D> poles,
                                   NCollection_Array2<double> weights,
                                   out int inversionProblem)
    {
        int ii, jj, ll, kk, dimension;
        int uLength = uParameters.Length();
        int vLength = vParameters.Length();
        Span<double> poles_array;

        // extraction of iso u
        dimension = 4 * uLength;
        NCollection_Array2<double> points = new(1, vLength, 1, dimension);

        NCollection_HArray1<int> contactOrder = new(1, vLength);
        contactOrder.Init(0);

        for (ii = 1; ii <= vLength; ii++)
        {

            for (jj = 1, ll = 1; jj <= uLength; jj++, ll += 4)
            {
                points[ii, ll] = poles[jj, ii].X;
                points[ii, ll + 1] = poles[jj, ii].Y;
                points[ii, ll + 2] = poles[jj, ii].Z;
                points[ii, ll + 3] = weights[jj, ii];
            }
        }

        // interpolation of iso u
        poles_array = points.AsSpan();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(vDegree,
                             vFlatKnots,
                             vParameters,
                             contactOrder,
                             dimension,
                             poles_array,
                             out inversionProblem);
        if (inversionProblem != 0)
        {
            return;
        }

        // extraction of iso v

        dimension = vLength * 4;
        NCollection_Array2<double> isoPoles = new(1, uLength, 1, dimension);

        contactOrder = new NCollection_HArray1<int>(1, uLength);
        contactOrder.Init(0);
        poles_array = isoPoles.AsSpan();

        for (ii = 1, kk = 1; ii <= uLength; ii++, kk += 4)
        {

            for (jj = 1, ll = 1; jj <= vLength; jj++, ll += 4)
            {
                isoPoles[ii, ll] = points[jj, kk];
                isoPoles[ii, ll + 1] = points[jj, kk + 1];
                isoPoles[ii, ll + 2] = points[jj, kk + 2];
                isoPoles[ii, ll + 3] = points[jj, kk + 3];
            }
        }
        // interpolation of iso v
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(uDegree,
                             uFlatKnots,
                             uParameters,
                             contactOrder,
                             dimension,
                             poles_array,
                             out inversionProblem);

        // return results

        for (ii = 1; ii <= uLength; ii++)
        {

            for (jj = 1, ll = 1; jj <= vLength; jj++, ll += 4)
            {
                Point3D pnt = new(isoPoles[ii, ll], isoPoles[ii, ll + 1], isoPoles[ii, ll + 2]);
                poles.SetValue(ii, jj, pnt);
                weights.SetValue(ii, jj, isoPoles[ii, ll + 3]);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Interpolates the data points given in Poles (indexed from 1), which are replaced by the interpolating
    /// surface's poles (see the rational overload).
    /// </summary>
    public static void Interpolate(int uDegree,
                                   int vDegree,
                                   NCollection_Array1<double> uFlatKnots,
                                   NCollection_Array1<double> vFlatKnots,
                                   NCollection_Array1<double> uParameters,
                                   NCollection_Array1<double> vParameters,
                                   NCollection_Array2<Point3D> poles,
                                   out int inversionProblem)
    {
        int ii, jj, ll, kk, dimension;
        int uLength = uParameters.Length();
        int vLength = vParameters.Length();
        Span<double> poles_array;

        // extraction of iso u
        dimension = 3 * uLength;
        NCollection_Array2<double> points = new(1, vLength, 1, dimension);

        NCollection_HArray1<int> contactOrder = new(1, vLength);
        contactOrder.Init(0);

        for (ii = 1; ii <= vLength; ii++)
        {

            for (jj = 1, ll = 1; jj <= uLength; jj++, ll += 3)
            {
                points[ii, ll] = poles[jj, ii].X;
                points[ii, ll + 1] = poles[jj, ii].Y;
                points[ii, ll + 2] = poles[jj, ii].Z;
            }
        }

        // interpolation of iso u
        poles_array = points.AsSpan();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(vDegree,
                             vFlatKnots,
                             vParameters,
                             contactOrder,
                             dimension,
                             poles_array,
                             out inversionProblem);
        if (inversionProblem != 0)
        {
            return;
        }

        // extraction of iso v

        dimension = vLength * 3;
        NCollection_Array2<double> isoPoles = new(1, uLength, 1, dimension);

        contactOrder = new NCollection_HArray1<int>(1, uLength);
        contactOrder.Init(0);
        poles_array = isoPoles.AsSpan();

        for (ii = 1, kk = 1; ii <= uLength; ii++, kk += 3)
        {

            for (jj = 1, ll = 1; jj <= vLength; jj++, ll += 3)
            {
                isoPoles[ii, ll] = points[jj, kk];
                isoPoles[ii, ll + 1] = points[jj, kk + 1];
                isoPoles[ii, ll + 2] = points[jj, kk + 2];
            }
        }
        // interpolation of iso v
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(uDegree,
                             uFlatKnots,
                             uParameters,
                             contactOrder,
                             dimension,
                             poles_array,
                             out inversionProblem);

        // return results

        for (ii = 1; ii <= uLength; ii++)
        {

            for (jj = 1, ll = 1; jj <= vLength; jj++, ll += 3)
            {
                Point3D pnt = new(isoPoles[ii, ll], isoPoles[ii, ll + 1], isoPoles[ii, ll + 2]);
                poles.SetValue(ii, jj, pnt);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Multiplies a B-spline numerator N(u, v) and denominator D(u, v) (degrees, knots and multiplicities, poles and
    /// weights indexed [1..NbUPoles][1..NbVPoles]) by a function a(u, v), where a * N and a * D must be B-splines of
    /// degrees UNewDegree, VNewDegree on the knots UFlatKnots, VFlatKnots, and a has the range of N and D; nothing
    /// is checked. The result is computed by interpolation at the Schoenberg points; theStatus is 0, or the faulty
    /// pivot of the inverted matrix.
    /// </summary>
    public static void FunctionMultiply(SurfaceEvaluatorFunction function,
                                        int ubSplineDegree,
                                        int vbSplineDegree,
                                        NCollection_Array1<double> ubSplineKnots,
                                        NCollection_Array1<double> vbSplineKnots,
                                        NCollection_Array1<int>? uMults,
                                        NCollection_Array1<int>? vMults,
                                        NCollection_Array2<Point3D> poles,
                                        NCollection_Array2<double>? weights,
                                        NCollection_Array1<double> uFlatKnots,
                                        NCollection_Array1<double> vFlatKnots,
                                        int uNewDegree,
                                        int vNewDegree,
                                        NCollection_Array2<Point3D> newNumerator,
                                        NCollection_Array2<double> newDenominator,
                                        out int status)
    {
        int num_uparameters,
          //  ii,jj,kk,
          ii, jj, error_code, num_vparameters;
        double result;

        num_uparameters = uFlatKnots.Length() - uNewDegree - 1;
        num_vparameters = vFlatKnots.Length() - vNewDegree - 1;
        NCollection_Array1<double> uParameters = new(1, num_uparameters);
        NCollection_Array1<double> vParameters = new(1, num_vparameters);

        if ((newNumerator.ColLength() == num_uparameters) && (newNumerator.RowLength() == num_vparameters)
            && (newDenominator.ColLength() == num_uparameters)
            && (newDenominator.RowLength() == num_vparameters))
        {

            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(uNewDegree, uFlatKnots, uParameters);

            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(vNewDegree, vFlatKnots, vParameters);

            for (ii = 1; ii <= num_uparameters; ii++)
            {

                for (jj = 1; jj <= num_vparameters; jj++)
                {
                    // OCCT: URat and VRat are passed true whatever Weights is, so a null Weights fails in
                    // PrepareEval (a null dereference in OCCT, a NullReferenceException here)
                    HomogeneousD0(uParameters[ii],
                                  vParameters[jj],
                                  0,
                                  0,
                                  poles,
                                  weights,
                                  ubSplineKnots,
                                  vbSplineKnots,
                                  uMults,
                                  vMults,
                                  ubSplineDegree,
                                  vbSplineDegree,
                                  true,
                                  true,
                                  false,
                                  false,
                                  out newDenominator[ii, jj],
                                  out newNumerator[ii, jj]);

                    function.Evaluate(0, uParameters[ii], vParameters[jj], out result, out error_code);
                    if (error_code != 0)
                    {
                        throw new ConstructionErrorException();
                    }
                    ref Point3D p = ref newNumerator[ii, jj];
                    p = p.WithX(p.X * result);
                    p = p.WithY(p.Y * result);
                    p = p.WithZ(p.Z * result);
                    newDenominator[ii, jj] *= result;
                }
            }
            Interpolate(uNewDegree,
                        vNewDegree,
                        uFlatKnots,
                        vFlatKnots,
                        uParameters,
                        vParameters,
                        newNumerator,
                        newDenominator,
                        out status);
        }
        else
        {
            throw new ConstructionErrorException();
        }
    }

    //=================================================================================================

    /// <summary>
    /// An array [1, theNbUPoles] x [1, theNbVPoles] of weights equal to 1. Up to
    /// <see cref="BSplCLib.MaxUnitWeightsSize"/> elements it is a non-owning view of a shared static array (no
    /// allocation), as in OCCT: shared, don't modify. Above that it is a new array.
    /// </summary>
    public static NCollection_Array2<double> UnitWeights(int nbUPoles, int nbVPoles)
    {
        int total = nbUPoles * nbVPoles;
        if (total <= global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxUnitWeightsSize())
        {
            // Non-owning view over global static array (zero allocation).
            // Note: the port allocates the two small view objects (the 1D view and the 2D view over it)
            return new NCollection_Array2<double>(global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(total),
                                                  1,
                                                  1,
                                                  nbUPoles,
                                                  1,
                                                  nbVPoles);
        }
        // Rare: allocate fresh array.
        NCollection_Array2<double> result = new(1, nbUPoles, 1, nbVPoles);
        result.Init(1.0);
        return result;
    }
}
