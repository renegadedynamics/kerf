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

using System.Runtime.InteropServices;

namespace Occt;

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
public static partial class BSplSLib
{
    // for null derivatives
    private static ReadOnlySpan<double> BSplSLib_zero => [0.0, 0.0, 0.0];

    //! Maximum supported degree for B-spline surfaces.
    private static readonly int THE_MAX_DEGREE = BSplCLib.MaxDegree();

    //! Validates that the given degrees do not exceed the maximum supported degree.
    private static void validateBSplineDegree(int theUDegree, int theVDegree)
    {
        Standard_OutOfRange_Raise_if(theUDegree > THE_MAX_DEGREE || theVDegree > THE_MAX_DEGREE,
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
    private static int dcPolesSize(int theUDegree, int theVDegree) => 4 * (theUDegree + 1) * (theVDegree + 1);

    private static int dcKnotsSize(int theUDegree, int theVDegree) => 2 * Max(theUDegree, theVDegree);

    private const int dcDersSize = 48;

    // A span of the coordinates of the points of a 2D array from the point (theRow, theCol) on: OCCT's
    // `(double*)&Poles(theRow, theCol)`.
    private static ReadOnlySpan<double> coordsFrom(NCollection_Array2<gp_Pnt> theArray, int theRow, int theCol)
    {
        int anOffset = (theRow - theArray.LowerRow()) * theArray.RowLength() + (theCol - theArray.LowerCol());
        return MemoryMarshal.Cast<gp_Pnt, double>(theArray.AsSpan()).Slice(anOffset * 3);
    }

    // Stands for OCCT's `(double*)&Weights(theRow, theCol)`.
    private static ReadOnlySpan<double> valuesFrom(NCollection_Array2<double> theArray, int theRow, int theCol)
    {
        int anOffset = (theRow - theArray.LowerRow()) * theArray.RowLength() + (theCol - theArray.LowerCol());
        return theArray.AsSpan().Slice(anOffset);
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
    /// denominator. <paramref name="HDerivatives"/> holds, for p = 0..UDeg and q = 0..VDeg, the (p, q) derivative
    /// of the numerator (3 values) followed by that of the denominator; <paramref name="RDerivatives"/> receives,
    /// for p = 0..N and q = 0..M, the (p, q) derivative of the rational function (3 values each) if
    /// <paramref name="All"/> is true, or only the (N, M) derivative otherwise.
    /// </summary>
    public static void RationalDerivative(int UDeg, int VDeg, int N, int M, ReadOnlySpan<double> HDerivatives,
                                          Span<double> RDerivatives, bool All = true)
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
        int M1, M3, M4, N1, iiM1, iiM3, jjM1, ppM1, ppM3;
        int MinN, MinN1, MinM, MinM1;
        int index_u, index_u1, index_v, index_v1, index_w;

        M1 = M + 1;
        N1 = N + 1;
        ii = N1 * M1;
        M3 = (M1 << 1) + M1;
        M4 = (VDeg + 1) << 2;

        int aStoreSize = All ? 0 : ii * 3;
        Span<double> StoreDerivatives = aStoreSize <= 128 ? stackalloc double[aStoreSize] : new double[aStoreSize];
        Span<double> RArray = All ? RDerivatives : StoreDerivatives;
        Span<double> StoreW = ii <= 128 ? stackalloc double[ii] : new double[ii];
        ReadOnlySpan<double> HomogeneousArray = HDerivatives;
        double denominator, Pii, Pip, Pjq;

        denominator = 1.0e0 / HomogeneousArray[3];
        index_u = 0;
        index_u1 = 0;
        if (UDeg < N)
        {
            MinN = UDeg;
        }
        else
        {
            MinN = N;
        }
        if (VDeg < M)
        {
            MinM = VDeg;
        }
        else
        {
            MinM = M;
        }
        MinN1 = MinN + 1;
        MinM1 = MinM + 1;
        iiM1 = -M1;

        for (ii = 0; ii < MinN1; ii++)
        {
            iiM1 += M1;
            index_v = index_u;
            index_v1 = index_u1;
            index_w = iiM1;

            for (jj = 0; jj < MinM1; jj++)
            {
                RArray[index_v++] = HomogeneousArray[index_v1++];
                RArray[index_v++] = HomogeneousArray[index_v1++];
                RArray[index_v++] = HomogeneousArray[index_v1++];
                StoreW[index_w++] = HomogeneousArray[index_v1++];
            }

            for (jj = MinM1; jj < M1; jj++)
            {
                RArray[index_v++] = 0.0;
                RArray[index_v++] = 0.0;
                RArray[index_v++] = 0.0;
                StoreW[index_w++] = 0.0;
            }
            index_u1 += M4;
            index_u += M3;
        }
        index_v = MinN1 * M3;
        index_w = MinN1 * M1;

        for (ii = MinN1; ii < N1; ii++)
        {

            for (jj = 0; jj < M1; jj++)
            {
                RArray[index_v++] = 0.0e0;
                RArray[index_v++] = 0.0e0;
                RArray[index_v++] = 0.0e0;
                StoreW[index_w++] = 0.0e0;
            }
        }

        // ---------------  Calculation ----------------

        iiM1 = -M1;
        iiM3 = -M3;

        for (ii = 0; ii <= N; ii++)
        {
            iiM1 += M1;
            iiM3 += M3;
            index1 = iiM3 - 3;
            jjM1 = iiM1;

            for (jj = 0; jj <= M; jj++)
            {
                jjM1++;
                ppM1 = -M1;
                ppM3 = -M3;
                index1 += 3;

                for (pp = 0; pp < ii; pp++)
                {
                    ppM1 += M1;
                    ppM3 += M3;
                    index = ppM3;
                    index2 = jjM1 - ppM1;
                    Pip = PLib.Bin(ii, pp);

                    for (qq = 0; qq <= jj; qq++)
                    {
                        index2--;
                        Pjq = Pip * PLib.Bin(jj, qq) * StoreW[index2];
                        RArray[index1] -= Pjq * RArray[index];
                        index++;
                        index1++;
                        RArray[index1] -= Pjq * RArray[index];
                        index++;
                        index1++;
                        RArray[index1] -= Pjq * RArray[index];
                        index++;
                        index1 -= 2;
                    }
                }
                index = iiM3;
                index2 = jj + 1;
                Pii = PLib.Bin(ii, ii);

                for (qq = 0; qq < jj; qq++)
                {
                    index2--;
                    Pjq = Pii * PLib.Bin(jj, qq) * StoreW[index2];
                    RArray[index1] -= Pjq * RArray[index];
                    index++;
                    index1++;
                    RArray[index1] -= Pjq * RArray[index];
                    index++;
                    index1++;
                    RArray[index1] -= Pjq * RArray[index];
                    index++;
                    index1 -= 2;
                }
                RArray[index1] *= denominator;
                index1++;
                RArray[index1] *= denominator;
                index1++;
                RArray[index1] *= denominator;
                index1 -= 2;
            }
        }
        if (!All)
        {
            RArray = RDerivatives;
            index = N * M1 + M;
            index = (index << 1) + index;
            RArray[0] = StoreDerivatives[index];
            index++;
            RArray[1] = StoreDerivatives[index];
            index++;
            RArray[2] = StoreDerivatives[index];
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

    private static bool PrepareEval(double U,
                                    double V,
                                    int Uindex,
                                    int Vindex,
                                    int UDegree,
                                    int VDegree,
                                    bool URat,
                                    bool VRat,
                                    bool UPer,
                                    bool VPer,
                                    NCollection_Array2<gp_Pnt> Poles,
                                    NCollection_Array2<double>? Weights,
                                    NCollection_Array1<double> UKnots,
                                    NCollection_Array1<double> VKnots,
                                    NCollection_Array1<int>? UMults,
                                    NCollection_Array1<int>? VMults,
                                    out double u1, // first  parameter to use
                                    out double u2, // second parameter to use
                                    out int d1,    // first degree
                                    out int d2,    // second degree
                                    out bool rational,
                                    BSplSLib_DataContainer dc)
    {
        rational = URat || VRat;
        int uindex = Uindex;
        int vindex = Vindex;
        int UKLower = UKnots.Lower();
        int UKUpper = UKnots.Upper();
        int VKLower = VKnots.Lower();
        int VKUpper = VKnots.Upper();

        if (UDegree <= VDegree)
        {
            // compute the indices
            if (uindex < UKLower || uindex > UKUpper)
            {
                BSplCLib.LocateParameter(UDegree, UKnots, UMults, U, UPer, ref uindex, out u1);
            }
            else
            {
                u1 = U;
            }

            if (vindex < VKLower || vindex > VKUpper)
            {
                BSplCLib.LocateParameter(VDegree, VKnots, VMults, V, VPer, ref vindex, out u2);
            }
            else
            {
                u2 = V;
            }

            // get the knots
            d1 = UDegree;
            d2 = VDegree;
            BSplCLib.BuildKnots(UDegree, uindex, UPer, UKnots, UMults, dc.knots1);
            BSplCLib.BuildKnots(VDegree, vindex, VPer, VKnots, VMults, dc.knots2);

            if (UMults == null)
            {
                uindex -= UKLower + UDegree;
            }
            else
            {
                uindex = BSplCLib.PoleIndex(UDegree, uindex, UPer, UMults);
            }

            if (VMults == null)
            {
                vindex -= VKLower + VDegree;
            }
            else
            {
                vindex = BSplCLib.PoleIndex(VDegree, vindex, VPer, VMults);
            }

            // get the poles
            int i, j, ip, jp;
            double w;
            Span<double> poles = dc.poles;
            int pole = 0;
            d1 = UDegree;
            d2 = VDegree;
            int PLowerRow = Poles.LowerRow();
            int PUpperRow = Poles.UpperRow();
            int PLowerCol = Poles.LowerCol();
            int PUpperCol = Poles.UpperCol();

            // verify if locally non rational
            if (rational)
            {
                rational = false;
                ip = PLowerRow + uindex;
                jp = PLowerCol + vindex;

                if (ip < PLowerRow)
                {
                    ip = PUpperRow;
                }
                if (jp < PLowerCol)
                {
                    jp = PUpperCol;
                }

                w = Weights!.Value(ip, jp);
                double eps = Epsilon(w);
                double dw;

                for (i = 0; i <= UDegree && !rational; i++)
                {
                    jp = PLowerCol + vindex;

                    if (jp < PLowerCol)
                    {
                        jp = PUpperCol;
                    }

                    for (j = 0; j <= VDegree && !rational; j++)
                    {
                        dw = Weights.Value(ip, jp) - w;
                        if (dw < 0)
                        {
                            dw = -dw;
                        }

                        rational = (dw > eps);

                        jp++;

                        if (jp > PUpperCol)
                        {
                            jp = PLowerCol;
                        }
                    }

                    ip++;

                    if (ip > PUpperRow)
                    {
                        ip = PLowerRow;
                    }
                }
            }

            // copy the poles
            ip = PLowerRow + uindex;

            if (ip < PLowerRow)
            {
                ip = PUpperRow;
            }

            if (rational)
            {
                for (i = 0; i <= d1; i++)
                {
                    jp = PLowerCol + vindex;

                    if (jp < PLowerCol)
                    {
                        jp = PUpperCol;
                    }

                    for (j = 0; j <= d2; j++)
                    {
                        gp_Pnt P = Poles.Value(ip, jp);
                        poles[pole + 3] = w = Weights!.Value(ip, jp);
                        poles[pole + 0] = P.X() * w;
                        poles[pole + 1] = P.Y() * w;
                        poles[pole + 2] = P.Z() * w;
                        pole += 4;
                        jp++;

                        if (jp > PUpperCol)
                        {
                            jp = PLowerCol;
                        }
                    }

                    ip++;

                    if (ip > PUpperRow)
                    {
                        ip = PLowerRow;
                    }
                }
            }
            else
            {
                for (i = 0; i <= d1; i++)
                {
                    jp = PLowerCol + vindex;

                    if (jp < PLowerCol)
                    {
                        jp = PUpperCol;
                    }

                    for (j = 0; j <= d2; j++)
                    {
                        gp_Pnt P = Poles.Value(ip, jp);
                        poles[pole + 0] = P.X();
                        poles[pole + 1] = P.Y();
                        poles[pole + 2] = P.Z();
                        pole += 3;
                        jp++;

                        if (jp > PUpperCol)
                        {
                            jp = PLowerCol;
                        }
                    }

                    ip++;

                    if (ip > PUpperRow)
                    {
                        ip = PLowerRow;
                    }
                }
            }

            return true;
        }
        else
        {
            // compute the indices
            if (uindex < UKLower || uindex > UKUpper)
            {
                BSplCLib.LocateParameter(UDegree, UKnots, UMults, U, UPer, ref uindex, out u2);
            }
            else
            {
                u2 = U;
            }

            if (vindex < VKLower || vindex > VKUpper)
            {
                BSplCLib.LocateParameter(VDegree, VKnots, VMults, V, VPer, ref vindex, out u1);
            }
            else
            {
                u1 = V;
            }

            // get the knots

            d2 = UDegree;
            d1 = VDegree;

            BSplCLib.BuildKnots(UDegree, uindex, UPer, UKnots, UMults, dc.knots2);
            BSplCLib.BuildKnots(VDegree, vindex, VPer, VKnots, VMults, dc.knots1);

            if (UMults == null)
            {
                uindex -= UKLower + UDegree;
            }
            else
            {
                uindex = BSplCLib.PoleIndex(UDegree, uindex, UPer, UMults);
            }

            if (VMults == null)
            {
                vindex -= VKLower + VDegree;
            }
            else
            {
                vindex = BSplCLib.PoleIndex(VDegree, vindex, VPer, VMults);
            }

            // get the poles
            int i, j, ip, jp;
            double w;
            Span<double> poles = dc.poles;
            int pole = 0;
            d1 = VDegree;
            d2 = UDegree;
            int PLowerRow = Poles.LowerRow();
            int PUpperRow = Poles.UpperRow();
            int PLowerCol = Poles.LowerCol();
            int PUpperCol = Poles.UpperCol();

            // verify if locally non rational
            if (rational)
            {
                rational = false;
                ip = PLowerRow + uindex;
                jp = PLowerCol + vindex;

                if (ip < PLowerRow)
                {
                    ip = PUpperRow;
                }

                if (jp < PLowerCol)
                {
                    jp = PUpperCol;
                }

                w = Weights!.Value(ip, jp);
                double eps = Epsilon(w);
                double dw;

                for (i = 0; i <= UDegree && !rational; i++)
                {
                    jp = PLowerCol + vindex;

                    if (jp < PLowerCol)
                    {
                        jp = PUpperCol;
                    }

                    for (j = 0; j <= VDegree && !rational; j++)
                    {
                        dw = Weights.Value(ip, jp) - w;
                        if (dw < 0)
                        {
                            dw = -dw;
                        }
                        rational = dw > eps;

                        jp++;

                        if (jp > PUpperCol)
                        {
                            jp = PLowerCol;
                        }
                    }

                    ip++;

                    if (ip > PUpperRow)
                    {
                        ip = PLowerRow;
                    }
                }
            }

            // copy the poles
            jp = PLowerCol + vindex;

            if (jp < PLowerCol)
            {
                jp = PUpperCol;
            }

            if (rational)
            {
                for (i = 0; i <= d1; i++)
                {
                    ip = PLowerRow + uindex;

                    if (ip < PLowerRow)
                    {
                        ip = PUpperRow;
                    }

                    for (j = 0; j <= d2; j++)
                    {
                        gp_Pnt P = Poles.Value(ip, jp);
                        poles[pole + 3] = w = Weights!.Value(ip, jp);
                        poles[pole + 0] = P.X() * w;
                        poles[pole + 1] = P.Y() * w;
                        poles[pole + 2] = P.Z() * w;
                        pole += 4;
                        ip++;

                        if (ip > PUpperRow)
                        {
                            ip = PLowerRow;
                        }
                    }

                    jp++;

                    if (jp > PUpperCol)
                    {
                        jp = PLowerCol;
                    }
                }
            }
            else
            {
                for (i = 0; i <= d1; i++)
                {
                    ip = PLowerRow + uindex;

                    if (ip < PLowerRow)
                    {
                        ip = PUpperRow;
                    }

                    if (ip > PUpperRow)
                    {
                        ip = PLowerRow;
                    }

                    for (j = 0; j <= d2; j++)
                    {
                        gp_Pnt P = Poles.Value(ip, jp);
                        poles[pole + 0] = P.X();
                        poles[pole + 1] = P.Y();
                        poles[pole + 2] = P.Z();
                        pole += 3;
                        ip++;

                        if (ip > PUpperRow)
                        {
                            ip = PLowerRow;
                        }
                    }

                    jp++;

                    if (jp > PUpperCol)
                    {
                        jp = PLowerCol;
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
    public static void D0(double U,
                          double V,
                          int UIndex,
                          int VIndex,
                          NCollection_Array2<gp_Pnt> Poles,
                          NCollection_Array2<double>? Weights,
                          NCollection_Array1<double> UKnots,
                          NCollection_Array1<double> VKnots,
                          NCollection_Array1<int>? UMults,
                          NCollection_Array1<int>? VMults,
                          int UDegree,
                          int VDegree,
                          bool URat,
                          bool VRat,
                          bool UPer,
                          bool VPer,
                          out gp_Pnt P)
    {
        //  int k ;
        HomogeneousD0(U,
                      V,
                      UIndex,
                      VIndex,
                      Poles,
                      Weights,
                      UKnots,
                      VKnots,
                      UMults,
                      VMults,
                      UDegree,
                      VDegree,
                      URat,
                      VRat,
                      UPer,
                      VPer,
                      out double W,
                      out P);
        P.SetX(P.X() / W);
        P.SetY(P.Y() / W);
        P.SetZ(P.Z() / W);
    }

    //=================================================================================================

    /// <summary>
    /// Homogeneous evaluation of the poles and weights at (U, V): P is the numerator and W the denominator if
    /// there are weights, otherwise 1.
    /// </summary>
    public static void HomogeneousD0(double U,
                                     double V,
                                     int UIndex,
                                     int VIndex,
                                     NCollection_Array2<gp_Pnt> Poles,
                                     NCollection_Array2<double>? Weights,
                                     NCollection_Array1<double> UKnots,
                                     NCollection_Array1<double> VKnots,
                                     NCollection_Array1<int>? UMults,
                                     NCollection_Array1<int>? VMults,
                                     int UDegree,
                                     int VDegree,
                                     bool URat,
                                     bool VRat,
                                     bool UPer,
                                     bool VPer,
                                     out double W,
                                     out gp_Pnt P)
    {
        bool rational;
        //  int k,dim;
        int dim;
        double u1, u2;
        int d1, d2;
        W = 1.0e0;

        validateBSplineDegree(UDegree, VDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[dcPolesSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcDersSize]);
        PrepareEval(U,
                    V,
                    UIndex,
                    VIndex,
                    UDegree,
                    VDegree,
                    URat,
                    VRat,
                    UPer,
                    VPer,
                    Poles,
                    Weights,
                    UKnots,
                    VKnots,
                    UMults,
                    VMults,
                    out u1,
                    out u2,
                    out d1,
                    out d2,
                    out rational,
                    dc);
        if (rational)
        {
            dim = 4;
            BSplCLib.Eval(u1, d1, dc.knots1, dim * (d2 + 1), dc.poles);
            BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles);
            W = dc.poles[3];
            P = new gp_Pnt(dc.poles[0], dc.poles[1], dc.poles[2]);
        }
        else
        {
            dim = 3;
            BSplCLib.Eval(u1, d1, dc.knots1, dim * (d2 + 1), dc.poles);
            BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles);
            P = new gp_Pnt(dc.poles[0], dc.poles[1], dc.poles[2]);
        }
    }

    //=================================================================================================

    /// <summary>Computes the point and the first derivatives of the surface at (U, V) (see <see cref="D0"/>).</summary>
    public static void D1(double U,
                          double V,
                          int UIndex,
                          int VIndex,
                          NCollection_Array2<gp_Pnt> Poles,
                          NCollection_Array2<double>? Weights,
                          NCollection_Array1<double> UKnots,
                          NCollection_Array1<double> VKnots,
                          NCollection_Array1<int>? UMults,
                          NCollection_Array1<int>? VMults,
                          int UDegree,
                          int VDegree,
                          bool URat,
                          bool VRat,
                          bool UPer,
                          bool VPer,
                          out gp_Pnt P,
                          out gp_Vec Vu,
                          out gp_Vec Vv)
    {
        bool rational;
        //  int k,dim,dim2;
        int dim, dim2;
        double u1, u2;
        int d1, d2;
        scoped ReadOnlySpan<double> result, resVu, resVv;
        validateBSplineDegree(UDegree, VDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[dcPolesSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcDersSize]);
        if (PrepareEval(U,
                        V,
                        UIndex,
                        VIndex,
                        UDegree,
                        VDegree,
                        URat,
                        VRat,
                        UPer,
                        VPer,
                        Poles,
                        Weights,
                        UKnots,
                        VKnots,
                        UMults,
                        VMults,
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
                BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
                BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[dim2..]);
                BSplSLib.RationalDerivative(d1, d2, 1, 1, dc.poles, dc.ders);
                result = dc.ders;
                resVu = result[6..];
                resVv = result[3..];
            }
            else
            {
                dim = 3;
                dim2 = d2 + 1;
                dim2 = (dim2 << 1) + dim2;
                BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
                BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[dim2..]);
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
                BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
                BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[dim2..]);
                BSplSLib.RationalDerivative(d1, d2, 1, 1, dc.poles, dc.ders);
                result = dc.ders;
                resVu = result[3..];
                resVv = result[6..];
            }
            else
            {
                dim = 3;
                dim2 = d2 + 1;
                dim2 = (dim2 << 1) + dim2;
                BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
                BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[dim2..]);
                result = dc.poles;
                resVu = result[3..];
                resVv = result[dim2..];
            }
        }

        P = new gp_Pnt(result[0], result[1], result[2]);
        Vu = new gp_Vec(resVu[0], resVu[1], resVu[2]);
        Vv = new gp_Vec(resVv[0], resVv[1], resVv[2]);
    }

    //=================================================================================================

    /// <summary>
    /// Homogeneous evaluation of the point and first derivatives at (U, V): N, Nu, Nv for the numerator and
    /// D, Du, Dv for the denominator (1, 0, 0 if there are no weights).
    /// </summary>
    public static void HomogeneousD1(double U,
                                     double V,
                                     int UIndex,
                                     int VIndex,
                                     NCollection_Array2<gp_Pnt> Poles,
                                     NCollection_Array2<double>? Weights,
                                     NCollection_Array1<double> UKnots,
                                     NCollection_Array1<double> VKnots,
                                     NCollection_Array1<int>? UMults,
                                     NCollection_Array1<int>? VMults,
                                     int UDegree,
                                     int VDegree,
                                     bool URat,
                                     bool VRat,
                                     bool UPer,
                                     bool VPer,
                                     out gp_Pnt N,
                                     out gp_Vec Nu,
                                     out gp_Vec Nv,
                                     out double D,
                                     out double Du,
                                     out double Dv)
    {
        bool rational;
        //  int k,dim;
        int dim;
        double u1, u2;
        int d1, d2;

        D = 1.0e0;
        Du = 0.0e0;
        Dv = 0.0e0;
        validateBSplineDegree(UDegree, VDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[dcPolesSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcDersSize]);
        bool ufirst = PrepareEval(U,
                                  V,
                                  UIndex,
                                  VIndex,
                                  UDegree,
                                  VDegree,
                                  URat,
                                  VRat,
                                  UPer,
                                  VPer,
                                  Poles,
                                  Weights,
                                  UKnots,
                                  VKnots,
                                  UMults,
                                  VMults,
                                  out u1,
                                  out u2,
                                  out d1,
                                  out d2,
                                  out rational,
                                  dc);
        dim = rational ? 4 : 3;

        BSplCLib.Bohm(u1, d1, 1, dc.knots1, dim * (d2 + 1), dc.poles);
        BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles);
        BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim * (d2 + 1))..]);

        scoped ReadOnlySpan<double> result, resVu, resVv;
        result = dc.poles;
        resVu = result[(ufirst ? dim * (d2 + 1) : dim)..];
        resVv = result[(ufirst ? dim : dim * (d2 + 1))..];

        N = new gp_Pnt(result[0], result[1], result[2]);
        Nu = new gp_Vec(resVu[0], resVu[1], resVu[2]);
        Nv = new gp_Vec(resVv[0], resVv[1], resVv[2]);

        if (rational)
        {
            D = result[3];
            Du = resVu[3];
            Dv = resVv[3];
        }
    }

    //=================================================================================================

    /// <summary>Computes the point and the derivatives up to order 2 at (U, V) (see <see cref="D0"/>).</summary>
    public static void D2(double U,
                          double V,
                          int UIndex,
                          int VIndex,
                          NCollection_Array2<gp_Pnt> Poles,
                          NCollection_Array2<double>? Weights,
                          NCollection_Array1<double> UKnots,
                          NCollection_Array1<double> VKnots,
                          NCollection_Array1<int>? UMults,
                          NCollection_Array1<int>? VMults,
                          int UDegree,
                          int VDegree,
                          bool URat,
                          bool VRat,
                          bool UPer,
                          bool VPer,
                          out gp_Pnt P,
                          out gp_Vec Vu,
                          out gp_Vec Vv,
                          out gp_Vec Vuu,
                          out gp_Vec Vvv,
                          out gp_Vec Vuv)
    {
        bool rational;
        //  int k,dim,dim2;
        int dim, dim2;
        double u1, u2;
        int d1, d2;
        scoped ReadOnlySpan<double> result;
        scoped ReadOnlySpan<double> resVu, resVv, resVuu, resVvv, resVuv;
        validateBSplineDegree(UDegree, VDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[dcPolesSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcDersSize]);
        if (PrepareEval(U,
                        V,
                        UIndex,
                        VIndex,
                        UDegree,
                        VDegree,
                        URat,
                        VRat,
                        UPer,
                        VPer,
                        Poles,
                        Weights,
                        UKnots,
                        VKnots,
                        UMults,
                        VMults,
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
                BSplCLib.Bohm(u1, d1, 2, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles);
                BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                BSplSLib.RationalDerivative(d1, d2, 2, 2, dc.poles, dc.ders);
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
                BSplCLib.Bohm(u1, d1, 2, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles);
                BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                result = dc.poles;
                resVu = result[dim2..];
                resVv = result[3..];
                if (UDegree <= 1)
                {
                    resVuu = BSplSLib_zero;
                }
                else
                {
                    resVuu = result[(dim2 << 1)..];
                }
                if (VDegree <= 1)
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
                BSplCLib.Bohm(u1, d1, 2, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles);
                BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                BSplSLib.RationalDerivative(d1, d2, 2, 2, dc.poles, dc.ders);
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
                BSplCLib.Bohm(u1, d1, 2, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles);
                BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                result = dc.poles;
                resVu = result[3..];
                resVv = result[dim2..];
                if (UDegree <= 1)
                {
                    resVuu = BSplSLib_zero;
                }
                else
                {
                    resVuu = result[6..];
                }
                if (VDegree <= 1)
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

        P = new gp_Pnt(result[0], result[1], result[2]);
        Vu = new gp_Vec(resVu[0], resVu[1], resVu[2]);
        Vv = new gp_Vec(resVv[0], resVv[1], resVv[2]);
        Vuu = new gp_Vec(resVuu[0], resVuu[1], resVuu[2]);
        Vvv = new gp_Vec(resVvv[0], resVvv[1], resVvv[2]);
        Vuv = new gp_Vec(resVuv[0], resVuv[1], resVuv[2]);
    }

    //=================================================================================================

    /// <summary>Computes the point and the derivatives up to order 3 at (U, V) (see <see cref="D0"/>).</summary>
    public static void D3(double U,
                          double V,
                          int UIndex,
                          int VIndex,
                          NCollection_Array2<gp_Pnt> Poles,
                          NCollection_Array2<double>? Weights,
                          NCollection_Array1<double> UKnots,
                          NCollection_Array1<double> VKnots,
                          NCollection_Array1<int>? UMults,
                          NCollection_Array1<int>? VMults,
                          int UDegree,
                          int VDegree,
                          bool URat,
                          bool VRat,
                          bool UPer,
                          bool VPer,
                          out gp_Pnt P,
                          out gp_Vec Vu,
                          out gp_Vec Vv,
                          out gp_Vec Vuu,
                          out gp_Vec Vvv,
                          out gp_Vec Vuv,
                          out gp_Vec Vuuu,
                          out gp_Vec Vvvv,
                          out gp_Vec Vuuv,
                          out gp_Vec Vuvv)
    {
        bool rational;
        //  int k,dim,dim2;
        int dim, dim2;
        double u1, u2;
        int d1, d2;
        scoped ReadOnlySpan<double> result;
        scoped ReadOnlySpan<double> resVu, resVv, resVuu, resVvv, resVuv, resVuuu, resVvvv, resVuuv, resVuvv;
        validateBSplineDegree(UDegree, VDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[dcPolesSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcDersSize]);
        if (PrepareEval(U,
                        V,
                        UIndex,
                        VIndex,
                        UDegree,
                        VDegree,
                        URat,
                        VRat,
                        UPer,
                        VPer,
                        Poles,
                        Weights,
                        UKnots,
                        VKnots,
                        UMults,
                        VMults,
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
                BSplCLib.Bohm(u1, d1, 3, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 3, dc.knots2, dim, dc.poles);
                BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                if (d1 > 2)
                {
                    BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[((dim2 << 1) + dim2)..]);
                }
                BSplSLib.RationalDerivative(d1, d2, 3, 3, dc.poles, dc.ders);
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
                BSplCLib.Bohm(u1, d1, 3, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 3, dc.knots2, dim, dc.poles);
                BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                if (d1 > 2)
                {
                    BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[((dim2 << 1) + dim2)..]);
                }
                result = dc.poles;
                resVu = result[dim2..];
                resVv = result[3..];
                if (UDegree <= 1)
                {
                    resVuu = BSplSLib_zero;
                    resVuuv = BSplSLib_zero;
                }
                else
                {
                    resVuu = result[(dim2 << 1)..];
                    resVuuv = result[((dim2 << 1) + 3)..];
                }
                if (VDegree <= 1)
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
                if (UDegree <= 2)
                {
                    resVuuu = BSplSLib_zero;
                }
                else
                {
                    resVuuu = result[((dim2 << 1) + dim2)..];
                }
                if (VDegree <= 2)
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
                BSplCLib.Bohm(u1, d1, 3, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 3, dc.knots2, dim, dc.poles);
                BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                if (d1 > 2)
                {
                    BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[((dim2 << 1) + dim2)..]);
                }
                BSplSLib.RationalDerivative(d1, d2, 3, 3, dc.poles, dc.ders);
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
                BSplCLib.Bohm(u1, d1, 3, dc.knots1, dim2, dc.poles);
                BSplCLib.Bohm(u2, d2, 3, dc.knots2, dim, dc.poles);
                BSplCLib.Bohm(u2, d2, 2, dc.knots2, dim, dc.poles[dim2..]);
                if (d1 > 1)
                {
                    BSplCLib.Bohm(u2, d2, 1, dc.knots2, dim, dc.poles[(dim2 << 1)..]);
                }
                if (d1 > 2)
                {
                    BSplCLib.Eval(u2, d2, dc.knots2, dim, dc.poles[((dim2 << 1) + dim2)..]);
                }
                result = dc.poles;
                resVu = result[3..];
                resVv = result[dim2..];
                if (UDegree <= 1)
                {
                    resVuu = BSplSLib_zero;
                    resVuuv = BSplSLib_zero;
                }
                else
                {
                    resVuu = result[6..];
                    resVuuv = result[(dim2 + 6)..];
                }
                if (VDegree <= 1)
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
                if (UDegree <= 2)
                {
                    resVuuu = BSplSLib_zero;
                }
                else
                {
                    resVuuu = result[9..];
                }
                if (VDegree <= 2)
                {
                    resVvvv = BSplSLib_zero;
                }
                else
                {
                    resVvvv = result[((dim2 << 1) + dim2)..];
                }
            }
        }

        P = new gp_Pnt(result[0], result[1], result[2]);
        Vu = new gp_Vec(resVu[0], resVu[1], resVu[2]);
        Vv = new gp_Vec(resVv[0], resVv[1], resVv[2]);
        Vuu = new gp_Vec(resVuu[0], resVuu[1], resVuu[2]);
        Vvv = new gp_Vec(resVvv[0], resVvv[1], resVvv[2]);
        Vuv = new gp_Vec(resVuv[0], resVuv[1], resVuv[2]);
        Vuuu = new gp_Vec(resVuuu[0], resVuuu[1], resVuuu[2]);
        Vvvv = new gp_Vec(resVvvv[0], resVvvv[1], resVvvv[2]);
        Vuuv = new gp_Vec(resVuuv[0], resVuuv[1], resVuuv[2]);
        Vuvv = new gp_Vec(resVuvv[0], resVuvv[1], resVuvv[2]);
    }

    //=================================================================================================

    /// <summary>Computes the derivative of order (Nu, Nv) at (U, V) (see <see cref="D0"/>).</summary>
    public static void DN(double U,
                          double V,
                          int Nu,
                          int Nv,
                          int UIndex,
                          int VIndex,
                          NCollection_Array2<gp_Pnt> Poles,
                          NCollection_Array2<double>? Weights,
                          NCollection_Array1<double> UKnots,
                          NCollection_Array1<double> VKnots,
                          NCollection_Array1<int>? UMults,
                          NCollection_Array1<int>? VMults,
                          int UDegree,
                          int VDegree,
                          bool URat,
                          bool VRat,
                          bool UPer,
                          bool VPer,
                          out gp_Vec Vn)
    {
        bool rational;
        int k, dim;
        double u1, u2;
        int d1, d2;

        validateBSplineDegree(UDegree, VDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[dcPolesSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcDersSize]);
        bool ufirst = PrepareEval(U,
                                  V,
                                  UIndex,
                                  VIndex,
                                  UDegree,
                                  VDegree,
                                  URat,
                                  VRat,
                                  UPer,
                                  VPer,
                                  Poles,
                                  Weights,
                                  UKnots,
                                  VKnots,
                                  UMults,
                                  VMults,
                                  out u1,
                                  out u2,
                                  out d1,
                                  out d2,
                                  out rational,
                                  dc);
        dim = rational ? 4 : 3;

        if (!rational)
        {
            if ((Nu > UDegree) || (Nv > VDegree))
            {
                Vn = new gp_Vec(0.0, 0.0, 0.0);
                return;
            }
        }

        int n1 = ufirst ? Nu : Nv;
        int n2 = ufirst ? Nv : Nu;

        BSplCLib.Bohm(u1, d1, n1, dc.knots1, dim * (d2 + 1), dc.poles);

        for (k = 0; k <= Min(n1, d1); k++)
        {
            BSplCLib.Bohm(u2, d2, n2, dc.knots2, dim, dc.poles[(k * dim * (d2 + 1))..]);
        }

        scoped ReadOnlySpan<double> result;
        if (rational)
        {
            BSplSLib.RationalDerivative(d1, d2, n1, n2, dc.poles, dc.ders, false);
            result = dc.ders; // because false ci-dessus.
        }
        else
        {
            result = dc.poles[((n1 * (d2 + 1) + n2) * dim)..];
        }

        Vn = new gp_Vec(result[0], result[1], result[2]);
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
    /// Computes the poles and weights of an isoparametric curve at <paramref name="Param"/> (a U iso if
    /// <paramref name="IsU"/>, a V iso otherwise). CWeights is required for a rational surface; for a non-rational
    /// one it is filled with 1 if given.
    /// </summary>
    public static void Iso(double Param,
                           bool IsU,
                           NCollection_Array2<gp_Pnt> Poles,
                           NCollection_Array2<double>? Weights,
                           NCollection_Array1<double> Knots,
                           NCollection_Array1<int>? Mults,
                           int Degree,
                           bool Periodic,
                           NCollection_Array1<gp_Pnt> CPoles,
                           NCollection_Array1<double>? CWeights)
    {
        int index = 0;
        double u = Param;
        bool rational = Weights != null;
        int dim = rational ? 4 : 3;

        // compute local knots

        int aKnotsSize = 2 * Degree;
        Span<double> locknots1 = aKnotsSize <= 128 ? stackalloc double[aKnotsSize] : new double[aKnotsSize];
        BSplCLib.LocateParameter(Degree, Knots, Mults, u, Periodic, ref index, out u);
        BSplCLib.BuildKnots(Degree, index, Periodic, Knots, Mults, locknots1);
        if (Mults == null)
        {
            index -= Knots.Lower() + Degree;
        }
        else
        {
            index = BSplCLib.PoleIndex(Degree, index, Periodic, Mults);
        }

        // copy the local poles

        //  int f1,l1,f2,l2,i,j,k;
        int f1, l1, f2, l2, i, j;

        if (IsU)
        {
            f1 = Poles.LowerRow();
            l1 = Poles.UpperRow();
            f2 = Poles.LowerCol();
            l2 = Poles.UpperCol();
        }
        else
        {
            f1 = Poles.LowerCol();
            l1 = Poles.UpperCol();
            f2 = Poles.LowerRow();
            l2 = Poles.UpperRow();
        }

        int aPolesSize = (Degree + 1) * (l2 - f2 + 1) * dim;
        Span<double> locpoles = aPolesSize <= 128 ? stackalloc double[aPolesSize] : new double[aPolesSize];

        double w;
        int pole = 0;
        index += f1;

        for (i = 0; i <= Degree; i++)
        {

            for (j = f2; j <= l2; j++)
            {

                gp_Pnt P = IsU ? Poles[index, j] : Poles[j, index];
                if (rational)
                {
                    locpoles[pole + 3] = w = IsU ? Weights![index, j] : Weights![j, index];
                    locpoles[pole + 0] = P.X() * w;
                    locpoles[pole + 1] = P.Y() * w;
                    locpoles[pole + 2] = P.Z() * w;
                }
                else
                {
                    locpoles[pole + 0] = P.X();
                    locpoles[pole + 1] = P.Y();
                    locpoles[pole + 2] = P.Z();
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
        BSplCLib.Eval(u, Degree, locknots1, (l2 - f2 + 1) * dim, locpoles);

        // get the result
        pole = 0;

        for (i = CPoles.Lower(); i <= CPoles.Upper(); i++)
        {
            if (rational)
            {
                CWeights![i] = w = locpoles[pole + 3];
                CPoles[i] = new gp_Pnt(locpoles[pole + 0] / w, locpoles[pole + 1] / w, locpoles[pole + 2] / w);
            }
            else
            {
                CPoles[i] = new gp_Pnt(locpoles[pole + 0], locpoles[pole + 1], locpoles[pole + 2]);
            }
            pole += dim;
        }

        // if the input is not rational but weights are wanted
        if (!rational && (CWeights != null))
        {

            for (i = CWeights.Lower(); i <= CWeights.Upper(); i++)
            {
                CWeights[i] = 1.0;
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Reverses the array of poles in one direction. <paramref name="Last"/> is the index of the new first row
    /// (column): Poles.Upper() on a non-periodic surface, the number of flat knots - degree - 1 (or the sum of the
    /// multiplicities but the last + degree - 1) on a periodic one.
    /// </summary>
    public static void Reverse(NCollection_Array2<gp_Pnt> Poles, int Last, bool UDirection)
    {
        int i, j, l = Last;
        if (UDirection)
        {
            l = Poles.LowerRow() + (l - Poles.LowerRow()) % (Poles.ColLength());
            NCollection_Array2<gp_Pnt> temp = new(0, Poles.ColLength() - 1, Poles.LowerCol(), Poles.UpperCol());

            for (i = Poles.LowerRow(); i <= l; i++)
            {

                for (j = Poles.LowerCol(); j <= Poles.UpperCol(); j++)
                {
                    temp[l - i, j] = Poles[i, j];
                }
            }

            for (i = l + 1; i <= Poles.UpperRow(); i++)
            {

                for (j = Poles.LowerCol(); j <= Poles.UpperCol(); j++)
                {
                    temp[l + Poles.ColLength() - i, j] = Poles[i, j];
                }
            }

            for (i = Poles.LowerRow(); i <= Poles.UpperRow(); i++)
            {

                for (j = Poles.LowerCol(); j <= Poles.UpperCol(); j++)
                {
                    Poles[i, j] = temp[i - Poles.LowerRow(), j];
                }
            }
        }
        else
        {
            l = Poles.LowerCol() + (l - Poles.LowerCol()) % (Poles.RowLength());
            NCollection_Array2<gp_Pnt> temp = new(Poles.LowerRow(), Poles.UpperRow(), 0, Poles.RowLength() - 1);

            for (j = Poles.LowerCol(); j <= l; j++)
            {

                for (i = Poles.LowerRow(); i <= Poles.UpperRow(); i++)
                {
                    temp[i, l - j] = Poles[i, j];
                }
            }

            for (j = l + 1; j <= Poles.UpperCol(); j++)
            {

                for (i = Poles.LowerRow(); i <= Poles.UpperRow(); i++)
                {
                    temp[i, l + Poles.RowLength() - j] = Poles[i, j];
                }
            }

            for (i = Poles.LowerRow(); i <= Poles.UpperRow(); i++)
            {

                for (j = Poles.LowerCol(); j <= Poles.UpperCol(); j++)
                {
                    Poles[i, j] = temp[i, j - Poles.LowerCol()];
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Reverses the array of weights in one direction (see the poles' <see cref="Reverse(NCollection_Array2{gp_Pnt}, int, bool)"/>).</summary>
    public static void Reverse(NCollection_Array2<double> Weights, int Last, bool UDirection)
    {
        int i, j, l = Last;
        if (UDirection)
        {
            l = Weights.LowerRow() + (l - Weights.LowerRow()) % (Weights.ColLength());
            NCollection_Array2<double> temp = new(0,
                                                  Weights.ColLength() - 1,
                                                  Weights.LowerCol(),
                                                  Weights.UpperCol());

            for (i = Weights.LowerRow(); i <= l; i++)
            {

                for (j = Weights.LowerCol(); j <= Weights.UpperCol(); j++)
                {
                    temp[l - i, j] = Weights[i, j];
                }
            }

            for (i = l + 1; i <= Weights.UpperRow(); i++)
            {

                for (j = Weights.LowerCol(); j <= Weights.UpperCol(); j++)
                {
                    temp[l + Weights.ColLength() - i, j] = Weights[i, j];
                }
            }

            for (i = Weights.LowerRow(); i <= Weights.UpperRow(); i++)
            {

                for (j = Weights.LowerCol(); j <= Weights.UpperCol(); j++)
                {
                    Weights[i, j] = temp[i - Weights.LowerRow(), j];
                }
            }
        }
        else
        {
            l = Weights.LowerCol() + (l - Weights.LowerCol()) % (Weights.RowLength());
            NCollection_Array2<double> temp = new(Weights.LowerRow(),
                                                  Weights.UpperRow(),
                                                  0,
                                                  Weights.RowLength() - 1);

            for (j = Weights.LowerCol(); j <= l; j++)
            {

                for (i = Weights.LowerRow(); i <= Weights.UpperRow(); i++)
                {
                    temp[i, l - j] = Weights[i, j];
                }
            }

            for (j = l + 1; j <= Weights.UpperCol(); j++)
            {

                for (i = Weights.LowerRow(); i <= Weights.UpperRow(); i++)
                {
                    temp[i, l + Weights.RowLength() - j] = Weights[i, j];
                }
            }

            for (i = Weights.LowerRow(); i <= Weights.UpperRow(); i++)
            {

                for (j = Weights.LowerCol(); j <= Weights.UpperCol(); j++)
                {
                    Weights[i, j] = temp[i, j - Weights.LowerCol()];
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// False if all the weights in the area [I1, I2] x [J1, J2] are identical, compared with
    /// <paramref name="Epsi"/>, or with the Epsilon of the first weight if Epsi is 0.
    /// </summary>
    public static bool IsRational(NCollection_Array2<double> Weights,
                                  int I1,
                                  int I2,
                                  int J1,
                                  int J2,
                                  double Epsi = 0.0)
    {
        // OCCT: the default tolerance is taken from Weights(I1, I2) (a row index used as a column index), not
        // Weights(I1, J1). OCCT's release build reads that address without a range check; the port reads the same
        // element of the storage (row I1, column I2 counted along the rows) rather than throwing.
        double eps = (Epsi > 0.0)
                         ? Epsi
                         : Epsilon(Weights.AsSpan()[(I1 - Weights.LowerRow()) * Weights.RowLength() + (I2 - Weights.LowerCol())]);
        int i, j;
        int fi = Weights.LowerRow(), li = Weights.ColLength();
        int fj = Weights.LowerCol(), lj = Weights.RowLength();

        for (i = I1 - fi; i < I2 - fi; i++)
        {

            for (j = J1 - fj; j < J2 - fj; j++)
            {
                if (Math.Abs(Weights[fi + i % li, fj + j % lj] - Weights[fi + (i + 1) % li, fj + j % lj])
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
    /// Copies the coordinates of the poles into FP, row by row if <paramref name="UDirection"/>, column by column
    /// otherwise.
    /// </summary>
    public static void SetPoles(NCollection_Array2<gp_Pnt> Poles,
                                NCollection_Array1<double> FP,
                                bool UDirection)
    {
        int i, j, l = FP.Lower();
        int PLowerRow = Poles.LowerRow();
        int PUpperRow = Poles.UpperRow();
        int PLowerCol = Poles.LowerCol();
        int PUpperCol = Poles.UpperCol();
        if (UDirection)
        {

            for (i = PLowerRow; i <= PUpperRow; i++)
            {

                for (j = PLowerCol; j <= PUpperCol; j++)
                {
                    gp_Pnt P = Poles.Value(i, j);
                    FP[l] = P.X();
                    l++;
                    FP[l] = P.Y();
                    l++;
                    FP[l] = P.Z();
                    l++;
                }
            }
        }
        else
        {

            for (j = PLowerCol; j <= PUpperCol; j++)
            {

                for (i = PLowerRow; i <= PUpperRow; i++)
                {
                    gp_Pnt P = Poles.Value(i, j);
                    FP[l] = P.X();
                    l++;
                    FP[l] = P.Y();
                    l++;
                    FP[l] = P.Z();
                    l++;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Copies the homogeneous coordinates of the poles (x * w, y * w, z * w, w) into FP, row by row if
    /// <paramref name="UDirection"/>, column by column otherwise.
    /// </summary>
    public static void SetPoles(NCollection_Array2<gp_Pnt> Poles,
                                NCollection_Array2<double> Weights,
                                NCollection_Array1<double> FP,
                                bool UDirection)
    {
        int i, j, l = FP.Lower();
        int PLowerRow = Poles.LowerRow();
        int PUpperRow = Poles.UpperRow();
        int PLowerCol = Poles.LowerCol();
        int PUpperCol = Poles.UpperCol();
        if (UDirection)
        {

            for (i = PLowerRow; i <= PUpperRow; i++)
            {

                for (j = PLowerCol; j <= PUpperCol; j++)
                {
                    gp_Pnt P = Poles.Value(i, j);
                    double w = Weights.Value(i, j);
                    FP[l] = P.X() * w;
                    l++;
                    FP[l] = P.Y() * w;
                    l++;
                    FP[l] = P.Z() * w;
                    l++;
                    FP[l] = w;
                    l++;
                }
            }
        }
        else
        {

            for (j = PLowerCol; j <= PUpperCol; j++)
            {

                for (i = PLowerRow; i <= PUpperRow; i++)
                {
                    gp_Pnt P = Poles.Value(i, j);
                    double w = Weights.Value(i, j);
                    FP[l] = P.X() * w;
                    l++;
                    FP[l] = P.Y() * w;
                    l++;
                    FP[l] = P.Z() * w;
                    l++;
                    FP[l] = w;
                    l++;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Gets the coordinates of the poles from FP (the inverse of <see cref="SetPoles(NCollection_Array2{gp_Pnt}, NCollection_Array1{double}, bool)"/>).</summary>
    public static void GetPoles(NCollection_Array1<double> FP,
                                NCollection_Array2<gp_Pnt> Poles,
                                bool UDirection)
    {
        int i, j, l = FP.Lower();
        int PLowerRow = Poles.LowerRow();
        int PUpperRow = Poles.UpperRow();
        int PLowerCol = Poles.LowerCol();
        int PUpperCol = Poles.UpperCol();
        if (UDirection)
        {

            for (i = PLowerRow; i <= PUpperRow; i++)
            {

                for (j = PLowerCol; j <= PUpperCol; j++)
                {
                    ref gp_Pnt P = ref Poles.ChangeValue(i, j);
                    P.SetX(FP[l]);
                    l++;
                    P.SetY(FP[l]);
                    l++;
                    P.SetZ(FP[l]);
                    l++;
                }
            }
        }
        else
        {

            for (j = PLowerCol; j <= PUpperCol; j++)
            {

                for (i = PLowerRow; i <= PUpperRow; i++)
                {
                    ref gp_Pnt P = ref Poles.ChangeValue(i, j);
                    P.SetX(FP[l]);
                    l++;
                    P.SetY(FP[l]);
                    l++;
                    P.SetZ(FP[l]);
                    l++;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Gets the poles and weights from the homogeneous coordinates in FP (the inverse of
    /// <see cref="SetPoles(NCollection_Array2{gp_Pnt}, NCollection_Array2{double}, NCollection_Array1{double}, bool)"/>).
    /// </summary>
    public static void GetPoles(NCollection_Array1<double> FP,
                                NCollection_Array2<gp_Pnt> Poles,
                                NCollection_Array2<double> Weights,
                                bool UDirection)
    {
        int i, j, l = FP.Lower();
        int PLowerRow = Poles.LowerRow();
        int PUpperRow = Poles.UpperRow();
        int PLowerCol = Poles.LowerCol();
        int PUpperCol = Poles.UpperCol();
        if (UDirection)
        {

            for (i = PLowerRow; i <= PUpperRow; i++)
            {

                for (j = PLowerCol; j <= PUpperCol; j++)
                {
                    double w = FP[l + 3];
                    Weights[i, j] = w;
                    ref gp_Pnt P = ref Poles.ChangeValue(i, j);
                    P.SetX(FP[l] / w);
                    l++;
                    P.SetY(FP[l] / w);
                    l++;
                    P.SetZ(FP[l] / w);
                    l++;
                    l++;
                }
            }
        }
        else
        {

            for (j = PLowerCol; j <= PUpperCol; j++)
            {

                for (i = PLowerRow; i <= PUpperRow; i++)
                {
                    double w = FP[l + 3];
                    Weights[i, j] = w;
                    ref gp_Pnt P = ref Poles.ChangeValue(i, j);
                    P.SetX(FP[l] / w);
                    l++;
                    P.SetY(FP[l] / w);
                    l++;
                    P.SetZ(FP[l] / w);
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
    public static void InsertKnots(bool UDirection,
                                   int Degree,
                                   bool Periodic,
                                   NCollection_Array2<gp_Pnt> Poles,
                                   NCollection_Array2<double>? Weights,
                                   NCollection_Array1<double> Knots,
                                   NCollection_Array1<int> Mults,
                                   NCollection_Array1<double> AddKnots,
                                   NCollection_Array1<int>? AddMults,
                                   NCollection_Array2<gp_Pnt> NewPoles,
                                   NCollection_Array2<double>? NewWeights,
                                   NCollection_Array1<double> NewKnots,
                                   NCollection_Array1<int> NewMults,
                                   double Epsilon,
                                   bool Add = true)
    {
        bool rational = Weights != null;
        int dim = 3;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.RowLength() * Poles.ColLength());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.RowLength() * NewPoles.ColLength());

        if (rational)
        {
            SetPoles(Poles, Weights!, poles, UDirection);
        }
        else
        {
            SetPoles(Poles, poles, UDirection);
        }

        if (UDirection)
        {
            dim *= Poles.RowLength();
        }
        else
        {
            dim *= Poles.ColLength();
        }
        BSplCLib.InsertKnots(Degree,
                             Periodic,
                             dim,
                             poles,
                             Knots,
                             Mults,
                             AddKnots,
                             AddMults,
                             newpoles,
                             NewKnots,
                             NewMults,
                             Epsilon,
                             Add);

        if (rational)
        {
            GetPoles(newpoles, NewPoles, NewWeights!, UDirection);
        }
        else
        {
            GetPoles(newpoles, NewPoles, UDirection);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Removes a knot in one direction of the surface, if the result stays within <paramref name="Tolerance"/>
    /// (see <see cref="BSplCLib"/>'s RemoveKnot). Returns false, leaving the outputs unchanged, otherwise.
    /// </summary>
    public static bool RemoveKnot(bool UDirection,
                                  int Index,
                                  int Mult,
                                  int Degree,
                                  bool Periodic,
                                  NCollection_Array2<gp_Pnt> Poles,
                                  NCollection_Array2<double>? Weights,
                                  NCollection_Array1<double> Knots,
                                  NCollection_Array1<int> Mults,
                                  NCollection_Array2<gp_Pnt> NewPoles,
                                  NCollection_Array2<double>? NewWeights,
                                  NCollection_Array1<double> NewKnots,
                                  NCollection_Array1<int> NewMults,
                                  double Tolerance)
    {
        bool rational = Weights != null;
        int dim = 3;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.RowLength() * Poles.ColLength());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.RowLength() * NewPoles.ColLength());

        if (rational)
        {
            SetPoles(Poles, Weights!, poles, UDirection);
        }
        else
        {
            SetPoles(Poles, poles, UDirection);
        }

        if (UDirection)
        {
            dim *= Poles.RowLength();
        }
        else
        {
            dim *= Poles.ColLength();
        }

        if (!BSplCLib.RemoveKnot(Index,
                                 Mult,
                                 Degree,
                                 Periodic,
                                 dim,
                                 poles,
                                 Knots,
                                 Mults,
                                 newpoles,
                                 NewKnots,
                                 NewMults,
                                 Tolerance))
        {
            return false;
        }

        if (rational)
        {
            GetPoles(newpoles, NewPoles, NewWeights!, UDirection);
        }
        else
        {
            GetPoles(newpoles, NewPoles, UDirection);
        }
        return true;
    }

    //=================================================================================================

    /// <summary>Raises the degree in one direction of the surface (see <see cref="BSplCLib"/>'s IncreaseDegree).</summary>
    public static void IncreaseDegree(bool UDirection,
                                      int Degree,
                                      int NewDegree,
                                      bool Periodic,
                                      NCollection_Array2<gp_Pnt> Poles,
                                      NCollection_Array2<double>? Weights,
                                      NCollection_Array1<double> Knots,
                                      NCollection_Array1<int> Mults,
                                      NCollection_Array2<gp_Pnt> NewPoles,
                                      NCollection_Array2<double>? NewWeights,
                                      NCollection_Array1<double> NewKnots,
                                      NCollection_Array1<int> NewMults)
    {
        bool rational = Weights != null;
        int dim = 3;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.RowLength() * Poles.ColLength());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.RowLength() * NewPoles.ColLength());

        if (rational)
        {
            SetPoles(Poles, Weights!, poles, UDirection);
        }
        else
        {
            SetPoles(Poles, poles, UDirection);
        }

        if (UDirection)
        {
            dim *= Poles.RowLength();
        }
        else
        {
            dim *= Poles.ColLength();
        }

        BSplCLib.IncreaseDegree(Degree,
                                NewDegree,
                                Periodic,
                                dim,
                                poles,
                                Knots,
                                Mults,
                                newpoles,
                                NewKnots,
                                NewMults);

        if (rational)
        {
            GetPoles(newpoles, NewPoles, NewWeights!, UDirection);
        }
        else
        {
            GetPoles(newpoles, NewPoles, UDirection);
        }
    }

    //=================================================================================================

    /// <summary>Makes one direction of a periodic surface non-periodic (see <see cref="BSplCLib"/>'s Unperiodize).</summary>
    public static void Unperiodize(bool UDirection,
                                   int Degree,
                                   NCollection_Array1<int> Mults,
                                   NCollection_Array1<double> Knots,
                                   NCollection_Array2<gp_Pnt> Poles,
                                   NCollection_Array2<double>? Weights,
                                   NCollection_Array1<int> NewMults,
                                   NCollection_Array1<double> NewKnots,
                                   NCollection_Array2<gp_Pnt> NewPoles,
                                   NCollection_Array2<double>? NewWeights)
    {
        bool rational = Weights != null;
        int dim = 3;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.RowLength() * Poles.ColLength());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.RowLength() * NewPoles.ColLength());

        if (rational)
        {
            SetPoles(Poles, Weights!, poles, UDirection);
        }
        else
        {
            SetPoles(Poles, poles, UDirection);
        }

        if (UDirection)
        {
            dim *= Poles.RowLength();
        }
        else
        {
            dim *= Poles.ColLength();
        }

        BSplCLib.Unperiodize(Degree, dim, Mults, Knots, poles, NewMults, NewKnots, newpoles);

        if (rational)
        {
            GetPoles(newpoles, NewPoles, NewWeights!, UDirection);
        }
        else
        {
            GetPoles(newpoles, NewPoles, UDirection);
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
    public static void BuildCache(double U,
                                  double V,
                                  double USpanDomain,
                                  double VSpanDomain,
                                  bool UPeriodic,
                                  bool VPeriodic,
                                  int UDegree,
                                  int VDegree,
                                  int UIndex,
                                  int VIndex,
                                  NCollection_Array1<double> UFlatKnots,
                                  NCollection_Array1<double> VFlatKnots,
                                  NCollection_Array2<gp_Pnt> Poles,
                                  NCollection_Array2<double>? Weights,
                                  NCollection_Array2<gp_Pnt> CachePoles,
                                  NCollection_Array2<double>? CacheWeights)
    {
        bool rational, rational_u, rational_v, flag_u_or_v;
        int kk, d1, d1p1, d2, d2p1, ii, jj, iii, jjj, Index;
        double u1, min_degree_domain, max_degree_domain, f, u2;
        Span<double> factor = stackalloc double[2];
        if (Weights != null)
        {
            rational_u = rational_v = true;
        }
        else
        {
            rational_u = rational_v = false;
        }
        validateBSplineDegree(UDegree, VDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[dcPolesSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcKnotsSize(UDegree, VDegree)],
                                        stackalloc double[dcDersSize]);
        flag_u_or_v = PrepareEval(U,
                                  V,
                                  UIndex,
                                  VIndex,
                                  UDegree,
                                  VDegree,
                                  rational_u,
                                  rational_v,
                                  UPeriodic,
                                  VPeriodic,
                                  Poles,
                                  Weights,
                                  UFlatKnots,
                                  VFlatKnots,
                                  (BSplCLib.NoMults()),
                                  (BSplCLib.NoMults()),
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
            BSplCLib.Bohm(u1, d1, d1, dc.knots1, 4 * d2p1, dc.poles);

            for (kk = 0; kk <= d1; kk++)
            {
                BSplCLib.Bohm(u2, d2, d2, dc.knots2, 4, dc.poles[(kk * 4 * d2p1)..]);
            }
            if (flag_u_or_v)
            {
                min_degree_domain = USpanDomain;
                max_degree_domain = VSpanDomain;
            }
            else
            {
                min_degree_domain = VSpanDomain;
                max_degree_domain = USpanDomain;
            }
            factor[0] = 1.0e0;

            for (ii = 0; ii <= d2; ii++)
            {
                iii = ii + 1;
                factor[1] = 1.0e0;

                for (jj = 0; jj <= d1; jj++)
                {
                    jjj = jj + 1;
                    Index = jj * d2p1 + ii;
                    Index = Index << 2;
                    f = factor[0] * factor[1];
                    double aX = f * dc.poles[Index];
                    Index++;
                    double aY = f * dc.poles[Index];
                    Index++;
                    double aZ = f * dc.poles[Index];
                    CachePoles[iii, jjj] = new gp_Pnt(aX, aY, aZ);
                    Index++;
                    CacheWeights![iii, jjj] = f * dc.poles[Index];
                    factor[1] *= min_degree_domain / (double)(jjj);
                }
                factor[0] *= max_degree_domain / (double)(iii);
            }
        }
        else
        {
            BSplCLib.Bohm(u1, d1, d1, dc.knots1, 3 * d2p1, dc.poles);

            for (kk = 0; kk <= d1; kk++)
            {
                BSplCLib.Bohm(u2, d2, d2, dc.knots2, 3, dc.poles[(kk * 3 * d2p1)..]);
            }
            if (flag_u_or_v)
            {
                min_degree_domain = USpanDomain;
                max_degree_domain = VSpanDomain;
            }
            else
            {
                min_degree_domain = VSpanDomain;
                max_degree_domain = USpanDomain;
            }
            factor[0] = 1.0e0;

            for (ii = 0; ii <= d2; ii++)
            {
                iii = ii + 1;
                factor[1] = 1.0e0;

                for (jj = 0; jj <= d1; jj++)
                {
                    jjj = jj + 1;
                    Index = jj * d2p1 + ii;
                    Index = (Index << 1) + Index;
                    f = factor[0] * factor[1];
                    double aX = f * dc.poles[Index];
                    Index++;
                    double aY = f * dc.poles[Index];
                    Index++;
                    double aZ = f * dc.poles[Index];
                    CachePoles[iii, jjj] = new gp_Pnt(aX, aY, aZ);
                    factor[1] *= min_degree_domain / (double)(jjj);
                }
                factor[0] *= max_degree_domain / (double)(iii);
            }
            if (Weights != null)
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
                        CacheWeights![ii, jj] = 0.0e0;
                    }
                }
                CacheWeights![1, 1] = 1.0e0;
            }
        }
    }

    /// <summary>
    /// Computes the Taylor expansion of the span of (theU, theV), normalized to [0, 1], into theCacheArray in the
    /// layout of <c>BSplSLib_Cache</c>: (max degree + 1) rows of (min degree + 1) coefficients of x, y, z (and w for a
    /// rational surface).
    /// </summary>
    public static void BuildCache(double theU,
                                  double theV,
                                  double theUSpanDomain,
                                  double theVSpanDomain,
                                  bool theUPeriodic,
                                  bool theVPeriodic,
                                  int theUDegree,
                                  int theVDegree,
                                  int theUIndex,
                                  int theVIndex,
                                  NCollection_Array1<double> theUFlatKnots,
                                  NCollection_Array1<double> theVFlatKnots,
                                  NCollection_Array2<gp_Pnt> thePoles,
                                  NCollection_Array2<double>? theWeights,
                                  NCollection_Array2<double> theCacheArray)
    {
        bool flag_u_or_v;
        int d1, d2;
        double u1, u2;
        bool isRationalOnParam = (theWeights != null);
        bool isRational;

        validateBSplineDegree(theUDegree, theVDegree);
        BSplSLib_DataContainer dc = new(stackalloc double[dcPolesSize(theUDegree, theVDegree)],
                                        stackalloc double[dcKnotsSize(theUDegree, theVDegree)],
                                        stackalloc double[dcKnotsSize(theUDegree, theVDegree)],
                                        stackalloc double[dcDersSize]);
        flag_u_or_v = PrepareEval(theU,
                                  theV,
                                  theUIndex,
                                  theVIndex,
                                  theUDegree,
                                  theVDegree,
                                  isRationalOnParam,
                                  isRationalOnParam,
                                  theUPeriodic,
                                  theVPeriodic,
                                  thePoles,
                                  theWeights,
                                  theUFlatKnots,
                                  theVFlatKnots,
                                  (BSplCLib.NoMults()),
                                  (BSplCLib.NoMults()),
                                  out u1,
                                  out u2,
                                  out d1,
                                  out d2,
                                  out isRational,
                                  dc);

        int d2p1 = d2 + 1;
        int aDimension = isRational ? 4 : 3;
        int aCacheShift = // helps to store weights when PrepareEval did not found that the
                          // surface is locally polynomial
            (isRationalOnParam && !isRational) ? aDimension + 1 : aDimension;

        Span<double> aDomains = stackalloc double[2];
        // aDomains[0] corresponds to variable with minimal degree
        // aDomains[1] corresponds to variable with maximal degree
        if (flag_u_or_v)
        {
            aDomains[0] = theUSpanDomain;
            aDomains[1] = theVSpanDomain;
        }
        else
        {
            aDomains[0] = theVSpanDomain;
            aDomains[1] = theUSpanDomain;
        }

        BSplCLib.Bohm(u1, d1, d1, dc.knots1, aDimension * d2p1, dc.poles);
        for (int kk = 0; kk <= d1; kk++)
        {
            BSplCLib.Bohm(u2, d2, d2, dc.knots2, aDimension, dc.poles[(kk * aDimension * d2p1)..]);
        }

        // theCacheArray(theCacheArray.LowerRow(), theCacheArray.LowerCol()) on
        Span<double> aCacheArray = theCacheArray.AsSpan();
        int aCache = 0;

        Span<double> aFactors = stackalloc double[2];
        // aFactors[0] corresponds to variable with minimal degree
        // aFactors[1] corresponds to variable with maximal degree
        aFactors[1] = 1.0;
        int aRow, aCol, i;
        double aCoeff;
        for (aRow = 0; aRow <= d2; aRow++)
        {
            aFactors[0] = 1.0;
            for (aCol = 0; aCol <= d1; aCol++)
            {
                ReadOnlySpan<double> aPolyCoeffs = dc.poles[((aCol * d2p1 + aRow) * aDimension)..];
                aCoeff = aFactors[0] * aFactors[1];
                for (i = 0; i < aDimension; i++)
                {
                    aCacheArray[aCache + i] = aPolyCoeffs[i] * aCoeff;
                }
                aCache += aCacheShift;
                aFactors[0] *= aDomains[0] / (aCol + 1);
            }
            aFactors[1] *= aDomains[1] / (aRow + 1);
        }

        // Fill the weights for the surface which is not locally polynomial
        if (aCacheShift > aDimension)
        {
            aCache = 0;
            aCache += aCacheShift - 1;
            for (aRow = 0; aRow <= d2; aRow++)
            {
                for (aCol = 0; aCol <= d1; aCol++)
                {
                    aCacheArray[aCache] = 0.0;
                    aCache += aCacheShift;
                }
            }
            theCacheArray.SetValue(theCacheArray.LowerRow(),
                                   theCacheArray.LowerCol() + aCacheShift - 1,
                                   1.0);
        }
    }

    //=======================================================================
    // function : CacheD0
    // purpose  : Evaluates the polynomial cache of the Bspline Curve
    //
    //=======================================================================

    /// <summary>
    /// Evaluates the point at (U, V) from the cache built by <see cref="BuildCache(double, double, double, double, bool, bool, int, int, int, int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array2{gp_Pnt}, NCollection_Array2{double}?, NCollection_Array2{gp_Pnt}, NCollection_Array2{double}?)"/>:
    /// UCacheParameter and VCacheParameter are where the cache was built, USpanLenght and VSpanLength normalize
    /// the polynomials. The cache must be valid for the point.
    /// </summary>
    public static void CacheD0(double UParameter,
                               double VParameter,
                               int UDegree,
                               int VDegree,
                               double UCacheParameter,
                               double VCacheParameter,
                               double USpanLenght,
                               double VSpanLenght,
                               NCollection_Array2<gp_Pnt> PolesArray,
                               NCollection_Array2<double>? WeightsArray,
                               out gp_Pnt aPoint)
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
        ReadOnlySpan<double> PArray = coordsFrom(PolesArray, PolesArray.LowerCol(), PolesArray.LowerRow());
        Span<double> myPoint = stackalloc double[3];
        if (UDegree <= VDegree)
        {
            min_degree = UDegree;
            max_degree = VDegree;
            new_parameter[1] = (UParameter - UCacheParameter) / USpanLenght;
            new_parameter[0] = (VParameter - VCacheParameter) / VSpanLenght;
            dimension = 3 * (UDegree + 1);
        }
        else
        {
            min_degree = VDegree;
            max_degree = UDegree;
            new_parameter[0] = (UParameter - UCacheParameter) / USpanLenght;
            new_parameter[1] = (VParameter - VCacheParameter) / VSpanLenght;
            dimension = 3 * (VDegree + 1);
        }
        Span<double> locpoles = dimension <= 128 ? stackalloc double[dimension] : new double[dimension];

        PLib.NoDerivativeEvalPolynomial(new_parameter[0],
                                        max_degree,
                                        dimension,
                                        max_degree * dimension,
                                        PArray,
                                        locpoles);

        PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                        min_degree,
                                        3,
                                        (min_degree << 1) + min_degree,
                                        locpoles,
                                        myPoint);
        if (WeightsArray != null)
        {
            dimension = min_degree + 1;
            NCollection_Array2<double> refWeights = WeightsArray;
            // OCCT: row and column swapped, as for the poles
            ReadOnlySpan<double> WArray = valuesFrom(refWeights, refWeights.LowerCol(), refWeights.LowerRow());
            PLib.NoDerivativeEvalPolynomial(new_parameter[0],
                                            max_degree,
                                            dimension,
                                            max_degree * dimension,
                                            WArray,
                                            locpoles);

            PLib.NoDerivativeEvalPolynomial(new_parameter[1],
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
        aPoint = new gp_Pnt(myPoint[0], myPoint[1], myPoint[2]);
    }

    /// <summary>Evaluates a Bezier surface at (U, V) from the coefficients computed by <see cref="PolesCoefficients(NCollection_Array2{gp_Pnt}, NCollection_Array2{double}?, NCollection_Array2{gp_Pnt}, NCollection_Array2{double}?)"/> (Bezier surfaces only).</summary>
    public static void CoefsD0(double U,
                               double V,
                               NCollection_Array2<gp_Pnt> Poles,
                               NCollection_Array2<double>? Weights,
                               out gp_Pnt Point)
    {
        // OCCT: the V degree (RowLength - 1) is passed as UDegree and the U degree as VDegree, while
        // PolesCoefficients returns the coefficients indexed (U, V): with at least as many U poles as V poles
        // this evaluates the surface at (V, U) with the U and V derivatives exchanged, with fewer it is
        // meaningless. No OCCT code calls the Coefs functions.
        BSplSLib.CacheD0(U,
                         V,
                         Poles.RowLength() - 1,
                         Poles.ColLength() - 1,
                         0.0,
                         0.0,
                         1.0,
                         1.0,
                         Poles,
                         Weights,
                         out Point);
    }

    //=======================================================================
    // function : CacheD1
    // purpose  : Evaluates the polynomial cache of the Bspline Curve
    //
    //=======================================================================

    /// <summary>Evaluates the point and first derivatives at (U, V) from the cache (see <see cref="CacheD0"/>).</summary>
    public static void CacheD1(double UParameter,
                               double VParameter,
                               int UDegree,
                               int VDegree,
                               double UCacheParameter,
                               double VCacheParameter,
                               double USpanLenght,
                               double VSpanLenght,
                               NCollection_Array2<gp_Pnt> PolesArray,
                               NCollection_Array2<double>? WeightsArray,
                               out gp_Pnt aPoint,
                               out gp_Vec aVecU,
                               out gp_Vec aVecV)
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
        ReadOnlySpan<double> PArray = coordsFrom(PolesArray, PolesArray.LowerCol(), PolesArray.LowerRow());
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

        bool aUMin;
        if (UDegree <= VDegree)
        {
            min_degree = UDegree;
            max_degree = VDegree;
            inverse_min = 1.0e0 / USpanLenght;
            inverse_max = 1.0e0 / VSpanLenght;
            new_parameter[0] = (VParameter - VCacheParameter) * inverse_max;
            new_parameter[1] = (UParameter - UCacheParameter) * inverse_min;

            dimension = 3 * (UDegree + 1);
            aUMin = true; // my_vec_min is aVecU, my_vec_max is aVecV
        }
        else
        {
            min_degree = VDegree;
            max_degree = UDegree;
            inverse_min = 1.0e0 / VSpanLenght;
            inverse_max = 1.0e0 / USpanLenght;
            new_parameter[0] = (UParameter - UCacheParameter) * inverse_max;
            new_parameter[1] = (VParameter - VCacheParameter) * inverse_min;
            dimension = 3 * (VDegree + 1);
            aUMin = false; // my_vec_min is aVecV, my_vec_max is aVecU
        }

        int aLocSize = 2 * dimension;
        Span<double> locpoles = aLocSize <= 128 ? stackalloc double[aLocSize] : new double[aLocSize];

        PLib.EvalPolynomial(new_parameter[0], 1, max_degree, dimension, PArray, locpoles);

        PLib.EvalPolynomial(new_parameter[1], 1, min_degree, 3, locpoles, local_poles_array);
        PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                        min_degree,
                                        3,
                                        (min_degree << 1) + min_degree,
                                        locpoles[dimension..],
                                        local_poles_array[6..]);

        if (WeightsArray != null)
        {
            dimension = min_degree + 1;
            NCollection_Array2<double> refWeights = WeightsArray;
            // OCCT: row and column swapped (see CacheD0)
            ReadOnlySpan<double> WArray = valuesFrom(refWeights, refWeights.LowerCol(), refWeights.LowerRow());
            PLib.EvalPolynomial(new_parameter[0], 1, max_degree, dimension, WArray, locpoles);

            PLib.EvalPolynomial(new_parameter[1],
                                1,
                                min_degree,
                                1,
                                locpoles,
                                local_weights_array);
            PLib.NoDerivativeEvalPolynomial(new_parameter[1],
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

            BSplSLib.RationalDerivative(1,
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

        aPoint = new gp_Pnt(my_point[0], my_point[1], my_point[2]);
        gp_Vec aVecMin = new(my_vec_min[0], my_vec_min[1], my_vec_min[2]);
        gp_Vec aVecMax = new(my_vec_max[0], my_vec_max[1], my_vec_max[2]);
        aVecU = aUMin ? aVecMin : aVecMax;
        aVecV = aUMin ? aVecMax : aVecMin;
    }

    /// <summary>Evaluates a Bezier surface and its first derivatives at (U, V) from its coefficients (see <see cref="CoefsD0"/>).</summary>
    public static void CoefsD1(double U,
                               double V,
                               NCollection_Array2<gp_Pnt> Poles,
                               NCollection_Array2<double>? Weights,
                               out gp_Pnt Point,
                               out gp_Vec VecU,
                               out gp_Vec VecV)
    {
        // OCCT: the V degree (RowLength - 1) is passed as UDegree and the U degree as VDegree, while
        // PolesCoefficients returns the coefficients indexed (U, V): with at least as many U poles as V poles
        // this evaluates the surface at (V, U) with the U and V derivatives exchanged, with fewer it is
        // meaningless. No OCCT code calls the Coefs functions.
        BSplSLib.CacheD1(U,
                         V,
                         Poles.RowLength() - 1,
                         Poles.ColLength() - 1,
                         0.0,
                         0.0,
                         1.0,
                         1.0,
                         Poles,
                         Weights,
                         out Point,
                         out VecU,
                         out VecV);
    }

    //=======================================================================
    // function : CacheD2
    // purpose  : Evaluates the polynomial cache of the Bspline Curve
    //
    //=======================================================================

    /// <summary>Evaluates the point and the derivatives up to order 2 at (U, V) from the cache (see <see cref="CacheD0"/>).</summary>
    public static void CacheD2(double UParameter,
                               double VParameter,
                               int UDegree,
                               int VDegree,
                               double UCacheParameter,
                               double VCacheParameter,
                               double USpanLenght,
                               double VSpanLenght,
                               NCollection_Array2<gp_Pnt> PolesArray,
                               NCollection_Array2<double>? WeightsArray,
                               out gp_Pnt aPoint,
                               out gp_Vec aVecU,
                               out gp_Vec aVecV,
                               out gp_Vec aVecUU,
                               out gp_Vec aVecUV,
                               out gp_Vec aVecVV)
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
        ReadOnlySpan<double> PArray = coordsFrom(PolesArray, PolesArray.LowerCol(), PolesArray.LowerRow());
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

        bool aUMin;
        if (UDegree <= VDegree)
        {
            min_degree = UDegree;
            max_degree = VDegree;
            inverse_min = 1.0e0 / USpanLenght;
            inverse_max = 1.0e0 / VSpanLenght;
            new_parameter[0] = (VParameter - VCacheParameter) * inverse_max;
            new_parameter[1] = (UParameter - UCacheParameter) * inverse_min;

            dimension = 3 * (UDegree + 1);
            aUMin = true; // min is U (aVecU, aVecUU), max is V (aVecV, aVecVV)
        }
        else
        {
            min_degree = VDegree;
            max_degree = UDegree;
            inverse_min = 1.0e0 / VSpanLenght;
            inverse_max = 1.0e0 / USpanLenght;
            new_parameter[0] = (UParameter - UCacheParameter) * inverse_max;
            new_parameter[1] = (VParameter - VCacheParameter) * inverse_min;
            dimension = 3 * (VDegree + 1);
            aUMin = false; // min is V (aVecV, aVecVV), max is U (aVecU, aVecUU)
        }

        int aLocSize = 3 * dimension;
        Span<double> locpoles = aLocSize <= 128 ? stackalloc double[aLocSize] : new double[aLocSize];

        //
        // initialize in case min or max degree are less than 2
        //
        int MinIndMax = 2;
        if (max_degree < 2)
        {
            MinIndMax = max_degree;
        }
        int MinIndMin = 2;
        if (min_degree < 2)
        {
            MinIndMin = min_degree;
        }

        index = MinIndMax * dimension;

        for (ii = MinIndMax; ii < 3; ii++)
        {

            for (kk = 0; kk < dimension; kk++)
            {
                locpoles[index] = 0.0e0;
                index += 1;
            }
        }

        PLib.EvalPolynomial(new_parameter[0], MinIndMax, max_degree, dimension, PArray, locpoles);

        PLib.EvalPolynomial(new_parameter[1],
                            MinIndMin,
                            min_degree,
                            3,
                            locpoles,
                            local_poles_array);
        PLib.EvalPolynomial(new_parameter[1],
                            1,
                            min_degree,
                            3,
                            locpoles[dimension..],
                            local_poles_array[9..]);

        PLib.NoDerivativeEvalPolynomial(new_parameter[1],
                                        min_degree,
                                        3,
                                        (min_degree << 1) + min_degree,
                                        locpoles[(dimension + dimension)..],
                                        local_poles_array[18..]);

        if (WeightsArray != null)
        {
            // OCCT: locpoles is zeroed above row MinIndMax for the poles' dimension only; with the weights' smaller
            // dimension, a max degree below 2 leaves row 2 holding pole values, which are then taken as the second
            // derivative of the weights: the second derivatives of a rational surface of degrees (1, 1) are wrong
            dimension = min_degree + 1;
            NCollection_Array2<double> refWeights = WeightsArray;
            // OCCT: row and column swapped (see CacheD0)
            ReadOnlySpan<double> WArray = valuesFrom(refWeights, refWeights.LowerCol(), refWeights.LowerRow());
            PLib.EvalPolynomial(new_parameter[0],
                                MinIndMax,
                                max_degree,
                                dimension,
                                WArray,
                                locpoles);

            PLib.EvalPolynomial(new_parameter[1],
                                MinIndMin,
                                min_degree,
                                1,
                                locpoles,
                                local_weights_array);
            PLib.EvalPolynomial(new_parameter[1],
                                1,
                                min_degree,
                                1,
                                locpoles[dimension..],
                                local_weights_array[3..]);
            PLib.NoDerivativeEvalPolynomial(new_parameter[1],
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

            BSplSLib.RationalDerivative(2,
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

        aPoint = new gp_Pnt(my_point[0], my_point[1], my_point[2]);
        gp_Vec aVecMin = new(my_vec_min[0], my_vec_min[1], my_vec_min[2]);
        gp_Vec aVecMax = new(my_vec_max[0], my_vec_max[1], my_vec_max[2]);
        gp_Vec aVecMinMin = new(my_vec_min_min[0], my_vec_min_min[1], my_vec_min_min[2]);
        gp_Vec aVecMaxMax = new(my_vec_max_max[0], my_vec_max_max[1], my_vec_max_max[2]);
        aVecUV = new gp_Vec(my_vec_min_max[0], my_vec_min_max[1], my_vec_min_max[2]);
        aVecU = aUMin ? aVecMin : aVecMax;
        aVecV = aUMin ? aVecMax : aVecMin;
        aVecUU = aUMin ? aVecMinMin : aVecMaxMax;
        aVecVV = aUMin ? aVecMaxMax : aVecMinMin;
    }

    /// <summary>Evaluates a Bezier surface and its derivatives up to order 2 at (U, V) from its coefficients (see <see cref="CoefsD0"/>).</summary>
    public static void CoefsD2(double U,
                               double V,
                               NCollection_Array2<gp_Pnt> Poles,
                               NCollection_Array2<double>? Weights,
                               out gp_Pnt Point,
                               out gp_Vec VecU,
                               out gp_Vec VecV,
                               out gp_Vec VecUU,
                               out gp_Vec VecUV,
                               out gp_Vec VecVV)
    {
        // OCCT: the V degree (RowLength - 1) is passed as UDegree and the U degree as VDegree, while
        // PolesCoefficients returns the coefficients indexed (U, V): with at least as many U poles as V poles
        // this evaluates the surface at (V, U) with the U and V derivatives exchanged, with fewer it is
        // meaningless. No OCCT code calls the Coefs functions.
        BSplSLib.CacheD2(U,
                         V,
                         Poles.RowLength() - 1,
                         Poles.ColLength() - 1,
                         0.0,
                         0.0,
                         1.0,
                         1.0,
                         Poles,
                         Weights,
                         out Point,
                         out VecU,
                         out VecV,
                         out VecUU,
                         out VecUV,
                         out VecVV);
    }

    /// <summary>
    /// The Taylor expansion of a non-rational Bezier surface at (0, 0) (Bezier surfaces only), for
    /// <see cref="CoefsD0"/>.
    /// </summary>
    public static void PolesCoefficients(NCollection_Array2<gp_Pnt> Poles,
                                         NCollection_Array2<gp_Pnt> CachePoles)
    {
        BSplSLib.PolesCoefficients(Poles, BSplSLib.NoWeights(), CachePoles, BSplSLib.NoWeights());
    }

    //=================================================================================================

    /// <summary>
    /// The Taylor expansion of a Bezier surface at (0, 0), through <see cref="BuildCache(double, double, double, double, bool, bool, int, int, int, int, NCollection_Array1{double}, NCollection_Array1{double}, NCollection_Array2{gp_Pnt}, NCollection_Array2{double}?, NCollection_Array2{gp_Pnt}, NCollection_Array2{double}?)"/>
    /// (Bezier surfaces only); CacheWeights is required when Weights is given.
    /// </summary>
    public static void PolesCoefficients(NCollection_Array2<gp_Pnt> Poles,
                                         NCollection_Array2<double>? Weights,
                                         NCollection_Array2<gp_Pnt> CachePoles,
                                         NCollection_Array2<double>? CacheWeights)
    {
        int i;
        int uclas = Poles.ColLength();
        int vclas = Poles.RowLength();
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
            BSplSLib.BuildCache(0.0,
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
                                Poles,
                                Weights,
                                CachePoles,
                                CacheWeights);
        }
        else
        {
            // BuilCache exige que les resultats soient formates en [MaxCoeff,MinCoeff]
            NCollection_Array2<gp_Pnt> CPoles = new(1, vclas, 1, uclas);
            NCollection_Array2<double> CWeights = new(1, vclas, 1, uclas);
            int ii, jj;
            BSplSLib.BuildCache(0.0,
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
                                Poles,
                                Weights,
                                CPoles,
                                CWeights);
            if (Weights == null)
            {

                for (ii = 1; ii <= uclas; ii++)
                {

                    for (jj = 1; jj <= vclas; jj++)
                    {
                        CachePoles[ii, jj] = CPoles[jj, ii];
                    }
                }
            }
            else
            {

                for (ii = 1; ii <= uclas; ii++)
                {

                    for (jj = 1; jj <= vclas; jj++)
                    {
                        CachePoles[ii, jj] = CPoles[jj, ii];
                        CacheWeights![ii, jj] = CWeights[jj, ii];
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
    /// Finds the new poles that move the point at (U, V) by <paramref name="Displ"/>. UIndex1..UIndex2 and
    /// VIndex1..VIndex2 are the ranges of poles that may move ((1, NbPoles - 1) or (2, NbPoles): no constraint on
    /// one side; (2, NbPoles - 1): the ends stay; (1, NbPoles): a rigid move). If the B-spline basis can't be
    /// computed, the indices are 0 and NewPoles is unchanged.
    /// </summary>
    public static void MovePoint(double U,
                                 double V,
                                 in gp_Vec Displ,
                                 int UIndex1,
                                 int UIndex2,
                                 int VIndex1,
                                 int VIndex2,
                                 int UDegree,
                                 int VDegree,
                                 bool Rational,
                                 NCollection_Array2<gp_Pnt> Poles,
                                 NCollection_Array2<double> Weights,
                                 NCollection_Array1<double> UFlatKnots,
                                 NCollection_Array1<double> VFlatKnots,
                                 out int UFirstIndex,
                                 out int ULastIndex,
                                 out int VFirstIndex,
                                 out int VLastIndex,
                                 NCollection_Array2<gp_Pnt> NewPoles)
    {
        // calculate the UBSplineBasis in the parameter U
        math_Matrix UBSplineBasis = new(1, 1, 1, UDegree + 1);
        int ErrorCod1 = BSplCLib.EvalBsplineBasis(0,
                                                  UDegree + 1,
                                                  UFlatKnots,
                                                  U,
                                                  out int UFirstNonZeroBsplineIndex,
                                                  UBSplineBasis);
        // calculate the VBSplineBasis in the parameter V
        math_Matrix VBSplineBasis = new(1, 1, 1, VDegree + 1);
        int ErrorCod2 = BSplCLib.EvalBsplineBasis(0,
                                                  VDegree + 1,
                                                  VFlatKnots,
                                                  V,
                                                  out int VFirstNonZeroBsplineIndex,
                                                  VBSplineBasis);
        if (ErrorCod1 != 0 || ErrorCod2 != 0)
        {
            UFirstIndex = 0;
            ULastIndex = 0;
            VFirstIndex = 0;
            VLastIndex = 0;
            return;
        }

        // find the span which is predominant for parameter U
        UFirstIndex = UFirstNonZeroBsplineIndex;
        ULastIndex = UFirstNonZeroBsplineIndex + UDegree;
        if (UFirstIndex < UIndex1)
        {
            UFirstIndex = UIndex1;
        }
        if (ULastIndex > UIndex2)
        {
            ULastIndex = UIndex2;
        }

        double maxValue = 0.0;
        int ukk1 = 0, ukk2;

        for (int i = UFirstIndex - UFirstNonZeroBsplineIndex + 1;
             i <= ULastIndex - UFirstNonZeroBsplineIndex + 1;
             i++)
        {
            if (UBSplineBasis[1, i] > maxValue)
            {
                ukk1 = i + UFirstNonZeroBsplineIndex - 1;
                maxValue = UBSplineBasis[1, i];
            }
        }

        // find a ukk2 if symmetry
        ukk2 = ukk1;
        // OCCT: when [UFirstIndex, ULastIndex] holds none of the poles acting at U, ukk1 stays 0 and the basis
        // matrix is read out of range below (unchecked in OCCT's release build; the port throws Standard_OutOfRange)
        if ((ukk1 + 1) <= ULastIndex)
        {
            if (Math.Abs(UBSplineBasis[1, ukk1 - UFirstNonZeroBsplineIndex + 2] - maxValue) < 1.0e-10)
            {
                ukk2 = ukk1 + 1;
            }
        }

        // find the span which is predominant for parameter V
        VFirstIndex = VFirstNonZeroBsplineIndex;
        VLastIndex = VFirstNonZeroBsplineIndex + VDegree;

        if (VFirstIndex < VIndex1)
        {
            VFirstIndex = VIndex1;
        }
        if (VLastIndex > VIndex2)
        {
            VLastIndex = VIndex2;
        }

        maxValue = 0.0;
        int vkk1 = 0, vkk2;

        for (int j = VFirstIndex - VFirstNonZeroBsplineIndex + 1;
             j <= VLastIndex - VFirstNonZeroBsplineIndex + 1;
             j++)
        {
            if (VBSplineBasis[1, j] > maxValue)
            {
                vkk1 = j + VFirstNonZeroBsplineIndex - 1;
                maxValue = VBSplineBasis[1, j];
            }
        }

        // find a vkk2 if symmetry
        vkk2 = vkk1;
        // OCCT: the same for V (see ukk1 above)
        if ((vkk1 + 1) <= VLastIndex)
        {
            if (Math.Abs(VBSplineBasis[1, vkk1 - VFirstNonZeroBsplineIndex + 2] - maxValue) < 1.0e-10)
            {
                vkk2 = vkk1 + 1;
            }
        }

        // compute the vector of displacement
        double D1 = 0.0;
        double D2 = 0.0;
        double hN, Coef, DvalU, DvalV;

        for (int i = 1; i <= UDegree + 1; i++)
        {
            int ii = i + UFirstNonZeroBsplineIndex - 1;
            if (ii < ukk1)
            {
                DvalU = ukk1 - ii;
            }
            else if (ii > ukk2)
            {
                DvalU = ii - ukk2;
            }
            else
            {
                DvalU = 0.0;
            }

            for (int j = 1; j <= VDegree + 1; j++)
            {
                int jj = j + VFirstNonZeroBsplineIndex - 1;
                if (Rational)
                {
                    hN = Weights[ii, jj] * UBSplineBasis[1, i] * VBSplineBasis[1, j];
                    D2 += hN;
                }
                else
                {
                    hN = UBSplineBasis[1, i] * VBSplineBasis[1, j];
                }
                if (ii >= UFirstIndex && ii <= ULastIndex && jj >= VFirstIndex && jj <= VLastIndex)
                {
                    if (jj < vkk1)
                    {
                        DvalV = vkk1 - jj;
                    }
                    else if (jj > vkk2)
                    {
                        DvalV = jj - vkk2;
                    }
                    else
                    {
                        DvalV = 0.0;
                    }
                    D1 += 1.0 / (DvalU + DvalV + 1.0) * hN;
                }
            }
        }

        if (Rational)
        {
            Coef = D2 / D1;
        }
        else
        {
            Coef = 1.0 / D1;
        }

        // compute the new poles

        for (int i = Poles.LowerRow(); i <= Poles.UpperRow(); i++)
        {
            if (i < ukk1)
            {
                DvalU = ukk1 - i;
            }
            else if (i > ukk2)
            {
                DvalU = i - ukk2;
            }
            else
            {
                DvalU = 0.0;
            }

            for (int j = Poles.LowerCol(); j <= Poles.UpperCol(); j++)
            {
                if (i >= UFirstIndex && i <= ULastIndex && j >= VFirstIndex && j <= VLastIndex)
                {
                    if (j < vkk1)
                    {
                        DvalV = vkk1 - j;
                    }
                    else if (j > vkk2)
                    {
                        DvalV = j - vkk2;
                    }
                    else
                    {
                        DvalV = 0.0;
                    }
                    NewPoles[i, j] = Poles[i, j].Translated((Coef / (DvalU + DvalV + 1.0)) * Displ);
                }
                else
                {
                    NewPoles[i, j] = Poles[i, j];
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
    public static void Resolution(NCollection_Array2<gp_Pnt> Poles,
                                  NCollection_Array2<double>? Weights,
                                  NCollection_Array1<double> UKnots,
                                  NCollection_Array1<double> VKnots,
                                  NCollection_Array1<int> UMults,
                                  NCollection_Array1<int> VMults,
                                  int UDegree,
                                  int VDegree,
                                  bool URational,
                                  bool VRational,
                                  bool UPeriodic,
                                  bool VPeriodic,
                                  double Tolerance3D,
                                  out double UTolerance,
                                  out double VTolerance)
    {
        double Wij, Wmj, Wji, Wjm;
        double Xij, Xmj, Xji, Xjm, Xpq, Xqp;
        double Yij, Ymj, Yji, Yjm, Ypq, Yqp;
        double Zij, Zmj, Zji, Zjm, Zpq, Zqp;
        double factor, value, min, min_weights = 0, inverse;
        Span<double> max_derivative = stackalloc double[2];

        max_derivative[0] = max_derivative[1] = 0.0e0;

        int PRowLength, PColLength;
        int ii, jj, pp, qq, ii_index, jj_index, pp_index, qq_index;
        int ii_minus;
        Span<int> upper = stackalloc int[2];
        Span<int> lower = stackalloc int[2];
        Span<int> poles_length = stackalloc int[2];
        Span<int> num_poles = stackalloc int[2];
        Span<int> num_flat_knots = stackalloc int[2];

        num_flat_knots[0] = BSplCLib.KnotSequenceLength(UMults, UDegree, UPeriodic);
        num_flat_knots[1] = BSplCLib.KnotSequenceLength(VMults, VDegree, VPeriodic);
        NCollection_Array1<double> flat_knots_in_u = new(1, num_flat_knots[0]);
        NCollection_Array1<double> flat_knots_in_v = new(1, num_flat_knots[1]);
        BSplCLib.KnotSequence(UKnots, UMults, UDegree, UPeriodic, flat_knots_in_u);
        BSplCLib.KnotSequence(VKnots, VMults, VDegree, VPeriodic, flat_knots_in_v);
        PRowLength = Poles.RowLength();
        PColLength = Poles.ColLength();
        if (URational || VRational)
        {
            int Wsize = PRowLength * PColLength;
            NCollection_Array2<double> refWights = Weights!;
            ReadOnlySpan<double> WG = valuesFrom(refWights, refWights.LowerRow(), refWights.LowerCol());
            min_weights = WG[0];

            for (ii = 1; ii < Wsize; ii++)
            {
                min = WG[ii];
                if (min_weights > min)
                {
                    min_weights = min;
                }
            }
        }
        int UD1 = UDegree + 1;
        int VD1 = VDegree + 1;
        num_poles[0] = num_flat_knots[0] - UD1;
        num_poles[1] = num_flat_knots[1] - VD1;
        poles_length[0] = PColLength;
        poles_length[1] = PRowLength;
        if (URational)
        {
            int UD2 = UDegree << 1;
            int VD2 = VDegree << 1;

            for (ii = 2; ii <= num_poles[0]; ii++)
            {
                ii_index = (ii - 1) % poles_length[0] + 1;
                ii_minus = (ii - 2) % poles_length[0] + 1;
                inverse = flat_knots_in_u[ii + UDegree] - flat_knots_in_u[ii];
                inverse = 1.0e0 / inverse;
                lower[0] = ii - UD1;
                if (lower[0] < 1)
                {
                    lower[0] = 1;
                }
                upper[0] = ii + UD2 + 1;
                if (upper[0] > num_poles[0])
                {
                    upper[0] = num_poles[0];
                }

                for (jj = 1; jj <= num_poles[1]; jj++)
                {
                    jj_index = (jj - 1) % poles_length[1] + 1;
                    lower[1] = jj - VD1;
                    if (lower[1] < 1)
                    {
                        lower[1] = 1;
                    }
                    upper[1] = jj + VD2 + 1;
                    if (upper[1] > num_poles[1])
                    {
                        upper[1] = num_poles[1];
                    }
                    gp_Pnt Pij = Poles.Value(ii_index, jj_index);
                    Wij = Weights!.Value(ii_index, jj_index);
                    gp_Pnt Pmj = Poles.Value(ii_minus, jj_index);
                    Wmj = Weights.Value(ii_minus, jj_index);
                    Xij = Pij.X();
                    Yij = Pij.Y();
                    Zij = Pij.Z();
                    Xmj = Pmj.X();
                    Ymj = Pmj.Y();
                    Zmj = Pmj.Z();

                    for (pp = lower[0]; pp <= upper[0]; pp++)
                    {
                        pp_index = (pp - 1) % poles_length[0] + 1;

                        for (qq = lower[1]; qq <= upper[1]; qq++)
                        {
                            value = 0.0e0;
                            qq_index = (qq - 1) % poles_length[1] + 1;
                            gp_Pnt Ppq = Poles.Value(pp_index, qq_index);
                            Xpq = Ppq.X();
                            Ypq = Ppq.Y();
                            Zpq = Ppq.Z();
                            factor = (Xpq - Xij) * Wij;
                            factor -= (Xpq - Xmj) * Wmj;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            factor = (Ypq - Yij) * Wij;
                            factor -= (Ypq - Ymj) * Wmj;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            factor = (Zpq - Zij) * Wij;
                            factor -= (Zpq - Zmj) * Wmj;
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
                inverse = flat_knots_in_u[ii + UDegree] - flat_knots_in_u[ii];
                inverse = 1.0e0 / inverse;

                for (jj = 1; jj <= num_poles[1]; jj++)
                {
                    jj_index = (jj - 1) % poles_length[1] + 1;
                    value = 0.0e0;
                    gp_Pnt Pij = Poles.Value(ii_index, jj_index);
                    gp_Pnt Pmj = Poles.Value(ii_minus, jj_index);
                    factor = Pij.X() - Pmj.X();
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    factor = Pij.Y() - Pmj.Y();
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    factor = Pij.Z() - Pmj.Z();
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
        max_derivative[0] *= UDegree;
        if (VRational)
        {
            int UD2 = UDegree << 1;
            int VD2 = VDegree << 1;

            for (ii = 2; ii <= num_poles[1]; ii++)
            {
                ii_index = (ii - 1) % poles_length[1] + 1;
                ii_minus = (ii - 2) % poles_length[1] + 1;
                inverse = flat_knots_in_v[ii + VDegree] - flat_knots_in_v[ii];
                inverse = 1.0e0 / inverse;
                lower[0] = ii - VD1;
                if (lower[0] < 1)
                {
                    lower[0] = 1;
                }
                upper[0] = ii + VD2 + 1;
                if (upper[0] > num_poles[1])
                {
                    upper[0] = num_poles[1];
                }

                for (jj = 1; jj <= num_poles[0]; jj++)
                {
                    jj_index = (jj - 1) % poles_length[0] + 1;
                    lower[1] = jj - UD1;
                    if (lower[1] < 1)
                    {
                        lower[1] = 1;
                    }
                    upper[1] = jj + UD2 + 1;
                    if (upper[1] > num_poles[0])
                    {
                        upper[1] = num_poles[0];
                    }
                    gp_Pnt Pji = Poles.Value(jj_index, ii_index);
                    Wji = Weights!.Value(jj_index, ii_index);
                    gp_Pnt Pjm = Poles.Value(jj_index, ii_minus);
                    Wjm = Weights.Value(jj_index, ii_minus);
                    Xji = Pji.X();
                    Yji = Pji.Y();
                    Zji = Pji.Z();
                    Xjm = Pjm.X();
                    Yjm = Pjm.Y();
                    Zjm = Pjm.Z();

                    for (pp = lower[1]; pp <= upper[1]; pp++)
                    {
                        pp_index = (pp - 1) % poles_length[1] + 1;

                        for (qq = lower[0]; qq <= upper[0]; qq++)
                        {
                            value = 0.0e0;
                            qq_index = (qq - 1) % poles_length[0] + 1;
                            gp_Pnt Pqp = Poles.Value(qq_index, pp_index);
                            Xqp = Pqp.X();
                            Yqp = Pqp.Y();
                            Zqp = Pqp.Z();
                            factor = (Xqp - Xji) * Wji;
                            factor -= (Xqp - Xjm) * Wjm;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            factor = (Yqp - Yji) * Wji;
                            factor -= (Yqp - Yjm) * Wjm;
                            if (factor < 0)
                            {
                                factor = -factor;
                            }
                            value += factor;
                            factor = (Zqp - Zji) * Wji;
                            factor -= (Zqp - Zjm) * Wjm;
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
                inverse = flat_knots_in_v[ii + VDegree] - flat_knots_in_v[ii];
                inverse = 1.0e0 / inverse;

                for (jj = 1; jj <= num_poles[0]; jj++)
                {
                    jj_index = (jj - 1) % poles_length[0] + 1;
                    value = 0.0e0;
                    gp_Pnt Pji = Poles.Value(jj_index, ii_index);
                    gp_Pnt Pjm = Poles.Value(jj_index, ii_minus);
                    factor = Pji.X() - Pjm.X();
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    factor = Pji.Y() - Pjm.Y();
                    if (factor < 0)
                    {
                        factor = -factor;
                    }
                    value += factor;
                    factor = Pji.Z() - Pjm.Z();
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
        max_derivative[1] *= VDegree;
        max_derivative[0] *= M_SQRT2;
        max_derivative[1] *= M_SQRT2;
        if (max_derivative[0] != 0.0 && max_derivative[1] != 0.0)
        {
            UTolerance = Tolerance3D / max_derivative[0];
            VTolerance = Tolerance3D / max_derivative[1];
        }
        else
        {
            UTolerance = VTolerance = 0.0;
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
    public static void Interpolate(int UDegree,
                                   int VDegree,
                                   NCollection_Array1<double> UFlatKnots,
                                   NCollection_Array1<double> VFlatKnots,
                                   NCollection_Array1<double> UParameters,
                                   NCollection_Array1<double> VParameters,
                                   NCollection_Array2<gp_Pnt> Poles,
                                   NCollection_Array2<double> Weights,
                                   out int InversionProblem)
    {
        int ii, jj, ll, kk, dimension;
        int ULength = UParameters.Length();
        int VLength = VParameters.Length();
        Span<double> poles_array;

        // extraction of iso u
        dimension = 4 * ULength;
        NCollection_Array2<double> Points = new(1, VLength, 1, dimension);

        NCollection_HArray1<int> ContactOrder = new(1, VLength);
        ContactOrder.Init(0);

        for (ii = 1; ii <= VLength; ii++)
        {

            for (jj = 1, ll = 1; jj <= ULength; jj++, ll += 4)
            {
                Points[ii, ll] = Poles[jj, ii].X();
                Points[ii, ll + 1] = Poles[jj, ii].Y();
                Points[ii, ll + 2] = Poles[jj, ii].Z();
                Points[ii, ll + 3] = Weights[jj, ii];
            }
        }

        // interpolation of iso u
        poles_array = Points.AsSpan();
        BSplCLib.Interpolate(VDegree,
                             VFlatKnots,
                             VParameters,
                             ContactOrder,
                             dimension,
                             poles_array,
                             out InversionProblem);
        if (InversionProblem != 0)
        {
            return;
        }

        // extraction of iso v

        dimension = VLength * 4;
        NCollection_Array2<double> IsoPoles = new(1, ULength, 1, dimension);

        ContactOrder = new NCollection_HArray1<int>(1, ULength);
        ContactOrder.Init(0);
        poles_array = IsoPoles.AsSpan();

        for (ii = 1, kk = 1; ii <= ULength; ii++, kk += 4)
        {

            for (jj = 1, ll = 1; jj <= VLength; jj++, ll += 4)
            {
                IsoPoles[ii, ll] = Points[jj, kk];
                IsoPoles[ii, ll + 1] = Points[jj, kk + 1];
                IsoPoles[ii, ll + 2] = Points[jj, kk + 2];
                IsoPoles[ii, ll + 3] = Points[jj, kk + 3];
            }
        }
        // interpolation of iso v
        BSplCLib.Interpolate(UDegree,
                             UFlatKnots,
                             UParameters,
                             ContactOrder,
                             dimension,
                             poles_array,
                             out InversionProblem);

        // return results

        for (ii = 1; ii <= ULength; ii++)
        {

            for (jj = 1, ll = 1; jj <= VLength; jj++, ll += 4)
            {
                gp_Pnt Pnt = new(IsoPoles[ii, ll], IsoPoles[ii, ll + 1], IsoPoles[ii, ll + 2]);
                Poles.SetValue(ii, jj, Pnt);
                Weights.SetValue(ii, jj, IsoPoles[ii, ll + 3]);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Interpolates the data points given in Poles (indexed from 1), which are replaced by the interpolating
    /// surface's poles (see the rational overload).
    /// </summary>
    public static void Interpolate(int UDegree,
                                   int VDegree,
                                   NCollection_Array1<double> UFlatKnots,
                                   NCollection_Array1<double> VFlatKnots,
                                   NCollection_Array1<double> UParameters,
                                   NCollection_Array1<double> VParameters,
                                   NCollection_Array2<gp_Pnt> Poles,
                                   out int InversionProblem)
    {
        int ii, jj, ll, kk, dimension;
        int ULength = UParameters.Length();
        int VLength = VParameters.Length();
        Span<double> poles_array;

        // extraction of iso u
        dimension = 3 * ULength;
        NCollection_Array2<double> Points = new(1, VLength, 1, dimension);

        NCollection_HArray1<int> ContactOrder = new(1, VLength);
        ContactOrder.Init(0);

        for (ii = 1; ii <= VLength; ii++)
        {

            for (jj = 1, ll = 1; jj <= ULength; jj++, ll += 3)
            {
                Points[ii, ll] = Poles[jj, ii].X();
                Points[ii, ll + 1] = Poles[jj, ii].Y();
                Points[ii, ll + 2] = Poles[jj, ii].Z();
            }
        }

        // interpolation of iso u
        poles_array = Points.AsSpan();
        BSplCLib.Interpolate(VDegree,
                             VFlatKnots,
                             VParameters,
                             ContactOrder,
                             dimension,
                             poles_array,
                             out InversionProblem);
        if (InversionProblem != 0)
        {
            return;
        }

        // extraction of iso v

        dimension = VLength * 3;
        NCollection_Array2<double> IsoPoles = new(1, ULength, 1, dimension);

        ContactOrder = new NCollection_HArray1<int>(1, ULength);
        ContactOrder.Init(0);
        poles_array = IsoPoles.AsSpan();

        for (ii = 1, kk = 1; ii <= ULength; ii++, kk += 3)
        {

            for (jj = 1, ll = 1; jj <= VLength; jj++, ll += 3)
            {
                IsoPoles[ii, ll] = Points[jj, kk];
                IsoPoles[ii, ll + 1] = Points[jj, kk + 1];
                IsoPoles[ii, ll + 2] = Points[jj, kk + 2];
            }
        }
        // interpolation of iso v
        BSplCLib.Interpolate(UDegree,
                             UFlatKnots,
                             UParameters,
                             ContactOrder,
                             dimension,
                             poles_array,
                             out InversionProblem);

        // return results

        for (ii = 1; ii <= ULength; ii++)
        {

            for (jj = 1, ll = 1; jj <= VLength; jj++, ll += 3)
            {
                gp_Pnt Pnt = new(IsoPoles[ii, ll], IsoPoles[ii, ll + 1], IsoPoles[ii, ll + 2]);
                Poles.SetValue(ii, jj, Pnt);
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
    public static void FunctionMultiply(BSplSLib_EvaluatorFunction Function,
                                        int UBSplineDegree,
                                        int VBSplineDegree,
                                        NCollection_Array1<double> UBSplineKnots,
                                        NCollection_Array1<double> VBSplineKnots,
                                        NCollection_Array1<int>? UMults,
                                        NCollection_Array1<int>? VMults,
                                        NCollection_Array2<gp_Pnt> Poles,
                                        NCollection_Array2<double>? Weights,
                                        NCollection_Array1<double> UFlatKnots,
                                        NCollection_Array1<double> VFlatKnots,
                                        int UNewDegree,
                                        int VNewDegree,
                                        NCollection_Array2<gp_Pnt> NewNumerator,
                                        NCollection_Array2<double> NewDenominator,
                                        out int theStatus)
    {
        int num_uparameters,
          //  ii,jj,kk,
          ii, jj, error_code, num_vparameters;
        double result;

        num_uparameters = UFlatKnots.Length() - UNewDegree - 1;
        num_vparameters = VFlatKnots.Length() - VNewDegree - 1;
        NCollection_Array1<double> UParameters = new(1, num_uparameters);
        NCollection_Array1<double> VParameters = new(1, num_vparameters);

        if ((NewNumerator.ColLength() == num_uparameters) && (NewNumerator.RowLength() == num_vparameters)
            && (NewDenominator.ColLength() == num_uparameters)
            && (NewDenominator.RowLength() == num_vparameters))
        {

            BSplCLib.BuildSchoenbergPoints(UNewDegree, UFlatKnots, UParameters);

            BSplCLib.BuildSchoenbergPoints(VNewDegree, VFlatKnots, VParameters);

            for (ii = 1; ii <= num_uparameters; ii++)
            {

                for (jj = 1; jj <= num_vparameters; jj++)
                {
                    // OCCT: URat and VRat are passed true whatever Weights is, so a null Weights fails in
                    // PrepareEval (a null dereference in OCCT, a NullReferenceException here)
                    HomogeneousD0(UParameters[ii],
                                  VParameters[jj],
                                  0,
                                  0,
                                  Poles,
                                  Weights,
                                  UBSplineKnots,
                                  VBSplineKnots,
                                  UMults,
                                  VMults,
                                  UBSplineDegree,
                                  VBSplineDegree,
                                  true,
                                  true,
                                  false,
                                  false,
                                  out NewDenominator[ii, jj],
                                  out NewNumerator[ii, jj]);

                    Function.Evaluate(0, UParameters[ii], VParameters[jj], out result, out error_code);
                    if (error_code != 0)
                    {
                        throw new Standard_ConstructionError();
                    }
                    ref gp_Pnt P = ref NewNumerator[ii, jj];
                    P.SetX(P.X() * result);
                    P.SetY(P.Y() * result);
                    P.SetZ(P.Z() * result);
                    NewDenominator[ii, jj] *= result;
                }
            }
            Interpolate(UNewDegree,
                        VNewDegree,
                        UFlatKnots,
                        VFlatKnots,
                        UParameters,
                        VParameters,
                        NewNumerator,
                        NewDenominator,
                        out theStatus);
        }
        else
        {
            throw new Standard_ConstructionError();
        }
    }

    //=================================================================================================

    /// <summary>
    /// An array [1, theNbUPoles] x [1, theNbVPoles] of weights equal to 1. Up to
    /// <see cref="BSplCLib.MaxUnitWeightsSize"/> elements it is a non-owning view of a shared static array (no
    /// allocation), as in OCCT: shared, don't modify. Above that it is a new array.
    /// </summary>
    public static NCollection_Array2<double> UnitWeights(int theNbUPoles, int theNbVPoles)
    {
        int aTotal = theNbUPoles * theNbVPoles;
        if (aTotal <= BSplCLib.MaxUnitWeightsSize())
        {
            // Non-owning view over global static array (zero allocation).
            // Note: the port allocates the two small view objects (the 1D view and the 2D view over it)
            return new NCollection_Array2<double>(BSplCLib.UnitWeights(aTotal),
                                                  1,
                                                  1,
                                                  theNbUPoles,
                                                  1,
                                                  theNbVPoles);
        }
        // Rare: allocate fresh array.
        NCollection_Array2<double> aResult = new(1, theNbUPoles, 1, theNbVPoles);
        aResult.Init(1.0);
        return aResult;
    }
}
