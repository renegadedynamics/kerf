// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BSplCLib_CurveComputation;

namespace Occt;

// BSpline Curve in 3d space
// ***************************

// Explicit template instantiations for 3D types (BSplCLib_CurveComputation.cs)

public static partial class BSplCLib
{
    /// <summary>
    /// Reverses the array of poles. <paramref name="L"/> is the index of the new first pole: Poles.Upper() on a
    /// non-periodic curve, (number of flat knots - degree - 1) on a periodic one.
    /// </summary>
    public static void Reverse(NCollection_Array1<gp_Pnt> Poles, int L)
    {
        BSplCLib_Reverse(Poles, L);
    }

    /// <summary>
    /// Decrements the multiplicity of Knots(Index) to <paramref name="Mult"/> (removes the knot if Mult is 0).
    /// Returns false, leaving the new arrays unchanged, if a new pole would move by more than
    /// <paramref name="Tolerance"/>.
    /// </summary>
    public static bool RemoveKnot(int Index,
                                  int Mult,
                                  int Degree,
                                  bool Periodic,
                                  NCollection_Array1<gp_Pnt> Poles,
                                  NCollection_Array1<double>? Weights,
                                  NCollection_Array1<double> Knots,
                                  NCollection_Array1<int> Mults,
                                  NCollection_Array1<gp_Pnt> NewPoles,
                                  NCollection_Array1<double>? NewWeights,
                                  NCollection_Array1<double> NewKnots,
                                  NCollection_Array1<int> NewMults,
                                  double Tolerance)
    {
        return BSplCLib_RemoveKnot<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Index,
                                                                           Mult,
                                                                           Degree,
                                                                           Periodic,
                                                                           Poles,
                                                                           Weights,
                                                                           Knots,
                                                                           Mults,
                                                                           NewPoles,
                                                                           NewWeights,
                                                                           NewKnots,
                                                                           NewMults,
                                                                           Tolerance);
    }

    /// <summary>
    /// Inserts the knots <paramref name="AddKnots"/> with multiplicities <paramref name="AddMults"/> (null: 1 each)
    /// into the curve. Epsilon compares knots; with <paramref name="Add"/> the multiplicities of equal knots are
    /// added, otherwise the larger is kept. Size the new arrays with PrepareInsertKnots.
    /// </summary>
    public static void InsertKnots(int Degree,
                                   bool Periodic,
                                   NCollection_Array1<gp_Pnt> Poles,
                                   NCollection_Array1<double>? Weights,
                                   NCollection_Array1<double> Knots,
                                   NCollection_Array1<int> Mults,
                                   NCollection_Array1<double> AddKnots,
                                   NCollection_Array1<int>? AddMults,
                                   NCollection_Array1<gp_Pnt> NewPoles,
                                   NCollection_Array1<double>? NewWeights,
                                   NCollection_Array1<double> NewKnots,
                                   NCollection_Array1<int> NewMults,
                                   double Epsilon,
                                   bool Add = true)
    {
        BSplCLib_InsertKnots<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Degree,
                                                                     Periodic,
                                                                     Poles,
                                                                     Weights,
                                                                     Knots,
                                                                     Mults,
                                                                     AddKnots,
                                                                     AddMults,
                                                                     NewPoles,
                                                                     NewWeights,
                                                                     NewKnots,
                                                                     NewMults,
                                                                     Epsilon,
                                                                     Add);
    }

    /// <summary>
    /// Inserts the knot <paramref name="U"/> with multiplicity <paramref name="UMult"/> and returns the new poles;
    /// Knots and Mults are not updated. <paramref name="UIndex"/> is unused.
    /// </summary>
    public static void InsertKnot(int UIndex,
                                  double U,
                                  int UMult,
                                  int Degree,
                                  bool Periodic,
                                  NCollection_Array1<gp_Pnt> Poles,
                                  NCollection_Array1<double>? Weights,
                                  NCollection_Array1<double> Knots,
                                  NCollection_Array1<int> Mults,
                                  NCollection_Array1<gp_Pnt> NewPoles,
                                  NCollection_Array1<double>? NewWeights)
    {
        _ = UIndex; // unnamed (unused) in OCCT
        BSplCLib_InsertKnot<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(0,
                                                                    U,
                                                                    UMult,
                                                                    Degree,
                                                                    Periodic,
                                                                    Poles,
                                                                    Weights,
                                                                    Knots,
                                                                    Mults,
                                                                    NewPoles,
                                                                    NewWeights);
    }

    /// <summary>
    /// Raises the multiplicity of Knots(KnotIndex) to <paramref name="Mult"/> and returns the new poles; Knots and
    /// Mults are not updated.
    /// </summary>
    public static void RaiseMultiplicity(int KnotIndex,
                                         int Mult,
                                         int Degree,
                                         bool Periodic,
                                         NCollection_Array1<gp_Pnt> Poles,
                                         NCollection_Array1<double>? Weights,
                                         NCollection_Array1<double> Knots,
                                         NCollection_Array1<int> Mults,
                                         NCollection_Array1<gp_Pnt> NewPoles,
                                         NCollection_Array1<double>? NewWeights)
    {
        BSplCLib_RaiseMultiplicity<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(KnotIndex,
                                                                           Mult,
                                                                           Degree,
                                                                           Periodic,
                                                                           Poles,
                                                                           Weights,
                                                                           Knots,
                                                                           Mults,
                                                                           NewPoles,
                                                                           NewWeights);
    }

    /// <summary>
    /// Raises the degree of the curve from <paramref name="Degree"/> to <paramref name="NewDegree"/>. Size the new
    /// knot arrays with IncreaseDegreeCountKnots.
    /// </summary>
    public static void IncreaseDegree(int Degree,
                                      int NewDegree,
                                      bool Periodic,
                                      NCollection_Array1<gp_Pnt> Poles,
                                      NCollection_Array1<double>? Weights,
                                      NCollection_Array1<double> Knots,
                                      NCollection_Array1<int> Mults,
                                      NCollection_Array1<gp_Pnt> NewPoles,
                                      NCollection_Array1<double>? NewWeights,
                                      NCollection_Array1<double> NewKnots,
                                      NCollection_Array1<int> NewMults)
    {
        BSplCLib_IncreaseDegree<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Degree,
                                                                        NewDegree,
                                                                        Periodic,
                                                                        Poles,
                                                                        Weights,
                                                                        Knots,
                                                                        Mults,
                                                                        NewPoles,
                                                                        NewWeights,
                                                                        NewKnots,
                                                                        NewMults);
    }

    /// <summary>Builds the non-periodic curve identical to a periodic one. Size the arrays with PrepareUnperiodize.</summary>
    public static void Unperiodize(int Degree,
                                   NCollection_Array1<int> Mults,
                                   NCollection_Array1<double> Knots,
                                   NCollection_Array1<gp_Pnt> Poles,
                                   NCollection_Array1<double>? Weights,
                                   NCollection_Array1<int> NewMults,
                                   NCollection_Array1<double> NewKnots,
                                   NCollection_Array1<gp_Pnt> NewPoles,
                                   NCollection_Array1<double>? NewWeights)
    {
        BSplCLib_Unperiodize<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Degree,
                                                                     Mults,
                                                                     Knots,
                                                                     Poles,
                                                                     Weights,
                                                                     NewMults,
                                                                     NewKnots,
                                                                     NewPoles,
                                                                     NewWeights);
    }

    /// <summary>Trims the curve to [U1, U2]. Size the arrays with PrepareTrimming.</summary>
    public static void Trimming(int Degree,
                                bool Periodic,
                                NCollection_Array1<double> Knots,
                                NCollection_Array1<int> Mults,
                                NCollection_Array1<gp_Pnt> Poles,
                                NCollection_Array1<double>? Weights,
                                double U1,
                                double U2,
                                NCollection_Array1<double> NewKnots,
                                NCollection_Array1<int> NewMults,
                                NCollection_Array1<gp_Pnt> NewPoles,
                                NCollection_Array1<double>? NewWeights)
    {
        BSplCLib_Trimming<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Degree,
                                                                  Periodic,
                                                                  Knots,
                                                                  Mults,
                                                                  Poles,
                                                                  Weights,
                                                                  U1,
                                                                  U2,
                                                                  NewKnots,
                                                                  NewMults,
                                                                  NewPoles,
                                                                  NewWeights);
    }

    /// <summary>
    /// Copies into <paramref name="LP"/> the Degree + 1 poles (and weights) for the Eval scheme, starting from
    /// Poles(Poles.Lower() + Index).
    /// </summary>
    public static void BuildEval(int Degree,
                                 int Index,
                                 NCollection_Array1<gp_Pnt> Poles,
                                 NCollection_Array1<double>? Weights,
                                 Span<double> LP)
    {
        BSplCLib_BuildEval<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Degree,
                                                                   Index,
                                                                   Poles,
                                                                   Weights,
                                                                   LP);
    }

    /// <summary>
    /// The point at <paramref name="U"/>. <paramref name="Index"/> locates U in the knots (searched when out of
    /// range); pass <c>NoWeights()</c> for a non-rational curve and <c>NoMults()</c> for flat knots.
    /// </summary>
    public static void D0(double U,
                          int Index,
                          int Degree,
                          bool Periodic,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults,
                          out gp_Pnt P)
    {
        BSplCLib_D0<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(U,
                                                            Index,
                                                            Degree,
                                                            Periodic,
                                                            Poles,
                                                            Weights,
                                                            Knots,
                                                            Mults,
                                                            out P);
    }

    /// <summary>The point and first derivative at <paramref name="U"/> (see D0).</summary>
    public static void D1(double U,
                          int Index,
                          int Degree,
                          bool Periodic,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults,
                          out gp_Pnt P,
                          out gp_Vec V)
    {
        BSplCLib_D1<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(U,
                                                            Index,
                                                            Degree,
                                                            Periodic,
                                                            Poles,
                                                            Weights,
                                                            Knots,
                                                            Mults,
                                                            out P,
                                                            out V);
    }

    /// <summary>The point and first two derivatives at <paramref name="U"/> (see D0).</summary>
    public static void D2(double U,
                          int Index,
                          int Degree,
                          bool Periodic,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults,
                          out gp_Pnt P,
                          out gp_Vec V1,
                          out gp_Vec V2)
    {
        BSplCLib_D2<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(U,
                                                            Index,
                                                            Degree,
                                                            Periodic,
                                                            Poles,
                                                            Weights,
                                                            Knots,
                                                            Mults,
                                                            out P,
                                                            out V1,
                                                            out V2);
    }

    /// <summary>The point and first three derivatives at <paramref name="U"/> (see D0).</summary>
    public static void D3(double U,
                          int Index,
                          int Degree,
                          bool Periodic,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults,
                          out gp_Pnt P,
                          out gp_Vec V1,
                          out gp_Vec V2,
                          out gp_Vec V3)
    {
        BSplCLib_D3<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(U,
                                                            Index,
                                                            Degree,
                                                            Periodic,
                                                            Poles,
                                                            Weights,
                                                            Knots,
                                                            Mults,
                                                            out P,
                                                            out V1,
                                                            out V2,
                                                            out V3);
    }

    /// <summary>The derivative of order <paramref name="N"/> at <paramref name="U"/> (see D0).</summary>
    public static void DN(double U,
                          int N,
                          int Index,
                          int Degree,
                          bool Periodic,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          NCollection_Array1<double> Knots,
                          NCollection_Array1<int>? Mults,
                          out gp_Vec VN)
    {
        BSplCLib_DN<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(U,
                                                            N,
                                                            Index,
                                                            Degree,
                                                            Periodic,
                                                            Poles,
                                                            Weights,
                                                            Knots,
                                                            Mults,
                                                            out VN);
    }

    /// <summary>
    /// Solves Matrix . X = B with the LU-factored banded <paramref name="Matrix"/>, B and the result being
    /// <paramref name="PolesArray"/>. Returns 0 on success.
    /// </summary>
    public static int SolveBandedSystem(math_Matrix Matrix,
                                        int UpperBandWidth,
                                        int LowerBandWidth,
                                        NCollection_Array1<gp_Pnt> PolesArray)
    {
        return BSplCLib_SolveBandedSystem<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Matrix,
                                                                                  UpperBandWidth,
                                                                                  LowerBandWidth,
                                                                                  PolesArray);
    }

    /// <summary>
    /// Solves Matrix . X = B for rational poles. If <paramref name="HomogeneousFlag"/> is false the poles are
    /// multiplied by the weights on entry and divided by the interpolated weights at the end; if true, poles and
    /// weights are interpolated as they are.
    /// </summary>
    public static int SolveBandedSystem(math_Matrix Matrix,
                                        int UpperBandWidth,
                                        int LowerBandWidth,
                                        bool HomogeneousFlag,
                                        NCollection_Array1<gp_Pnt> PolesArray,
                                        NCollection_Array1<double> WeightsArray)
    {
        return BSplCLib_SolveBandedSystem<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Matrix,
                                                                                  UpperBandWidth,
                                                                                  LowerBandWidth,
                                                                                  HomogeneousFlag,
                                                                                  PolesArray,
                                                                                  WeightsArray);
    }

    /// <summary>
    /// The point and weight of a rational curve given by flat knots at <paramref name="Parameter"/>; with
    /// <paramref name="HomogeneousFlag"/> the point is left in homogeneous coordinates (not divided by the weight).
    /// </summary>
    public static void Eval(double Parameter,
                            bool PeriodicFlag,
                            bool HomogeneousFlag,
                            ref int ExtrapMode,
                            int Degree,
                            NCollection_Array1<double> FlatKnots,
                            NCollection_Array1<gp_Pnt> PolesArray,
                            NCollection_Array1<double> WeightsArray,
                            out gp_Pnt aPoint,
                            out double aWeight)
    {
        BSplCLib_Eval<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Parameter,
                                                              PeriodicFlag,
                                                              HomogeneousFlag,
                                                              ref ExtrapMode,
                                                              Degree,
                                                              FlatKnots,
                                                              PolesArray,
                                                              WeightsArray,
                                                              out aPoint,
                                                              out aWeight);
    }

    /// <summary>
    /// The point at <paramref name="Parameter"/> on a span stored as a polynomial cache (see BuildCache) starting
    /// at <paramref name="CacheParameter"/> with length <paramref name="SpanLenght"/>.
    /// </summary>
    public static void CacheD0(double Parameter,
                               int Degree,
                               double CacheParameter,
                               double SpanLenght,
                               NCollection_Array1<gp_Pnt> PolesArray,
                               NCollection_Array1<double>? WeightsArray,
                               out gp_Pnt aPoint)
    {
        BSplCLib_CacheD0<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Parameter,
                                                                 Degree,
                                                                 CacheParameter,
                                                                 SpanLenght,
                                                                 PolesArray,
                                                                 WeightsArray,
                                                                 out aPoint);
    }

    /// <summary>The point and first derivative from a polynomial cache (see CacheD0).</summary>
    public static void CacheD1(double Parameter,
                               int Degree,
                               double CacheParameter,
                               double SpanLenght,
                               NCollection_Array1<gp_Pnt> PolesArray,
                               NCollection_Array1<double>? WeightsArray,
                               out gp_Pnt aPoint,
                               out gp_Vec aVector)
    {
        BSplCLib_CacheD1<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Parameter,
                                                                 Degree,
                                                                 CacheParameter,
                                                                 SpanLenght,
                                                                 PolesArray,
                                                                 WeightsArray,
                                                                 out aPoint,
                                                                 out aVector);
    }

    /// <summary>The point and first two derivatives from a polynomial cache (see CacheD0).</summary>
    public static void CacheD2(double Parameter,
                               int Degree,
                               double CacheParameter,
                               double SpanLenght,
                               NCollection_Array1<gp_Pnt> PolesArray,
                               NCollection_Array1<double>? WeightsArray,
                               out gp_Pnt aPoint,
                               out gp_Vec aVector1,
                               out gp_Vec aVector2)
    {
        BSplCLib_CacheD2<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Parameter,
                                                                 Degree,
                                                                 CacheParameter,
                                                                 SpanLenght,
                                                                 PolesArray,
                                                                 WeightsArray,
                                                                 out aPoint,
                                                                 out aVector1,
                                                                 out aVector2);
    }

    /// <summary>The point and first three derivatives from a polynomial cache (see CacheD0).</summary>
    public static void CacheD3(double Parameter,
                               int Degree,
                               double CacheParameter,
                               double SpanLenght,
                               NCollection_Array1<gp_Pnt> PolesArray,
                               NCollection_Array1<double>? WeightsArray,
                               out gp_Pnt aPoint,
                               out gp_Vec aVector1,
                               out gp_Vec aVector2,
                               out gp_Vec aVector3)
    {
        BSplCLib_CacheD3<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Parameter,
                                                                 Degree,
                                                                 CacheParameter,
                                                                 SpanLenght,
                                                                 PolesArray,
                                                                 WeightsArray,
                                                                 out aPoint,
                                                                 out aVector1,
                                                                 out aVector2,
                                                                 out aVector3);
    }

    /// <summary>
    /// Stores in <paramref name="CachePoles"/> (and <paramref name="CacheWeights"/>) the polynomial coefficients of
    /// the span containing <paramref name="U"/>, scaled to the span domain (1 / <paramref name="InverseOfSpanDomain"/>
    /// in OCCT's naming; the value is used as the span length). Arrays are indexed from 1.
    /// </summary>
    public static void BuildCache(double U,
                                  double InverseOfSpanDomain,
                                  bool PeriodicFlag,
                                  int Degree,
                                  NCollection_Array1<double> FlatKnots,
                                  NCollection_Array1<gp_Pnt> Poles,
                                  NCollection_Array1<double>? Weights,
                                  NCollection_Array1<gp_Pnt> CachePoles,
                                  NCollection_Array1<double>? CacheWeights)
    {
        BSplCLib_BuildCache<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(U,
                                                                    InverseOfSpanDomain,
                                                                    PeriodicFlag,
                                                                    Degree,
                                                                    FlatKnots,
                                                                    Poles,
                                                                    Weights,
                                                                    CachePoles,
                                                                    CacheWeights);
    }

    /// <summary>
    /// Stores the polynomial coefficients of the span <paramref name="theSpanIndex"/> into
    /// <paramref name="theCacheArray"/>, one row per coefficient: the coordinates, then the weight for a rational
    /// curve (0 except the first, 1, when the weights are all equal).
    /// </summary>
    public static void BuildCache(double theParameter,
                                  double theSpanDomain,
                                  bool thePeriodicFlag,
                                  int theDegree,
                                  int theSpanIndex,
                                  NCollection_Array1<double> theFlatKnots,
                                  NCollection_Array1<gp_Pnt> thePoles,
                                  NCollection_Array1<double>? theWeights,
                                  NCollection_Array2<double> theCacheArray)
    {
        BSplCLib_BuildCache<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(theParameter,
                                                                    theSpanDomain,
                                                                    thePeriodicFlag,
                                                                    theDegree,
                                                                    theSpanIndex,
                                                                    theFlatKnots,
                                                                    thePoles,
                                                                    theWeights,
                                                                    theCacheArray);
    }

    /// <summary>
    /// Interpolates the points (in <paramref name="Poles"/> on entry, the poles on return) at
    /// <paramref name="Parameters"/>, with the contact orders <paramref name="ContactOrderArray"/> (0: the point,
    /// 1: the first derivative, ...). <paramref name="InversionProblem"/> is 0 on success.
    /// </summary>
    public static void Interpolate(int Degree,
                                   NCollection_Array1<double> FlatKnots,
                                   NCollection_Array1<double> Parameters,
                                   NCollection_Array1<int> ContactOrderArray,
                                   NCollection_Array1<gp_Pnt> Poles,
                                   out int InversionProblem)
    {
        BSplCLib_Interpolate<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Degree,
                                                                     FlatKnots,
                                                                     Parameters,
                                                                     ContactOrderArray,
                                                                     Poles,
                                                                     out InversionProblem);
    }

    /// <summary>The rational form of Interpolate: the weights are interpolated too.</summary>
    public static void Interpolate(int Degree,
                                   NCollection_Array1<double> FlatKnots,
                                   NCollection_Array1<double> Parameters,
                                   NCollection_Array1<int> ContactOrderArray,
                                   NCollection_Array1<gp_Pnt> Poles,
                                   NCollection_Array1<double> Weights,
                                   out int InversionProblem)
    {
        BSplCLib_Interpolate<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Degree,
                                                                     FlatKnots,
                                                                     Parameters,
                                                                     ContactOrderArray,
                                                                     Poles,
                                                                     Weights,
                                                                     out InversionProblem);
    }

    /// <summary>
    /// Moves the point at <paramref name="U"/> by <paramref name="Displ"/>, changing only the poles between
    /// <paramref name="Index1"/> and <paramref name="Index2"/>; the poles changed are FirstIndex to LastIndex
    /// (both 0 when nothing could be done).
    /// </summary>
    public static void MovePoint(double U,
                                 in gp_Vec Displ,
                                 int Index1,
                                 int Index2,
                                 int Degree,
                                 NCollection_Array1<gp_Pnt> Poles,
                                 NCollection_Array1<double>? Weights,
                                 NCollection_Array1<double> FlatKnots,
                                 out int FirstIndex,
                                 out int LastIndex,
                                 NCollection_Array1<gp_Pnt> NewPoles)
    {
        BSplCLib_MovePoint<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(U,
                                                                   Displ,
                                                                   Index1,
                                                                   Index2,
                                                                   Degree,
                                                                   Poles,
                                                                   Weights,
                                                                   FlatKnots,
                                                                   out FirstIndex,
                                                                   out LastIndex,
                                                                   NewPoles);
    }

    /// <summary>
    /// Moves the point at <paramref name="U"/> by <paramref name="Delta"/> and its derivative by
    /// <paramref name="DeltaDerivative"/>, keeping the start and end of the curve to the given orders
    /// (-1: free, 0: the point, 1: the tangent, ...). <paramref name="ErrorStatus"/> is 0 on success.
    /// </summary>
    public static void MovePointAndTangent(double U,
                                           in gp_Vec Delta,
                                           in gp_Vec DeltaDerivative,
                                           double Tolerance,
                                           int Degree,
                                           int StartingCondition,
                                           int EndingCondition,
                                           NCollection_Array1<gp_Pnt> Poles,
                                           NCollection_Array1<double>? Weights,
                                           NCollection_Array1<double> FlatKnots,
                                           NCollection_Array1<gp_Pnt> NewPoles,
                                           out int ErrorStatus)
    {
        BSplCLib_MovePointAndTangent<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(U,
                                                                             Delta,
                                                                             DeltaDerivative,
                                                                             Tolerance,
                                                                             Degree,
                                                                             StartingCondition,
                                                                             EndingCondition,
                                                                             Poles,
                                                                             Weights,
                                                                             FlatKnots,
                                                                             NewPoles,
                                                                             out ErrorStatus);
    }

    /// <summary>
    /// The parametric tolerance <paramref name="UTolerance"/> such that |t1 - t0| &lt; UTolerance implies
    /// |C(t1) - C(t0)| &lt; <paramref name="Tolerance3D"/>.
    /// </summary>
    public static void Resolution(NCollection_Array1<gp_Pnt> Poles,
                                  NCollection_Array1<double>? Weights,
                                  int NumPoles,
                                  NCollection_Array1<double> FlatKnots,
                                  int Degree,
                                  double Tolerance3D,
                                  out double UTolerance)
    {
        BSplCLib_Resolution<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Poles,
                                                                    Weights,
                                                                    NumPoles,
                                                                    FlatKnots,
                                                                    Degree,
                                                                    Tolerance3D,
                                                                    out UTolerance);
    }

    /// <summary>
    /// Multiplies the B-spline F(t) by the function a(t) (<paramref name="FunctionPtr"/>), assuming F(t)a(t) is
    /// exactly a B-spline of degree <paramref name="NewDegree"/> on <paramref name="FlatKnots"/> (not checked).
    /// <paramref name="theStatus"/> is 0, or the pivot index of the failed matrix inversion.
    /// </summary>
    public static void FunctionMultiply(BSplCLib_EvaluatorFunction FunctionPtr,
                                        int BSplineDegree,
                                        NCollection_Array1<double> BSplineFlatKnots,
                                        NCollection_Array1<gp_Pnt> Poles,
                                        NCollection_Array1<double> FlatKnots,
                                        int NewDegree,
                                        NCollection_Array1<gp_Pnt> NewPoles,
                                        out int theStatus)
    {
        BSplCLib_FunctionMultiply<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(FunctionPtr,
                                                                          BSplineDegree,
                                                                          BSplineFlatKnots,
                                                                          Poles,
                                                                          FlatKnots,
                                                                          NewDegree,
                                                                          NewPoles,
                                                                          out theStatus);
    }

    /// <summary>
    /// Composes the B-spline F(t) with the function a(t) (<paramref name="FunctionPtr"/>), assuming F(a(t)) is
    /// exactly a B-spline of degree <paramref name="NewDegree"/> on <paramref name="FlatKnots"/> and a(t) maps the
    /// range of FlatKnots onto that of BSplineFlatKnots (not checked). <paramref name="theStatus"/> is 0, or the
    /// pivot index of the failed matrix inversion.
    /// </summary>
    public static void FunctionReparameterise(BSplCLib_EvaluatorFunction FunctionPtr,
                                              int BSplineDegree,
                                              NCollection_Array1<double> BSplineFlatKnots,
                                              NCollection_Array1<gp_Pnt> Poles,
                                              NCollection_Array1<double> FlatKnots,
                                              int NewDegree,
                                              NCollection_Array1<gp_Pnt> NewPoles,
                                              out int theStatus)
    {
        BSplCLib_FunctionReparameterise<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(FunctionPtr,
                                                                                BSplineDegree,
                                                                                BSplineFlatKnots,
                                                                                Poles,
                                                                                FlatKnots,
                                                                                NewDegree,
                                                                                NewPoles,
                                                                                out theStatus);
    }
}
