// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Splines.CurveComputation;

namespace Kerf.Engine.Geometry.Splines;

// BSpline Curve in 3d space
// ***************************

// Explicit template instantiations for 3D types (BSplCLib_CurveComputation.cs)

internal static partial class BSplCLib
{
    /// <summary>
    /// Reverses the array of poles. <paramref name="l"/> is the index of the new first pole: Poles.Upper() on a
    /// non-periodic curve, (number of flat knots - degree - 1) on a periodic one.
    /// </summary>
    public static void Reverse(NCollection_Array1<Point3D> poles, int l)
    {
        BSplCLib_Reverse(poles, l);
    }

    /// <summary>
    /// Decrements the multiplicity of Knots(Index) to <paramref name="mult"/> (removes the knot if Mult is 0).
    /// Returns false, leaving the new arrays unchanged, if a new pole would move by more than
    /// <paramref name="tolerance"/>.
    /// </summary>
    public static bool RemoveKnot(int index,
                                  int mult,
                                  int degree,
                                  bool periodic,
                                  NCollection_Array1<Point3D> poles,
                                  NCollection_Array1<double>? weights,
                                  NCollection_Array1<double> knots,
                                  NCollection_Array1<int> mults,
                                  NCollection_Array1<Point3D> newPoles,
                                  NCollection_Array1<double>? newWeights,
                                  NCollection_Array1<double> newKnots,
                                  NCollection_Array1<int> newMults,
                                  double tolerance)
    {
        return BSplCLib_RemoveKnot<Point3D, Vector3D, CurveTraits3d>(index,
                                                                           mult,
                                                                           degree,
                                                                           periodic,
                                                                           poles,
                                                                           weights,
                                                                           knots,
                                                                           mults,
                                                                           newPoles,
                                                                           newWeights,
                                                                           newKnots,
                                                                           newMults,
                                                                           tolerance);
    }

    /// <summary>
    /// Inserts the knots <paramref name="addKnots"/> with multiplicities <paramref name="addMults"/> (null: 1 each)
    /// into the curve. Epsilon compares knots; with <paramref name="add"/> the multiplicities of equal knots are
    /// added, otherwise the larger is kept. Size the new arrays with PrepareInsertKnots.
    /// </summary>
    public static void InsertKnots(int degree,
                                   bool periodic,
                                   NCollection_Array1<Point3D> poles,
                                   NCollection_Array1<double>? weights,
                                   NCollection_Array1<double> knots,
                                   NCollection_Array1<int> mults,
                                   NCollection_Array1<double> addKnots,
                                   NCollection_Array1<int>? addMults,
                                   NCollection_Array1<Point3D> newPoles,
                                   NCollection_Array1<double>? newWeights,
                                   NCollection_Array1<double> newKnots,
                                   NCollection_Array1<int> newMults,
                                   double epsilon,
                                   bool add = true)
    {
        BSplCLib_InsertKnots<Point3D, Vector3D, CurveTraits3d>(degree,
                                                                     periodic,
                                                                     poles,
                                                                     weights,
                                                                     knots,
                                                                     mults,
                                                                     addKnots,
                                                                     addMults,
                                                                     newPoles,
                                                                     newWeights,
                                                                     newKnots,
                                                                     newMults,
                                                                     epsilon,
                                                                     add);
    }

    /// <summary>
    /// Inserts the knot <paramref name="u"/> with multiplicity <paramref name="uMult"/> and returns the new poles;
    /// Knots and Mults are not updated. <paramref name="uIndex"/> is unused.
    /// </summary>
    public static void InsertKnot(int uIndex,
                                  double u,
                                  int uMult,
                                  int degree,
                                  bool periodic,
                                  NCollection_Array1<Point3D> poles,
                                  NCollection_Array1<double>? weights,
                                  NCollection_Array1<double> knots,
                                  NCollection_Array1<int> mults,
                                  NCollection_Array1<Point3D> newPoles,
                                  NCollection_Array1<double>? newWeights)
    {
        _ = uIndex; // unnamed (unused) in OCCT
        BSplCLib_InsertKnot<Point3D, Vector3D, CurveTraits3d>(0,
                                                                    u,
                                                                    uMult,
                                                                    degree,
                                                                    periodic,
                                                                    poles,
                                                                    weights,
                                                                    knots,
                                                                    mults,
                                                                    newPoles,
                                                                    newWeights);
    }

    /// <summary>
    /// Raises the multiplicity of Knots(KnotIndex) to <paramref name="mult"/> and returns the new poles; Knots and
    /// Mults are not updated.
    /// </summary>
    public static void RaiseMultiplicity(int knotIndex,
                                         int mult,
                                         int degree,
                                         bool periodic,
                                         NCollection_Array1<Point3D> poles,
                                         NCollection_Array1<double>? weights,
                                         NCollection_Array1<double> knots,
                                         NCollection_Array1<int> mults,
                                         NCollection_Array1<Point3D> newPoles,
                                         NCollection_Array1<double>? newWeights)
    {
        BSplCLib_RaiseMultiplicity<Point3D, Vector3D, CurveTraits3d>(knotIndex,
                                                                           mult,
                                                                           degree,
                                                                           periodic,
                                                                           poles,
                                                                           weights,
                                                                           knots,
                                                                           mults,
                                                                           newPoles,
                                                                           newWeights);
    }

    /// <summary>
    /// Raises the degree of the curve from <paramref name="degree"/> to <paramref name="newDegree"/>. Size the new
    /// knot arrays with IncreaseDegreeCountKnots.
    /// </summary>
    public static void IncreaseDegree(int degree,
                                      int newDegree,
                                      bool periodic,
                                      NCollection_Array1<Point3D> poles,
                                      NCollection_Array1<double>? weights,
                                      NCollection_Array1<double> knots,
                                      NCollection_Array1<int> mults,
                                      NCollection_Array1<Point3D> newPoles,
                                      NCollection_Array1<double>? newWeights,
                                      NCollection_Array1<double> newKnots,
                                      NCollection_Array1<int> newMults)
    {
        BSplCLib_IncreaseDegree<Point3D, Vector3D, CurveTraits3d>(degree,
                                                                        newDegree,
                                                                        periodic,
                                                                        poles,
                                                                        weights,
                                                                        knots,
                                                                        mults,
                                                                        newPoles,
                                                                        newWeights,
                                                                        newKnots,
                                                                        newMults);
    }

    /// <summary>Builds the non-periodic curve identical to a periodic one. Size the arrays with PrepareUnperiodize.</summary>
    public static void Unperiodize(int degree,
                                   NCollection_Array1<int> mults,
                                   NCollection_Array1<double> knots,
                                   NCollection_Array1<Point3D> poles,
                                   NCollection_Array1<double>? weights,
                                   NCollection_Array1<int> newMults,
                                   NCollection_Array1<double> newKnots,
                                   NCollection_Array1<Point3D> newPoles,
                                   NCollection_Array1<double>? newWeights)
    {
        BSplCLib_Unperiodize<Point3D, Vector3D, CurveTraits3d>(degree,
                                                                     mults,
                                                                     knots,
                                                                     poles,
                                                                     weights,
                                                                     newMults,
                                                                     newKnots,
                                                                     newPoles,
                                                                     newWeights);
    }

    /// <summary>Trims the curve to [U1, U2]. Size the arrays with PrepareTrimming.</summary>
    public static void Trimming(int degree,
                                bool periodic,
                                NCollection_Array1<double> knots,
                                NCollection_Array1<int> mults,
                                NCollection_Array1<Point3D> poles,
                                NCollection_Array1<double>? weights,
                                double u1,
                                double u2,
                                NCollection_Array1<double> newKnots,
                                NCollection_Array1<int> newMults,
                                NCollection_Array1<Point3D> newPoles,
                                NCollection_Array1<double>? newWeights)
    {
        BSplCLib_Trimming<Point3D, Vector3D, CurveTraits3d>(degree,
                                                                  periodic,
                                                                  knots,
                                                                  mults,
                                                                  poles,
                                                                  weights,
                                                                  u1,
                                                                  u2,
                                                                  newKnots,
                                                                  newMults,
                                                                  newPoles,
                                                                  newWeights);
    }

    /// <summary>
    /// Copies into <paramref name="lp"/> the Degree + 1 poles (and weights) for the Eval scheme, starting from
    /// Poles(Poles.Lower() + Index).
    /// </summary>
    public static void BuildEval(int degree,
                                 int index,
                                 NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<double>? weights,
                                 Span<double> lp)
    {
        BSplCLib_BuildEval<Point3D, Vector3D, CurveTraits3d>(degree,
                                                                   index,
                                                                   poles,
                                                                   weights,
                                                                   lp);
    }

    /// <summary>
    /// The point at <paramref name="u"/>. <paramref name="index"/> locates U in the knots (searched when out of
    /// range); pass <c>NoWeights()</c> for a non-rational curve and <c>NoMults()</c> for flat knots.
    /// </summary>
    public static void D0(double u,
                          int index,
                          int degree,
                          bool periodic,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults,
                          out Point3D p)
    {
        BSplCLib_D0<Point3D, Vector3D, CurveTraits3d>(u,
                                                            index,
                                                            degree,
                                                            periodic,
                                                            poles,
                                                            weights,
                                                            knots,
                                                            mults,
                                                            out p);
    }

    /// <summary>The point and first derivative at <paramref name="u"/> (see D0).</summary>
    public static void D1(double u,
                          int index,
                          int degree,
                          bool periodic,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults,
                          out Point3D p,
                          out Vector3D v)
    {
        BSplCLib_D1<Point3D, Vector3D, CurveTraits3d>(u,
                                                            index,
                                                            degree,
                                                            periodic,
                                                            poles,
                                                            weights,
                                                            knots,
                                                            mults,
                                                            out p,
                                                            out v);
    }

    /// <summary>The point and first two derivatives at <paramref name="u"/> (see D0).</summary>
    public static void D2(double u,
                          int index,
                          int degree,
                          bool periodic,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults,
                          out Point3D p,
                          out Vector3D v1,
                          out Vector3D v2)
    {
        BSplCLib_D2<Point3D, Vector3D, CurveTraits3d>(u,
                                                            index,
                                                            degree,
                                                            periodic,
                                                            poles,
                                                            weights,
                                                            knots,
                                                            mults,
                                                            out p,
                                                            out v1,
                                                            out v2);
    }

    /// <summary>The point and first three derivatives at <paramref name="u"/> (see D0).</summary>
    public static void D3(double u,
                          int index,
                          int degree,
                          bool periodic,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults,
                          out Point3D p,
                          out Vector3D v1,
                          out Vector3D v2,
                          out Vector3D v3)
    {
        BSplCLib_D3<Point3D, Vector3D, CurveTraits3d>(u,
                                                            index,
                                                            degree,
                                                            periodic,
                                                            poles,
                                                            weights,
                                                            knots,
                                                            mults,
                                                            out p,
                                                            out v1,
                                                            out v2,
                                                            out v3);
    }

    /// <summary>The derivative of order <paramref name="n"/> at <paramref name="u"/> (see D0).</summary>
    public static void DN(double u,
                          int n,
                          int index,
                          int degree,
                          bool periodic,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          NCollection_Array1<double> knots,
                          NCollection_Array1<int>? mults,
                          out Vector3D vn)
    {
        BSplCLib_DN<Point3D, Vector3D, CurveTraits3d>(u,
                                                            n,
                                                            index,
                                                            degree,
                                                            periodic,
                                                            poles,
                                                            weights,
                                                            knots,
                                                            mults,
                                                            out vn);
    }

    /// <summary>
    /// Solves Matrix . X = B with the LU-factored banded <paramref name="matrix"/>, B and the result being
    /// <paramref name="polesArray"/>. Returns 0 on success.
    /// </summary>
    public static int SolveBandedSystem(Matrix matrix,
                                        int upperBandWidth,
                                        int lowerBandWidth,
                                        NCollection_Array1<Point3D> polesArray)
    {
        return BSplCLib_SolveBandedSystem<Point3D, Vector3D, CurveTraits3d>(matrix,
                                                                                  upperBandWidth,
                                                                                  lowerBandWidth,
                                                                                  polesArray);
    }

    /// <summary>
    /// Solves Matrix . X = B for rational poles. If <paramref name="homogeneousFlag"/> is false the poles are
    /// multiplied by the weights on entry and divided by the interpolated weights at the end; if true, poles and
    /// weights are interpolated as they are.
    /// </summary>
    public static int SolveBandedSystem(Matrix matrix,
                                        int upperBandWidth,
                                        int lowerBandWidth,
                                        bool homogeneousFlag,
                                        NCollection_Array1<Point3D> polesArray,
                                        NCollection_Array1<double> weightsArray)
    {
        return BSplCLib_SolveBandedSystem<Point3D, Vector3D, CurveTraits3d>(matrix,
                                                                                  upperBandWidth,
                                                                                  lowerBandWidth,
                                                                                  homogeneousFlag,
                                                                                  polesArray,
                                                                                  weightsArray);
    }

    /// <summary>
    /// The point and weight of a rational curve given by flat knots at <paramref name="parameter"/>; with
    /// <paramref name="homogeneousFlag"/> the point is left in homogeneous coordinates (not divided by the weight).
    /// </summary>
    public static void Eval(double parameter,
                            bool periodicFlag,
                            bool homogeneousFlag,
                            ReadOnlySpan<int> extrapMode,
                            int degree,
                            NCollection_Array1<double> flatKnots,
                            NCollection_Array1<Point3D> polesArray,
                            NCollection_Array1<double> weightsArray,
                            out Point3D aPoint,
                            out double aWeight)
    {
        BSplCLib_Eval<Point3D, Vector3D, CurveTraits3d>(parameter,
                                                              periodicFlag,
                                                              homogeneousFlag,
                                                              extrapMode,
                                                              degree,
                                                              flatKnots,
                                                              polesArray,
                                                              weightsArray,
                                                              out aPoint,
                                                              out aWeight);
    }

    /// <summary>
    /// The point at <paramref name="parameter"/> on a span stored as a polynomial cache (see BuildCache) starting
    /// at <paramref name="cacheParameter"/> with length <paramref name="spanLenght"/>.
    /// </summary>
    public static void CacheD0(double parameter,
                               int degree,
                               double cacheParameter,
                               double spanLenght,
                               NCollection_Array1<Point3D> polesArray,
                               NCollection_Array1<double>? weightsArray,
                               out Point3D aPoint)
    {
        BSplCLib_CacheD0<Point3D, Vector3D, CurveTraits3d>(parameter,
                                                                 degree,
                                                                 cacheParameter,
                                                                 spanLenght,
                                                                 polesArray,
                                                                 weightsArray,
                                                                 out aPoint);
    }

    /// <summary>The point and first derivative from a polynomial cache (see CacheD0).</summary>
    public static void CacheD1(double parameter,
                               int degree,
                               double cacheParameter,
                               double spanLenght,
                               NCollection_Array1<Point3D> polesArray,
                               NCollection_Array1<double>? weightsArray,
                               out Point3D aPoint,
                               out Vector3D aVector)
    {
        BSplCLib_CacheD1<Point3D, Vector3D, CurveTraits3d>(parameter,
                                                                 degree,
                                                                 cacheParameter,
                                                                 spanLenght,
                                                                 polesArray,
                                                                 weightsArray,
                                                                 out aPoint,
                                                                 out aVector);
    }

    /// <summary>The point and first two derivatives from a polynomial cache (see CacheD0).</summary>
    public static void CacheD2(double parameter,
                               int degree,
                               double cacheParameter,
                               double spanLenght,
                               NCollection_Array1<Point3D> polesArray,
                               NCollection_Array1<double>? weightsArray,
                               out Point3D aPoint,
                               out Vector3D aVector1,
                               out Vector3D aVector2)
    {
        BSplCLib_CacheD2<Point3D, Vector3D, CurveTraits3d>(parameter,
                                                                 degree,
                                                                 cacheParameter,
                                                                 spanLenght,
                                                                 polesArray,
                                                                 weightsArray,
                                                                 out aPoint,
                                                                 out aVector1,
                                                                 out aVector2);
    }

    /// <summary>The point and first three derivatives from a polynomial cache (see CacheD0).</summary>
    public static void CacheD3(double parameter,
                               int degree,
                               double cacheParameter,
                               double spanLenght,
                               NCollection_Array1<Point3D> polesArray,
                               NCollection_Array1<double>? weightsArray,
                               out Point3D aPoint,
                               out Vector3D aVector1,
                               out Vector3D aVector2,
                               out Vector3D aVector3)
    {
        BSplCLib_CacheD3<Point3D, Vector3D, CurveTraits3d>(parameter,
                                                                 degree,
                                                                 cacheParameter,
                                                                 spanLenght,
                                                                 polesArray,
                                                                 weightsArray,
                                                                 out aPoint,
                                                                 out aVector1,
                                                                 out aVector2,
                                                                 out aVector3);
    }

    /// <summary>
    /// Stores in <paramref name="cachePoles"/> (and <paramref name="cacheWeights"/>) the polynomial coefficients of
    /// the span containing <paramref name="u"/>, scaled to the span domain (1 / <paramref name="inverseOfSpanDomain"/>
    /// in OCCT's naming; the value is used as the span length). Arrays are indexed from 1.
    /// </summary>
    public static void BuildCache(double u,
                                  double inverseOfSpanDomain,
                                  bool periodicFlag,
                                  int degree,
                                  NCollection_Array1<double> flatKnots,
                                  NCollection_Array1<Point3D> poles,
                                  NCollection_Array1<double>? weights,
                                  NCollection_Array1<Point3D> cachePoles,
                                  NCollection_Array1<double>? cacheWeights)
    {
        BSplCLib_BuildCache<Point3D, Vector3D, CurveTraits3d>(u,
                                                                    inverseOfSpanDomain,
                                                                    periodicFlag,
                                                                    degree,
                                                                    flatKnots,
                                                                    poles,
                                                                    weights,
                                                                    cachePoles,
                                                                    cacheWeights);
    }

    /// <summary>
    /// Stores the polynomial coefficients of the span <paramref name="spanIndex"/> into
    /// <paramref name="cacheArray"/>, one row per coefficient: the coordinates, then the weight for a rational
    /// curve (0 except the first, 1, when the weights are all equal).
    /// </summary>
    public static void BuildCache(double parameter,
                                  double spanDomain,
                                  bool periodicFlag,
                                  int degree,
                                  int spanIndex,
                                  NCollection_Array1<double> flatKnots,
                                  NCollection_Array1<Point3D> poles,
                                  NCollection_Array1<double>? weights,
                                  NCollection_Array2<double> cacheArray)
    {
        BSplCLib_BuildCache<Point3D, Vector3D, CurveTraits3d>(parameter,
                                                                    spanDomain,
                                                                    periodicFlag,
                                                                    degree,
                                                                    spanIndex,
                                                                    flatKnots,
                                                                    poles,
                                                                    weights,
                                                                    cacheArray);
    }

    /// <summary>
    /// Interpolates the points (in <paramref name="poles"/> on entry, the poles on return) at
    /// <paramref name="parameters"/>, with the contact orders <paramref name="contactOrderArray"/> (0: the point,
    /// 1: the first derivative, ...). <paramref name="inversionProblem"/> is 0 on success.
    /// </summary>
    public static void Interpolate(int degree,
                                   NCollection_Array1<double> flatKnots,
                                   NCollection_Array1<double> parameters,
                                   NCollection_Array1<int> contactOrderArray,
                                   NCollection_Array1<Point3D> poles,
                                   out int inversionProblem)
    {
        BSplCLib_Interpolate<Point3D, Vector3D, CurveTraits3d>(degree,
                                                                     flatKnots,
                                                                     parameters,
                                                                     contactOrderArray,
                                                                     poles,
                                                                     out inversionProblem);
    }

    /// <summary>The rational form of Interpolate: the weights are interpolated too.</summary>
    public static void Interpolate(int degree,
                                   NCollection_Array1<double> flatKnots,
                                   NCollection_Array1<double> parameters,
                                   NCollection_Array1<int> contactOrderArray,
                                   NCollection_Array1<Point3D> poles,
                                   NCollection_Array1<double> weights,
                                   out int inversionProblem)
    {
        BSplCLib_Interpolate<Point3D, Vector3D, CurveTraits3d>(degree,
                                                                     flatKnots,
                                                                     parameters,
                                                                     contactOrderArray,
                                                                     poles,
                                                                     weights,
                                                                     out inversionProblem);
    }

    /// <summary>
    /// Moves the point at <paramref name="u"/> by <paramref name="displ"/>, changing only the poles between
    /// <paramref name="index1"/> and <paramref name="index2"/>; the poles changed are FirstIndex to LastIndex
    /// (both 0 when nothing could be done).
    /// </summary>
    public static void MovePoint(double u,
                                 in Vector3D displ,
                                 int index1,
                                 int index2,
                                 int degree,
                                 NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<double>? weights,
                                 NCollection_Array1<double> flatKnots,
                                 out int firstIndex,
                                 out int lastIndex,
                                 NCollection_Array1<Point3D> newPoles)
    {
        BSplCLib_MovePoint<Point3D, Vector3D, CurveTraits3d>(u,
                                                                   displ,
                                                                   index1,
                                                                   index2,
                                                                   degree,
                                                                   poles,
                                                                   weights,
                                                                   flatKnots,
                                                                   out firstIndex,
                                                                   out lastIndex,
                                                                   newPoles);
    }

    /// <summary>
    /// Moves the point at <paramref name="u"/> by <paramref name="delta"/> and its derivative by
    /// <paramref name="deltaDerivative"/>, keeping the start and end of the curve to the given orders
    /// (-1: free, 0: the point, 1: the tangent, ...). <paramref name="errorStatus"/> is 0 on success.
    /// </summary>
    public static void MovePointAndTangent(double u,
                                           in Vector3D delta,
                                           in Vector3D deltaDerivative,
                                           double tolerance,
                                           int degree,
                                           int startingCondition,
                                           int endingCondition,
                                           NCollection_Array1<Point3D> poles,
                                           NCollection_Array1<double>? weights,
                                           NCollection_Array1<double> flatKnots,
                                           NCollection_Array1<Point3D> newPoles,
                                           out int errorStatus)
    {
        BSplCLib_MovePointAndTangent<Point3D, Vector3D, CurveTraits3d>(u,
                                                                             delta,
                                                                             deltaDerivative,
                                                                             tolerance,
                                                                             degree,
                                                                             startingCondition,
                                                                             endingCondition,
                                                                             poles,
                                                                             weights,
                                                                             flatKnots,
                                                                             newPoles,
                                                                             out errorStatus);
    }

    /// <summary>
    /// The parametric tolerance <paramref name="uTolerance"/> such that |t1 - t0| &lt; UTolerance implies
    /// |C(t1) - C(t0)| &lt; <paramref name="tolerance3D"/>.
    /// </summary>
    public static void Resolution(NCollection_Array1<Point3D> poles,
                                  NCollection_Array1<double>? weights,
                                  int numPoles,
                                  NCollection_Array1<double> flatKnots,
                                  int degree,
                                  double tolerance3D,
                                  out double uTolerance)
    {
        BSplCLib_Resolution<Point3D, Vector3D, CurveTraits3d>(poles,
                                                                    weights,
                                                                    numPoles,
                                                                    flatKnots,
                                                                    degree,
                                                                    tolerance3D,
                                                                    out uTolerance);
    }

    /// <summary>
    /// Multiplies the B-spline F(t) by the function a(t) (<paramref name="functionPtr"/>), assuming F(t)a(t) is
    /// exactly a B-spline of degree <paramref name="newDegree"/> on <paramref name="flatKnots"/> (not checked).
    /// <paramref name="status"/> is 0, or the pivot index of the failed matrix inversion.
    /// </summary>
    public static void FunctionMultiply(EvaluatorFunction functionPtr,
                                        int bSplineDegree,
                                        NCollection_Array1<double> bSplineFlatKnots,
                                        NCollection_Array1<Point3D> poles,
                                        NCollection_Array1<double> flatKnots,
                                        int newDegree,
                                        NCollection_Array1<Point3D> newPoles,
                                        out int status)
    {
        BSplCLib_FunctionMultiply<Point3D, Vector3D, CurveTraits3d>(functionPtr,
                                                                          bSplineDegree,
                                                                          bSplineFlatKnots,
                                                                          poles,
                                                                          flatKnots,
                                                                          newDegree,
                                                                          newPoles,
                                                                          out status);
    }

    /// <summary>
    /// Composes the B-spline F(t) with the function a(t) (<paramref name="functionPtr"/>), assuming F(a(t)) is
    /// exactly a B-spline of degree <paramref name="newDegree"/> on <paramref name="flatKnots"/> and a(t) maps the
    /// range of FlatKnots onto that of BSplineFlatKnots (not checked). <paramref name="status"/> is 0, or the
    /// pivot index of the failed matrix inversion.
    /// </summary>
    public static void FunctionReparameterise(EvaluatorFunction functionPtr,
                                              int bSplineDegree,
                                              NCollection_Array1<double> bSplineFlatKnots,
                                              NCollection_Array1<Point3D> poles,
                                              NCollection_Array1<double> flatKnots,
                                              int newDegree,
                                              NCollection_Array1<Point3D> newPoles,
                                              out int status)
    {
        BSplCLib_FunctionReparameterise<Point3D, Vector3D, CurveTraits3d>(functionPtr,
                                                                                bSplineDegree,
                                                                                bSplineFlatKnots,
                                                                                poles,
                                                                                flatKnots,
                                                                                newDegree,
                                                                                newPoles,
                                                                                out status);
    }
}
