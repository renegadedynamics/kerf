// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Splines;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Engine.Geometry.NormalStatus;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Geometry.TransformForm;

namespace Occt;

/// <summary>
/// Geom Library: basic computations on the geometry of Geom and Geom2d (conversions to 3D, range changes,
/// extensions, inertia axes, closure checks, ...).
/// </summary>
/// <remarks>
/// The functions that OCCT writes through a handle reference (<c>SameRange</c>, <c>BuildCurve3d</c>,
/// <c>AdjustExtremity</c>, <c>ExtendCurveToPoint</c>, <c>ExtendSurfByLength</c>, <c>CancelDenominatorDerivative</c>)
/// take it by <c>ref</c> and give it a new object the caller owns; they never change the caller's geometry.
/// </remarks>
public static partial class GeomLib
{
    //=================================================================================================

    private static void ComputeLambda(Matrix Constraint,
                                      Matrix Hermit,
                                      double Length,
                                      ref double Lambda)
    {
        int size = Hermit.RowNumber();
        int Continuity = size - 2;
        int ii, jj, ip, pp;

        // Minimization
        Matrix HDer = new(1, size - 1, 1, size);
        for (jj = 1; jj <= size; jj++)
        {
            for (ii = 1; ii < size; ii++)
            {
                HDer[ii, jj] = ii * Hermit[jj, ii + 1];
            }
        }

        math_Vector V = new(1, size);
        math_Vector Vec1 = new(1, Constraint.RowNumber());
        math_Vector Vec2 = new(1, Constraint.RowNumber());
        math_Vector Vec3 = new(1, Constraint.RowNumber());
        math_Vector Vec4 = new(1, Constraint.RowNumber());

        // OCCT's pointers to HDer(1, 1) and V(1)
        Vec2.Initialized(Constraint.Col(2));
        Vec2 /= Length;
        double t, squared1 = Vec2.Norm2(), GW;
        //  math_Matrix Vec(1, Constraint.RowNumber(), 1, size-1);
        //  gp_Vec Vfirst(p0.XYZ()), Vlast(Point.XYZ());
        //  NCollection_Array1<gp_Vec> Der(2, 4);
        //  Der(2) = d1; Der(3) = d2; Der(4) = d3;

        int GOrdre = 4 + 4 * Continuity, DDim = Continuity * (Continuity + 2);
        math_Vector GaussP = new(1, GOrdre), GaussW = new(1, GOrdre), pol2 = new(1, 2 * Continuity + 1),
          pol4 = new(1, 4 * Continuity + 1);
        NumericTables.GaussPoints(GOrdre, GaussP);
        NumericTables.GaussWeights(GOrdre, GaussW);
        pol4.Init(0.0);

        for (ip = 1; ip <= GOrdre; ip++)
        {
            t = (GaussP[ip] + 1.0) / 2;
            GW = GaussW[ip];
            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(t, Continuity, Continuity + 2, DDim, HDer.AsSpan(), V.AsSpan());
            V /= Length; // Normalisation

            //                      i
            // C'(t) = SUM Vi*Lambda
            Vec1.Initialized(Constraint.Col(1));
            Vec1 *= V[1];
            Vec1 += V[size] * Constraint.Col(size);
            Vec2.Initialized(Constraint.Col(2));
            Vec2 *= V[2];
            if (Continuity > 1)
            {
                Vec3.Initialized(Constraint.Col(3));
                Vec3 *= V[3];
                if (Continuity > 2)
                {
                    Vec4.Initialized(Constraint.Col(4));
                    Vec4 *= V[4];
                }
            }

            //   2          2
            // C'(t) - C'(0)

            pol2[1] = Vec1.Norm2();
            pol2[2] = 2 * (Vec1.Multiplied(Vec2));
            pol2[3] = Vec2.Norm2() - squared1;
            if (Continuity > 1)
            {
                pol2[3] += 2 * (Vec1.Multiplied(Vec3));
                pol2[4] = 2 * (Vec2.Multiplied(Vec3));
                pol2[5] = Vec3.Norm2();
                if (Continuity > 2)
                {
                    pol2[4] += 2 * (Vec1.Multiplied(Vec4));
                    pol2[5] += 2 * (Vec2.Multiplied(Vec4));
                    pol2[6] = 2 * (Vec3.Multiplied(Vec4));
                    pol2[7] = Vec4.Norm2();
                }
            }

            //                     2      2  2
            // Integrale de ( C'(t) - C'(0) )
            for (ii = 1; ii <= pol2.Length(); ii++)
            {
                pp = ii;
                for (jj = 1; jj < ii; jj++, pp++)
                {
                    pol4[pp] += 2 * GW * pol2[ii] * pol2[jj];
                }
                pol4[2 * ii - 1] += GW * Math.Pow(pol2[ii], 2);
            }
        }

        double EMin, E;
        Span<double> aRes = stackalloc double[1];
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(Lambda, pol4.Length() - 1, 1, pol4.Length() - 1, pol4.AsSpan(), aRes);
        EMin = aRes[0];

        if (EMin > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // Search for extrema of the function
            GeomLib_PolyFunc FF = new(pol4);
            GeomLib_LogSample S = new(Lambda / 1000, 50 * Lambda, 100);
            FunctionAllRoots Solve = new(FF,
                                              S,
                                              global::Kerf.Engine.Foundation.Precision.Confusion(),
                                              global::Kerf.Engine.Foundation.Precision.Confusion() * (Length + 1),
                                              1.0e-15);
            if (Solve.IsDone)
            {
                for (ii = 1; ii <= Solve.PointCount; ii++)
                {
                    t = Solve.GetPoint(ii);
                    global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(t, pol4.Length() - 1, 1, pol4.Length() - 1, pol4.AsSpan(), aRes);
                    E = aRes[0];
                    if (E < EMin)
                    {
                        Lambda = t;
                        EMin = E;
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Keeps at most about NumPoints of the increasing parameters InParameters: one in each of NumPoints uniform
    /// segments of their range (the first to exceed the segment's midpoint), with the first and last parameters.
    /// </summary>
    public static void RemovePointsFromArray(int NumPoints,
                                             NCollection_Array1<double> InParameters,
                                             out NCollection_HArray1<double> OutParameters)
    {
        int ii, jj, add_one_point, loc_num_points, num_points, index;
        double delta, current_parameter;

        loc_num_points = Max(0, NumPoints - 2);
        delta = InParameters[InParameters.Upper()] - InParameters[InParameters.Lower()];
        delta /= (double)(loc_num_points + 1);
        num_points = 1;
        current_parameter = InParameters[InParameters.Lower()] + delta * 0.5e0;
        ii = InParameters.Lower() + 1;
        for (jj = 0; ii < InParameters.Upper() && jj < NumPoints; jj++)
        {
            add_one_point = 0;
            while (ii < InParameters.Upper() && InParameters[ii] < current_parameter)
            {
                ii += 1;
                add_one_point = 1;
            }
            num_points += add_one_point;
            current_parameter += delta;
        }
        if (NumPoints <= 2)
        {
            num_points = 2;
        }
        index = 2;
        current_parameter = InParameters[InParameters.Lower()] + delta * 0.5e0;
        OutParameters = new NCollection_HArray1<double>(1, num_points);
        OutParameters.ChangeArray1()[1] = InParameters[InParameters.Lower()];
        ii = InParameters.Lower() + 1;
        for (jj = 0; ii < InParameters.Upper() && jj < NumPoints; jj++)
        {
            add_one_point = 0;
            while (ii < InParameters.Upper() && InParameters[ii] < current_parameter)
            {
                ii += 1;
                add_one_point = 1;
            }
            if (add_one_point != 0 && index <= num_points)
            {
                OutParameters.ChangeArray1()[index] = InParameters[ii - 1];
                index += 1;
            }
            current_parameter += delta;
        }
        OutParameters.ChangeArray1()[num_points] = InParameters[InParameters.Upper()];
    }

    //=================================================================================================

    /// <summary>
    /// Makes sure there are at least MinNumPoints parameters, adding uniformly spaced ones between the
    /// non-decreasing InParameters (which are copied unchanged when they are not in order).
    /// </summary>
    public static void DensifyArray1OfReal(int MinNumPoints,
                                           NCollection_Array1<double> InParameters,
                                           out NCollection_HArray1<double> OutParameters)
    {
        int ii, in_order, num_points, num_parameters_to_add, index;
        double delta, current_parameter;

        in_order = 1;
        if (MinNumPoints > InParameters.Length())
        {

            //
            // checks the parameters are in increasing order
            //
            for (ii = InParameters.Lower(); ii < InParameters.Upper(); ii++)
            {
                if (InParameters[ii] > InParameters[ii + 1])
                {
                    in_order = 0;
                    break;
                }
            }
            if (in_order != 0)
            {
                num_parameters_to_add = MinNumPoints - InParameters.Length();
                delta = InParameters[InParameters.Upper()] - InParameters[InParameters.Lower()];
                delta /= (double)(num_parameters_to_add + 1);
                num_points = MinNumPoints;
                OutParameters = new NCollection_HArray1<double>(1, num_points);
                index = 1;
                current_parameter = InParameters[InParameters.Lower()];
                OutParameters.ChangeArray1()[index] = current_parameter;
                index += 1;
                current_parameter += delta;
                for (ii = InParameters.Lower() + 1; index <= num_points && ii <= InParameters.Upper(); ii++)
                {
                    while (current_parameter < InParameters[ii] && index <= num_points)
                    {
                        OutParameters.ChangeArray1()[index] = current_parameter;
                        index += 1;
                        current_parameter += delta;
                    }
                    if (index <= num_points)
                    {
                        OutParameters.ChangeArray1()[index] = InParameters[ii];
                    }
                    index += 1;
                }
                //
                // beware of roundoff !
                //
                OutParameters.ChangeArray1()[num_points] = InParameters[InParameters.Upper()];
            }
            else
            {
                index = 1;
                num_points = InParameters.Length();
                OutParameters = new NCollection_HArray1<double>(1, num_points);
                for (ii = InParameters.Lower(); ii <= InParameters.Upper(); ii++)
                {
                    OutParameters.ChangeArray1()[index] = InParameters[ii];
                    index += 1;
                }
            }
        }
        else
        {
            index = 1;
            num_points = InParameters.Length();
            OutParameters = new NCollection_HArray1<double>(1, num_points);
            for (ii = InParameters.Lower(); ii <= InParameters.Upper(); ii++)
            {
                OutParameters.ChangeArray1()[index] = InParameters[ii];
                index += 1;
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Fuses the sorted intervals Interval1 and Interval2 into Fusion. Values closer than Confusion are merged: to
    /// their mean, or to the value of Interval1 when IsAdjustToFirstInterval is true.
    /// </summary>
    public static void FuseIntervals(NCollection_Array1<double> I1,
                                     NCollection_Array1<double> I2,
                                     NCollection_Sequence<double> Seq,
                                     double Epspar = 1.0e-9,
                                     bool IsAdjustToFirstInterval = false)
    {
        int ind1 = 1, ind2 = 1;
        double v1, v2;
        // Initializations: IND1 and IND2 point to the 1st element
        // of each of the 2 tables to process. INDS points to the last
        // created element of TABSOR

        //--- Fill TABSOR by traversing TABLE1 and TABLE2 simultaneously ---
        //------------------ eliminating multiple occurrences ----------------

        while ((ind1 <= I1.Upper()) && (ind2 <= I2.Upper()))
        {
            v1 = I1[ind1];
            v2 = I2[ind2];
            if (Math.Abs(v1 - v2) <= Epspar)
            {
                // Here the elements of I1 and I2 match.
                if (IsAdjustToFirstInterval)
                {
                    Seq.Append(v1);
                }
                else
                {
                    Seq.Append((v1 + v2) / 2);
                }
                ind1++;
                ind2++;
            }
            else if (v1 < v2)
            {
                // Here the element of I1 is suitable.
                Seq.Append(v1);
                ind1++;
            }
            else
            {
                // Here the element of TABLE2 is suitable.
                Seq.Append(v2);
                ind2++;
            }
        }

        if (ind1 > I1.Upper())
        {
            //----- Here I1 is exhausted, complete with the end of TABLE2 -------

            for (; ind2 <= I2.Upper(); ind2++)
            {
                Seq.Append(I2[ind2]);
            }
        }

        if (ind2 > I2.Upper())
        {
            //----- Here I2 is exhausted, complete with the end of I1 -------
            for (; ind1 <= I1.Upper(); ind1++)
            {
                Seq.Append(I1[ind1]);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// The largest distance between the two curves at the given parameters (the curves evaluated at the same
    /// parameter). The tolerance is not used.
    /// </summary>
    public static void EvalMaxParametricDistance(Adaptor3d_Curve ACurve,
                                                 Adaptor3d_Curve AReferenceCurve,
                                                 //			       const double  Tolerance,
                                                 double _,
                                                 NCollection_Array1<double> Parameters,
                                                 out double MaxDistance)
    {
        int ii;

        double max_squared = 0.0e0,
          //    tolerance_squared,
          local_distance_squared;

        //  tolerance_squared = Tolerance * Tolerance ;
        Point3D Point1;
        Point3D Point2;
        for (ii = Parameters.Lower(); ii <= Parameters.Upper(); ii++)
        {
            ACurve.D0(Parameters[ii], out Point1);
            AReferenceCurve.D0(Parameters[ii], out Point2);
            local_distance_squared = Point1.SquareDistance(Point2);
            max_squared = Max(max_squared, local_distance_squared);
        }
        if (max_squared > 0.0e0)
        {
            MaxDistance = Math.Sqrt(max_squared);
        }
        else
        {
            MaxDistance = 0.0e0;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The largest distance from the curve at the given parameters to the reference curve: where the points at the
    /// same parameter are farther apart than Tolerance, the point is projected on the reference curve. At least
    /// Tolerance.
    /// </summary>
    public static void EvalMaxDistanceAlongParameter(Adaptor3d_Curve ACurve,
                                                     Adaptor3d_Curve AReferenceCurve,
                                                     double Tolerance,
                                                     NCollection_Array1<double> Parameters,
                                                     out double MaxDistance)
    {
        int ii;
        double max_squared = 0.0e0, tolerance_squared = Tolerance * Tolerance, other_parameter,
               para_tolerance, local_distance_squared;
        Point3D Point1;
        Point3D Point2;

        para_tolerance = AReferenceCurve.Resolution(Tolerance);
        other_parameter = Parameters[Parameters.Lower()];
        ACurve.D0(other_parameter, out Point1);
        Extrema_LocateExtPC a_projector = new(Point1, AReferenceCurve, other_parameter, para_tolerance);
        for (ii = Parameters.Lower(); ii <= Parameters.Upper(); ii++)
        {
            ACurve.D0(Parameters[ii], out Point1);
            AReferenceCurve.D0(Parameters[ii], out Point2);
            local_distance_squared = Point1.SquareDistance(Point2);

            if (local_distance_squared > tolerance_squared)
            {

                a_projector.Perform(Point1, other_parameter);
                if (a_projector.IsDone())
                {
                    other_parameter = a_projector.Point().Parameter();
                    AReferenceCurve.D0(other_parameter, out Point2);
                    local_distance_squared = Point1.SquareDistance(Point2);
                }
                else
                {
                    local_distance_squared = 0.0e0;
                    other_parameter = Parameters[ii];
                }
            }
            else
            {
                other_parameter = Parameters[ii];
            }

            max_squared = Max(max_squared, local_distance_squared);
        }
        if (max_squared > tolerance_squared)
        {
            MaxDistance = Math.Sqrt(max_squared);
        }
        else
        {
            MaxDistance = Tolerance;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The 3D curve of the 2D curve Curve2d drawn on the plane Position. Raises Standard_NotImplemented for a
    /// curve of another kind.
    /// </summary>
    public static Geom_Curve To3d(in Frame3D Position, Geom2d_Curve Curve2d)
    {
        Geom_Curve Curve3d;

        if (Curve2d.GetType() == typeof(Geom2d_TrimmedCurve))
        {
            Geom2d_TrimmedCurve Ct = (Geom2d_TrimmedCurve)Curve2d;
            double U1 = Ct.FirstParameter();
            double U2 = Ct.LastParameter();
            Geom2d_Curve CBasis2d = Ct.BasisCurve();
            Geom_Curve CC = GeomLib.To3d(Position, CBasis2d);
            Curve3d = new Geom_TrimmedCurve(CC, U1, U2);
        }
        else if (Curve2d.GetType() == typeof(Geom2d_OffsetCurve))
        {
            Geom2d_OffsetCurve Co = (Geom2d_OffsetCurve)Curve2d;
            double Offset = Co.Offset();
            Geom2d_Curve CBasis2d = Co.BasisCurve();
            Geom_Curve CC = GeomLib.To3d(Position, CBasis2d);
            Curve3d = new Geom_OffsetCurve(CC, Offset, Position.Direction());
        }
        else if (Curve2d.GetType() == typeof(Geom2d_BezierCurve))
        {
            Geom2d_BezierCurve CBez2d = (Geom2d_BezierCurve)Curve2d;
            NCollection_Array1<Point2D> Poles2d = CBez2d.InternalPoles();
            int Nbpoles = Poles2d.Length();
            NCollection_Array1<Point3D> Poles3d = new(1, Nbpoles);
            for (int i = 1; i <= Nbpoles; i++)
            {
                Poles3d[i] = global::Kerf.Engine.Geometry.ElCLib.To3d(Position, Poles2d[i]);
            }
            Geom_BezierCurve CBez3d;
            if (CBez2d.IsRational())
            {
                CBez3d = new Geom_BezierCurve(Poles3d, CBez2d.InternalWeightsArray());
            }
            else
            {
                CBez3d = new Geom_BezierCurve(Poles3d);
            }
            Curve3d = CBez3d;
        }
        else if (Curve2d.GetType() == typeof(Geom2d_BSplineCurve))
        {
            Geom2d_BSplineCurve CBSpl2d = (Geom2d_BSplineCurve)Curve2d;
            int TheDegree = CBSpl2d.Degree();
            bool IsPeriodic = CBSpl2d.IsPeriodic();
            NCollection_Array1<Point2D> Poles2d = CBSpl2d.InternalPoles();
            int Nbpoles = Poles2d.Length();
            NCollection_Array1<Point3D> Poles3d = new(1, Nbpoles);
            for (int i = 1; i <= Nbpoles; i++)
            {
                Poles3d[i] = global::Kerf.Engine.Geometry.ElCLib.To3d(Position, Poles2d[i]);
            }
            NCollection_Array1<double> TheKnots = CBSpl2d.InternalKnots();
            NCollection_Array1<int> TheMults = CBSpl2d.InternalMultiplicities();
            Geom_BSplineCurve CBSpl3d;
            if (CBSpl2d.IsRational())
            {
                CBSpl3d = new Geom_BSplineCurve(Poles3d,
                                                CBSpl2d.InternalWeightsArray(),
                                                TheKnots,
                                                TheMults,
                                                TheDegree,
                                                IsPeriodic);
            }
            else
            {
                CBSpl3d = new Geom_BSplineCurve(Poles3d, TheKnots, TheMults, TheDegree, IsPeriodic);
            }
            Curve3d = CBSpl3d;
        }
        else if (Curve2d.GetType() == typeof(Geom2d_Line))
        {
            Geom2d_Line Line2d = (Geom2d_Line)Curve2d;
            Line2D L2d = Line2d.Lin2d();
            Line3D L3d = global::Kerf.Engine.Geometry.ElCLib.To3d(Position, L2d);
            Geom_Line GeomL3d = new(L3d);
            Curve3d = GeomL3d;
        }
        else if (Curve2d.GetType() == typeof(Geom2d_Circle))
        {
            Geom2d_Circle Circle2d = (Geom2d_Circle)Curve2d;
            Circle2D C2d = Circle2d.Circ2d();
            Circle3D C3d = global::Kerf.Engine.Geometry.ElCLib.To3d(Position, C2d);
            Geom_Circle GeomC3d = new(C3d);
            Curve3d = GeomC3d;
        }
        else if (Curve2d.GetType() == typeof(Geom2d_Ellipse))
        {
            Geom2d_Ellipse Ellipse2d = (Geom2d_Ellipse)Curve2d;
            Ellipse2D E2d = Ellipse2d.Elips2d();
            Ellipse3D E3d = global::Kerf.Engine.Geometry.ElCLib.To3d(Position, E2d);
            Geom_Ellipse GeomE3d = new(E3d);
            Curve3d = GeomE3d;
        }
        else if (Curve2d.GetType() == typeof(Geom2d_Parabola))
        {
            Geom2d_Parabola Parabola2d = (Geom2d_Parabola)Curve2d;
            Parabola2D Prb2d = Parabola2d.Parab2d();
            Parabola3D Prb3d = global::Kerf.Engine.Geometry.ElCLib.To3d(Position, Prb2d);
            Geom_Parabola GeomPrb3d = new(Prb3d);
            Curve3d = GeomPrb3d;
        }
        else if (Curve2d.GetType() == typeof(Geom2d_Hyperbola))
        {
            Geom2d_Hyperbola Hyperbola2d = (Geom2d_Hyperbola)Curve2d;
            Hyperbola2D H2d = Hyperbola2d.Hypr2d();
            Hyperbola3D H3d = global::Kerf.Engine.Geometry.ElCLib.To3d(Position, H2d);
            Geom_Hyperbola GeomH3d = new(H3d);
            Curve3d = GeomH3d;
        }
        else
        {
            throw new MethodNotImplementedException();
        }

        return Curve3d;
    }

    //=================================================================================================

    /// <summary>
    /// The 2D curve transformed by GTrsf. Null when it cannot be computed: an offset curve, or an infinite
    /// parabola or hyperbola under an affinity.
    /// </summary>
    public static Geom2d_Curve? GTransform(Geom2d_Curve Curve, in GeneralTransform2D GTrsf)
    {
        TransformForm Form = GTrsf.Form();

        if (Form != Other)
        {

            // Then, the GTrsf is actually a Trsf.
            // The curve geometry will then remain unchanged.

            Geom2d_Curve? C = Curve.Transformed(GTrsf.Trsf2d()) as Geom2d_Curve;
            return C;
        }
        else
        {

            // So, the GTrsf is an other Transformation.
            // The geometry of the curves is then changed, and the conics will have to
            // be converted to BSplines.

            Type TheType = Curve.GetType();

            if (TheType == typeof(Geom2d_TrimmedCurve))
            {

                // We will recurse on the BasisCurve

                Geom2d_TrimmedCurve C = (Geom2d_TrimmedCurve)Curve.Copy();

                Type TheBasisType = (C.BasisCurve()).GetType();

                if (TheBasisType == typeof(Geom2d_BSplineCurve)
                    || TheBasisType == typeof(Geom2d_BezierCurve))
                {

                    // In these cases the parameterization is preserved on the transformed curve,
                    // so we can trim it with the parameters of the base curve.

                    double U1 = C.FirstParameter();
                    double U2 = C.LastParameter();

                    Geom2d_TrimmedCurve result =
                      new(GTransform(C.BasisCurve(), GTrsf)!, U1, U2);
                    return result;
                }
                else if (TheBasisType == typeof(Geom2d_Line))
                {

                    // In this case, the parameterization is no longer preserved.
                    // We must recompute the trimming parameters on the resulting
                    // curve (by projecting the transformed start and end points using ElCLib).

                    Geom2d_Line L = (Geom2d_Line)GTransform(C.BasisCurve(), GTrsf)!;
                    Line2D Lin = L.Lin2d();

                    Point2D P1 = C.StartPoint();
                    Point2D P2 = C.EndPoint();
                    P1.SetXY(GTrsf.Transformed(P1.XY()));
                    P2.SetXY(GTrsf.Transformed(P2.XY()));
                    double U1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(Lin, P1);
                    double U2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(Lin, P2);

                    Geom2d_TrimmedCurve result = new(L, U1, U2);
                    return result;
                }
                else if (TheBasisType == typeof(Geom2d_Circle)
                         || TheBasisType == typeof(Geom2d_Ellipse)
                         || TheBasisType == typeof(Geom2d_Parabola)
                         || TheBasisType == typeof(Geom2d_Hyperbola))
                {

                    // In these cases, the curve geometry is not preserved;
                    // convert it to BSpline before applying the Trsf.

                    Geom2d_BSplineCurve BS = Geom2dConvert.CurveToBSplineCurve(C);
                    return GTransform(BS, GTrsf);
                }
                else
                {

                    // The transform of an OffsetCurve is not supported.

                    return null;
                }
            }
            else if (TheType == typeof(Geom2d_Line))
            {

                Geom2d_Line L = (Geom2d_Line)Curve.Copy();
                Line2D Lin = L.Lin2d();
                Point2D P = Lin.Location();
                Point2D PP = L.Value(10.0); // arbitrary point
                P.SetXY(GTrsf.Transformed(P.XY()));
                PP.SetXY(GTrsf.Transformed(PP.XY()));
                L.SetLocation(P);
                Vector2D V = new(P, PP);
                L.SetDirection(new Direction2D(V));
                return L;
            }
            else if (TheType == typeof(Geom2d_BezierCurve))
            {

                // Since GTrsf is a linear operation, the transform of a pole-based curve
                // is the curve whose poles are the transforms of the base curve's poles.

                Geom2d_BezierCurve C = (Geom2d_BezierCurve)Curve.Copy();
                int NbPoles = C.NbPoles();
                NCollection_Array1<Point2D> Poles = new(C.InternalPoles());
                for (int i = 1; i <= NbPoles; i++)
                {
                    Poles[i].SetXY(GTrsf.Transformed(Poles[i].XY()));
                    C.SetPole(i, Poles[i]);
                }
                return C;
            }
            else if (TheType == typeof(Geom2d_BSplineCurve))
            {

                // See comment for Bezier curves above.

                Geom2d_BSplineCurve C = (Geom2d_BSplineCurve)Curve.Copy();
                int NbPoles = C.NbPoles();
                NCollection_Array1<Point2D> Poles = new(C.InternalPoles());
                for (int i = 1; i <= NbPoles; i++)
                {
                    Poles[i].SetXY(GTrsf.Transformed(Poles[i].XY()));
                    C.SetPole(i, Poles[i]);
                }
                return C;
            }
            else if (TheType == typeof(Geom2d_Circle) || TheType == typeof(Geom2d_Ellipse))
            {

                // In these cases, the curve geometry is not preserved;
                // convert it to BSpline before applying the Trsf.

                Geom2d_BSplineCurve C = Geom2dConvert.CurveToBSplineCurve(Curve);
                return GTransform(C, GTrsf);
            }
            else if (TheType == typeof(Geom2d_Parabola) || TheType == typeof(Geom2d_Hyperbola)
                     || TheType == typeof(Geom2d_OffsetCurve))
            {

                // Not supported: return a null Handle;

                return null;
            }
        }

        return null; // portage Windows.
    }

    //=================================================================================================

    /// <summary>
    /// Gives the 2D curve the range RequestedFirst to RequestedLast as economically as possible: unchanged when the
    /// ranges match within Tolerance, translated or rotated (lines, circles) or re-trimmed when only the origin
    /// changes, otherwise a B-spline reparametrised to the range. NewCurvePtr can be CurvePtr itself; it is left
    /// unchanged where OCCT leaves it (equal lengths, a curve none of the cases handles, a null-length range).
    /// </summary>
    public static void SameRange(double Tolerance,
                                 Geom2d_Curve? CurvePtr,
                                 double FirstOnCurve,
                                 double LastOnCurve,
                                 double RequestedFirst,
                                 double RequestedLast,
                                 ref Geom2d_Curve? NewCurvePtr)
    {
        if (CurvePtr is null)
        {
            throw new FailureException();
        }
        if (Math.Abs(LastOnCurve - RequestedLast) <= Tolerance
            && Math.Abs(FirstOnCurve - RequestedFirst) <= Tolerance)
        {
            NewCurvePtr = CurvePtr;
            return;
        }

        // the parametrisation length  must at least be the same.
        if (Math.Abs(LastOnCurve - FirstOnCurve - RequestedLast + RequestedFirst) <= Tolerance)
        {
            if (CurvePtr is Geom2d_Line)
            {
                Geom2d_Line Line = (Geom2d_Line)CurvePtr.Copy();
                double dU = FirstOnCurve - RequestedFirst;
                Direction2D D = Line.Direction();
                Line.Translate(dU * new Vector2D(D));
                NewCurvePtr = Line;
            }
            else if (CurvePtr is Geom2d_Circle)
            {
                Transform2D Trsf = new();
                NewCurvePtr = (Geom2d_Curve)CurvePtr.Copy();
                Geom2d_Circle Circ = (Geom2d_Circle)NewCurvePtr;
                Point2D P = Circ.Location();
                double dU;
                if (Circ.Circ2d().IsDirect)
                {
                    dU = FirstOnCurve - RequestedFirst;
                }
                else
                {
                    dU = RequestedFirst - FirstOnCurve;
                }
                Trsf.SetRotation(P, dU);
                NewCurvePtr.Transform(Trsf);
            }
            else if (CurvePtr is Geom2d_TrimmedCurve)
            {
                Geom2d_TrimmedCurve TC = (Geom2d_TrimmedCurve)CurvePtr;
                GeomLib.SameRange(Tolerance,
                                  TC.BasisCurve(),
                                  FirstOnCurve,
                                  LastOnCurve,
                                  RequestedFirst,
                                  RequestedLast,
                                  ref NewCurvePtr);
                NewCurvePtr = new Geom2d_TrimmedCurve(NewCurvePtr!, RequestedFirst, RequestedLast);
            }
            //
            //  Beware of limitation issues: use the SAME test as in
            //  Geom2d_TrimmedCurve::SetTrim, otherwise since we may re-trim on
            //  RequestedFirst and RequestedLast we will have a problem
            //
            //
            else if (Math.Abs(LastOnCurve - FirstOnCurve) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                     || Math.Abs(RequestedLast + RequestedFirst) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {

                Geom2d_TrimmedCurve TC =
                  new(CurvePtr, FirstOnCurve, LastOnCurve);

                Geom2d_BSplineCurve BS = Geom2dConvert.CurveToBSplineCurve(TC);
                NCollection_Array1<double> Knots = new(BS.InternalKnots());

                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(RequestedFirst, RequestedLast, Knots);

                BS.SetKnots(Knots);
                NewCurvePtr = BS;
            }
        }
        else
        { // We segment the result
            Geom2d_TrimmedCurve TC;
            Geom2d_Curve aCCheck = CurvePtr;

            if (aCCheck is Geom2d_TrimmedCurve)
            {
                aCCheck = ((Geom2d_TrimmedCurve)aCCheck).BasisCurve();
            }

            if (aCCheck.IsPeriodic())
            {
                if (Math.Abs(LastOnCurve - FirstOnCurve) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    TC = new Geom2d_TrimmedCurve(CurvePtr, FirstOnCurve, LastOnCurve);
                }
                else
                {
                    TC =
                      new Geom2d_TrimmedCurve(CurvePtr, CurvePtr.FirstParameter(), CurvePtr.LastParameter());
                }
            }
            else
            {
                double Udeb = Max(CurvePtr.FirstParameter(), FirstOnCurve);
                double Ufin = Min(CurvePtr.LastParameter(), LastOnCurve);
                if (Math.Abs(Ufin - Udeb) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    TC = new Geom2d_TrimmedCurve(CurvePtr, Udeb, Ufin);
                }
                else
                {
                    TC =
                      new Geom2d_TrimmedCurve(CurvePtr, CurvePtr.FirstParameter(), CurvePtr.LastParameter());
                }
            }

            //
            Geom2d_BSplineCurve BS = Geom2dConvert.CurveToBSplineCurve(TC);
            NCollection_Array1<double> Knots = new(BS.InternalKnots());

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(RequestedFirst, RequestedLast, Knots);

            BS.SetKnots(Knots);
            NewCurvePtr = BS;
        }
    }

    //=================================================================================================

    private sealed class GeomLib_CurveOnSurfaceEvaluator : AdvApprox_EvaluatorFunction
    {
        public GeomLib_CurveOnSurfaceEvaluator(Adaptor3d_CurveOnSurface theCurveOnSurface,
                                               double theFirst,
                                               double theLast)
        {
            CurveOnSurface = theCurveOnSurface;
            FirstParam = theFirst;
            LastParam = theLast;
        }

        public override void Evaluate(int Dimension, ReadOnlySpan<double> DebutFin, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ReturnCode)
        {
            Point3D Point;

            // Handle left / right positioning
            if ((DebutFin[0] != FirstParam) || (DebutFin[1] != LastParam))
            {
                TrimCurve = CurveOnSurface.Trim(DebutFin[0], DebutFin[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                FirstParam = DebutFin[0];
                LastParam = DebutFin[1];
            }

            // Positioning
            if (DerivativeRequest == 0)
            {
                TrimCurve!.D0(Parameter, out Point);

                for (int ii = 0; ii < 3; ii++)
                {
                    Result[ii] = Point.Coord(ii + 1);
                }
            }
            if (DerivativeRequest == 1)
            {
                Vector3D Vector;
                TrimCurve!.D1(Parameter, out Point, out Vector);
                for (int ii = 0; ii < 3; ii++)
                {
                    Result[ii] = Vector.Coord(ii + 1);
                }
            }
            if (DerivativeRequest == 2)
            {
                Vector3D Vector, VecBis;
                TrimCurve!.D2(Parameter, out Point, out VecBis, out Vector);
                for (int ii = 0; ii < 3; ii++)
                {
                    Result[ii] = Vector.Coord(ii + 1);
                }
            }
            ReturnCode = 0;
        }

        private readonly Adaptor3d_CurveOnSurface CurveOnSurface;
        private double FirstParam;
        private double LastParam;

        private Adaptor3d_Curve? TrimCurve;
    }

    //=================================================================================================

    /// <summary>
    /// Builds the 3D curve of a curve on a surface: on a plane, the 2D curve drawn on it (GeomLib.To3d); on an
    /// isoline, the surface's isoline; otherwise an AdvApprox approximation within Tolerance, with its maximum and
    /// average deviations. NewCurvePtr is left unchanged when the approximation has no result.
    /// </summary>
    public static void BuildCurve3d(double Tolerance,
                                    Adaptor3d_CurveOnSurface Curve,
                                    double FirstParameter,
                                    double LastParameter,
                                    ref Geom_Curve? NewCurvePtr,
                                    out double MaxDeviation,
                                    out double AverageDeviation,
                                    Continuity Continuity = C1,
                                    int MaxDegree = 14,
                                    int MaxSegment = 30)

    {

        MaxDeviation = 0.0e0;
        AverageDeviation = 0.0e0;
        GeomAdaptor_Surface? geom_adaptor_surface_ptr = Curve.GetSurface() as GeomAdaptor_Surface;
        Geom2dAdaptor_Curve? geom_adaptor_curve_ptr = Curve.GetCurve() as Geom2dAdaptor_Curve;

        if (geom_adaptor_curve_ptr is not null && geom_adaptor_surface_ptr is not null)
        {
            Geom_Plane? P;
            GeomAdaptor_Surface geom_surface = geom_adaptor_surface_ptr;

            Geom_RectangularTrimmedSurface? RT =
              geom_surface.Surface() as Geom_RectangularTrimmedSurface;
            if (RT is null)
            {
                P = geom_surface.Surface() as Geom_Plane;
            }
            else
            {
                P = RT.BasisSurface() as Geom_Plane;
            }

            if (P is not null)
            {
                // compute the 3d curve
                Frame3D axes = P.Position().Ax2();
                Geom2dAdaptor_Curve geom2d_curve = geom_adaptor_curve_ptr;
                NewCurvePtr = GeomLib.To3d(axes, geom2d_curve.Curve()!);
                return;
            }

            Adaptor2d_Curve2d TrimmedC2D =
              geom_adaptor_curve_ptr.Trim(FirstParameter, LastParameter, global::Kerf.Engine.Foundation.Precision.PConfusion());

            bool isU, isForward;
            double aParam;
            if (isIsoLine(TrimmedC2D, out isU, out aParam, out isForward))
            {
                NewCurvePtr = buildC3dOnIsoLine(TrimmedC2D,
                                                geom_adaptor_surface_ptr,
                                                FirstParameter,
                                                LastParameter,
                                                Tolerance,
                                                isU,
                                                aParam,
                                                isForward);
                if (NewCurvePtr is not null)
                {
                    return;
                }
            }
        }

        //
        // Entree
        //
        NCollection_HArray1<double>? Tolerance1DPtr = null, Tolerance2DPtr = null;
        NCollection_HArray1<double> Tolerance3DPtr = new(1, 1);
        Tolerance3DPtr.SetValue(1, Tolerance);

        // Search for discontinuities
        int NbIntervalC2 = Curve.NbIntervals(C2);
        NCollection_Array1<double> Param_de_decoupeC2 = new(1, NbIntervalC2 + 1);
        Curve.Intervals(Param_de_decoupeC2, C2);

        int NbIntervalC3 = Curve.NbIntervals(C3);
        NCollection_Array1<double> Param_de_decoupeC3 = new(1, NbIntervalC3 + 1);
        Curve.Intervals(Param_de_decoupeC3, C3);

        // Note extension of the parametric range
        // To force Trim on first evaluator call
        GeomLib_CurveOnSurfaceEvaluator ev = new(Curve, FirstParameter - 1.0, LastParameter + 1.0);

        // Approximation with preferential cutting
        AdvApprox_PrefAndRec Preferentiel = new(Param_de_decoupeC2, Param_de_decoupeC3);
        AdvApprox_ApproxAFunction anApproximator = new(0,
                                                       0,
                                                       1,
                                                       Tolerance1DPtr,
                                                       Tolerance2DPtr,
                                                       Tolerance3DPtr,
                                                       FirstParameter,
                                                       LastParameter,
                                                       Continuity,
                                                       MaxDegree,
                                                       MaxSegment,
                                                       ev,
                                                       //					      CurveOnSurfaceEvaluator,
                                                       Preferentiel);

        if (anApproximator.HasResult())
        {
            GeomLib_MakeCurvefromApprox aCurveBuilder = new(anApproximator);

            Geom_BSplineCurve aCurvePtr = aCurveBuilder.Curve(1);
            // Return the approximation results
            MaxDeviation = anApproximator.MaxError(3, 1);
            AverageDeviation = anApproximator.AverageError(3, 1);
            NewCurvePtr = aCurvePtr;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Deforms the curve (converted to a B-spline) so that it starts at P1 tangent to T1 and ends at P2 tangent to
    /// T2. Curve receives the new curve.
    /// </summary>
    public static void AdjustExtremity(ref Geom_BoundedCurve Curve,
                                       in Point3D P1,
                                       in Point3D P2,
                                       in Vector3D T1,
                                       in Vector3D T2)
    {
        // Convert the input (preserving the parameterization if possible)
        Geom_BSplineCurve aIn, aDef;
        aIn = GeomConvert.CurveToBSplineCurve(Curve, QuasiAngular);
        // The conversion copies the caller's curve; a frozen result would be shared (decision 244): change a copy.
        if (aIn.IsFrozen)
        {
            aIn = (Geom_BSplineCurve)aIn.Copy();
        }

        int ii, jj;
        Point3D P;
        Vector3D V, Vtan, DV;
        NCollection_Array1<Point3D> PolesDef = new(1, 4), Coeffs = new(1, 4);
        NCollection_Array1<double> FK = new(1, 8);
        NCollection_Array1<double> Ti = new(1, 4);
        NCollection_Array1<int> Contact = new(1, 4);

        Ti[1] = Ti[2] = aIn.FirstParameter();
        Ti[3] = Ti[4] = aIn.LastParameter();
        Contact[1] = Contact[3] = 0;
        Contact[2] = Contact[4] = 1;
        for (ii = 1; ii <= 4; ii++)
        {
            FK[ii] = aIn.FirstParameter();
            FK[ii] = aIn.LastParameter();
        }

        // Calculation of deformation constraints
        aIn.D1(Ti[1], out P, out V);
        PolesDef[1].ChangeCoord() = P1.XYZ() - P.XYZ();
        Vtan = T1;
        Vtan.Normalize();
        DV = Vtan * (Vtan * V) - V;
        PolesDef[2].ChangeCoord() = (Ti[4] - Ti[1]) * DV.XYZ();

        aIn.D1(Ti[4], out P, out V);
        PolesDef[3].ChangeCoord() = P2.XYZ() - P.XYZ();
        Vtan = T2;
        Vtan.Normalize();
        DV = Vtan * (Vtan * V) - V;
        PolesDef[4].ChangeCoord() = (Ti[4] - Ti[1]) * DV.XYZ();

        // Interpolation of constraints
        Matrix Mat = new(1, 4, 1, 4);
        if (!global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(0.0, 1.0, 1, 1, Mat))
        {
            throw new ConstructionErrorException();
        }

        for (jj = 1; jj <= 4; jj++)
        {
            Xyz aux = new(0.0, 0.0, 0.0);
            for (ii = 1; ii <= 4; ii++)
            {
                aux.SetLinearForm(Mat[ii, jj], PolesDef[ii].XYZ(), aux);
            }
            Coeffs[jj].SetXYZ(aux);
        }

        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(Coeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), PolesDef, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());

        // Add the deformation
        NCollection_Array1<double> K = new(1, 2);
        NCollection_Array1<int> M = new(1, 2);
        K[1] = Ti[1];
        K[2] = Ti[4];
        M.Init(4);

        aDef = new Geom_BSplineCurve(PolesDef, K, M, 3);
        if (aIn.Degree() < 3)
        {
            aIn.IncreaseDegree(3);
        }
        else
        {
            aDef.IncreaseDegree(aIn.Degree());
        }

        for (ii = 2; ii < aIn.NbKnots(); ii++)
        {
            aDef.InsertKnot(aIn.Knot(ii), aIn.Multiplicity(ii));
        }

        if (aDef.NbPoles() != aIn.NbPoles())
        {
            throw new ConstructionErrorException("Inconsistent poles's number");
        }

        for (ii = 1; ii <= aDef.NbPoles(); ii++)
        {
            P = aIn.Pole(ii);
            P.ChangeCoord() += aDef.Pole(ii).XYZ();
            aIn.SetPole(ii, P);
        }
        Curve = aIn;
    }

    //=================================================================================================

    /// <summary>
    /// Extends the bounded curve to Point, after its end if After is true or before its start otherwise, with
    /// continuity Continuity (1, 2 or 3; nothing is done otherwise, nor when Point is on the curve's end). Curve
    /// receives a new B-spline curve.
    /// </summary>
    public static void ExtendCurveToPoint(ref Geom_BoundedCurve Curve,
                                          in Point3D Point,
                                          int Continuity,
                                          bool After)
    {
        if (Continuity < 1 || Continuity > 3)
        {
            return;
        }
        int size = Continuity + 2;
        double Ubord, Tol = 1.0e-6;
        Matrix MatCoefs = new(1, size, 1, size);
        double Lambda, L1;
        int ii, jj;
        Vector3D d1, d2, d3;
        Point3D p0;
        // Convert the input (preserving the parameterization if possible)
        GeomConvert_CompCurveToBSplineCurve Concat = new(Curve, QuasiAngular);

        // Construction constraints
        NCollection_Array1<Xyz> Cont = new(1, size);
        if (After)
        {
            Ubord = Curve.LastParameter();
        }
        else
        {
            Ubord = Curve.FirstParameter();
        }
        global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(0,
                                 1, // The Bounds
                                 Continuity,
                                 0, // The constraint orders
                                 MatCoefs);

        Curve.D3(Ubord, out p0, out d1, out d2, out d3);
        if (!After)
        { // Invert the parameterization
            d1 *= -1;
            d3 *= -1;
        }

        L1 = p0.Distance(Point);
        if (L1 > Tol)
        {
            // Lambda is the ratio to apply to the derivative of the curve
            // to obtain the derivative of the extension (arbitrarily fixed to the
            // length of the segment end of curve - target point.
            // We try to have on the extension the average velocity that we
            // have on the curve.
            Vector3D daux;
            Point3D pp;
            double f = Curve.FirstParameter(), t, dt, norm;
            dt = (Curve.LastParameter() - f) / 9;
            norm = d1.Magnitude();
            for (ii = 1, t = f + dt; ii <= 8; ii++, t += dt)
            {
                Curve.D1(t, out pp, out daux);
                norm += daux.Magnitude();
            }
            norm /= 9;
            dt = d1.Magnitude() / norm;
            if ((dt < 1.5) && (dt > 0.75))
            { // The edge is within the average, keep it
                Lambda = ((double)1) / Max(d1.Magnitude() / L1, Tol);
            }
            else
            {
                Lambda = ((double)1) / Max(norm / L1, Tol);
            }
        }
        else
        {
            return; // No extension
        }

        // Optimization of Lambda
        Matrix Cons = new(1, 3, 1, size);
        Cons[1, 1] = p0.X();
        Cons[2, 1] = p0.Y();
        Cons[3, 1] = p0.Z();
        Cons[1, 2] = d1.X();
        Cons[2, 2] = d1.Y();
        Cons[3, 2] = d1.Z();
        Cons[1, size] = Point.X();
        Cons[2, size] = Point.Y();
        Cons[3, size] = Point.Z();
        if (Continuity >= 2)
        {
            Cons[1, 3] = d2.X();
            Cons[2, 3] = d2.Y();
            Cons[3, 3] = d2.Z();
        }
        if (Continuity >= 3)
        {
            Cons[1, 4] = d3.X();
            Cons[2, 4] = d3.Y();
            Cons[3, 4] = d3.Z();
        }
        ComputeLambda(Cons, MatCoefs, L1, ref Lambda);

        // Construction in the Polynomial Basis
        Cont[1] = p0.XYZ();
        Cont[2] = d1.XYZ() * Lambda;
        if (Continuity >= 2)
        {
            Cont[3] = d2.XYZ() * Math.Pow(Lambda, 2);
        }
        if (Continuity >= 3)
        {
            Cont[4] = d3.XYZ() * Math.Pow(Lambda, 3);
        }
        Cont[size] = Point.XYZ();

        NCollection_Array1<Point3D> ExtrapPoles = new(1, size);
        NCollection_Array1<Point3D> ExtraCoeffs = new(1, size);

        Point3D PNull = new(0.0, 0.0, 0.0);
        ExtraCoeffs.Init(PNull);
        for (ii = 1; ii <= size; ii++)
        {
            for (jj = 1; jj <= size; jj++)
            {
                ExtraCoeffs[jj].ChangeCoord() += MatCoefs[ii, jj] * Cont[ii];
            }
        }

        // Conversion to the Bernstein Basis
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(ExtraCoeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), ExtrapPoles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());

        Geom_BezierCurve Bezier = new(ExtrapPoles);

        double dist = ExtrapPoles[1].Distance(p0);
        bool Ok;
        Tol += dist;

        // Concatenation
        Ok = Concat.Add(Bezier, Tol, After);
        if (!Ok)
        {
            throw new ConstructionErrorException("ExtendCurveToPoint");
        }

        // The concatenation is discarded: its curve goes to the caller unfrozen (decision 244).
        Curve = Concat.InternalBSplineCurve()!;
    }

    //=================================================================================================
    private static bool ExtendKPart(ref Geom_RectangularTrimmedSurface? Surface,
                                    double Length,
                                    bool InU,
                                    bool After)
    {

        if (Surface is null)
        {
            return false;
        }

        bool Ok = true;
        double Uf, Ul, Vf, Vl;
        Geom_Surface Support = Surface.BasisSurface();
        SurfaceKind Type;

        Surface.Bounds(out Uf, out Ul, out Vf, out Vl);
        GeomAdaptor_Surface AS = new(Surface);
        Type = AS.GetType();

        if (InU)
        {
            switch (Type)
            {
                case Plane:
                {
                    if (After)
                    {
                        Ul += Length;
                    }
                    else
                    {
                        Uf -= Length;
                    }
                    Surface = new Geom_RectangularTrimmedSurface(Support, Uf, Ul, Vf, Vl);
                    break;
                }

                default:
                    Ok = false;
                    break;
            }
        }
        else
        {
            switch (Type)
            {
                case Plane:
                case SurfaceKind.Cylinder:
                case SurfaceOfExtrusion:
                {
                    if (After)
                    {
                        Vl += Length;
                    }
                    else
                    {
                        Vf -= Length;
                    }
                    Surface = new Geom_RectangularTrimmedSurface(Support, Uf, Ul, Vf, Vl);
                    break;
                }
                default:
                    Ok = false;
                    break;
            }
        }

        return Ok;
    }

    //=================================================================================================

    /// <summary>
    /// Extends the bounded surface by Length in U (InU) or V, after its end (After) or before its start, with
    /// continuity Continuity (0 to 3; nothing is done otherwise). Planes, cylinders and extrusions trimmed in the
    /// extended direction are re-trimmed; other surfaces become B-splines. Surface receives the new surface; the
    /// caller's surface is never changed (OCCT changes a B-spline surface in place: the port changes a copy).
    /// </summary>
    public static void ExtendSurfByLength(ref Geom_BoundedSurface Surface,
                                          double Length,
                                          int Continuity,
                                          bool InU,
                                          bool After)
    {
        if (Continuity < 0 || Continuity > 3)
        {
            return;
        }
        int Cont = Continuity;

        // Kpart ?
        Geom_RectangularTrimmedSurface? TS = Surface as Geom_RectangularTrimmedSurface;
        if (ExtendKPart(ref TS, Length, InU, After))
        {
            Surface = TS!;
            return;
        }

        //  BSplineSurface format with a degree sufficient for the desired continuity
        Geom_BSplineSurface? BS = Surface as Geom_BSplineSurface;
        if (BS is null)
        {
            // BS = GeomConvert::SurfaceToBSplineSurface(Surface);
            double Tol = global::Kerf.Engine.Foundation.Precision.Confusion(); // 1.e-4;
            Continuity UCont = C1, VCont = C1;
            int degU = 14, degV = 14;
            int nmax = 16;
            int thePrec = 1;
            Geom_Surface aSurf = Surface; // to resolve ambiguity
            GeomConvert_ApproxSurface theApprox = new(aSurf, Tol, UCont, VCont, degU, degV, nmax, thePrec);
            if (theApprox.HasResult())
            {
                BS = theApprox.Surface();
            }
            else
            {
                BS = GeomConvert.SurfaceToBSplineSurface(Surface);
            }
        }
        // OCCT changes the caller's B-spline surface (or ApproxSurface's frozen one) in place below: the port changes a
        // copy (decision 245), which the caller receives as OCCT's caller receives its own surface.
        BS = (Geom_BSplineSurface)BS!.Copy();
        if (InU && (BS.UDegree() < Continuity + 1))
        {
            BS.IncreaseDegree(Continuity + 1, BS.VDegree());
        }
        if (!InU && (BS.VDegree() < Continuity + 1))
        {
            BS.IncreaseDegree(BS.UDegree(), Continuity + 1);
        }

        // if BS was periodic in the extension direction, it will no longer be
        if ((InU && (BS.IsUPeriodic())) || (!InU && (BS.IsVPeriodic())))
        {
            double U0, U1, V0, V1;
            BS.Bounds(out U0, out U1, out V0, out V1);
            BS.Segment(U0, U1, V0, V1);
        }

        // IFV Fix OCC bug 0022694 - wrong result extrapolating rational surfaces
        //   bool rational = ( InU && BS->IsURational() )
        //                                   || ( !InU && BS->IsVRational() ) ;
        bool rational = (BS.IsURational() || BS.IsVRational());
        double EpsW = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        int gap = 3;
        if (rational)
        {
            gap++;
        }

        int Cdeg = 0, Cdim = 0, NbP = 0, Ksize = 0, Psize = 1;
        int ii, jj, ipole, Kount;
        double Tbord, lambmin = Length;
        bool Ok;
        NCollection_HArray1<double>? FKnots = null, Point = null, lambda = null, Tgte = null, Poles = null;
        Span<int> extrap_mode = stackalloc int[2];

        for (Kount = 0, Ok = false; Kount <= 2 && !Ok; Kount++)
        {
            //  Transform the surface into a non-rational BSpline in one variable
            //  of degree UDegree or VDegree and dimension 3 or 4 x NbVpoles or NbUpoles
            //  the number of poles equals NbUpoles or NbVpoles
            //  WARNING: in the rational case, a point with coordinates (x,y,z)
            //           and weight w becomes a point with coordinates (wx, wy, wz, w)

            if (InU)
            {
                Cdeg = BS.UDegree();
                NbP = BS.NbUPoles();
                Cdim = BS.NbVPoles() * gap;
            }
            else
            {
                Cdeg = BS.VDegree();
                NbP = BS.NbVPoles();
                Cdim = BS.NbUPoles() * gap;
            }

            //  the flat knots
            Ksize = NbP + Cdeg + 1;
            if (InU)
            {
                FKnots = new NCollection_HArray1<double>(BS.InternalUKnotSequence());
            }
            else
            {
                FKnots = new NCollection_HArray1<double>(BS.InternalVKnotSequence());
            }

            //  the parameter of the connection knot
            if (After)
            {
                Tbord = FKnots.Value(FKnots.Upper() - Cdeg);
            }
            else
            {
                Tbord = FKnots.Value(FKnots.Lower() + Cdeg);
            }

            //  the poles
            Psize = Cdim * NbP;
            Poles = new NCollection_HArray1<double>(1, Psize);

            if (InU)
            {
                for (ii = 1, ipole = 1; ii <= NbP; ii++)
                {
                    for (jj = 1; jj <= BS.NbVPoles(); jj++)
                    {
                        Poles.SetValue(ipole, BS.Pole(ii, jj).X());
                        Poles.SetValue(ipole + 1, BS.Pole(ii, jj).Y());
                        Poles.SetValue(ipole + 2, BS.Pole(ii, jj).Z());
                        if (rational)
                        {
                            Poles.SetValue(ipole + 3, BS.Weight(ii, jj));
                        }
                        ipole += gap;
                    }
                }
            }
            else
            {
                for (jj = 1, ipole = 1; jj <= NbP; jj++)
                {
                    for (ii = 1; ii <= BS.NbUPoles(); ii++)
                    {
                        Poles.SetValue(ipole, BS.Pole(ii, jj).X());
                        Poles.SetValue(ipole + 1, BS.Pole(ii, jj).Y());
                        Poles.SetValue(ipole + 2, BS.Pole(ii, jj).Z());
                        if (rational)
                        {
                            Poles.SetValue(ipole + 3, BS.Weight(ii, jj));
                        }
                        ipole += gap;
                    }
                }
            }
            // OCCT's Padr points at Poles(1): Poles.AsSpan() below

            //  calculation of the connection point and tangent
            Point = new NCollection_HArray1<double>(1, Cdim);
            Tgte = new NCollection_HArray1<double>(1, Cdim);
            lambda = new NCollection_HArray1<double>(1, Cdim);

            bool periodic_flag = false;
            int derivative_request = Max(Continuity, 1);
            extrap_mode[0] = extrap_mode[1] = Cdeg;
            NCollection_Array1<double> Result = new(1, Cdim * (derivative_request + 1));

            NCollection_Array1<double> tgte = Tgte.ChangeArray1();
            NCollection_Array1<double> point = Point.ChangeArray1();
            NCollection_Array1<double> lamb = lambda.ChangeArray1();

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(Tbord,
                          periodic_flag,
                          derivative_request,
                          extrap_mode,
                          Cdeg,
                          FKnots.Array1(),
                          Cdim,
                          Poles.AsSpan(),
                          Result.AsSpan());
            Ok = true;
            for (ii = 1; ii <= Cdim; ii++)
            {
                point[ii] = Result[ii];
                tgte[ii] = Result[ii + Cdim];
            }

            //  calculation of the constraint to reach

            Vector3D CurT = new(), OldT = new();

            double NTgte, val, Tgtol = 1.0e-12, OldN = 0.0;
            if (rational)
            {
                for (ii = gap; ii <= Cdim; ii += gap)
                {
                    tgte[ii] = 0.0;
                }
                for (ii = gap; ii <= Cdim; ii += gap)
                {
                    CurT.SetCoord(tgte[ii - 3], tgte[ii - 2], tgte[ii - 1]);
                    NTgte = CurT.Magnitude();
                    if (NTgte > Tgtol)
                    {
                        val = Length / NTgte;
                        // Attention to cases where the segment given by the poles
                        // is opposite to the direction of the derivative
                        // Example: Certain portions of torus.
                        if ((OldN > Tgtol) && (CurT.Angle(OldT) > 2))
                        {
                            Ok = false;
                        }

                        lamb[ii - 1] = lamb[ii - 2] = lamb[ii - 3] = val;
                        lamb[ii] = 0.0;
                        lambmin = Min(lambmin, val);
                    }
                    else
                    {
                        lamb[ii - 1] = lamb[ii - 2] = lamb[ii - 3] = 0.0;
                        lamb[ii] = 0.0;
                    }
                    OldT = CurT;
                    OldN = NTgte;
                }
            }
            else
            {
                for (ii = gap; ii <= Cdim; ii += gap)
                {
                    CurT.SetCoord(tgte[ii - 2], tgte[ii - 1], tgte[ii]);
                    NTgte = CurT.Magnitude();
                    if (NTgte > Tgtol)
                    {
                        val = Length / NTgte;
                        // Attention to cases where the segment given by the poles
                        // is opposite to the direction of the derivative
                        // Example: Certain portions of torus.
                        if ((OldN > Tgtol) && (CurT.Angle(OldT) > 2))
                        {
                            Ok = false;
                        }
                        lamb[ii] = lamb[ii - 1] = lamb[ii - 2] = val;
                        lambmin = Min(lambmin, val);
                    }
                    else
                    {
                        lamb[ii] = lamb[ii - 1] = lamb[ii - 2] = 0.0;
                    }
                    OldT = CurT;
                    OldN = NTgte;
                }
            }
            if (!Ok && Kount < 2)
            {
                // We increase the degree of the border iso to bring the poles of the surface closer
                // And we retry
                if (InU)
                {
                    BS.IncreaseDegree(BS.UDegree(), BS.VDegree() + 2);
                }
                else
                {
                    BS.IncreaseDegree(BS.UDegree() + 2, BS.VDegree());
                }
            }
        }

        NCollection_Array1<double> ConstraintPoint = new(1, Cdim);
        if (After)
        {
            for (ii = 1; ii <= Cdim; ii++)
            {
                ConstraintPoint[ii] = Point!.Value(ii) + lambda!.Value(ii) * Tgte!.Value(ii);
            }
        }
        else
        {
            for (ii = 1; ii <= Cdim; ii++)
            {
                ConstraintPoint[ii] = Point!.Value(ii) - lambda!.Value(ii) * Tgte!.Value(ii);
            }
        }

        //  special case of rational
        if (rational)
        {
            for (ipole = 1; ipole <= Psize; ipole += gap)
            {
                Poles![ipole] *= Poles.Value(ipole + 3);
                Poles[ipole + 1] *= Poles.Value(ipole + 3);
                Poles[ipole + 2] *= Poles.Value(ipole + 3);
            }
            for (ii = 1; ii <= Cdim; ii += gap)
            {
                ConstraintPoint[ii] *= ConstraintPoint[ii + 3];
                ConstraintPoint[ii + 1] *= ConstraintPoint[ii + 3];
                ConstraintPoint[ii + 2] *= ConstraintPoint[ii + 3];
            }
        }

        //  arrays needed for the extension
        int Ksize2 = Ksize + Cdeg, NbPoles, NbKnots = 0;
        NCollection_Array1<double> FK = new(1, Ksize2);

        int Psize2 = Psize + Cdeg * Cdim;
        NCollection_Array1<double> PRes = new(1, Psize2);
        double ww;
        bool ExtOk = false;
        NCollection_HArray2<Point3D>? NewPoles = null;
        NCollection_HArray2<double>? NewWeights = null;

        for (Kount = 1; Kount <= 5 && !ExtOk; Kount++)
        {
            //  extension
            global::Kerf.Engine.Geometry.Splines.BSplCLib.TangExtendToConstraint(FKnots!.Array1(),
                                            lambmin,
                                            NbP,
                                            Poles!.AsSpan(),
                                            Cdim,
                                            Cdeg,
                                            ConstraintPoint,
                                            Cont,
                                            After,
                                            out NbPoles,
                                            out NbKnots,
                                            FK.AsSpan(),
                                            PRes.AsSpan());

            //  Copy the result poles as 3D points and weights
            int NU, NV, indice;
            if (InU)
            {
                NU = NbPoles;
                NV = BS.NbVPoles();
            }
            else
            {
                NU = BS.NbUPoles();
                NV = NbPoles;
            }

            NewPoles = new NCollection_HArray2<Point3D>(1, NU, 1, NV);
            NCollection_Array2<Point3D> NewP = NewPoles.ChangeArray2();
            NewWeights = new NCollection_HArray2<double>(1, NU, 1, NV);
            NCollection_Array2<double> NewW = NewWeights.ChangeArray2();

            if (!rational)
            {
                NewW.Init(1.0);
            }
            bool NullWeight = false;

            if (InU)
            {
                for (ii = 1; ii <= NU && !NullWeight; ii++)
                {
                    for (jj = 1; jj <= NV && !NullWeight; jj++)
                    {
                        indice = 1 + (ii - 1) * Cdim + (jj - 1) * gap;
                        NewP[ii, jj].SetCoord(1, PRes[indice]);
                        NewP[ii, jj].SetCoord(2, PRes[indice + 1]);
                        NewP[ii, jj].SetCoord(3, PRes[indice + 2]);
                        if (rational)
                        {
                            ww = PRes[indice + 3];
                            if (Math.Abs(ww - 1.0) < EpsW)
                            {
                                ww = 1.0;
                            }
                            if (ww < EpsW)
                            {
                                NullWeight = true;
                            }
                            else
                            {
                                NewW[ii, jj] = ww;
                                NewP[ii, jj].ChangeCoord() /= ww;
                            }
                        }
                    }
                }
            }
            else
            {
                for (jj = 1; jj <= NV && !NullWeight; jj++)
                {
                    for (ii = 1; ii <= NU && !NullWeight; ii++)
                    {
                        indice = 1 + (ii - 1) * gap + (jj - 1) * Cdim;
                        NewP[ii, jj].SetCoord(1, PRes[indice]);
                        NewP[ii, jj].SetCoord(2, PRes[indice + 1]);
                        NewP[ii, jj].SetCoord(3, PRes[indice + 2]);
                        if (rational)
                        {
                            ww = PRes[indice + 3];
                            if (Math.Abs(ww - 1.0) < EpsW)
                            {
                                ww = 1.0;
                            }
                            if (ww < EpsW)
                            {
                                NullWeight = true;
                            }
                            else
                            {
                                NewW[ii, jj] = ww;
                                NewP[ii, jj].ChangeCoord() /= ww;
                            }
                        }
                    }
                }
            }

            if (NullWeight)
            {
                lambmin /= 3.0;
            }
            else
            {
                ExtOk = true;
            }
        }

        // Copy flat knots as knots with their multiplicities.
        // Compute the degrees of the result.
        int Usize = BS.NbUKnots(), Vsize = BS.NbVKnots(), UDeg, VDeg;
        if (InU)
        {
            Usize++;
        }
        else
        {
            Vsize++;
        }
        NCollection_Array1<double> UKnots = new(1, Usize);
        NCollection_Array1<double> VKnots = new(1, Vsize);
        NCollection_Array1<int> UMults = new(1, Usize);
        NCollection_Array1<int> VMults = new(1, Vsize);
        NCollection_Array1<double> FKRes = new(1, NbKnots);

        for (ii = 1; ii <= NbKnots; ii++)
        {
            FKRes[ii] = FK[ii];
        }

        if (InU)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Knots(FKRes, UKnots, UMults);
            UDeg = Cdeg;
            UMults[Usize] = UDeg + 1; // Petite verrue utile quand la continuite
                                      // n'est pas ok.
            VKnots.Assign(BS.InternalVKnots());
            VMults.Assign(BS.InternalVMultiplicities());
            VDeg = BS.VDegree();
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Knots(FKRes, VKnots, VMults);
            VDeg = Cdeg;
            VMults[Vsize] = VDeg + 1;
            UKnots.Assign(BS.InternalUKnots());
            UMults.Assign(BS.InternalUMultiplicities());
            UDeg = BS.UDegree();
        }

        //  Construct the resulting BSpline surface
        Geom_BSplineSurface Res = new(NewPoles!.Array2(),
                                      NewWeights!.Array2(),
                                      UKnots,
                                      VKnots,
                                      UMults,
                                      VMults,
                                      UDeg,
                                      VDeg,
                                      BS.IsUPeriodic(),
                                      BS.IsVPeriodic());
        Surface = Res;
    }

    //=================================================================================================

    /// <summary>
    /// Computes the principal axes of inertia of some points and their dispersions: XDir is the axis of largest
    /// inertia, YDir the next, and Xgap, Ygap, Zgap the square roots of the three eigenvalues.
    /// </summary>
    public static void Inertia(NCollection_Array1<Point3D> Points,
                               out Point3D Bary,
                               out Direction3D XDir,
                               out Direction3D YDir,
                               out double Xgap,
                               out double Ygap,
                               out double Zgap)
    {
        Xyz GB = new(0.0, 0.0, 0.0), Diff = new();
        //  gp_Vec A,B,C,D;

        int i, nb = Points.Length();
        GB.SetCoord(0.0, 0.0, 0.0);
        for (i = 1; i <= nb; i++)
        {
            GB += Points[i].XYZ();
        }

        GB /= nb;

        Matrix M = new(1, 3, 1, 3);
        M.Init(0.0);
        for (i = 1; i <= nb; i++)
        {
            Diff.SetLinearForm(-1, Points[i].XYZ(), GB);
            M[1, 1] += Diff.X() * Diff.X();
            M[2, 2] += Diff.Y() * Diff.Y();
            M[3, 3] += Diff.Z() * Diff.Z();
            M[1, 2] += Diff.X() * Diff.Y();
            M[1, 3] += Diff.X() * Diff.Z();
            M[2, 3] += Diff.Y() * Diff.Z();
        }

        M[2, 1] = M[1, 2];
        M[3, 1] = M[1, 3];
        M[3, 2] = M[2, 3];

        M /= nb;

        Jacobi J = new(M);
        // OCCT reports a failure of Jacobi in debug builds only.

        double n1, n2, n3;

        n1 = J.Value(1);
        n2 = J.Value(2);
        n3 = J.Value(3);

        double r1 = Min(Min(n1, n2), n3), r2;
        int m1, m2, m3;
        if (r1 == n1)
        {
            m1 = 1;
            r2 = Min(n2, n3);
            if (r2 == n2)
            {
                m2 = 2;
                m3 = 3;
            }
            else
            {
                m2 = 3;
                m3 = 2;
            }
        }
        else
        {
            if (r1 == n2)
            {
                m1 = 2;
                r2 = Min(n1, n3);
                if (r2 == n1)
                {
                    m2 = 1;
                    m3 = 3;
                }
                else
                {
                    m2 = 3;
                    m3 = 1;
                }
            }
            else
            {
                m1 = 3;
                r2 = Min(n1, n2);
                if (r2 == n1)
                {
                    m2 = 1;
                    m3 = 2;
                }
                else
                {
                    m2 = 2;
                    m3 = 1;
                }
            }
        }

        math_Vector V2 = new(1, 3), V3 = new(1, 3);
        J.Vector(m2, V2);
        J.Vector(m3, V3);

        Bary = new Point3D();
        Bary.SetXYZ(GB);
        XDir = new Direction3D();
        XDir.SetCoord(V3[1], V3[2], V3[3]);
        YDir = new Direction3D();
        YDir.SetCoord(V2[1], V2[2], V2[3]);

        Zgap = Math.Sqrt(Math.Abs(J.Value(m1)));
        Ygap = Math.Sqrt(Math.Abs(J.Value(m2)));
        Xgap = Math.Sqrt(Math.Abs(J.Value(m3)));
    }

    /// <summary>
    /// Computes the axes of inertia of some points: the location is their barycentre, the X direction the axis of
    /// largest inertia, the main direction the normal to the mean plane. IsSingular is true if the points are on a
    /// line (within Tol).
    /// </summary>
    public static void AxeOfInertia(NCollection_Array1<Point3D> Points,
                                    out Frame3D Axe,
                                    out bool IsSingular,
                                    double Tol = 1.0e-7)
    {
        Point3D Bary;
        Direction3D OX, OY, OZ;
        double gx, gy, gz;

        GeomLib.Inertia(Points, out Bary, out OX, out OY, out gx, out gy, out gz);

        if (gy * Points.Length() <= Tol)
        {
            Frame3D axe = new(Bary, OX);
            OY = axe.XDirection();
            IsSingular = true;
        }
        else
        {
            IsSingular = false;
        }

        OZ = OX ^ OY;
        Frame3D TheAxe = new(Bary, OZ, OX);
        Axe = TheAxe;
    }

    //=================================================================================================

    private static bool CanBeTreated(Geom_BSplineSurface BSurf)

    {
        int i;
        double lambda; // proportionnality coefficient
        bool AlreadyTreated = true;

        if (!BSurf.IsURational() || (BSurf.IsUPeriodic()))
        {
            return false;
        }
        else
        {
            lambda = (BSurf.Weight(1, 1) / BSurf.Weight(BSurf.NbUPoles(), 1));
            for (i = 1; i <= BSurf.NbVPoles(); i++)
            { //test of the proportionnality of the denominator on the boundaries
                if ((BSurf.Weight(1, i) / (lambda * BSurf.Weight(BSurf.NbUPoles(), i))
                     < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                    || (BSurf.Weight(1, i) / (lambda * BSurf.Weight(BSurf.NbUPoles(), i))
                        > (1 + global::Kerf.Engine.Foundation.Precision.Confusion())))
                {
                    return false;
                }
            }
            i = 1;
            while ((AlreadyTreated) && (i <= BSurf.NbVPoles()))
            { // tests if the surface has already been treated
                if (((BSurf.Weight(1, i) / (BSurf.Weight(2, i))) < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                    || ((BSurf.Weight(1, i) / (BSurf.Weight(2, i))) > (1 + global::Kerf.Engine.Foundation.Precision.Confusion()))
                    || ((BSurf.Weight(BSurf.NbUPoles() - 1, i) / (BSurf.Weight(BSurf.NbUPoles(), i)))
                        < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                    || ((BSurf.Weight(BSurf.NbUPoles() - 1, i) / (BSurf.Weight(BSurf.NbUPoles(), i)))
                        > (1 + global::Kerf.Engine.Foundation.Precision.Confusion())))
                {
                    AlreadyTreated = false;
                }
                i++;
            }
            if (AlreadyTreated)
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    private sealed class law_evaluator : SurfaceEvaluatorFunction
    {
        public law_evaluator(GeomLib_DenominatorMultiplier? theDenominatorPtr)
        {
            myDenominator = theDenominatorPtr;
        }

        public override void Evaluate(int theDerivativeRequest,
                                      double theUParameter,
                                      double theVParameter,
                                      out double theResult,
                                      out int theErrorCode)
        {
            if ((myDenominator is not null) && (theDerivativeRequest == 0))
            {
                theResult = myDenominator.Value(theUParameter, theVParameter);
                theErrorCode = 0;
            }
            else
            {
                // OCCT leaves the result unset (decision 32)
                theResult = 0.0;
                theErrorCode = 1;
            }
        }

        private readonly GeomLib_DenominatorMultiplier? myDenominator;
    }

    //=================================================================================================

    private static bool CheckIfKnotExists(NCollection_Array1<double> surface_knots, double knot)

    {
        int i;
        for (i = 1; i <= surface_knots.Length(); i++)
        {
            if ((surface_knots[i] - global::Kerf.Engine.Foundation.Precision.Confusion() <= knot)
                && (surface_knots[i] + global::Kerf.Engine.Foundation.Precision.Confusion() >= knot))
            {
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    private static void AddAKnot(NCollection_Array1<double> knots,
                                 NCollection_Array1<int> mults,
                                 double knotinserted,
                                 int deltasurface_degree,
                                 int finalsurfacedegree,
                                 out NCollection_HArray1<double> newknots,
                                 out NCollection_HArray1<int> newmults)

    {
        int i;

        newknots = new NCollection_HArray1<double>(1, knots.Length() + 1);
        newmults = new NCollection_HArray1<int>(1, knots.Length() + 1);
        i = 1;
        while (knots[i] < knotinserted)
        {
            newknots.SetValue(i, knots[i]);
            newmults.SetValue(i, mults[i] + deltasurface_degree);
            i++;
        }
        newknots.SetValue(i, knotinserted); // insertion of the new knot
        newmults.SetValue(i, finalsurfacedegree - 2);
        i++;
        while (i <= newknots.Length())
        {
            newknots.SetValue(i, knots[i - 1]);
            newmults.SetValue(i, mults[i - 1] + deltasurface_degree);
            i++;
        }
    }

    //=================================================================================================

    private static void BuildFlatKnot(NCollection_Array1<double> surface_knots,
                                      NCollection_Array1<int> surface_mults,
                                      int deltasurface_degree,
                                      int finalsurface_degree,
                                      double knotmin,
                                      double knotmax,
                                      out NCollection_HArray1<double> ResultKnots,
                                      out NCollection_HArray1<int> ResultMults)

    {
        int i;

        if (CheckIfKnotExists(surface_knots, knotmin) && CheckIfKnotExists(surface_knots, knotmax))
        {
            ResultKnots = new NCollection_HArray1<double>(1, surface_knots.Length());
            ResultMults = new NCollection_HArray1<int>(1, surface_knots.Length());
            for (i = 1; i <= surface_knots.Length(); i++)
            {
                ResultKnots.SetValue(i, surface_knots[i]);
                ResultMults.SetValue(i, surface_mults[i] + deltasurface_degree);
            }
        }
        else
        {
            if ((CheckIfKnotExists(surface_knots, knotmin)) && (!CheckIfKnotExists(surface_knots, knotmax)))
            {
                AddAKnot(surface_knots,
                         surface_mults,
                         knotmax,
                         deltasurface_degree,
                         finalsurface_degree,
                         out ResultKnots,
                         out ResultMults);
            }
            else
            {
                if ((!CheckIfKnotExists(surface_knots, knotmin))
                    && (CheckIfKnotExists(surface_knots, knotmax)))
                {
                    AddAKnot(surface_knots,
                             surface_mults,
                             knotmin,
                             deltasurface_degree,
                             finalsurface_degree,
                             out ResultKnots,
                             out ResultMults);
                }
                else
                {
                    if ((!CheckIfKnotExists(surface_knots, knotmin))
                        && (!CheckIfKnotExists(surface_knots, knotmax)) && (knotmin == knotmax))
                    {
                        AddAKnot(surface_knots,
                                 surface_mults,
                                 knotmin,
                                 deltasurface_degree,
                                 finalsurface_degree,
                                 out ResultKnots,
                                 out ResultMults);
                    }
                    else
                    {
                        NCollection_HArray1<double> IntermedKnots;
                        NCollection_HArray1<int> IntermedMults;
                        AddAKnot(surface_knots,
                                 surface_mults,
                                 knotmin,
                                 deltasurface_degree,
                                 finalsurface_degree,
                                 out IntermedKnots,
                                 out IntermedMults);
                        AddAKnot(IntermedKnots.ChangeArray1(),
                                 IntermedMults.ChangeArray1(),
                                 knotmax,
                                 0,
                                 finalsurface_degree,
                                 out ResultKnots,
                                 out ResultMults);
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static void FunctionMultiply(ref Geom_BSplineSurface BSurf,
                                         double knotmin,
                                         double knotmax)

    {
        NCollection_Array1<double> surface_u_knots = BSurf.InternalUKnots();
        NCollection_Array1<int> surface_u_mults = BSurf.InternalUMultiplicities();
        NCollection_Array1<double> surface_v_knots = BSurf.InternalVKnots();
        NCollection_Array1<int> surface_v_mults = BSurf.InternalVMultiplicities();
        NCollection_Array2<Point3D> surface_poles = BSurf.InternalPoles();
        NCollection_Array2<double>? surface_weights = BSurf.InternalWeights();
        int i, j, k, status, new_num_u_poles, new_num_v_poles, length = 0;
        NCollection_HArray1<double> newuknots, newvknots;
        NCollection_HArray1<int> newumults, newvmults;

        NCollection_Array1<double> Knots = new(1, 2);
        NCollection_Array1<int> Mults = new(1, 2);
        NCollection_HArray1<double> NewKnots;
        NCollection_HArray1<int> NewMults;

        Knots[1] = 0;
        Knots[2] = 1;
        Mults[1] = 4;
        Mults[2] = 4;
        BuildFlatKnot(Knots, Mults, 0, 3, knotmin, knotmax, out NewKnots, out NewMults);

        for (i = 1; i <= NewMults.Length(); i++)
        {
            length += NewMults.Value(i);
        }
        NCollection_Array1<double> FlatKnots = new(1, length);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(NewKnots.ChangeArray1(), NewMults.ChangeArray1(), FlatKnots);

        GeomLib_DenominatorMultiplier aDenominator = new(BSurf, FlatKnots);

        BuildFlatKnot(surface_u_knots,
                      surface_u_mults,
                      3,
                      BSurf.UDegree() + 3,
                      knotmin,
                      knotmax,
                      out newuknots,
                      out newumults);
        BuildFlatKnot(surface_v_knots,
                      surface_v_mults,
                      BSurf.VDegree(),
                      2 * (BSurf.VDegree()),
                      1.0,
                      0.0,
                      out newvknots,
                      out newvmults);
        length = 0;
        for (i = 1; i <= newumults.Length(); i++)
        {
            length += newumults.Value(i);
        }
        new_num_u_poles = (length - BSurf.UDegree() - 3 - 1);
        NCollection_Array1<double> newuflatknots = new(1, length);
        length = 0;
        for (i = 1; i <= newvmults.Length(); i++)
        {
            length += newvmults.Value(i);
        }
        new_num_v_poles = (length - 2 * BSurf.VDegree() - 1);
        NCollection_Array1<double> newvflatknots = new(1, length);

        NCollection_Array2<Point3D> NewNumerator = new(1, new_num_u_poles, 1, new_num_v_poles);
        NCollection_Array2<double> NewDenominator = new(1, new_num_u_poles, 1, new_num_v_poles);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(newuknots.ChangeArray1(), newumults.ChangeArray1(), newuflatknots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(newvknots.ChangeArray1(), newvmults.ChangeArray1(), newvflatknots);
        // POP for WNT
        law_evaluator ev = new(aDenominator);
        // BSplSLib::FunctionMultiply(law_evaluator,               //multiplication
        SurfaceBSplSLib.FunctionMultiply(ev, // multiplication
                                  BSurf.UDegree(),
                                  BSurf.VDegree(),
                                  surface_u_knots,
                                  surface_v_knots,
                                  surface_u_mults,
                                  surface_v_mults,
                                  surface_poles,
                                  surface_weights,
                                  newuflatknots,
                                  newvflatknots,
                                  BSurf.UDegree() + 3,
                                  2 * (BSurf.VDegree()),
                                  NewNumerator,
                                  NewDenominator,
                                  out status);
        if (status != 0)
        {
            throw new ConstructionErrorException("GeomLib Multiplication Error");
        }
        for (i = 1; i <= new_num_u_poles; i++)
        {
            for (j = 1; j <= new_num_v_poles; j++)
            {
                for (k = 1; k <= 3; k++)
                {
                    NewNumerator[i, j].SetCoord(k, NewNumerator[i, j].Coord(k) / NewDenominator[i, j]);
                }
            }
        }
        BSurf = new Geom_BSplineSurface(NewNumerator,
                                        NewDenominator,
                                        newuknots.ChangeArray1(),
                                        newvknots.ChangeArray1(),
                                        newumults.ChangeArray1(),
                                        newvmults.ChangeArray1(),
                                        BSurf.UDegree() + 3,
                                        2 * (BSurf.VDegree()));
    }

    //=================================================================================================

    private static void CancelDenominatorDerivative1D(ref Geom_BSplineSurface BSurf)

    {
        int i, j;
        double uknotmin = 1.0, uknotmax = 0.0, x, y, startu_value, endu_value;

        startu_value = BSurf.UKnot(1);
        endu_value = BSurf.UKnot(BSurf.NbUKnots());
        NCollection_Array1<double> BSurf_u_knots = new(BSurf.InternalUKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, BSurf_u_knots);
        BSurf.SetUKnots(BSurf_u_knots); // reparametrisation of the surface
        Geom_BSplineCurve BCurve;
        NCollection_Array1<double> BCurveWeights = new(1, BSurf.NbUPoles());
        NCollection_Array1<Point3D> BCurvePoles = new(1, BSurf.NbUPoles());

        // OCCT: when the surface can't be treated, it is left reparametrised to [0, 1]: the knots are restored only
        // after FunctionMultiply (occt-issues.md).
        if (CanBeTreated(BSurf))
        {
            NCollection_Array1<double> BCurveKnots = BSurf.InternalUKnots();
            NCollection_Array1<int> BCurveMults = BSurf.InternalUMultiplicities();
            for (i = 1; i <= BSurf.NbVPoles(); i++)
            { // loop on each pole function
                x = 1.0;
                y = 0.0;
                for (j = 1; j <= BSurf.NbUPoles(); j++)
                {
                    BCurveWeights[j] = BSurf.Weight(j, i);
                    BCurvePoles[j] = BSurf.Pole(j, i);
                }
                BCurve = new Geom_BSplineCurve(BCurvePoles, // building of a pole function
                                               BCurveWeights,
                                               BCurveKnots,
                                               BCurveMults,
                                               BSurf.UDegree());
                Hermit.Solutionbis(BCurve, ref x, ref y, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion());
                if (x < uknotmin)
                {
                    uknotmin = x; // uknotmin,uknotmax:extremal knots
                }
                if ((x != 1.0) && (x > uknotmax))
                {
                    uknotmax = x;
                }
                if ((y != 0.0) && (y < uknotmin))
                {
                    uknotmin = y;
                }
                if (y > uknotmax)
                {
                    uknotmax = y;
                }
            }

            FunctionMultiply(ref BSurf, uknotmin, uknotmax); // multiplication

            BSurf_u_knots.Assign(BSurf.InternalUKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(startu_value, endu_value, BSurf_u_knots);
            BSurf.SetUKnots(BSurf_u_knots);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Cancels, on the boundaries, the first derivative of the denominator of a rational surface in the requested
    /// directions, and sets its value to 1. BSurf receives a new surface; the caller's is never changed (OCCT changes
    /// it in place: the port changes a copy).
    /// </summary>
    public static void CancelDenominatorDerivative(ref Geom_BSplineSurface BSurf,
                                                   bool udirection,
                                                   bool vdirection)

    {
        if (udirection || vdirection)
        {
            // OCCT changes the caller's surface in place: the port changes a copy (decision 245).
            BSurf = (Geom_BSplineSurface)BSurf.Copy();
        }
        if (udirection && !vdirection)
        {
            CancelDenominatorDerivative1D(ref BSurf);
        }
        else
        {
            if (!udirection && vdirection)
            {
                BSurf.ExchangeUV();
                CancelDenominatorDerivative1D(ref BSurf);
                BSurf.ExchangeUV();
            }
            else
            {
                if (udirection && vdirection)
                { // optimize the treatment
                    if (BSurf.UDegree() <= BSurf.VDegree())
                    {
                        CancelDenominatorDerivative1D(ref BSurf);
                        BSurf.ExchangeUV();
                        CancelDenominatorDerivative1D(ref BSurf);
                        BSurf.ExchangeUV();
                    }
                    else
                    {
                        BSurf.ExchangeUV();
                        CancelDenominatorDerivative1D(ref BSurf);
                        BSurf.ExchangeUV();
                        CancelDenominatorDerivative1D(ref BSurf);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Estimates the normal of the surface at (U, V): 0 if from D1, 1 if from D2 (quasi-singular point), 2 or more
    /// when it fails (undefined or infinitely many normals). theNorm is left unchanged on failure.
    /// </summary>
    public static int NormEstim(Geom_Surface theSurf,
                                in Point2D theUV,
                                double theTol,
                                ref Direction3D theNorm)
    {
        double aTol2 = Square(theTol);

        Vector3D DU, DV;
        Point3D aDummyPnt;
        theSurf.D1(theUV.X(), theUV.Y(), out aDummyPnt, out DU, out DV);

        double MDU = DU.SquareMagnitude(), MDV = DV.SquareMagnitude();
        if (MDU >= aTol2 && MDV >= aTol2)
        {
            Vector3D aNorm = DU ^ DV;
            double aMagn = aNorm.SquareMagnitude();
            if (aMagn < aTol2)
            {
                return 3;
            }

            theNorm.SetXYZ(aNorm.XYZ());
            return 0;
        }

        Vector3D D2U, D2V, D2UV;
        bool isDone = false;
        NormalStatus aStatus;
        Direction3D aNormal = new();

        theSurf.D2(theUV.X(), theUV.Y(), out aDummyPnt, out DU, out DV, out D2U, out D2V, out D2UV);
        global::Kerf.Engine.Geometry.CSLib.Normal(DU, DV, D2U, D2V, D2UV, theTol, out isDone, out aStatus, ref aNormal);
        if (!isDone)
        {
            // computation is impossible
            return aStatus == D1NIsNull ? 2 : 3;
        }

        double Umin, Umax, Vmin, Vmax;
        double step = 1.0e-5;
        double eps = 1.0e-16;
        double sign = -1.0;
        theSurf.Bounds(out Umin, out Umax, out Vmin, out Vmax);

        // check for cone apex singularity point
        if ((theUV.Y() > Vmin + step) && (theUV.Y() < Vmax - step))
        {
            Direction3D aNormal1, aNormal2;
            double aConeSingularityAngleEps = 1.0e-4;
            theSurf.D1(theUV.X(), theUV.Y() - sign * step, out aDummyPnt, out DU, out DV);
            if ((DU.XYZ().SquareModulus() > eps) && (DV.XYZ().SquareModulus() > eps))
            {
                aNormal1 = new Direction3D(DU ^ DV);
                theSurf.D1(theUV.X(), theUV.Y() + sign * step, out aDummyPnt, out DU, out DV);
                if ((DU.XYZ().SquareModulus() > eps) && (DV.XYZ().SquareModulus() > eps))
                {
                    aNormal2 = new Direction3D(DU ^ DV);
                    if (aNormal1.IsOpposite(aNormal2, aConeSingularityAngleEps))
                    {
                        return 2;
                    }
                }
            }
        }

        // Along V
        if (MDU < aTol2 && MDV >= aTol2)
        {
            if ((Vmax - theUV.Y()) > (theUV.Y() - Vmin))
            {
                sign = 1.0;
            }

            theSurf.D1(theUV.X(), theUV.Y() + sign * step, out aDummyPnt, out DU, out DV);
            Vector3D Norm = DU ^ DV;
            if (Norm.SquareMagnitude() < eps)
            {
                double sign1 = -1.0;
                if ((Umax - theUV.X()) > (theUV.X() - Umin))
                {
                    sign1 = 1.0;
                }
                theSurf.D1(theUV.X() + sign1 * step, theUV.Y() + sign * step, out aDummyPnt, out DU, out DV);
                Norm = DU ^ DV;
            }
            if (Norm.SquareMagnitude() >= eps && Norm.Dot(new Vector3D(aNormal)) < 0.0)
            {
                aNormal.Reverse();
            }
        }

        // Along U
        if (MDV < aTol2 && MDU >= aTol2)
        {
            if ((Umax - theUV.X()) > (theUV.X() - Umin))
            {
                sign = 1.0;
            }

            theSurf.D1(theUV.X() + sign * step, theUV.Y(), out aDummyPnt, out DU, out DV);
            Vector3D Norm = DU ^ DV;
            if (Norm.SquareMagnitude() < eps)
            {
                double sign1 = -1.0;
                if ((Vmax - theUV.Y()) > (theUV.Y() - Vmin))
                {
                    sign1 = 1.0;
                }

                theSurf.D1(theUV.X() + sign * step, theUV.Y() + sign1 * step, out aDummyPnt, out DU, out DV);
                Norm = DU ^ DV;
            }
            if (Norm.SquareMagnitude() >= eps && Norm.Dot(new Vector3D(aNormal)) < 0.0)
            {
                aNormal.Reverse();
            }
        }

        // quasysingular
        if (aStatus == D1NuIsNull || aStatus == D1NvIsNull
            || aStatus == D1NuIsParallelD1Nv)
        {
            theNorm.SetXYZ(aNormal.XYZ());
            return 1;
        }

        return aStatus == InfinityOfSolutions ? 2 : 3;
    }

    //=================================================================================================

    /// <summary>
    /// Whether the opposite boundaries of the surface coincide within Tol, in U and in V.
    /// </summary>
    public static void IsClosed(Geom_Surface S,
                                double Tol,
                                out bool isUClosed,
                                out bool isVClosed)
    {
        isUClosed = false;
        isVClosed = false;
        //
        GeomAdaptor_Surface aGAS = new(S);
        SurfaceKind aSType = aGAS.GetType();
        //
        double u1, u2, v1, v2;
        u1 = aGAS.FirstUParameter();
        u2 = aGAS.LastUParameter();
        v1 = aGAS.FirstVParameter();
        v2 = aGAS.LastVParameter();
        //
        double Tol2 = Tol * Tol;
        switch (aSType)
        {
            case Plane:
            {
                return;
            }
            case SurfaceOfExtrusion:
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
                {
                    // not closed
                    return;
                }
            }
                goto case SurfaceKind.Cylinder;
            case SurfaceKind.Cylinder:
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1))
                {
                    v1 = 0.0;
                }
                Point3D p1 = aGAS.Value(u1, v1);
                Point3D p2 = aGAS.Value(u2, v1);
                isUClosed = p1.SquareDistance(p2) <= Tol2;
                return;
            }
            case SurfaceKind.Cone:
            {
                // find v with maximal distance from axis
                if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2)))
                {
                    Cone aCone = aGAS.Cone();
                    Point3D anApex = aCone.Apex();
                    Point3D P1 = aGAS.Value(u1, v1);
                    Point3D P2 = aGAS.Value(u1, v2);
                    if (P2.SquareDistance(anApex) > P1.SquareDistance(anApex))
                    {
                        v1 = v2;
                    }
                }
                else
                {
                    v1 = 0.0;
                }
                Point3D p1 = aGAS.Value(u1, v1);
                Point3D p2 = aGAS.Value(u2, v1);
                isUClosed = p1.SquareDistance(p2) <= Tol2;
                return;
            }
            case SurfaceKind.Sphere:
            {
                // find v with maximal distance from axis
                if (v1 * v2 <= 0.0)
                {
                    v1 = 0.0;
                }
                else
                {
                    if (v1 < 0.0)
                    {
                        v1 = v2;
                    }
                }
                Point3D p1 = aGAS.Value(u1, v1);
                Point3D p2 = aGAS.Value(u2, v1);
                isUClosed = p1.SquareDistance(p2) <= Tol2;
                return;
            }
            case SurfaceKind.Torus:
            {
                double ures = aGAS.UResolution(Tol);
                double vres = aGAS.VResolution(Tol);
                //
                isUClosed = (u2 - u1) >= aGAS.UPeriod() - ures;
                isVClosed = (v2 - v1) >= aGAS.VPeriod() - vres;
                return;
            }
            case BSplineSurface:
            {
                Geom_BSplineSurface aBSpl = aGAS.BSpline();
                isUClosed = GeomLib.IsBSplUClosed(aBSpl, u1, u2, Tol);
                isVClosed = GeomLib.IsBSplVClosed(aBSpl, v1, v2, Tol);
                return;
            }
            case BezierSurface:
            {
                Geom_BezierSurface aBz = aGAS.Bezier();
                isUClosed = GeomLib.IsBzUClosed(aBz, u1, u2, Tol);
                isVClosed = GeomLib.IsBzVClosed(aBz, v1, v2, Tol);
                return;
            }
            case SurfaceOfRevolution:
            case OffsetSurface:
            case OtherSurface:
            {
                int nbp = 23;
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1))
                {
                    v1 = Sign(1.0, v1);
                }
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
                {
                    v2 = Sign(1.0, v2);
                }
                //
                if (aSType == OffsetSurface || aSType == OtherSurface)
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1))
                    {
                        u1 = Sign(1.0, u1);
                    }
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
                    {
                        u2 = Sign(1.0, u2);
                    }
                }
                isUClosed = true;
                double dt = (v2 - v1) / (nbp - 1);
                double res = Max(aGAS.UResolution(Tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (dt <= res)
                {
                    nbp = RealToInt((v2 - v1) / (2.0 * res)) + 1;
                    nbp = Max(nbp, 2);
                    dt = (v2 - v1) / (nbp - 1);
                }
                double t;
                int i;
                for (i = 0; i < nbp; ++i)
                {
                    t = (i == nbp - 1 ? v2 : v1 + i * dt);
                    Point3D p1 = aGAS.Value(u1, t);
                    Point3D p2 = aGAS.Value(u2, t);
                    if (p1.SquareDistance(p2) > Tol2)
                    {
                        isUClosed = false;
                        break;
                    }
                }
                //
                nbp = 23;
                isVClosed = true;
                dt = (u2 - u1) / (nbp - 1);
                res = Max(aGAS.VResolution(Tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (dt <= res)
                {
                    nbp = RealToInt((u2 - u1) / (2.0 * res)) + 1;
                    nbp = Max(nbp, 2);
                    dt = (u2 - u1) / (nbp - 1);
                }
                for (i = 0; i < nbp; ++i)
                {
                    t = (i == nbp - 1 ? u2 : u1 + i * dt);
                    Point3D p1 = aGAS.Value(t, v1);
                    Point3D p2 = aGAS.Value(t, v2);
                    if (p1.SquareDistance(p2) > Tol2)
                    {
                        isVClosed = false;
                        break;
                    }
                }
                return;
            }
            default:
            {
                return;
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Whether the poles of the U1 and U2 isolines of the surface are equal within Tol (for rational surfaces, the
    /// weighted poles Weights(i)·Poles(i)).
    /// </summary>
    public static bool IsBSplUClosed(Geom_BSplineSurface S,
                                     double U1,
                                     double U2,
                                     double Tol)
    {
        Geom_Curve? aCUF = S.UIso(U1);
        Geom_Curve? aCUL = S.UIso(U2);
        if (aCUF is null || aCUL is null)
        {
            return false;
        }
        double Tol2 = 2.0 * Tol;
        Geom_BSplineCurve aBsF = (Geom_BSplineCurve)aCUF;
        Geom_BSplineCurve aBsL = (Geom_BSplineCurve)aCUL;
        NCollection_Array1<Point3D> aPF = aBsF.InternalPoles();
        NCollection_Array1<Point3D> aPL = aBsL.InternalPoles();
        NCollection_Array1<double>? WF = aBsF.InternalWeights();
        NCollection_Array1<double>? WL = aBsL.InternalWeights();
        return CompareWeightPoles(aPF, WF, aPL, WL, Tol2);
    }

    //=================================================================================================

    /// <summary>
    /// Whether the poles of the V1 and V2 isolines of the surface are equal within Tol (for rational surfaces, the
    /// weighted poles Weights(i)·Poles(i)).
    /// </summary>
    public static bool IsBSplVClosed(Geom_BSplineSurface S,
                                     double V1,
                                     double V2,
                                     double Tol)
    {
        Geom_Curve? aCVF = S.VIso(V1);
        Geom_Curve? aCVL = S.VIso(V2);
        if (aCVF is null || aCVL is null)
        {
            return false;
        }
        double Tol2 = 2.0 * Tol;
        Geom_BSplineCurve aBsF = (Geom_BSplineCurve)aCVF;
        Geom_BSplineCurve aBsL = (Geom_BSplineCurve)aCVL;
        NCollection_Array1<Point3D> aPF = aBsF.InternalPoles();
        NCollection_Array1<Point3D> aPL = aBsL.InternalPoles();
        NCollection_Array1<double>? WF = aBsF.InternalWeights();
        NCollection_Array1<double>? WL = aBsL.InternalWeights();
        return CompareWeightPoles(aPF, WF, aPL, WL, Tol2);
    }

    //=================================================================================================

    /// <summary>Whether the poles of the U1 and U2 isolines of the Bezier surface are equal within Tol.</summary>
    public static bool IsBzUClosed(Geom_BezierSurface S,
                                   double U1,
                                   double U2,
                                   double Tol)
    {
        Geom_Curve? aCUF = S.UIso(U1);
        Geom_Curve? aCUL = S.UIso(U2);
        if (aCUF is null || aCUL is null)
        {
            return false;
        }
        double Tol2 = 2.0 * Tol;
        Geom_BezierCurve aBzF = (Geom_BezierCurve)aCUF;
        Geom_BezierCurve aBzL = (Geom_BezierCurve)aCUL;
        NCollection_Array1<Point3D> aPF = aBzF.InternalPoles();
        NCollection_Array1<Point3D> aPL = aBzL.InternalPoles();
        //
        return CompareWeightPoles(aPF, null, aPL, null, Tol2);
    }

    //=================================================================================================

    /// <summary>Whether the poles of the V1 and V2 isolines of the Bezier surface are equal within Tol.</summary>
    public static bool IsBzVClosed(Geom_BezierSurface S,
                                   double V1,
                                   double V2,
                                   double Tol)
    {
        Geom_Curve? aCVF = S.VIso(V1);
        Geom_Curve? aCVL = S.VIso(V2);
        if (aCVF is null || aCVL is null)
        {
            return false;
        }
        double Tol2 = 2.0 * Tol;
        Geom_BezierCurve aBzF = (Geom_BezierCurve)aCVF;
        Geom_BezierCurve aBzL = (Geom_BezierCurve)aCVL;
        NCollection_Array1<Point3D> aPF = aBzF.InternalPoles();
        NCollection_Array1<Point3D> aPL = aBzL.InternalPoles();
        //
        return CompareWeightPoles(aPF, null, aPL, null, Tol2);
    }

    //=================================================================================================
    private static bool CompareWeightPoles(NCollection_Array1<Point3D> thePoles1,
                                           NCollection_Array1<double>? theW1,
                                           NCollection_Array1<Point3D> thePoles2,
                                           NCollection_Array1<double>? theW2,
                                           double theTol)
    {
        if (thePoles1.Length() != thePoles2.Length())
        {
            return false;
        }
        //
        int i = 1;
        for (i = 1; i <= thePoles1.Length(); i++)
        {
            double aW1 = (theW1 is null) ? 1.0 : theW1.Value(i);
            double aW2 = (theW2 is null) ? 1.0 : theW2.Value(i);

            Xyz aPole1 = thePoles1.Value(i).XYZ() * aW1;
            Xyz aPole2 = thePoles2.Value(i).XYZ() * aW2;
            if (!aPole1.IsEqual(aPole2, theTol))
            {
                return false;
            }
        }
        //
        return true;
    }

    //=================================================================================================

    /// <summary>
    /// Whether the 2D curve is an isoline with natural parametrisation: a line, or a B-spline or Bezier curve of
    /// degree 1 with two poles, along U (theIsU, U = theParam) or V. The outputs are assigned false and 0 when it is
    /// not (OCCT leaves them unset).
    /// </summary>
    public static bool isIsoLine(Adaptor2d_Curve2d theC2D,
                                 out bool theIsU,
                                 out double theParam,
                                 out bool theIsForward)
    {
        // OCCT leaves the outputs unset when it returns false; the port assigns them first (decision 32).
        theIsU = false;
        theParam = 0.0;
        theIsForward = false;

        // These variables are used to check line state (vertical or horizontal).
        bool isAppropriateType = false;
        Point2D aLoc2d = new();
        Direction2D aDir2d = new();

        // Test type.
        CurveKind aType = theC2D.GetType();
        if (aType == Line)
        {
            Line2D aLin2d = theC2D.Line();
            aLoc2d = aLin2d.Location();
            aDir2d = aLin2d.Direction();
            isAppropriateType = true;
        }
        else if (aType == BSplineCurve)
        {
            Geom2d_BSplineCurve aBSpline2d = theC2D.BSpline();
            if (aBSpline2d.Degree() != 1 || aBSpline2d.NbPoles() != 2)
            {
                return false; // Not a line or uneven parameterization.
            }

            aLoc2d = aBSpline2d.Pole(1);

            // Vector should be non-degenerated.
            Vector2D aVec2d = new(aBSpline2d.Pole(1), aBSpline2d.Pole(2));
            if (aVec2d.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false; // Degenerated spline.
            }
            aDir2d = new Direction2D(aVec2d);

            isAppropriateType = true;
        }
        else if (aType == BezierCurve)
        {
            Geom2d_BezierCurve aBezier2d = theC2D.Bezier();
            if (aBezier2d.Degree() != 1 || aBezier2d.NbPoles() != 2)
            {
                return false; // Not a line or uneven parameterization.
            }

            aLoc2d = aBezier2d.Pole(1);

            // Vector should be non-degenerated.
            Vector2D aVec2d = new(aBezier2d.Pole(1), aBezier2d.Pole(2));
            if (aVec2d.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false; // Degenerated spline.
            }
            aDir2d = new Direction2D(aVec2d);

            isAppropriateType = true;
        }

        if (!isAppropriateType)
        {
            return false;
        }

        // Check line to be vertical or horizontal.
        if (aDir2d.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Horizontal line. V = const.
            theIsU = false;
            theParam = aLoc2d.Y();
            theIsForward = aDir2d.Dot(Constants.DX2d()) > 0.0;
            return true;
        }
        else if (aDir2d.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Vertical line. U = const.
            theIsU = true;
            theParam = aLoc2d.X();
            theIsForward = aDir2d.Dot(Constants.DY2d()) > 0.0;
            return true;
        }

        return false;
    }

    //=================================================================================================

    /// <summary>
    /// Builds the 3D curve of an isoline from the surface's own isoline (a GeomAdaptor_Surface other than a sphere),
    /// reparametrised as the 2D curve. Null when it cannot, or when the result is not within theTolerance.
    /// </summary>
    public static Geom_Curve? buildC3dOnIsoLine(Adaptor2d_Curve2d theC2D,
                                                Adaptor3d_Surface theSurf,
                                                double theFirst,
                                                double theLast,
                                                double theTolerance,
                                                bool theIsU,
                                                double theParam,
                                                bool theIsForward)
    {
        // Convert adapter to the appropriate type.
        GeomAdaptor_Surface? aGeomAdapter = theSurf as GeomAdaptor_Surface;
        if (aGeomAdapter is null)
        {
            return null;
        }

        if (theSurf.GetType() == SurfaceKind.Sphere)
        {
            return null;
        }

        // Extract isoline
        Geom_Surface aSurf = aGeomAdapter.Surface()!;
        Geom_Curve aC3d;

        Point2D aF2d = theC2D.Value(theC2D.FirstParameter());
        Point2D aL2d = theC2D.Value(theC2D.LastParameter());

        bool isToTrim = true;
        double U1, U2, V1, V2;
        aSurf.Bounds(out U1, out U2, out V1, out V2);

        if (theIsU)
        {
            double aV1Param = Min(aF2d.Y(), aL2d.Y());
            double aV2Param = Max(aF2d.Y(), aL2d.Y());
            if (aV2Param < V1 - theTolerance || aV1Param > V2 + theTolerance)
            {
                return null;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(V1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(V2))
            {
                if (Math.Abs(aV2Param - aV1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return null;
                }
                aSurf = new Geom_RectangularTrimmedSurface(aSurf, U1, U2, aV1Param, aV2Param);
                isToTrim = false;
            }
            else
            {
                aV1Param = Max(aV1Param, V1);
                aV2Param = Min(aV2Param, V2);
                if (Math.Abs(aV2Param - aV1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return null;
                }
            }
            aC3d = aSurf.UIso(theParam);
            if (isToTrim)
            {
                aC3d = new Geom_TrimmedCurve(aC3d, aV1Param, aV2Param);
            }
        }
        else
        {
            double aU1Param = Min(aF2d.X(), aL2d.X());
            double aU2Param = Max(aF2d.X(), aL2d.X());
            if (aU2Param < U1 - theTolerance || aU1Param > U2 + theTolerance)
            {
                return null;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(U1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(U2))
            {
                if (Math.Abs(aU2Param - aU1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return null;
                }
                aSurf = new Geom_RectangularTrimmedSurface(aSurf, aU1Param, aU2Param, V1, V2);
                isToTrim = false;
            }
            else
            {
                aU1Param = Max(aU1Param, U1);
                aU2Param = Min(aU2Param, U2);
                if (Math.Abs(aU2Param - aU1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return null;
                }
            }
            aC3d = aSurf.VIso(theParam);
            if (isToTrim)
            {
                aC3d = new Geom_TrimmedCurve(aC3d, aU1Param, aU2Param);
            }
        }

        // Convert arbitrary curve type to the b-spline.
        Geom_BSplineCurve aCurve3d =
          GeomConvert.CurveToBSplineCurve(aC3d, QuasiAngular);
        // The conversion can return a frozen curve that others share (decision 244): change a copy.
        if (aCurve3d.IsFrozen)
        {
            aCurve3d = (Geom_BSplineCurve)aCurve3d.Copy();
        }
        if (!theIsForward)
        {
            aCurve3d.Reverse();
        }

        // Rebuild parameterization for the 3d curve to have the same parameterization with
        // a two-dimensional curve.
        NCollection_Array1<double> aKnots = new(aCurve3d.InternalKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(theC2D.FirstParameter(), theC2D.LastParameter(), aKnots);
        aCurve3d.SetKnots(aKnots);

        // Evaluate error.
        double anError3d = 0.0;

        double aParF = theFirst;
        double aParL = theLast;
        const int aNbPnt = 23;
        for (int anIdx = 0; anIdx <= aNbPnt; ++anIdx)
        {
            double aPar = aParF + ((aParL - aParF) * anIdx) / aNbPnt;

            Point2D aPnt2d = theC2D.Value(aPar);

            Point3D aPntC3D = aCurve3d.Value(aPar);
            Point3D aPntC2D = theSurf.Value(aPnt2d.X(), aPnt2d.Y());

            double aSqDeviation = aPntC3D.SquareDistance(aPntC2D);
            anError3d = Max(aSqDeviation, anError3d);
        }

        anError3d = Math.Sqrt(anError3d);

        // Target tolerance is not obtained. This situation happens for isolines on the sphere.
        // OCCT is unable to convert it keeping original parameterization, while the geometric
        // form of the result is entirely identical. In that case, it is better to utilize
        // a general-purpose approach.
        if (anError3d > theTolerance)
        {
            return null;
        }

        return aCurve3d;
    }
}
