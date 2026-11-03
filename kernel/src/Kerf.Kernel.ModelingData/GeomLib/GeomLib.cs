// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Splines;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Engine.Geometry.NormalStatus;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Geometry.Library;

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

    private static void ComputeLambda(Matrix constraint,
                                      Matrix hermit,
                                      double length,
                                      ref double lambda)
    {
        int size = hermit.RowNumber();
        int continuity = size - 2;
        int ii, jj, ip, pp;

        // Minimization
        Matrix hDer = new(1, size - 1, 1, size);
        for (jj = 1; jj <= size; jj++)
        {
            for (ii = 1; ii < size; ii++)
            {
                hDer[ii, jj] = ii * hermit[jj, ii + 1];
            }
        }

        math_Vector v = new(1, size);
        math_Vector vec1 = new(1, constraint.RowNumber());
        math_Vector vec2 = new(1, constraint.RowNumber());
        math_Vector vec3 = new(1, constraint.RowNumber());
        math_Vector vec4 = new(1, constraint.RowNumber());

        // OCCT's pointers to HDer(1, 1) and V(1)
        vec2.Initialized(constraint.Col(2));
        vec2 /= length;
        double t, squared1 = vec2.Norm2(), gw;
        //  math_Matrix Vec(1, Constraint.RowNumber(), 1, size-1);
        //  gp_Vec Vfirst(p0.XYZ()), Vlast(Point.XYZ());
        //  NCollection_Array1<gp_Vec> Der(2, 4);
        //  Der(2) = d1; Der(3) = d2; Der(4) = d3;

        int gOrdre = 4 + 4 * continuity, dDim = continuity * (continuity + 2);
        math_Vector gaussP = new(1, gOrdre), gaussW = new(1, gOrdre), pol2 = new(1, 2 * continuity + 1),
          pol4 = new(1, 4 * continuity + 1);
        NumericTables.GaussPoints(gOrdre, gaussP);
        NumericTables.GaussWeights(gOrdre, gaussW);
        pol4.Init(0.0);

        for (ip = 1; ip <= gOrdre; ip++)
        {
            t = (gaussP[ip] + 1.0) / 2;
            gw = gaussW[ip];
            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(t, continuity, continuity + 2, dDim, hDer.AsSpan(), v.AsSpan());
            v /= length; // Normalisation

            //                      i
            // C'(t) = SUM Vi*Lambda
            vec1.Initialized(constraint.Col(1));
            vec1 *= v[1];
            vec1 += v[size] * constraint.Col(size);
            vec2.Initialized(constraint.Col(2));
            vec2 *= v[2];
            if (continuity > 1)
            {
                vec3.Initialized(constraint.Col(3));
                vec3 *= v[3];
                if (continuity > 2)
                {
                    vec4.Initialized(constraint.Col(4));
                    vec4 *= v[4];
                }
            }

            //   2          2
            // C'(t) - C'(0)

            pol2[1] = vec1.Norm2();
            pol2[2] = 2 * (vec1.Multiplied(vec2));
            pol2[3] = vec2.Norm2() - squared1;
            if (continuity > 1)
            {
                pol2[3] += 2 * (vec1.Multiplied(vec3));
                pol2[4] = 2 * (vec2.Multiplied(vec3));
                pol2[5] = vec3.Norm2();
                if (continuity > 2)
                {
                    pol2[4] += 2 * (vec1.Multiplied(vec4));
                    pol2[5] += 2 * (vec2.Multiplied(vec4));
                    pol2[6] = 2 * (vec3.Multiplied(vec4));
                    pol2[7] = vec4.Norm2();
                }
            }

            //                     2      2  2
            // Integrale de ( C'(t) - C'(0) )
            for (ii = 1; ii <= pol2.Length(); ii++)
            {
                pp = ii;
                for (jj = 1; jj < ii; jj++, pp++)
                {
                    pol4[pp] += 2 * gw * pol2[ii] * pol2[jj];
                }
                pol4[2 * ii - 1] += gw * Math.Pow(pol2[ii], 2);
            }
        }

        double eMin, e;
        Span<double> res = stackalloc double[1];
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(lambda, pol4.Length() - 1, 1, pol4.Length() - 1, pol4.AsSpan(), res);
        eMin = res[0];

        if (eMin > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // Search for extrema of the function
            PolyFunc ff = new(pol4);
            LogSample s = new(lambda / 1000, 50 * lambda, 100);
            FunctionAllRoots solve = new(ff,
                                              s,
                                              global::Kerf.Engine.Foundation.Precision.Confusion(),
                                              global::Kerf.Engine.Foundation.Precision.Confusion() * (length + 1),
                                              1.0e-15);
            if (solve.IsDone)
            {
                for (ii = 1; ii <= solve.PointCount; ii++)
                {
                    t = solve.GetPoint(ii);
                    global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(t, pol4.Length() - 1, 1, pol4.Length() - 1, pol4.AsSpan(), res);
                    e = res[0];
                    if (e < eMin)
                    {
                        lambda = t;
                        eMin = e;
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
    public static void RemovePointsFromArray(int numPoints,
                                             NCollection_Array1<double> inParameters,
                                             out NCollection_HArray1<double> outParameters)
    {
        int ii, jj, add_one_point, loc_num_points, num_points, index;
        double delta, current_parameter;

        loc_num_points = Max(0, numPoints - 2);
        delta = inParameters[inParameters.Upper()] - inParameters[inParameters.Lower()];
        delta /= (double)(loc_num_points + 1);
        num_points = 1;
        current_parameter = inParameters[inParameters.Lower()] + delta * 0.5e0;
        ii = inParameters.Lower() + 1;
        for (jj = 0; ii < inParameters.Upper() && jj < numPoints; jj++)
        {
            add_one_point = 0;
            while (ii < inParameters.Upper() && inParameters[ii] < current_parameter)
            {
                ii += 1;
                add_one_point = 1;
            }
            num_points += add_one_point;
            current_parameter += delta;
        }
        if (numPoints <= 2)
        {
            num_points = 2;
        }
        index = 2;
        current_parameter = inParameters[inParameters.Lower()] + delta * 0.5e0;
        outParameters = new NCollection_HArray1<double>(1, num_points);
        outParameters.ChangeArray1()[1] = inParameters[inParameters.Lower()];
        ii = inParameters.Lower() + 1;
        for (jj = 0; ii < inParameters.Upper() && jj < numPoints; jj++)
        {
            add_one_point = 0;
            while (ii < inParameters.Upper() && inParameters[ii] < current_parameter)
            {
                ii += 1;
                add_one_point = 1;
            }
            if (add_one_point != 0 && index <= num_points)
            {
                outParameters.ChangeArray1()[index] = inParameters[ii - 1];
                index += 1;
            }
            current_parameter += delta;
        }
        outParameters.ChangeArray1()[num_points] = inParameters[inParameters.Upper()];
    }

    //=================================================================================================

    /// <summary>
    /// Makes sure there are at least MinNumPoints parameters, adding uniformly spaced ones between the
    /// non-decreasing InParameters (which are copied unchanged when they are not in order).
    /// </summary>
    public static void DensifyArray1OfReal(int minNumPoints,
                                           NCollection_Array1<double> inParameters,
                                           out NCollection_HArray1<double> outParameters)
    {
        int ii, in_order, num_points, num_parameters_to_add, index;
        double delta, current_parameter;

        in_order = 1;
        if (minNumPoints > inParameters.Length())
        {

            //
            // checks the parameters are in increasing order
            //
            for (ii = inParameters.Lower(); ii < inParameters.Upper(); ii++)
            {
                if (inParameters[ii] > inParameters[ii + 1])
                {
                    in_order = 0;
                    break;
                }
            }
            if (in_order != 0)
            {
                num_parameters_to_add = minNumPoints - inParameters.Length();
                delta = inParameters[inParameters.Upper()] - inParameters[inParameters.Lower()];
                delta /= (double)(num_parameters_to_add + 1);
                num_points = minNumPoints;
                outParameters = new NCollection_HArray1<double>(1, num_points);
                index = 1;
                current_parameter = inParameters[inParameters.Lower()];
                outParameters.ChangeArray1()[index] = current_parameter;
                index += 1;
                current_parameter += delta;
                for (ii = inParameters.Lower() + 1; index <= num_points && ii <= inParameters.Upper(); ii++)
                {
                    while (current_parameter < inParameters[ii] && index <= num_points)
                    {
                        outParameters.ChangeArray1()[index] = current_parameter;
                        index += 1;
                        current_parameter += delta;
                    }
                    if (index <= num_points)
                    {
                        outParameters.ChangeArray1()[index] = inParameters[ii];
                    }
                    index += 1;
                }
                //
                // beware of roundoff !
                //
                outParameters.ChangeArray1()[num_points] = inParameters[inParameters.Upper()];
            }
            else
            {
                index = 1;
                num_points = inParameters.Length();
                outParameters = new NCollection_HArray1<double>(1, num_points);
                for (ii = inParameters.Lower(); ii <= inParameters.Upper(); ii++)
                {
                    outParameters.ChangeArray1()[index] = inParameters[ii];
                    index += 1;
                }
            }
        }
        else
        {
            index = 1;
            num_points = inParameters.Length();
            outParameters = new NCollection_HArray1<double>(1, num_points);
            for (ii = inParameters.Lower(); ii <= inParameters.Upper(); ii++)
            {
                outParameters.ChangeArray1()[index] = inParameters[ii];
                index += 1;
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Fuses the sorted intervals Interval1 and Interval2 into Fusion. Values closer than Confusion are merged: to
    /// their mean, or to the value of Interval1 when IsAdjustToFirstInterval is true.
    /// </summary>
    public static void FuseIntervals(NCollection_Array1<double> i1,
                                     NCollection_Array1<double> i2,
                                     NCollection_Sequence<double> seq,
                                     double epspar = 1.0e-9,
                                     bool isAdjustToFirstInterval = false)
    {
        int ind1 = 1, ind2 = 1;
        double v1, v2;
        // Initializations: IND1 and IND2 point to the 1st element
        // of each of the 2 tables to process. INDS points to the last
        // created element of TABSOR

        //--- Fill TABSOR by traversing TABLE1 and TABLE2 simultaneously ---
        //------------------ eliminating multiple occurrences ----------------

        while ((ind1 <= i1.Upper()) && (ind2 <= i2.Upper()))
        {
            v1 = i1[ind1];
            v2 = i2[ind2];
            if (Math.Abs(v1 - v2) <= epspar)
            {
                // Here the elements of I1 and I2 match.
                if (isAdjustToFirstInterval)
                {
                    seq.Append(v1);
                }
                else
                {
                    seq.Append((v1 + v2) / 2);
                }
                ind1++;
                ind2++;
            }
            else if (v1 < v2)
            {
                // Here the element of I1 is suitable.
                seq.Append(v1);
                ind1++;
            }
            else
            {
                // Here the element of TABLE2 is suitable.
                seq.Append(v2);
                ind2++;
            }
        }

        if (ind1 > i1.Upper())
        {
            //----- Here I1 is exhausted, complete with the end of TABLE2 -------

            for (; ind2 <= i2.Upper(); ind2++)
            {
                seq.Append(i2[ind2]);
            }
        }

        if (ind2 > i2.Upper())
        {
            //----- Here I2 is exhausted, complete with the end of I1 -------
            for (; ind1 <= i1.Upper(); ind1++)
            {
                seq.Append(i1[ind1]);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// The largest distance between the two curves at the given parameters (the curves evaluated at the same
    /// parameter). The tolerance is not used.
    /// </summary>
    public static void EvalMaxParametricDistance(CurveAdaptorBase aCurve,
                                                 CurveAdaptorBase aReferenceCurve,
                                                 //			       const double  Tolerance,
                                                 double _,
                                                 NCollection_Array1<double> parameters,
                                                 out double maxDistance)
    {
        int ii;

        double max_squared = 0.0e0,
          //    tolerance_squared,
          local_distance_squared;

        //  tolerance_squared = Tolerance * Tolerance ;
        Point3D point1;
        Point3D point2;
        for (ii = parameters.Lower(); ii <= parameters.Upper(); ii++)
        {
            aCurve.D0(parameters[ii], out point1);
            aReferenceCurve.D0(parameters[ii], out point2);
            local_distance_squared = point1.SquareDistance(point2);
            max_squared = Max(max_squared, local_distance_squared);
        }
        if (max_squared > 0.0e0)
        {
            maxDistance = Math.Sqrt(max_squared);
        }
        else
        {
            maxDistance = 0.0e0;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The largest distance from the curve at the given parameters to the reference curve: where the points at the
    /// same parameter are farther apart than Tolerance, the point is projected on the reference curve. At least
    /// Tolerance.
    /// </summary>
    public static void EvalMaxDistanceAlongParameter(CurveAdaptorBase aCurve,
                                                     CurveAdaptorBase aReferenceCurve,
                                                     double tolerance,
                                                     NCollection_Array1<double> parameters,
                                                     out double maxDistance)
    {
        int ii;
        double max_squared = 0.0e0, tolerance_squared = tolerance * tolerance, other_parameter,
               para_tolerance, local_distance_squared;
        Point3D point1;
        Point3D point2;

        para_tolerance = aReferenceCurve.Resolution(tolerance);
        other_parameter = parameters[parameters.Lower()];
        aCurve.D0(other_parameter, out point1);
        LocateExtPC a_projector = new(point1, aReferenceCurve, other_parameter, para_tolerance);
        for (ii = parameters.Lower(); ii <= parameters.Upper(); ii++)
        {
            aCurve.D0(parameters[ii], out point1);
            aReferenceCurve.D0(parameters[ii], out point2);
            local_distance_squared = point1.SquareDistance(point2);

            if (local_distance_squared > tolerance_squared)
            {

                a_projector.Perform(point1, other_parameter);
                if (a_projector.IsDone)
                {
                    other_parameter = a_projector.Point().Parameter();
                    aReferenceCurve.D0(other_parameter, out point2);
                    local_distance_squared = point1.SquareDistance(point2);
                }
                else
                {
                    local_distance_squared = 0.0e0;
                    other_parameter = parameters[ii];
                }
            }
            else
            {
                other_parameter = parameters[ii];
            }

            max_squared = Max(max_squared, local_distance_squared);
        }
        if (max_squared > tolerance_squared)
        {
            maxDistance = Math.Sqrt(max_squared);
        }
        else
        {
            maxDistance = tolerance;
        }
    }

    //=================================================================================================

    /// <summary>
    /// The 3D curve of the 2D curve Curve2d drawn on the plane Position. Raises Standard_NotImplemented for a
    /// curve of another kind.
    /// </summary>
    public static Curve3D To3d(in Frame3D position, Curve2D curve2d)
    {
        Curve3D curve3d;

        if (curve2d.GetType() == typeof(TrimmedCurve2D))
        {
            TrimmedCurve2D ct = (TrimmedCurve2D)curve2d;
            double u1 = ct.FirstParameter();
            double u2 = ct.LastParameter();
            Curve2D cBasis2d = ct.BasisCurve();
            Curve3D cc = global::Kerf.Engine.Geometry.Library.GeomLib.To3d(position, cBasis2d);
            curve3d = new TrimmedCurve3D(cc, u1, u2);
        }
        else if (curve2d.GetType() == typeof(OffsetCurve2D))
        {
            OffsetCurve2D co = (OffsetCurve2D)curve2d;
            double offset = co.Offset();
            Curve2D cBasis2d = co.BasisCurve();
            Curve3D cc = global::Kerf.Engine.Geometry.Library.GeomLib.To3d(position, cBasis2d);
            curve3d = new OffsetCurve3D(cc, offset, position.Direction());
        }
        else if (curve2d.GetType() == typeof(BezierCurve2D))
        {
            BezierCurve2D cBez2d = (BezierCurve2D)curve2d;
            NCollection_Array1<Point2D> poles2d = cBez2d.InternalPoles();
            int nbpoles = poles2d.Length();
            NCollection_Array1<Point3D> poles3d = new(1, nbpoles);
            for (int i = 1; i <= nbpoles; i++)
            {
                poles3d[i] = global::Kerf.Engine.Geometry.ElCLib.To3d(position, poles2d[i]);
            }
            BezierCurve3D cBez3d;
            if (cBez2d.IsRational)
            {
                cBez3d = new BezierCurve3D(poles3d, cBez2d.InternalWeightsArray());
            }
            else
            {
                cBez3d = new BezierCurve3D(poles3d);
            }
            curve3d = cBez3d;
        }
        else if (curve2d.GetType() == typeof(BSplineCurve2D))
        {
            BSplineCurve2D cbSpl2d = (BSplineCurve2D)curve2d;
            int theDegree = cbSpl2d.Degree();
            bool isPeriodic = cbSpl2d.IsPeriodic;
            NCollection_Array1<Point2D> poles2d = cbSpl2d.InternalPoles();
            int nbpoles = poles2d.Length();
            NCollection_Array1<Point3D> poles3d = new(1, nbpoles);
            for (int i = 1; i <= nbpoles; i++)
            {
                poles3d[i] = global::Kerf.Engine.Geometry.ElCLib.To3d(position, poles2d[i]);
            }
            NCollection_Array1<double> theKnots = cbSpl2d.InternalKnots();
            NCollection_Array1<int> theMults = cbSpl2d.InternalMultiplicities();
            BSplineCurve3D cbSpl3d;
            if (cbSpl2d.IsRational)
            {
                cbSpl3d = new BSplineCurve3D(poles3d,
                                                cbSpl2d.InternalWeightsArray(),
                                                theKnots,
                                                theMults,
                                                theDegree,
                                                isPeriodic);
            }
            else
            {
                cbSpl3d = new BSplineCurve3D(poles3d, theKnots, theMults, theDegree, isPeriodic);
            }
            curve3d = cbSpl3d;
        }
        else if (curve2d.GetType() == typeof(LineCurve2D))
        {
            LineCurve2D line2d = (LineCurve2D)curve2d;
            Line2D l2d = line2d.Lin2d();
            Line3D l3d = global::Kerf.Engine.Geometry.ElCLib.To3d(position, l2d);
            LineCurve3D geomL3d = new(l3d);
            curve3d = geomL3d;
        }
        else if (curve2d.GetType() == typeof(CircleCurve2D))
        {
            CircleCurve2D circle2d = (CircleCurve2D)curve2d;
            Circle2D c2d = circle2d.Circ2d();
            Circle3D c3d = global::Kerf.Engine.Geometry.ElCLib.To3d(position, c2d);
            CircleCurve3D geomC3d = new(c3d);
            curve3d = geomC3d;
        }
        else if (curve2d.GetType() == typeof(EllipseCurve2D))
        {
            EllipseCurve2D ellipse2d = (EllipseCurve2D)curve2d;
            Ellipse2D e2d = ellipse2d.Elips2d();
            Ellipse3D e3d = global::Kerf.Engine.Geometry.ElCLib.To3d(position, e2d);
            EllipseCurve3D geomE3d = new(e3d);
            curve3d = geomE3d;
        }
        else if (curve2d.GetType() == typeof(ParabolaCurve2D))
        {
            ParabolaCurve2D parabola2d = (ParabolaCurve2D)curve2d;
            Parabola2D prb2d = parabola2d.Parab2d();
            Parabola3D prb3d = global::Kerf.Engine.Geometry.ElCLib.To3d(position, prb2d);
            ParabolaCurve3D geomPrb3d = new(prb3d);
            curve3d = geomPrb3d;
        }
        else if (curve2d.GetType() == typeof(HyperbolaCurve2D))
        {
            HyperbolaCurve2D hyperbola2d = (HyperbolaCurve2D)curve2d;
            Hyperbola2D h2d = hyperbola2d.Hypr2d();
            Hyperbola3D h3d = global::Kerf.Engine.Geometry.ElCLib.To3d(position, h2d);
            HyperbolaCurve3D geomH3d = new(h3d);
            curve3d = geomH3d;
        }
        else
        {
            throw new MethodNotImplementedException();
        }

        return curve3d;
    }

    //=================================================================================================

    /// <summary>
    /// The 2D curve transformed by GTrsf. Null when it cannot be computed: an offset curve, or an infinite
    /// parabola or hyperbola under an affinity.
    /// </summary>
    public static Curve2D? GTransform(Curve2D curve, in GeneralTransform2D gTrsf)
    {
        TransformForm form = gTrsf.Form();

        if (form != Other)
        {

            // Then, the GTrsf is actually a Trsf.
            // The curve geometry will then remain unchanged.

            Curve2D? c = curve.Transformed(gTrsf.Trsf2d()) as Curve2D;
            return c;
        }
        else
        {

            // So, the GTrsf is an other Transformation.
            // The geometry of the curves is then changed, and the conics will have to
            // be converted to BSplines.

            Type theType = curve.GetType();

            if (theType == typeof(TrimmedCurve2D))
            {

                // We will recurse on the BasisCurve

                TrimmedCurve2D c = (TrimmedCurve2D)curve.Copy();

                Type theBasisType = (c.BasisCurve()).GetType();

                if (theBasisType == typeof(BSplineCurve2D)
                    || theBasisType == typeof(BezierCurve2D))
                {

                    // In these cases the parameterization is preserved on the transformed curve,
                    // so we can trim it with the parameters of the base curve.

                    double u1 = c.FirstParameter();
                    double u2 = c.LastParameter();

                    TrimmedCurve2D result =
                      new(GTransform(c.BasisCurve(), gTrsf)!, u1, u2);
                    return result;
                }
                else if (theBasisType == typeof(LineCurve2D))
                {

                    // In this case, the parameterization is no longer preserved.
                    // We must recompute the trimming parameters on the resulting
                    // curve (by projecting the transformed start and end points using ElCLib).

                    LineCurve2D l = (LineCurve2D)GTransform(c.BasisCurve(), gTrsf)!;
                    Line2D lin = l.Lin2d();

                    Point2D p1 = c.StartPoint();
                    Point2D p2 = c.EndPoint();
                    p1 = p1.WithXY(gTrsf.Transformed(p1.XY()));
                    p2 = p2.WithXY(gTrsf.Transformed(p2.XY()));
                    double u1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, p1);
                    double u2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, p2);

                    TrimmedCurve2D result = new(l, u1, u2);
                    return result;
                }
                else if (theBasisType == typeof(CircleCurve2D)
                         || theBasisType == typeof(EllipseCurve2D)
                         || theBasisType == typeof(ParabolaCurve2D)
                         || theBasisType == typeof(HyperbolaCurve2D))
                {

                    // In these cases, the curve geometry is not preserved;
                    // convert it to BSpline before applying the Trsf.

                    BSplineCurve2D bs = Geom2dConvert2D.CurveToBSplineCurve(c);
                    return GTransform(bs, gTrsf);
                }
                else
                {

                    // The transform of an OffsetCurve is not supported.

                    return null;
                }
            }
            else if (theType == typeof(LineCurve2D))
            {

                LineCurve2D l = (LineCurve2D)curve.Copy();
                Line2D lin = l.Lin2d();
                Point2D p = lin.Location();
                Point2D pp = l.Value(10.0); // arbitrary point
                p = p.WithXY(gTrsf.Transformed(p.XY()));
                pp = pp.WithXY(gTrsf.Transformed(pp.XY()));
                l.SetLocation(p);
                Vector2D v = new(p, pp);
                l.SetDirection(new Direction2D(v));
                return l;
            }
            else if (theType == typeof(BezierCurve2D))
            {

                // Since GTrsf is a linear operation, the transform of a pole-based curve
                // is the curve whose poles are the transforms of the base curve's poles.

                BezierCurve2D c = (BezierCurve2D)curve.Copy();
                int nbPoles = c.PoleCount;
                NCollection_Array1<Point2D> poles = new(c.InternalPoles());
                for (int i = 1; i <= nbPoles; i++)
                {
                    poles[i] = poles[i].WithXY(gTrsf.Transformed(poles[i].XY()));
                    c.SetPole(i, poles[i]);
                }
                return c;
            }
            else if (theType == typeof(BSplineCurve2D))
            {

                // See comment for Bezier curves above.

                BSplineCurve2D c = (BSplineCurve2D)curve.Copy();
                int nbPoles = c.PoleCount;
                NCollection_Array1<Point2D> poles = new(c.InternalPoles());
                for (int i = 1; i <= nbPoles; i++)
                {
                    poles[i] = poles[i].WithXY(gTrsf.Transformed(poles[i].XY()));
                    c.SetPole(i, poles[i]);
                }
                return c;
            }
            else if (theType == typeof(CircleCurve2D) || theType == typeof(EllipseCurve2D))
            {

                // In these cases, the curve geometry is not preserved;
                // convert it to BSpline before applying the Trsf.

                BSplineCurve2D c = Geom2dConvert2D.CurveToBSplineCurve(curve);
                return GTransform(c, gTrsf);
            }
            else if (theType == typeof(ParabolaCurve2D) || theType == typeof(HyperbolaCurve2D)
                     || theType == typeof(OffsetCurve2D))
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
    public static void SameRange(double tolerance,
                                 Curve2D? curvePtr,
                                 double firstOnCurve,
                                 double lastOnCurve,
                                 double requestedFirst,
                                 double requestedLast,
                                 ref Curve2D? newCurvePtr)
    {
        if (curvePtr is null)
        {
            throw new FailureException();
        }
        if (Math.Abs(lastOnCurve - requestedLast) <= tolerance
            && Math.Abs(firstOnCurve - requestedFirst) <= tolerance)
        {
            newCurvePtr = curvePtr;
            return;
        }

        // the parametrisation length  must at least be the same.
        if (Math.Abs(lastOnCurve - firstOnCurve - requestedLast + requestedFirst) <= tolerance)
        {
            if (curvePtr is LineCurve2D)
            {
                LineCurve2D line = (LineCurve2D)curvePtr.Copy();
                double dU = firstOnCurve - requestedFirst;
                Direction2D d = line.Direction();
                line.Translate(dU * new Vector2D(d));
                newCurvePtr = line;
            }
            else if (curvePtr is CircleCurve2D)
            {
                Transform2D trsf = new();
                newCurvePtr = (Curve2D)curvePtr.Copy();
                CircleCurve2D circ = (CircleCurve2D)newCurvePtr;
                Point2D p = circ.Location();
                double dU;
                if (circ.Circ2d().IsDirect)
                {
                    dU = firstOnCurve - requestedFirst;
                }
                else
                {
                    dU = requestedFirst - firstOnCurve;
                }
                trsf = trsf.WithRotation(p, dU);
                newCurvePtr.Transform(trsf);
            }
            else if (curvePtr is TrimmedCurve2D)
            {
                TrimmedCurve2D tc = (TrimmedCurve2D)curvePtr;
                global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(tolerance,
                                  tc.BasisCurve(),
                                  firstOnCurve,
                                  lastOnCurve,
                                  requestedFirst,
                                  requestedLast,
                                  ref newCurvePtr);
                newCurvePtr = new TrimmedCurve2D(newCurvePtr!, requestedFirst, requestedLast);
            }
            //
            //  Beware of limitation issues: use the SAME test as in
            //  Geom2d_TrimmedCurve::SetTrim, otherwise since we may re-trim on
            //  RequestedFirst and RequestedLast we will have a problem
            //
            //
            else if (Math.Abs(lastOnCurve - firstOnCurve) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                     || Math.Abs(requestedLast + requestedFirst) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {

                TrimmedCurve2D tc =
                  new(curvePtr, firstOnCurve, lastOnCurve);

                BSplineCurve2D bs = Geom2dConvert2D.CurveToBSplineCurve(tc);
                NCollection_Array1<double> knots = new(bs.InternalKnots());

                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(requestedFirst, requestedLast, knots);

                bs.SetKnots(knots);
                newCurvePtr = bs;
            }
        }
        else
        { // We segment the result
            TrimmedCurve2D tc;
            Curve2D cCheck = curvePtr;

            if (cCheck is TrimmedCurve2D)
            {
                cCheck = ((TrimmedCurve2D)cCheck).BasisCurve();
            }

            if (cCheck.IsPeriodic)
            {
                if (Math.Abs(lastOnCurve - firstOnCurve) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    tc = new TrimmedCurve2D(curvePtr, firstOnCurve, lastOnCurve);
                }
                else
                {
                    tc =
                      new TrimmedCurve2D(curvePtr, curvePtr.FirstParameter(), curvePtr.LastParameter());
                }
            }
            else
            {
                double udeb = Max(curvePtr.FirstParameter(), firstOnCurve);
                double ufin = Min(curvePtr.LastParameter(), lastOnCurve);
                if (Math.Abs(ufin - udeb) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    tc = new TrimmedCurve2D(curvePtr, udeb, ufin);
                }
                else
                {
                    tc =
                      new TrimmedCurve2D(curvePtr, curvePtr.FirstParameter(), curvePtr.LastParameter());
                }
            }

            //
            BSplineCurve2D bs = Geom2dConvert2D.CurveToBSplineCurve(tc);
            NCollection_Array1<double> knots = new(bs.InternalKnots());

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(requestedFirst, requestedLast, knots);

            bs.SetKnots(knots);
            newCurvePtr = bs;
        }
    }

    //=================================================================================================

    private sealed class GeomLib_CurveOnSurfaceEvaluator : global::Kerf.Engine.Approximation.Advanced.EvaluatorFunction
    {
        public GeomLib_CurveOnSurfaceEvaluator(CurveOnSurface curveOnSurface,
                                               double first,
                                               double last)
        {
            _curveOnSurface = curveOnSurface;
            _firstParam = first;
            _lastParam = last;
        }

        public override void Evaluate(int dimension, ReadOnlySpan<double> debutFin, double parameter,
                                      int derivativeRequest, Span<double> result, out int returnCode)
        {
            Point3D point;

            // Handle left / right positioning
            if ((debutFin[0] != _firstParam) || (debutFin[1] != _lastParam))
            {
                _trimCurve = _curveOnSurface.Trim(debutFin[0], debutFin[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
                _firstParam = debutFin[0];
                _lastParam = debutFin[1];
            }

            // Positioning
            if (derivativeRequest == 0)
            {
                _trimCurve!.D0(parameter, out point);

                for (int ii = 0; ii < 3; ii++)
                {
                    result[ii] = point.Coord(ii + 1);
                }
            }
            if (derivativeRequest == 1)
            {
                Vector3D vector;
                _trimCurve!.D1(parameter, out point, out vector);
                for (int ii = 0; ii < 3; ii++)
                {
                    result[ii] = vector.Coord(ii + 1);
                }
            }
            if (derivativeRequest == 2)
            {
                Vector3D vector, vecBis;
                _trimCurve!.D2(parameter, out point, out vecBis, out vector);
                for (int ii = 0; ii < 3; ii++)
                {
                    result[ii] = vector.Coord(ii + 1);
                }
            }
            returnCode = 0;
        }

        private readonly CurveOnSurface _curveOnSurface;
        private double _firstParam;
        private double _lastParam;

        private CurveAdaptorBase? _trimCurve;
    }

    //=================================================================================================

    /// <summary>
    /// Builds the 3D curve of a curve on a surface: on a plane, the 2D curve drawn on it (GeomLib.To3d); on an
    /// isoline, the surface's isoline; otherwise an AdvApprox approximation within Tolerance, with its maximum and
    /// average deviations. NewCurvePtr is left unchanged when the approximation has no result.
    /// </summary>
    public static void BuildCurve3d(double tolerance,
                                    CurveOnSurface curve,
                                    double firstParameter,
                                    double lastParameter,
                                    ref Curve3D? newCurvePtr,
                                    out double maxDeviation,
                                    out double averageDeviation,
                                    Continuity continuity = C1,
                                    int maxDegree = 14,
                                    int maxSegment = 30)

    {

        maxDeviation = 0.0e0;
        averageDeviation = 0.0e0;
        SurfaceAdaptor? geom_adaptor_surface_ptr = curve.GetSurface() as SurfaceAdaptor;
        CurveAdaptor2D? geom_adaptor_curve_ptr = curve.GetCurve() as CurveAdaptor2D;

        if (geom_adaptor_curve_ptr is not null && geom_adaptor_surface_ptr is not null)
        {
            PlaneSurface? p;
            SurfaceAdaptor geom_surface = geom_adaptor_surface_ptr;

            TrimmedSurface? rt =
              geom_surface.Surface() as TrimmedSurface;
            if (rt is null)
            {
                p = geom_surface.Surface() as PlaneSurface;
            }
            else
            {
                p = rt.BasisSurface() as PlaneSurface;
            }

            if (p is not null)
            {
                // compute the 3d curve
                Frame3D axes = p.Position().Ax2();
                CurveAdaptor2D geom2d_curve = geom_adaptor_curve_ptr;
                newCurvePtr = global::Kerf.Engine.Geometry.Library.GeomLib.To3d(axes, geom2d_curve.Curve()!);
                return;
            }

            Curve2DAdaptorBase trimmedC2D =
              geom_adaptor_curve_ptr.Trim(firstParameter, lastParameter, global::Kerf.Engine.Foundation.Precision.PConfusion());

            bool isU, isForward;
            double param;
            if (IsIsoLine(trimmedC2D, out isU, out param, out isForward))
            {
                newCurvePtr = BuildC3dOnIsoLine(trimmedC2D,
                                                geom_adaptor_surface_ptr,
                                                firstParameter,
                                                lastParameter,
                                                tolerance,
                                                isU,
                                                param,
                                                isForward);
                if (newCurvePtr is not null)
                {
                    return;
                }
            }
        }

        //
        // Entree
        //
        NCollection_HArray1<double>? tolerance1DPtr = null, tolerance2DPtr = null;
        NCollection_HArray1<double> tolerance3DPtr = new(1, 1);
        tolerance3DPtr.SetValue(1, tolerance);

        // Search for discontinuities
        int nbIntervalC2 = curve.NbIntervals(C2);
        NCollection_Array1<double> Param_de_decoupeC2 = new(1, nbIntervalC2 + 1);
        curve.Intervals(Param_de_decoupeC2, C2);

        int nbIntervalC3 = curve.NbIntervals(C3);
        NCollection_Array1<double> Param_de_decoupeC3 = new(1, nbIntervalC3 + 1);
        curve.Intervals(Param_de_decoupeC3, C3);

        // Note extension of the parametric range
        // To force Trim on first evaluator call
        GeomLib_CurveOnSurfaceEvaluator ev = new(curve, firstParameter - 1.0, lastParameter + 1.0);

        // Approximation with preferential cutting
        PrefAndRec preferentiel = new(Param_de_decoupeC2, Param_de_decoupeC3);
        ApproxAFunction approximator = new(0,
                                                       0,
                                                       1,
                                                       tolerance1DPtr,
                                                       tolerance2DPtr,
                                                       tolerance3DPtr,
                                                       firstParameter,
                                                       lastParameter,
                                                       continuity,
                                                       maxDegree,
                                                       maxSegment,
                                                       ev,
                                                       //					      CurveOnSurfaceEvaluator,
                                                       preferentiel);

        if (approximator.HasResult)
        {
            MakeCurvefromApprox curveBuilder = new(approximator);

            BSplineCurve3D curvePtr = curveBuilder.Curve(1);
            // Return the approximation results
            maxDeviation = approximator.MaxError(3, 1);
            averageDeviation = approximator.AverageError(3, 1);
            newCurvePtr = curvePtr;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Deforms the curve (converted to a B-spline) so that it starts at P1 tangent to T1 and ends at P2 tangent to
    /// T2. Curve receives the new curve.
    /// </summary>
    public static void AdjustExtremity(ref BoundedCurve3D curve,
                                       in Point3D p1,
                                       in Point3D p2,
                                       in Vector3D t1,
                                       in Vector3D t2)
    {
        // Convert the input (preserving the parameterization if possible)
        BSplineCurve3D aIn, def;
        aIn = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(curve, QuasiAngular);
        // The conversion copies the caller's curve; a frozen result would be shared (decision 244): change a copy.
        if (aIn.IsFrozen)
        {
            aIn = (BSplineCurve3D)aIn.Copy();
        }

        int ii, jj;
        Point3D p;
        Vector3D v, vtan, dv;
        NCollection_Array1<Point3D> polesDef = new(1, 4), coeffs = new(1, 4);
        NCollection_Array1<double> fk = new(1, 8);
        NCollection_Array1<double> ti = new(1, 4);
        NCollection_Array1<int> contact = new(1, 4);

        ti[1] = ti[2] = aIn.FirstParameter();
        ti[3] = ti[4] = aIn.LastParameter();
        contact[1] = contact[3] = 0;
        contact[2] = contact[4] = 1;
        for (ii = 1; ii <= 4; ii++)
        {
            fk[ii] = aIn.FirstParameter();
            fk[ii] = aIn.LastParameter();
        }

        // Calculation of deformation constraints
        aIn.D1(ti[1], out p, out v);
        polesDef[1].ChangeCoord() = p1.XYZ() - p.XYZ();
        vtan = t1;
        vtan = vtan.Normalized();
        dv = vtan * (vtan * v) - v;
        polesDef[2].ChangeCoord() = (ti[4] - ti[1]) * dv.XYZ();

        aIn.D1(ti[4], out p, out v);
        polesDef[3].ChangeCoord() = p2.XYZ() - p.XYZ();
        vtan = t2;
        vtan = vtan.Normalized();
        dv = vtan * (vtan * v) - v;
        polesDef[4].ChangeCoord() = (ti[4] - ti[1]) * dv.XYZ();

        // Interpolation of constraints
        Matrix mat = new(1, 4, 1, 4);
        if (!global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(0.0, 1.0, 1, 1, mat))
        {
            throw new ConstructionErrorException();
        }

        for (jj = 1; jj <= 4; jj++)
        {
            Xyz aux = new(0.0, 0.0, 0.0);
            for (ii = 1; ii <= 4; ii++)
            {
                aux.SetLinearForm(mat[ii, jj], polesDef[ii].XYZ(), aux);
            }
            coeffs[jj] = coeffs[jj].WithXYZ(aux);
        }

        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(coeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), polesDef, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());

        // Add the deformation
        NCollection_Array1<double> k = new(1, 2);
        NCollection_Array1<int> m = new(1, 2);
        k[1] = ti[1];
        k[2] = ti[4];
        m.Init(4);

        def = new BSplineCurve3D(polesDef, k, m, 3);
        if (aIn.Degree() < 3)
        {
            aIn.IncreaseDegree(3);
        }
        else
        {
            def.IncreaseDegree(aIn.Degree());
        }

        for (ii = 2; ii < aIn.KnotCount; ii++)
        {
            def.InsertKnot(aIn.Knot(ii), aIn.Multiplicity(ii));
        }

        if (def.PoleCount != aIn.PoleCount)
        {
            throw new ConstructionErrorException("Inconsistent poles's number");
        }

        for (ii = 1; ii <= def.PoleCount; ii++)
        {
            p = aIn.Pole(ii);
            p.ChangeCoord() += def.Pole(ii).XYZ();
            aIn.SetPole(ii, p);
        }
        curve = aIn;
    }

    //=================================================================================================

    /// <summary>
    /// Extends the bounded curve to Point, after its end if After is true or before its start otherwise, with
    /// continuity Continuity (1, 2 or 3; nothing is done otherwise, nor when Point is on the curve's end). Curve
    /// receives a new B-spline curve.
    /// </summary>
    public static void ExtendCurveToPoint(ref BoundedCurve3D curve,
                                          in Point3D point,
                                          int continuity,
                                          bool after)
    {
        if (continuity < 1 || continuity > 3)
        {
            return;
        }
        int size = continuity + 2;
        double ubord, tol = 1.0e-6;
        Matrix matCoefs = new(1, size, 1, size);
        double lambda, l1;
        int ii, jj;
        Vector3D d1, d2, d3;
        Point3D p0;
        // Convert the input (preserving the parameterization if possible)
        CompCurveToBSplineCurve concat = new(curve, QuasiAngular);

        // Construction constraints
        NCollection_Array1<Xyz> cont = new(1, size);
        if (after)
        {
            ubord = curve.LastParameter();
        }
        else
        {
            ubord = curve.FirstParameter();
        }
        global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(0,
                                 1, // The Bounds
                                 continuity,
                                 0, // The constraint orders
                                 matCoefs);

        curve.D3(ubord, out p0, out d1, out d2, out d3);
        if (!after)
        { // Invert the parameterization
            d1 *= -1;
            d3 *= -1;
        }

        l1 = p0.Distance(point);
        if (l1 > tol)
        {
            // Lambda is the ratio to apply to the derivative of the curve
            // to obtain the derivative of the extension (arbitrarily fixed to the
            // length of the segment end of curve - target point.
            // We try to have on the extension the average velocity that we
            // have on the curve.
            Vector3D daux;
            Point3D pp;
            double f = curve.FirstParameter(), t, dt, norm;
            dt = (curve.LastParameter() - f) / 9;
            norm = d1.Magnitude();
            for (ii = 1, t = f + dt; ii <= 8; ii++, t += dt)
            {
                curve.D1(t, out pp, out daux);
                norm += daux.Magnitude();
            }
            norm /= 9;
            dt = d1.Magnitude() / norm;
            if ((dt < 1.5) && (dt > 0.75))
            { // The edge is within the average, keep it
                lambda = ((double)1) / Max(d1.Magnitude() / l1, tol);
            }
            else
            {
                lambda = ((double)1) / Max(norm / l1, tol);
            }
        }
        else
        {
            return; // No extension
        }

        // Optimization of Lambda
        Matrix cons = new(1, 3, 1, size);
        cons[1, 1] = p0.X();
        cons[2, 1] = p0.Y();
        cons[3, 1] = p0.Z();
        cons[1, 2] = d1.X();
        cons[2, 2] = d1.Y();
        cons[3, 2] = d1.Z();
        cons[1, size] = point.X();
        cons[2, size] = point.Y();
        cons[3, size] = point.Z();
        if (continuity >= 2)
        {
            cons[1, 3] = d2.X();
            cons[2, 3] = d2.Y();
            cons[3, 3] = d2.Z();
        }
        if (continuity >= 3)
        {
            cons[1, 4] = d3.X();
            cons[2, 4] = d3.Y();
            cons[3, 4] = d3.Z();
        }
        ComputeLambda(cons, matCoefs, l1, ref lambda);

        // Construction in the Polynomial Basis
        cont[1] = p0.XYZ();
        cont[2] = d1.XYZ() * lambda;
        if (continuity >= 2)
        {
            cont[3] = d2.XYZ() * Math.Pow(lambda, 2);
        }
        if (continuity >= 3)
        {
            cont[4] = d3.XYZ() * Math.Pow(lambda, 3);
        }
        cont[size] = point.XYZ();

        NCollection_Array1<Point3D> extrapPoles = new(1, size);
        NCollection_Array1<Point3D> extraCoeffs = new(1, size);

        Point3D pNull = new(0.0, 0.0, 0.0);
        extraCoeffs.Init(pNull);
        for (ii = 1; ii <= size; ii++)
        {
            for (jj = 1; jj <= size; jj++)
            {
                extraCoeffs[jj].ChangeCoord() += matCoefs[ii, jj] * cont[ii];
            }
        }

        // Conversion to the Bernstein Basis
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(extraCoeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), extrapPoles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());

        BezierCurve3D bezier = new(extrapPoles);

        double dist = extrapPoles[1].Distance(p0);
        bool ok;
        tol += dist;

        // Concatenation
        ok = concat.Add(bezier, tol, after);
        if (!ok)
        {
            throw new ConstructionErrorException("ExtendCurveToPoint");
        }

        // The concatenation is discarded: its curve goes to the caller unfrozen (decision 244).
        curve = concat.InternalBSplineCurve()!;
    }

    //=================================================================================================
    private static bool ExtendKPart(ref TrimmedSurface? surface,
                                    double length,
                                    bool inU,
                                    bool after)
    {

        if (surface is null)
        {
            return false;
        }

        bool ok = true;
        double uf, ul, vf, vl;
        Surface support = surface.BasisSurface();
        SurfaceKind type;

        surface.Bounds(out uf, out ul, out vf, out vl);
        SurfaceAdaptor AS = new(surface);
        type = AS.GetType();

        if (inU)
        {
            switch (type)
            {
                case Plane:
                {
                    if (after)
                    {
                        ul += length;
                    }
                    else
                    {
                        uf -= length;
                    }
                    surface = new TrimmedSurface(support, uf, ul, vf, vl);
                    break;
                }

                default:
                    ok = false;
                    break;
            }
        }
        else
        {
            switch (type)
            {
                case Plane:
                case SurfaceKind.Cylinder:
                case SurfaceOfExtrusion:
                {
                    if (after)
                    {
                        vl += length;
                    }
                    else
                    {
                        vf -= length;
                    }
                    surface = new TrimmedSurface(support, uf, ul, vf, vl);
                    break;
                }
                default:
                    ok = false;
                    break;
            }
        }

        return ok;
    }

    //=================================================================================================

    /// <summary>
    /// Extends the bounded surface by Length in U (InU) or V, after its end (After) or before its start, with
    /// continuity Continuity (0 to 3; nothing is done otherwise). Planes, cylinders and extrusions trimmed in the
    /// extended direction are re-trimmed; other surfaces become B-splines. Surface receives the new surface; the
    /// caller's surface is never changed (OCCT changes a B-spline surface in place: the port changes a copy).
    /// </summary>
    public static void ExtendSurfByLength(ref BoundedSurface surface,
                                          double length,
                                          int continuity,
                                          bool inU,
                                          bool after)
    {
        if (continuity < 0 || continuity > 3)
        {
            return;
        }
        int cont = continuity;

        // Kpart ?
        TrimmedSurface? ts = surface as TrimmedSurface;
        if (ExtendKPart(ref ts, length, inU, after))
        {
            surface = ts!;
            return;
        }

        //  BSplineSurface format with a degree sufficient for the desired continuity
        global::Kerf.Geometry.BSplineSurface? bs = surface as global::Kerf.Geometry.BSplineSurface;
        if (bs is null)
        {
            // BS = GeomConvert::SurfaceToBSplineSurface(Surface);
            double tol = global::Kerf.Engine.Foundation.Precision.Confusion(); // 1.e-4;
            Continuity uCont = C1, vCont = C1;
            int degU = 14, degV = 14;
            int nmax = 16;
            int thePrec = 1;
            Surface surf = surface; // to resolve ambiguity
            ApproxSurface theApprox = new(surf, tol, uCont, vCont, degU, degV, nmax, thePrec);
            if (theApprox.HasResult)
            {
                bs = theApprox.Surface();
            }
            else
            {
                bs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(surface);
            }
        }
        // OCCT changes the caller's B-spline surface (or ApproxSurface's frozen one) in place below: the port changes a
        // copy (decision 245), which the caller receives as OCCT's caller receives its own surface.
        bs = (global::Kerf.Geometry.BSplineSurface)bs!.Copy();
        if (inU && (bs.UDegree() < continuity + 1))
        {
            bs.IncreaseDegree(continuity + 1, bs.VDegree());
        }
        if (!inU && (bs.VDegree() < continuity + 1))
        {
            bs.IncreaseDegree(bs.UDegree(), continuity + 1);
        }

        // if BS was periodic in the extension direction, it will no longer be
        if ((inU && (bs.IsUPeriodic)) || (!inU && (bs.IsVPeriodic)))
        {
            double u0, u1, v0, v1;
            bs.Bounds(out u0, out u1, out v0, out v1);
            bs.Segment(u0, u1, v0, v1);
        }

        // IFV Fix OCC bug 0022694 - wrong result extrapolating rational surfaces
        //   bool rational = ( InU && BS->IsURational() )
        //                                   || ( !InU && BS->IsVRational() ) ;
        bool rational = (bs.IsURational || bs.IsVRational);
        double epsW = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        int gap = 3;
        if (rational)
        {
            gap++;
        }

        int cdeg = 0, cdim = 0, nbP = 0, ksize = 0, psize = 1;
        int ii, jj, ipole, kount;
        double tbord, lambmin = length;
        bool ok;
        NCollection_HArray1<double>? fKnots = null, Point = null, lambda = null, Tgte = null, poles = null;
        Span<int> extrap_mode = stackalloc int[2];

        for (kount = 0, ok = false; kount <= 2 && !ok; kount++)
        {
            //  Transform the surface into a non-rational BSpline in one variable
            //  of degree UDegree or VDegree and dimension 3 or 4 x NbVpoles or NbUpoles
            //  the number of poles equals NbUpoles or NbVpoles
            //  WARNING: in the rational case, a point with coordinates (x,y,z)
            //           and weight w becomes a point with coordinates (wx, wy, wz, w)

            if (inU)
            {
                cdeg = bs.UDegree();
                nbP = bs.UPoleCount;
                cdim = bs.VPoleCount * gap;
            }
            else
            {
                cdeg = bs.VDegree();
                nbP = bs.VPoleCount;
                cdim = bs.UPoleCount * gap;
            }

            //  the flat knots
            ksize = nbP + cdeg + 1;
            if (inU)
            {
                fKnots = new NCollection_HArray1<double>(bs.InternalUKnotSequence());
            }
            else
            {
                fKnots = new NCollection_HArray1<double>(bs.InternalVKnotSequence());
            }

            //  the parameter of the connection knot
            if (after)
            {
                tbord = fKnots.Value(fKnots.Upper() - cdeg);
            }
            else
            {
                tbord = fKnots.Value(fKnots.Lower() + cdeg);
            }

            //  the poles
            psize = cdim * nbP;
            poles = new NCollection_HArray1<double>(1, psize);

            if (inU)
            {
                for (ii = 1, ipole = 1; ii <= nbP; ii++)
                {
                    for (jj = 1; jj <= bs.VPoleCount; jj++)
                    {
                        poles.SetValue(ipole, bs.Pole(ii, jj).X());
                        poles.SetValue(ipole + 1, bs.Pole(ii, jj).Y());
                        poles.SetValue(ipole + 2, bs.Pole(ii, jj).Z());
                        if (rational)
                        {
                            poles.SetValue(ipole + 3, bs.Weight(ii, jj));
                        }
                        ipole += gap;
                    }
                }
            }
            else
            {
                for (jj = 1, ipole = 1; jj <= nbP; jj++)
                {
                    for (ii = 1; ii <= bs.UPoleCount; ii++)
                    {
                        poles.SetValue(ipole, bs.Pole(ii, jj).X());
                        poles.SetValue(ipole + 1, bs.Pole(ii, jj).Y());
                        poles.SetValue(ipole + 2, bs.Pole(ii, jj).Z());
                        if (rational)
                        {
                            poles.SetValue(ipole + 3, bs.Weight(ii, jj));
                        }
                        ipole += gap;
                    }
                }
            }
            // OCCT's Padr points at Poles(1): Poles.AsSpan() below

            //  calculation of the connection point and tangent
            Point = new NCollection_HArray1<double>(1, cdim);
            Tgte = new NCollection_HArray1<double>(1, cdim);
            lambda = new NCollection_HArray1<double>(1, cdim);

            bool periodic_flag = false;
            int derivative_request = Max(continuity, 1);
            extrap_mode[0] = extrap_mode[1] = cdeg;
            NCollection_Array1<double> result = new(1, cdim * (derivative_request + 1));

            NCollection_Array1<double> tgte = Tgte.ChangeArray1();
            NCollection_Array1<double> point = Point.ChangeArray1();
            NCollection_Array1<double> lamb = lambda.ChangeArray1();

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(tbord,
                          periodic_flag,
                          derivative_request,
                          extrap_mode,
                          cdeg,
                          fKnots.Array1(),
                          cdim,
                          poles.AsSpan(),
                          result.AsSpan());
            ok = true;
            for (ii = 1; ii <= cdim; ii++)
            {
                point[ii] = result[ii];
                tgte[ii] = result[ii + cdim];
            }

            //  calculation of the constraint to reach

            Vector3D curT = new(), oldT = new();

            double nTgte, val, tgtol = 1.0e-12, oldN = 0.0;
            if (rational)
            {
                for (ii = gap; ii <= cdim; ii += gap)
                {
                    tgte[ii] = 0.0;
                }
                for (ii = gap; ii <= cdim; ii += gap)
                {
                    curT = curT.WithCoord(tgte[ii - 3], tgte[ii - 2], tgte[ii - 1]);
                    nTgte = curT.Magnitude();
                    if (nTgte > tgtol)
                    {
                        val = length / nTgte;
                        // Attention to cases where the segment given by the poles
                        // is opposite to the direction of the derivative
                        // Example: Certain portions of torus.
                        if ((oldN > tgtol) && (curT.Angle(oldT) > 2))
                        {
                            ok = false;
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
                    oldT = curT;
                    oldN = nTgte;
                }
            }
            else
            {
                for (ii = gap; ii <= cdim; ii += gap)
                {
                    curT = curT.WithCoord(tgte[ii - 2], tgte[ii - 1], tgte[ii]);
                    nTgte = curT.Magnitude();
                    if (nTgte > tgtol)
                    {
                        val = length / nTgte;
                        // Attention to cases where the segment given by the poles
                        // is opposite to the direction of the derivative
                        // Example: Certain portions of torus.
                        if ((oldN > tgtol) && (curT.Angle(oldT) > 2))
                        {
                            ok = false;
                        }
                        lamb[ii] = lamb[ii - 1] = lamb[ii - 2] = val;
                        lambmin = Min(lambmin, val);
                    }
                    else
                    {
                        lamb[ii] = lamb[ii - 1] = lamb[ii - 2] = 0.0;
                    }
                    oldT = curT;
                    oldN = nTgte;
                }
            }
            if (!ok && kount < 2)
            {
                // We increase the degree of the border iso to bring the poles of the surface closer
                // And we retry
                if (inU)
                {
                    bs.IncreaseDegree(bs.UDegree(), bs.VDegree() + 2);
                }
                else
                {
                    bs.IncreaseDegree(bs.UDegree() + 2, bs.VDegree());
                }
            }
        }

        NCollection_Array1<double> constraintPoint = new(1, cdim);
        if (after)
        {
            for (ii = 1; ii <= cdim; ii++)
            {
                constraintPoint[ii] = Point!.Value(ii) + lambda!.Value(ii) * Tgte!.Value(ii);
            }
        }
        else
        {
            for (ii = 1; ii <= cdim; ii++)
            {
                constraintPoint[ii] = Point!.Value(ii) - lambda!.Value(ii) * Tgte!.Value(ii);
            }
        }

        //  special case of rational
        if (rational)
        {
            for (ipole = 1; ipole <= psize; ipole += gap)
            {
                poles![ipole] *= poles.Value(ipole + 3);
                poles[ipole + 1] *= poles.Value(ipole + 3);
                poles[ipole + 2] *= poles.Value(ipole + 3);
            }
            for (ii = 1; ii <= cdim; ii += gap)
            {
                constraintPoint[ii] *= constraintPoint[ii + 3];
                constraintPoint[ii + 1] *= constraintPoint[ii + 3];
                constraintPoint[ii + 2] *= constraintPoint[ii + 3];
            }
        }

        //  arrays needed for the extension
        int ksize2 = ksize + cdeg, nbPoles, nbKnots = 0;
        NCollection_Array1<double> fk = new(1, ksize2);

        int psize2 = psize + cdeg * cdim;
        NCollection_Array1<double> pRes = new(1, psize2);
        double ww;
        bool extOk = false;
        NCollection_HArray2<Point3D>? newPoles = null;
        NCollection_HArray2<double>? newWeights = null;

        for (kount = 1; kount <= 5 && !extOk; kount++)
        {
            //  extension
            global::Kerf.Engine.Geometry.Splines.BSplCLib.TangExtendToConstraint(fKnots!.Array1(),
                                            lambmin,
                                            nbP,
                                            poles!.AsSpan(),
                                            cdim,
                                            cdeg,
                                            constraintPoint,
                                            cont,
                                            after,
                                            out nbPoles,
                                            out nbKnots,
                                            fk.AsSpan(),
                                            pRes.AsSpan());

            //  Copy the result poles as 3D points and weights
            int nu, nv, indice;
            if (inU)
            {
                nu = nbPoles;
                nv = bs.VPoleCount;
            }
            else
            {
                nu = bs.UPoleCount;
                nv = nbPoles;
            }

            newPoles = new NCollection_HArray2<Point3D>(1, nu, 1, nv);
            NCollection_Array2<Point3D> newP = newPoles.ChangeArray2();
            newWeights = new NCollection_HArray2<double>(1, nu, 1, nv);
            NCollection_Array2<double> newW = newWeights.ChangeArray2();

            if (!rational)
            {
                newW.Init(1.0);
            }
            bool nullWeight = false;

            if (inU)
            {
                for (ii = 1; ii <= nu && !nullWeight; ii++)
                {
                    for (jj = 1; jj <= nv && !nullWeight; jj++)
                    {
                        indice = 1 + (ii - 1) * cdim + (jj - 1) * gap;
                        newP[ii, jj] = newP[ii, jj].WithCoord(1, pRes[indice]);
                        newP[ii, jj] = newP[ii, jj].WithCoord(2, pRes[indice + 1]);
                        newP[ii, jj] = newP[ii, jj].WithCoord(3, pRes[indice + 2]);
                        if (rational)
                        {
                            ww = pRes[indice + 3];
                            if (Math.Abs(ww - 1.0) < epsW)
                            {
                                ww = 1.0;
                            }
                            if (ww < epsW)
                            {
                                nullWeight = true;
                            }
                            else
                            {
                                newW[ii, jj] = ww;
                                newP[ii, jj].ChangeCoord() /= ww;
                            }
                        }
                    }
                }
            }
            else
            {
                for (jj = 1; jj <= nv && !nullWeight; jj++)
                {
                    for (ii = 1; ii <= nu && !nullWeight; ii++)
                    {
                        indice = 1 + (ii - 1) * gap + (jj - 1) * cdim;
                        newP[ii, jj] = newP[ii, jj].WithCoord(1, pRes[indice]);
                        newP[ii, jj] = newP[ii, jj].WithCoord(2, pRes[indice + 1]);
                        newP[ii, jj] = newP[ii, jj].WithCoord(3, pRes[indice + 2]);
                        if (rational)
                        {
                            ww = pRes[indice + 3];
                            if (Math.Abs(ww - 1.0) < epsW)
                            {
                                ww = 1.0;
                            }
                            if (ww < epsW)
                            {
                                nullWeight = true;
                            }
                            else
                            {
                                newW[ii, jj] = ww;
                                newP[ii, jj].ChangeCoord() /= ww;
                            }
                        }
                    }
                }
            }

            if (nullWeight)
            {
                lambmin /= 3.0;
            }
            else
            {
                extOk = true;
            }
        }

        // Copy flat knots as knots with their multiplicities.
        // Compute the degrees of the result.
        int usize = bs.UKnotCount, vsize = bs.VKnotCount, uDeg, vDeg;
        if (inU)
        {
            usize++;
        }
        else
        {
            vsize++;
        }
        NCollection_Array1<double> uKnots = new(1, usize);
        NCollection_Array1<double> vKnots = new(1, vsize);
        NCollection_Array1<int> uMults = new(1, usize);
        NCollection_Array1<int> vMults = new(1, vsize);
        NCollection_Array1<double> fkRes = new(1, nbKnots);

        for (ii = 1; ii <= nbKnots; ii++)
        {
            fkRes[ii] = fk[ii];
        }

        if (inU)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Knots(fkRes, uKnots, uMults);
            uDeg = cdeg;
            uMults[usize] = uDeg + 1; // Petite verrue utile quand la continuite
                                      // n'est pas ok.
            vKnots.Assign(bs.InternalVKnots());
            vMults.Assign(bs.InternalVMultiplicities());
            vDeg = bs.VDegree();
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Knots(fkRes, vKnots, vMults);
            vDeg = cdeg;
            vMults[vsize] = vDeg + 1;
            uKnots.Assign(bs.InternalUKnots());
            uMults.Assign(bs.InternalUMultiplicities());
            uDeg = bs.UDegree();
        }

        //  Construct the resulting BSpline surface
        global::Kerf.Geometry.BSplineSurface res = new(newPoles!.Array2(),
                                      newWeights!.Array2(),
                                      uKnots,
                                      vKnots,
                                      uMults,
                                      vMults,
                                      uDeg,
                                      vDeg,
                                      bs.IsUPeriodic,
                                      bs.IsVPeriodic);
        surface = res;
    }

    //=================================================================================================

    /// <summary>
    /// Computes the principal axes of inertia of some points and their dispersions: XDir is the axis of largest
    /// inertia, YDir the next, and Xgap, Ygap, Zgap the square roots of the three eigenvalues.
    /// </summary>
    public static void Inertia(NCollection_Array1<Point3D> points,
                               out Point3D bary,
                               out Direction3D xDir,
                               out Direction3D yDir,
                               out double xgap,
                               out double ygap,
                               out double zgap)
    {
        Xyz gb = new(0.0, 0.0, 0.0), diff = new();
        //  gp_Vec A,B,C,D;

        int i, nb = points.Length();
        gb.SetCoord(0.0, 0.0, 0.0);
        for (i = 1; i <= nb; i++)
        {
            gb += points[i].XYZ();
        }

        gb /= nb;

        Matrix m = new(1, 3, 1, 3);
        m.Init(0.0);
        for (i = 1; i <= nb; i++)
        {
            diff.SetLinearForm(-1, points[i].XYZ(), gb);
            m[1, 1] += diff.X() * diff.X();
            m[2, 2] += diff.Y() * diff.Y();
            m[3, 3] += diff.Z() * diff.Z();
            m[1, 2] += diff.X() * diff.Y();
            m[1, 3] += diff.X() * diff.Z();
            m[2, 3] += diff.Y() * diff.Z();
        }

        m[2, 1] = m[1, 2];
        m[3, 1] = m[1, 3];
        m[3, 2] = m[2, 3];

        m /= nb;

        Jacobi j = new(m);
        // OCCT reports a failure of Jacobi in debug builds only.

        double n1, n2, n3;

        n1 = j.Value(1);
        n2 = j.Value(2);
        n3 = j.Value(3);

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

        math_Vector v2 = new(1, 3), v3 = new(1, 3);
        j.Vector(m2, v2);
        j.Vector(m3, v3);

        bary = new Point3D();
        bary = bary.WithXYZ(gb);
        xDir = new Direction3D();
        xDir = xDir.WithCoord(v3[1], v3[2], v3[3]);
        yDir = new Direction3D();
        yDir = yDir.WithCoord(v2[1], v2[2], v2[3]);

        zgap = Math.Sqrt(Math.Abs(j.Value(m1)));
        ygap = Math.Sqrt(Math.Abs(j.Value(m2)));
        xgap = Math.Sqrt(Math.Abs(j.Value(m3)));
    }

    /// <summary>
    /// Computes the axes of inertia of some points: the location is their barycentre, the X direction the axis of
    /// largest inertia, the main direction the normal to the mean plane. IsSingular is true if the points are on a
    /// line (within Tol).
    /// </summary>
    public static void AxeOfInertia(NCollection_Array1<Point3D> points,
                                    out Frame3D Axe,
                                    out bool isSingular,
                                    double tol = 1.0e-7)
    {
        Point3D bary;
        Direction3D ox, oy, oz;
        double gx, gy, gz;

        global::Kerf.Engine.Geometry.Library.GeomLib.Inertia(points, out bary, out ox, out oy, out gx, out gy, out gz);

        if (gy * points.Length() <= tol)
        {
            Frame3D axe = new(bary, ox);
            oy = axe.XDirection();
            isSingular = true;
        }
        else
        {
            isSingular = false;
        }

        oz = ox ^ oy;
        Frame3D theAxe = new(bary, oz, ox);
        Axe = theAxe;
    }

    //=================================================================================================

    private static bool CanBeTreated(global::Kerf.Geometry.BSplineSurface bSurf)

    {
        int i;
        double lambda; // proportionnality coefficient
        bool alreadyTreated = true;

        if (!bSurf.IsURational || (bSurf.IsUPeriodic))
        {
            return false;
        }
        else
        {
            lambda = (bSurf.Weight(1, 1) / bSurf.Weight(bSurf.UPoleCount, 1));
            for (i = 1; i <= bSurf.VPoleCount; i++)
            { //test of the proportionnality of the denominator on the boundaries
                if ((bSurf.Weight(1, i) / (lambda * bSurf.Weight(bSurf.UPoleCount, i))
                     < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                    || (bSurf.Weight(1, i) / (lambda * bSurf.Weight(bSurf.UPoleCount, i))
                        > (1 + global::Kerf.Engine.Foundation.Precision.Confusion())))
                {
                    return false;
                }
            }
            i = 1;
            while ((alreadyTreated) && (i <= bSurf.VPoleCount))
            { // tests if the surface has already been treated
                if (((bSurf.Weight(1, i) / (bSurf.Weight(2, i))) < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                    || ((bSurf.Weight(1, i) / (bSurf.Weight(2, i))) > (1 + global::Kerf.Engine.Foundation.Precision.Confusion()))
                    || ((bSurf.Weight(bSurf.UPoleCount - 1, i) / (bSurf.Weight(bSurf.UPoleCount, i)))
                        < (1 - global::Kerf.Engine.Foundation.Precision.Confusion()))
                    || ((bSurf.Weight(bSurf.UPoleCount - 1, i) / (bSurf.Weight(bSurf.UPoleCount, i)))
                        > (1 + global::Kerf.Engine.Foundation.Precision.Confusion())))
                {
                    alreadyTreated = false;
                }
                i++;
            }
            if (alreadyTreated)
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    private sealed class LawEvaluator : SurfaceEvaluatorFunction
    {
        public LawEvaluator(DenominatorMultiplier? denominatorPtr)
        {
            _denominator = denominatorPtr;
        }

        public override void Evaluate(int derivativeRequest,
                                      double uParameter,
                                      double vParameter,
                                      out double result,
                                      out int errorCode)
        {
            if ((_denominator is not null) && (derivativeRequest == 0))
            {
                result = _denominator.Value(uParameter, vParameter);
                errorCode = 0;
            }
            else
            {
                // OCCT leaves the result unset (decision 32)
                result = 0.0;
                errorCode = 1;
            }
        }

        private readonly DenominatorMultiplier? _denominator;
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
                                      out NCollection_HArray1<double> resultKnots,
                                      out NCollection_HArray1<int> resultMults)

    {
        int i;

        if (CheckIfKnotExists(surface_knots, knotmin) && CheckIfKnotExists(surface_knots, knotmax))
        {
            resultKnots = new NCollection_HArray1<double>(1, surface_knots.Length());
            resultMults = new NCollection_HArray1<int>(1, surface_knots.Length());
            for (i = 1; i <= surface_knots.Length(); i++)
            {
                resultKnots.SetValue(i, surface_knots[i]);
                resultMults.SetValue(i, surface_mults[i] + deltasurface_degree);
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
                         out resultKnots,
                         out resultMults);
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
                             out resultKnots,
                             out resultMults);
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
                                 out resultKnots,
                                 out resultMults);
                    }
                    else
                    {
                        NCollection_HArray1<double> intermedKnots;
                        NCollection_HArray1<int> intermedMults;
                        AddAKnot(surface_knots,
                                 surface_mults,
                                 knotmin,
                                 deltasurface_degree,
                                 finalsurface_degree,
                                 out intermedKnots,
                                 out intermedMults);
                        AddAKnot(intermedKnots.ChangeArray1(),
                                 intermedMults.ChangeArray1(),
                                 knotmax,
                                 0,
                                 finalsurface_degree,
                                 out resultKnots,
                                 out resultMults);
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static void FunctionMultiply(ref global::Kerf.Geometry.BSplineSurface bSurf,
                                         double knotmin,
                                         double knotmax)

    {
        NCollection_Array1<double> surface_u_knots = bSurf.InternalUKnots();
        NCollection_Array1<int> surface_u_mults = bSurf.InternalUMultiplicities();
        NCollection_Array1<double> surface_v_knots = bSurf.InternalVKnots();
        NCollection_Array1<int> surface_v_mults = bSurf.InternalVMultiplicities();
        NCollection_Array2<Point3D> surface_poles = bSurf.InternalPoles();
        NCollection_Array2<double>? surface_weights = bSurf.InternalWeights();
        int i, j, k, status, new_num_u_poles, new_num_v_poles, length = 0;
        NCollection_HArray1<double> newuknots, newvknots;
        NCollection_HArray1<int> newumults, newvmults;

        NCollection_Array1<double> knots = new(1, 2);
        NCollection_Array1<int> mults = new(1, 2);
        NCollection_HArray1<double> newKnots;
        NCollection_HArray1<int> newMults;

        knots[1] = 0;
        knots[2] = 1;
        mults[1] = 4;
        mults[2] = 4;
        BuildFlatKnot(knots, mults, 0, 3, knotmin, knotmax, out newKnots, out newMults);

        for (i = 1; i <= newMults.Length(); i++)
        {
            length += newMults.Value(i);
        }
        NCollection_Array1<double> flatKnots = new(1, length);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(newKnots.ChangeArray1(), newMults.ChangeArray1(), flatKnots);

        DenominatorMultiplier denominator = new(bSurf, flatKnots);

        BuildFlatKnot(surface_u_knots,
                      surface_u_mults,
                      3,
                      bSurf.UDegree() + 3,
                      knotmin,
                      knotmax,
                      out newuknots,
                      out newumults);
        BuildFlatKnot(surface_v_knots,
                      surface_v_mults,
                      bSurf.VDegree(),
                      2 * (bSurf.VDegree()),
                      1.0,
                      0.0,
                      out newvknots,
                      out newvmults);
        length = 0;
        for (i = 1; i <= newumults.Length(); i++)
        {
            length += newumults.Value(i);
        }
        new_num_u_poles = (length - bSurf.UDegree() - 3 - 1);
        NCollection_Array1<double> newuflatknots = new(1, length);
        length = 0;
        for (i = 1; i <= newvmults.Length(); i++)
        {
            length += newvmults.Value(i);
        }
        new_num_v_poles = (length - 2 * bSurf.VDegree() - 1);
        NCollection_Array1<double> newvflatknots = new(1, length);

        NCollection_Array2<Point3D> newNumerator = new(1, new_num_u_poles, 1, new_num_v_poles);
        NCollection_Array2<double> newDenominator = new(1, new_num_u_poles, 1, new_num_v_poles);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(newuknots.ChangeArray1(), newumults.ChangeArray1(), newuflatknots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(newvknots.ChangeArray1(), newvmults.ChangeArray1(), newvflatknots);
        // POP for WNT
        LawEvaluator ev = new(denominator);
        // BSplSLib::FunctionMultiply(LawEvaluator,               //multiplication
        SurfaceBSplSLib.FunctionMultiply(ev, // multiplication
                                  bSurf.UDegree(),
                                  bSurf.VDegree(),
                                  surface_u_knots,
                                  surface_v_knots,
                                  surface_u_mults,
                                  surface_v_mults,
                                  surface_poles,
                                  surface_weights,
                                  newuflatknots,
                                  newvflatknots,
                                  bSurf.UDegree() + 3,
                                  2 * (bSurf.VDegree()),
                                  newNumerator,
                                  newDenominator,
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
                    newNumerator[i, j] = newNumerator[i, j].WithCoord(k, newNumerator[i, j].Coord(k) / newDenominator[i, j]);
                }
            }
        }
        bSurf = new global::Kerf.Geometry.BSplineSurface(newNumerator,
                                        newDenominator,
                                        newuknots.ChangeArray1(),
                                        newvknots.ChangeArray1(),
                                        newumults.ChangeArray1(),
                                        newvmults.ChangeArray1(),
                                        bSurf.UDegree() + 3,
                                        2 * (bSurf.VDegree()));
    }

    //=================================================================================================

    private static void CancelDenominatorDerivative1D(ref global::Kerf.Geometry.BSplineSurface bSurf)

    {
        int i, j;
        double uknotmin = 1.0, uknotmax = 0.0, x, y, startu_value, endu_value;

        startu_value = bSurf.UKnot(1);
        endu_value = bSurf.UKnot(bSurf.UKnotCount);
        NCollection_Array1<double> BSurf_u_knots = new(bSurf.InternalUKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, BSurf_u_knots);
        bSurf.SetUKnots(BSurf_u_knots); // reparametrisation of the surface
        BSplineCurve3D bCurve;
        NCollection_Array1<double> bCurveWeights = new(1, bSurf.UPoleCount);
        NCollection_Array1<Point3D> bCurvePoles = new(1, bSurf.UPoleCount);

        // OCCT: when the surface can't be treated, it is left reparametrised to [0, 1]: the knots are restored only
        // after FunctionMultiply (occt-issues.md).
        if (CanBeTreated(bSurf))
        {
            NCollection_Array1<double> bCurveKnots = bSurf.InternalUKnots();
            NCollection_Array1<int> bCurveMults = bSurf.InternalUMultiplicities();
            for (i = 1; i <= bSurf.VPoleCount; i++)
            { // loop on each pole function
                x = 1.0;
                y = 0.0;
                for (j = 1; j <= bSurf.UPoleCount; j++)
                {
                    bCurveWeights[j] = bSurf.Weight(j, i);
                    bCurvePoles[j] = bSurf.Pole(j, i);
                }
                bCurve = new BSplineCurve3D(bCurvePoles, // building of a pole function
                                               bCurveWeights,
                                               bCurveKnots,
                                               bCurveMults,
                                               bSurf.UDegree());
                global::Kerf.Engine.Approximation.Hermit.Solutionbis(bCurve, ref x, ref y, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Confusion());
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

            FunctionMultiply(ref bSurf, uknotmin, uknotmax); // multiplication

            BSurf_u_knots.Assign(bSurf.InternalUKnots());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(startu_value, endu_value, BSurf_u_knots);
            bSurf.SetUKnots(BSurf_u_knots);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Cancels, on the boundaries, the first derivative of the denominator of a rational surface in the requested
    /// directions, and sets its value to 1. BSurf receives a new surface; the caller's is never changed (OCCT changes
    /// it in place: the port changes a copy).
    /// </summary>
    public static void CancelDenominatorDerivative(ref global::Kerf.Geometry.BSplineSurface bSurf,
                                                   bool udirection,
                                                   bool vdirection)

    {
        if (udirection || vdirection)
        {
            // OCCT changes the caller's surface in place: the port changes a copy (decision 245).
            bSurf = (global::Kerf.Geometry.BSplineSurface)bSurf.Copy();
        }
        if (udirection && !vdirection)
        {
            CancelDenominatorDerivative1D(ref bSurf);
        }
        else
        {
            if (!udirection && vdirection)
            {
                bSurf.ExchangeUV();
                CancelDenominatorDerivative1D(ref bSurf);
                bSurf.ExchangeUV();
            }
            else
            {
                if (udirection && vdirection)
                { // optimize the treatment
                    if (bSurf.UDegree() <= bSurf.VDegree())
                    {
                        CancelDenominatorDerivative1D(ref bSurf);
                        bSurf.ExchangeUV();
                        CancelDenominatorDerivative1D(ref bSurf);
                        bSurf.ExchangeUV();
                    }
                    else
                    {
                        bSurf.ExchangeUV();
                        CancelDenominatorDerivative1D(ref bSurf);
                        bSurf.ExchangeUV();
                        CancelDenominatorDerivative1D(ref bSurf);
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
    public static int NormEstim(Surface surf,
                                in Point2D uv,
                                double tol,
                                ref Direction3D norm)
    {
        double tol2 = Square(tol);

        Vector3D du, dv;
        Point3D dummyPnt;
        surf.D1(uv.X(), uv.Y(), out dummyPnt, out du, out dv);

        double mdu = du.SquareMagnitude(), mdv = dv.SquareMagnitude();
        if (mdu >= tol2 && mdv >= tol2)
        {
            Vector3D aNorm = du ^ dv;
            double magn = aNorm.SquareMagnitude();
            if (magn < tol2)
            {
                return 3;
            }

            norm = norm.WithXYZ(aNorm.XYZ());
            return 0;
        }

        Vector3D d2U, d2V, d2UV;
        bool isDone = false;
        NormalStatus status;
        Direction3D normal = new();

        surf.D2(uv.X(), uv.Y(), out dummyPnt, out du, out dv, out d2U, out d2V, out d2UV);
        global::Kerf.Engine.Geometry.CSLib.Normal(du, dv, d2U, d2V, d2UV, tol, out isDone, out status, ref normal);
        if (!isDone)
        {
            // computation is impossible
            return status == D1NIsNull ? 2 : 3;
        }

        double umin, umax, vmin, vmax;
        double step = 1.0e-5;
        double eps = 1.0e-16;
        double sign = -1.0;
        surf.Bounds(out umin, out umax, out vmin, out vmax);

        // check for cone apex singularity point
        if ((uv.Y() > vmin + step) && (uv.Y() < vmax - step))
        {
            Direction3D normal1, normal2;
            double coneSingularityAngleEps = 1.0e-4;
            surf.D1(uv.X(), uv.Y() - sign * step, out dummyPnt, out du, out dv);
            if ((du.XYZ().SquareModulus() > eps) && (dv.XYZ().SquareModulus() > eps))
            {
                normal1 = new Direction3D(du ^ dv);
                surf.D1(uv.X(), uv.Y() + sign * step, out dummyPnt, out du, out dv);
                if ((du.XYZ().SquareModulus() > eps) && (dv.XYZ().SquareModulus() > eps))
                {
                    normal2 = new Direction3D(du ^ dv);
                    if (normal1.IsOpposite(normal2, coneSingularityAngleEps))
                    {
                        return 2;
                    }
                }
            }
        }

        // Along V
        if (mdu < tol2 && mdv >= tol2)
        {
            if ((vmax - uv.Y()) > (uv.Y() - vmin))
            {
                sign = 1.0;
            }

            surf.D1(uv.X(), uv.Y() + sign * step, out dummyPnt, out du, out dv);
            Vector3D Norm = du ^ dv;
            if (Norm.SquareMagnitude() < eps)
            {
                double sign1 = -1.0;
                if ((umax - uv.X()) > (uv.X() - umin))
                {
                    sign1 = 1.0;
                }
                surf.D1(uv.X() + sign1 * step, uv.Y() + sign * step, out dummyPnt, out du, out dv);
                Norm = du ^ dv;
            }
            if (Norm.SquareMagnitude() >= eps && Norm.Dot(new Vector3D(normal)) < 0.0)
            {
                normal = normal.Reversed();
            }
        }

        // Along U
        if (mdv < tol2 && mdu >= tol2)
        {
            if ((umax - uv.X()) > (uv.X() - umin))
            {
                sign = 1.0;
            }

            surf.D1(uv.X() + sign * step, uv.Y(), out dummyPnt, out du, out dv);
            Vector3D Norm = du ^ dv;
            if (Norm.SquareMagnitude() < eps)
            {
                double sign1 = -1.0;
                if ((vmax - uv.Y()) > (uv.Y() - vmin))
                {
                    sign1 = 1.0;
                }

                surf.D1(uv.X() + sign * step, uv.Y() + sign1 * step, out dummyPnt, out du, out dv);
                Norm = du ^ dv;
            }
            if (Norm.SquareMagnitude() >= eps && Norm.Dot(new Vector3D(normal)) < 0.0)
            {
                normal = normal.Reversed();
            }
        }

        // quasysingular
        if (status == D1NuIsNull || status == D1NvIsNull
            || status == D1NuIsParallelD1Nv)
        {
            norm = norm.WithXYZ(normal.XYZ());
            return 1;
        }

        return status == InfinityOfSolutions ? 2 : 3;
    }

    //=================================================================================================

    /// <summary>
    /// Whether the opposite boundaries of the surface coincide within Tol, in U and in V.
    /// </summary>
    public static void IsClosed(Surface s,
                                double tol,
                                out bool isUClosed,
                                out bool isVClosed)
    {
        isUClosed = false;
        isVClosed = false;
        //
        SurfaceAdaptor gas = new(s);
        SurfaceKind sType = gas.GetType();
        //
        double u1, u2, v1, v2;
        u1 = gas.FirstUParameter();
        u2 = gas.LastUParameter();
        v1 = gas.FirstVParameter();
        v2 = gas.LastVParameter();
        //
        double tol2 = tol * tol;
        switch (sType)
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
                Point3D p1 = gas.Value(u1, v1);
                Point3D p2 = gas.Value(u2, v1);
                isUClosed = p1.SquareDistance(p2) <= tol2;
                return;
            }
            case SurfaceKind.Cone:
            {
                // find v with maximal distance from axis
                if (!(global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2)))
                {
                    Cone cone = gas.Cone();
                    Point3D apex = cone.Apex();
                    Point3D P1 = gas.Value(u1, v1);
                    Point3D P2 = gas.Value(u1, v2);
                    if (P2.SquareDistance(apex) > P1.SquareDistance(apex))
                    {
                        v1 = v2;
                    }
                }
                else
                {
                    v1 = 0.0;
                }
                Point3D p1 = gas.Value(u1, v1);
                Point3D p2 = gas.Value(u2, v1);
                isUClosed = p1.SquareDistance(p2) <= tol2;
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
                Point3D p1 = gas.Value(u1, v1);
                Point3D p2 = gas.Value(u2, v1);
                isUClosed = p1.SquareDistance(p2) <= tol2;
                return;
            }
            case SurfaceKind.Torus:
            {
                double ures = gas.UResolution(tol);
                double vres = gas.VResolution(tol);
                //
                isUClosed = (u2 - u1) >= gas.UPeriod() - ures;
                isVClosed = (v2 - v1) >= gas.VPeriod() - vres;
                return;
            }
            case SurfaceKind.BSplineSurface:
            {
                global::Kerf.Geometry.BSplineSurface bSpl = gas.BSpline();
                isUClosed = global::Kerf.Engine.Geometry.Library.GeomLib.IsBSplUClosed(bSpl, u1, u2, tol);
                isVClosed = global::Kerf.Engine.Geometry.Library.GeomLib.IsBSplVClosed(bSpl, v1, v2, tol);
                return;
            }
            case SurfaceKind.BezierSurface:
            {
                global::Kerf.Geometry.BezierSurface bz = gas.Bezier();
                isUClosed = global::Kerf.Engine.Geometry.Library.GeomLib.IsBzUClosed(bz, u1, u2, tol);
                isVClosed = global::Kerf.Engine.Geometry.Library.GeomLib.IsBzVClosed(bz, v1, v2, tol);
                return;
            }
            case SurfaceOfRevolution:
            case SurfaceKind.OffsetSurface:
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
                if (sType == SurfaceKind.OffsetSurface || sType == OtherSurface)
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
                double res = Max(gas.UResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
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
                    Point3D p1 = gas.Value(u1, t);
                    Point3D p2 = gas.Value(u2, t);
                    if (p1.SquareDistance(p2) > tol2)
                    {
                        isUClosed = false;
                        break;
                    }
                }
                //
                nbp = 23;
                isVClosed = true;
                dt = (u2 - u1) / (nbp - 1);
                res = Max(gas.VResolution(tol), global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (dt <= res)
                {
                    nbp = RealToInt((u2 - u1) / (2.0 * res)) + 1;
                    nbp = Max(nbp, 2);
                    dt = (u2 - u1) / (nbp - 1);
                }
                for (i = 0; i < nbp; ++i)
                {
                    t = (i == nbp - 1 ? u2 : u1 + i * dt);
                    Point3D p1 = gas.Value(t, v1);
                    Point3D p2 = gas.Value(t, v2);
                    if (p1.SquareDistance(p2) > tol2)
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
    public static bool IsBSplUClosed(global::Kerf.Geometry.BSplineSurface s,
                                     double u1,
                                     double u2,
                                     double tol)
    {
        Curve3D? cuf = s.UIso(u1);
        Curve3D? cul = s.UIso(u2);
        if (cuf is null || cul is null)
        {
            return false;
        }
        double tol2 = 2.0 * tol;
        BSplineCurve3D bsF = (BSplineCurve3D)cuf;
        BSplineCurve3D bsL = (BSplineCurve3D)cul;
        NCollection_Array1<Point3D> pf = bsF.InternalPoles();
        NCollection_Array1<Point3D> pl = bsL.InternalPoles();
        NCollection_Array1<double>? wf = bsF.InternalWeights();
        NCollection_Array1<double>? wl = bsL.InternalWeights();
        return CompareWeightPoles(pf, wf, pl, wl, tol2);
    }

    //=================================================================================================

    /// <summary>
    /// Whether the poles of the V1 and V2 isolines of the surface are equal within Tol (for rational surfaces, the
    /// weighted poles Weights(i)·Poles(i)).
    /// </summary>
    public static bool IsBSplVClosed(global::Kerf.Geometry.BSplineSurface s,
                                     double v1,
                                     double v2,
                                     double tol)
    {
        Curve3D? cvf = s.VIso(v1);
        Curve3D? cvl = s.VIso(v2);
        if (cvf is null || cvl is null)
        {
            return false;
        }
        double tol2 = 2.0 * tol;
        BSplineCurve3D bsF = (BSplineCurve3D)cvf;
        BSplineCurve3D bsL = (BSplineCurve3D)cvl;
        NCollection_Array1<Point3D> pf = bsF.InternalPoles();
        NCollection_Array1<Point3D> pl = bsL.InternalPoles();
        NCollection_Array1<double>? wf = bsF.InternalWeights();
        NCollection_Array1<double>? wl = bsL.InternalWeights();
        return CompareWeightPoles(pf, wf, pl, wl, tol2);
    }

    //=================================================================================================

    /// <summary>Whether the poles of the U1 and U2 isolines of the Bezier surface are equal within Tol.</summary>
    public static bool IsBzUClosed(global::Kerf.Geometry.BezierSurface s,
                                   double u1,
                                   double u2,
                                   double tol)
    {
        Curve3D? cuf = s.UIso(u1);
        Curve3D? cul = s.UIso(u2);
        if (cuf is null || cul is null)
        {
            return false;
        }
        double tol2 = 2.0 * tol;
        BezierCurve3D bzF = (BezierCurve3D)cuf;
        BezierCurve3D bzL = (BezierCurve3D)cul;
        NCollection_Array1<Point3D> pf = bzF.InternalPoles();
        NCollection_Array1<Point3D> pl = bzL.InternalPoles();
        //
        return CompareWeightPoles(pf, null, pl, null, tol2);
    }

    //=================================================================================================

    /// <summary>Whether the poles of the V1 and V2 isolines of the Bezier surface are equal within Tol.</summary>
    public static bool IsBzVClosed(global::Kerf.Geometry.BezierSurface s,
                                   double v1,
                                   double v2,
                                   double tol)
    {
        Curve3D? cvf = s.VIso(v1);
        Curve3D? cvl = s.VIso(v2);
        if (cvf is null || cvl is null)
        {
            return false;
        }
        double tol2 = 2.0 * tol;
        BezierCurve3D bzF = (BezierCurve3D)cvf;
        BezierCurve3D bzL = (BezierCurve3D)cvl;
        NCollection_Array1<Point3D> pf = bzF.InternalPoles();
        NCollection_Array1<Point3D> pl = bzL.InternalPoles();
        //
        return CompareWeightPoles(pf, null, pl, null, tol2);
    }

    //=================================================================================================
    private static bool CompareWeightPoles(NCollection_Array1<Point3D> poles1,
                                           NCollection_Array1<double>? w1,
                                           NCollection_Array1<Point3D> poles2,
                                           NCollection_Array1<double>? w2,
                                           double tol)
    {
        if (poles1.Length() != poles2.Length())
        {
            return false;
        }
        //
        int i = 1;
        for (i = 1; i <= poles1.Length(); i++)
        {
            double aW1 = (w1 is null) ? 1.0 : w1.Value(i);
            double aW2 = (w2 is null) ? 1.0 : w2.Value(i);

            Xyz pole1 = poles1.Value(i).XYZ() * aW1;
            Xyz pole2 = poles2.Value(i).XYZ() * aW2;
            if (!pole1.IsEqual(pole2, tol))
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
    public static bool IsIsoLine(Curve2DAdaptorBase c2D,
                                 out bool isU,
                                 out double param,
                                 out bool isForward)
    {
        // OCCT leaves the outputs unset when it returns false; the port assigns them first (decision 32).
        isU = false;
        param = 0.0;
        isForward = false;

        // These variables are used to check line state (vertical or horizontal).
        bool isAppropriateType = false;
        Point2D loc2d = new();
        Direction2D dir2d = new();

        // Test type.
        CurveKind type = c2D.GetType();
        if (type == Line)
        {
            Line2D lin2d = c2D.Line();
            loc2d = lin2d.Location();
            dir2d = lin2d.Direction();
            isAppropriateType = true;
        }
        else if (type == BSplineCurve)
        {
            BSplineCurve2D bSpline2d = c2D.BSpline();
            if (bSpline2d.Degree() != 1 || bSpline2d.PoleCount != 2)
            {
                return false; // Not a line or uneven parameterization.
            }

            loc2d = bSpline2d.Pole(1);

            // Vector should be non-degenerated.
            Vector2D vec2d = new(bSpline2d.Pole(1), bSpline2d.Pole(2));
            if (vec2d.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false; // Degenerated spline.
            }
            dir2d = new Direction2D(vec2d);

            isAppropriateType = true;
        }
        else if (type == BezierCurve)
        {
            BezierCurve2D bezier2d = c2D.Bezier();
            if (bezier2d.Degree() != 1 || bezier2d.PoleCount != 2)
            {
                return false; // Not a line or uneven parameterization.
            }

            loc2d = bezier2d.Pole(1);

            // Vector should be non-degenerated.
            Vector2D vec2d = new(bezier2d.Pole(1), bezier2d.Pole(2));
            if (vec2d.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return false; // Degenerated spline.
            }
            dir2d = new Direction2D(vec2d);

            isAppropriateType = true;
        }

        if (!isAppropriateType)
        {
            return false;
        }

        // Check line to be vertical or horizontal.
        if (dir2d.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Horizontal line. V = const.
            isU = false;
            param = loc2d.Y();
            isForward = dir2d.Dot(Constants.DX2d()) > 0.0;
            return true;
        }
        else if (dir2d.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Vertical line. U = const.
            isU = true;
            param = loc2d.X();
            isForward = dir2d.Dot(Constants.DY2d()) > 0.0;
            return true;
        }

        return false;
    }

    //=================================================================================================

    /// <summary>
    /// Builds the 3D curve of an isoline from the surface's own isoline (a GeomAdaptor_Surface other than a sphere),
    /// reparametrised as the 2D curve. Null when it cannot, or when the result is not within theTolerance.
    /// </summary>
    public static Curve3D? BuildC3dOnIsoLine(Curve2DAdaptorBase c2D,
                                                SurfaceAdaptorBase surf,
                                                double first,
                                                double last,
                                                double tolerance,
                                                bool isU,
                                                double param,
                                                bool isForward)
    {
        // Convert adapter to the appropriate type.
        SurfaceAdaptor? geomAdapter = surf as SurfaceAdaptor;
        if (geomAdapter is null)
        {
            return null;
        }

        if (surf.GetType() == SurfaceKind.Sphere)
        {
            return null;
        }

        // Extract isoline
        Surface aSurf = geomAdapter.Surface()!;
        Curve3D c3d;

        Point2D f2d = c2D.Value(c2D.FirstParameter());
        Point2D l2d = c2D.Value(c2D.LastParameter());

        bool isToTrim = true;
        double u1, u2, v1, v2;
        aSurf.Bounds(out u1, out u2, out v1, out v2);

        if (isU)
        {
            double v1Param = Min(f2d.Y(), l2d.Y());
            double v2Param = Max(f2d.Y(), l2d.Y());
            if (v2Param < v1 - tolerance || v1Param > v2 + tolerance)
            {
                return null;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
            {
                if (Math.Abs(v2Param - v1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return null;
                }
                aSurf = new TrimmedSurface(aSurf, u1, u2, v1Param, v2Param);
                isToTrim = false;
            }
            else
            {
                v1Param = Max(v1Param, v1);
                v2Param = Min(v2Param, v2);
                if (Math.Abs(v2Param - v1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return null;
                }
            }
            c3d = aSurf.UIso(param);
            if (isToTrim)
            {
                c3d = new TrimmedCurve3D(c3d, v1Param, v2Param);
            }
        }
        else
        {
            double u1Param = Min(f2d.X(), l2d.X());
            double u2Param = Max(f2d.X(), l2d.X());
            if (u2Param < u1 - tolerance || u1Param > u2 + tolerance)
            {
                return null;
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
            {
                if (Math.Abs(u2Param - u1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return null;
                }
                aSurf = new TrimmedSurface(aSurf, u1Param, u2Param, v1, v2);
                isToTrim = false;
            }
            else
            {
                u1Param = Max(u1Param, u1);
                u2Param = Min(u2Param, u2);
                if (Math.Abs(u2Param - u1Param) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return null;
                }
            }
            c3d = aSurf.VIso(param);
            if (isToTrim)
            {
                c3d = new TrimmedCurve3D(c3d, u1Param, u2Param);
            }
        }

        // Convert arbitrary curve type to the b-spline.
        BSplineCurve3D curve3d =
          global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(c3d, QuasiAngular);
        // The conversion can return a frozen curve that others share (decision 244): change a copy.
        if (curve3d.IsFrozen)
        {
            curve3d = (BSplineCurve3D)curve3d.Copy();
        }
        if (!isForward)
        {
            curve3d.Reverse();
        }

        // Rebuild parameterization for the 3d curve to have the same parameterization with
        // a two-dimensional curve.
        NCollection_Array1<double> knots = new(curve3d.InternalKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(c2D.FirstParameter(), c2D.LastParameter(), knots);
        curve3d.SetKnots(knots);

        // Evaluate error.
        double error3d = 0.0;

        double parF = first;
        double parL = last;
        const int nbPnt = 23;
        for (int idx = 0; idx <= nbPnt; ++idx)
        {
            double par = parF + ((parL - parF) * idx) / nbPnt;

            Point2D pnt2d = c2D.Value(par);

            Point3D pntC3D = curve3d.Value(par);
            Point3D pntC2D = surf.Value(pnt2d.X(), pnt2d.Y());

            double sqDeviation = pntC3D.SquareDistance(pntC2D);
            error3d = Max(sqDeviation, error3d);
        }

        error3d = Math.Sqrt(error3d);

        // Target tolerance is not obtained. This situation happens for isolines on the sphere.
        // OCCT is unable to convert it keeping original parameterization, while the geometric
        // form of the result is entirely identical. In that case, it is better to utilize
        // a general-purpose approach.
        if (error3d > tolerance)
        {
            return null;
        }

        return curve3d;
    }
}
