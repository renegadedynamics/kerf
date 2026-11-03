// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// General utilities for BRep: the default precision and plane of the makers, the computation of missing 3D curves
/// and pcurves, SameParameter and SameRange, tolerances, regularity, orientation of closed solids.
/// </summary>
public static partial class BRepLib
{
    // OCCT's file statics. Process-wide as in OCCT; their setters are internal, so callers always see the defaults.
    private static double thePrecision = Kerf.Engine.Foundation.Precision.Confusion();
    private static PlaneSurface? thePlane;

    /// <summary>Sets the default precision (internal: process-wide state).</summary>
    internal static void Precision(double P)
    {
        thePrecision = P;
    }

    /// <summary>Returns the default precision of the makers.</summary>
    public static double Precision()
    {
        return thePrecision;
    }

    /// <summary>Sets the current plane (internal: process-wide state).</summary>
    internal static void Plane(PlaneSurface P)
    {
        P.Freeze();
        thePlane = P;
    }

    /// <summary>Returns the current plane of the 2D makers (frozen).</summary>
    public static PlaneSurface Plane()
    {
        if (thePlane is null)
        {
            var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
            aPlane.Freeze();
            Interlocked.CompareExchange(ref thePlane, aPlane, null);
        }
        return thePlane;
    }

    //=================================================================================================

    /// <summary>
    /// True if all the curve representations of the edge have the same range within <paramref name="Confusion"/>,
    /// ignoring the edge's SameRange flag.
    /// </summary>
    public static bool CheckSameRange(in Edge AnEdge, double Confusion = 1.0e-12)
    {
        double Tolerance = Confusion;
        bool IsSameRange = true, first_time_in = true;

        // OCCT.NET: frozen. OCCT iterates ChangeCurves() to read them; Curves() is the same list.
        var an_Iterator = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)AnEdge.TShape()!).Curves());

        double first, last;
        double current_first = 0.0, current_last = 0.0;
        GCurve? geometric_representation_ptr;

        while (IsSameRange && an_Iterator.More())
        {
            geometric_representation_ptr = an_Iterator.Value() as GCurve;
            if (geometric_representation_ptr is not null)
            {

                first = geometric_representation_ptr.First();
                last = geometric_representation_ptr.Last();
                if (first_time_in)
                {
                    current_first = first;
                    current_last = last;
                    first_time_in = false;
                }
                else
                {
                    IsSameRange = (Math.Abs(current_first - first) <= Tolerance)
                                  && (Math.Abs(current_last - last) <= Tolerance);
                }
            }
            an_Iterator.Next();
        }
        return IsSameRange;
    }

    //=================================================================================================

    /// <summary>
    /// Gives all the curve representations of the edge the same range: the 3D curve's if there is one, else the
    /// first representation's; then sets SameRange. Ignores the SameRange flag. Writes into the edge.
    /// </summary>
    internal static void SameRange(in Edge AnEdge, double Tolerance = 1.0e-5)
    {
        var an_Iterator = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)AnEdge.TShape()!).ChangeCurves());

        Curve2D? Curve2dPtr = null, Curve2dPtr2 = null, NewCurve2dPtr = null, NewCurve2dPtr2 = null;

        bool first_time_in = true, has_curve, has_closed_curve;
        GCurve? geometric_representation_ptr;
        double first, current_first, last, current_last;

        Curve3D? C = Tool.Curve(AnEdge, out _, out current_first, out current_last);
        if (C is not null)
        {
            first_time_in = false;
        }

        while (an_Iterator.More())
        {
            geometric_representation_ptr = an_Iterator.Value() as GCurve;
            if (geometric_representation_ptr is not null)
            {
                has_closed_curve = has_curve = false;
                first = geometric_representation_ptr.First();
                last = geometric_representation_ptr.Last();
                if (geometric_representation_ptr.IsCurveOnSurface())
                {
                    Curve2dPtr = geometric_representation_ptr.PCurve();
                    has_curve = true;
                }
                if (geometric_representation_ptr.IsCurveOnClosedSurface)
                {
                    Curve2dPtr2 = geometric_representation_ptr.PCurve2();
                    has_closed_curve = true;
                }
                if (has_curve || has_closed_curve)
                {
                    if (first_time_in)
                    {
                        current_first = first;
                        current_last = last;
                        first_time_in = false;
                    }

                    if (Math.Abs(first - current_first) > Kerf.Engine.Foundation.Precision.Confusion()
                        || Math.Abs(last - current_last) > Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        if (has_curve)
                        {
                            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(Tolerance,
                                              Curve2dPtr,
                                              geometric_representation_ptr.First(),
                                              geometric_representation_ptr.Last(),
                                              current_first,
                                              current_last,
                                              ref NewCurve2dPtr);
                            geometric_representation_ptr.PCurve(NewCurve2dPtr);
                        }
                        if (has_closed_curve)
                        {
                            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(Tolerance,
                                              Curve2dPtr2,
                                              geometric_representation_ptr.First(),
                                              geometric_representation_ptr.Last(),
                                              current_first,
                                              current_last,
                                              ref NewCurve2dPtr2);
                            geometric_representation_ptr.PCurve2(NewCurve2dPtr2);
                        }
                    }
                }
            }
            an_Iterator.Next();
        }
        var B = new BrepBuilder();
        B.Range(AnEdge, current_first, current_last);

        B.SameRange(AnEdge, true);
    }

    //=======================================================================
    // function : EvaluateMaxSegment
    // purpose  : return MaxSegment to pass in approximation, if MaxSegment==0 provided
    //=======================================================================

    private static int evaluateMaxSegment(int aMaxSegment, CurveOnSurface aCurveOnSurface)
    {
        if (aMaxSegment != 0)
        {
            return aMaxSegment;
        }

        SurfaceAdaptorBase aSurf = aCurveOnSurface.GetSurface()!;
        Curve2DAdaptorBase aCurv2d = aCurveOnSurface.GetCurve()!;

        double aNbSKnots = 0, aNbC2dKnots = 0;

        if (aSurf.GetType() == SurfaceKind.BSplineSurface)
        {
            global::Kerf.Geometry.BSplineSurface aBSpline = aSurf.BSpline();
            aNbSKnots = Max(aBSpline.UKnotCount, aBSpline.VKnotCount);
        }
        if (aCurv2d.GetType() == BSplineCurve)
        {
            aNbC2dKnots = aCurv2d.KnotCount;
        }
        int aReturn = (int)(30 + Max(aNbSKnots, aNbC2dKnots));
        return aReturn;
    }

    //=================================================================================================

    /// <summary>
    /// Computes the 3D curve of the edge if it has none: from its pcurve on a plane, else by approximation of its
    /// first curve on a surface. True if the curve was computed or existed. Writes into the edge.
    /// </summary>
    /// <remarks><paramref name="MaxSegment"/> 0 means 30 plus the number of knots of the surface or pcurve.</remarks>
    internal static bool BuildCurve3d(in Edge AnEdge,
                                      double Tolerance = 1.0e-5,
                                      Continuity Continuity = C1,
                                      int MaxDegree = 14,
                                      int MaxSegment = 0)
    {
        int ii, jj;

        Placement LocalLoc = new(), LC;
        var L = new Placement[] { new(), new() };
        double f, l, fc, lc, tolerance, max_deviation;
        var first = new double[2];
        var last = new double[2];
        Curve2D? Curve2dPtr = null;
        var Curve2dArray = new Curve2D?[2];
        Surface? SurfacePtr = null;
        var SurfaceArray = new Surface?[2];

        bool not_done;
        // if the edge has a 3d curve returns true

        Curve3D? C = Tool.Curve(AnEdge, out LocalLoc, out f, out l);
        if (C is not null)
        {
            return true;
        }
        //
        // this should not exists but UpdateEdge makes funny things
        // if the edge is not same range
        //
        if (!CheckSameRange(AnEdge, Kerf.Engine.Foundation.Precision.Confusion()))
        {
            SameRange(AnEdge, Tolerance);
        }

        // search a curve on a plane
        Surface? S = null;
        Curve2D? PC = null;
        int i = 0;
        PlaneSurface? P = null;
        not_done = true;

        while (not_done)
        {
            i++;
            Tool.CurveOnSurface(AnEdge, ref PC, ref S, ref LocalLoc, ref f, ref l, i);
            if (S is not TrimmedSurface RT)
            {
                P = S as PlaneSurface;
            }
            else
            {
                P = RT.BasisSurface() as PlaneSurface;
            }
            not_done = S is not null && P is null;
        }
        if (P is not null)
        {
            // compute the 3d curve
            Frame3D axes = P.Position().Ax2();
            Curve3D? C3d = global::Kerf.Engine.Geometry.Library.GeomLib.To3d(axes, PC!);
            if (C3d is null)
            {
                return false;
            }
            // update the edge
            double First, Last;

            var B = new BrepBuilder();
            B.UpdateEdge(AnEdge, C3d, LocalLoc, 0.0e0);
            LC = new Placement();
            Tool.Range(AnEdge, S, LC, out First, out Last);
            B.Range(AnEdge, First, Last); // Do not forget 3D range.(PRO6412)
        }
        else
        {
            //
            // compute the 3d curve using existing surface
            //
            fc = f;
            lc = l;
            if (!Tool.Degenerated(AnEdge))
            {
                jj = 0;
                // BRep_Tool::CurveOnSurface expects Index >= 1, so start from 1.
                for (ii = 1; ii < 3; ii++)
                {
                    Tool.CurveOnSurface(AnEdge, ref Curve2dPtr, ref SurfacePtr, ref LocalLoc, ref fc, ref lc, ii);

                    if (Curve2dPtr is not null && jj < 2)
                    {
                        Curve2dArray[jj] = Curve2dPtr;
                        SurfaceArray[jj] = SurfacePtr;
                        L[jj] = LocalLoc;
                        first[jj] = fc;
                        last[jj] = lc;
                        jj += 1;
                    }
                }
                f = first[0];
                l = last[0];
                Curve2dPtr = Curve2dArray[0];
                SurfacePtr = SurfaceArray[0];

                // OCCT copies a local adaptor into a handle; the port makes the handle directly (decision 331).
                var AnAdaptor3dCurve2dPtr = new CurveAdaptor2D(Curve2dPtr!, f, l);
                var AnAdaptor3dSurfacePtr = new SurfaceAdaptor(SurfacePtr!);
                var CurveOnSurface = new CurveOnSurface(AnAdaptor3dCurve2dPtr, AnAdaptor3dSurfacePtr);

                Curve3D? NewCurvePtr = null;

                global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(Tolerance,
                                     CurveOnSurface,
                                     f,
                                     l,
                                     ref NewCurvePtr,
                                     out max_deviation,
                                     out _,
                                     Continuity,
                                     MaxDegree,
                                     evaluateMaxSegment(MaxSegment, CurveOnSurface));
                var B = new BrepBuilder();
                tolerance = Tool.Tolerance(AnEdge);
                // Patch
                // max_deviation = std::max(tolerance, max_deviation) ;
                max_deviation = Max(tolerance, Tolerance);
                if (NewCurvePtr is null)
                {
                    return false;
                }
                B.UpdateEdge(AnEdge, NewCurvePtr, L[0], max_deviation);
                if (jj == 1)
                {
                    //
                    // if there is only one curve on surface attached to the edge
                    // than it can be qualified sameparameter
                    //
                    B.SameParameter(AnEdge, true);
                }
            }
            else
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    /// <summary>Computes the 3D curves of the edges of <paramref name="S"/> with a tolerance of 1e-5.</summary>
    internal static bool BuildCurves3d(in Shape S)

    {
        return BuildCurves3d(S, 1.0e-5);
    }

    //=================================================================================================

    /// <summary>
    /// Computes the missing 3D curves of all the edges of <paramref name="S"/>; false if one of the computations
    /// failed. Writes into the edges.
    /// </summary>
    internal static bool BuildCurves3d(in Shape S,
                                       double Tolerance,
                                       Continuity Continuity = C1,
                                       int MaxDegree = 14,
                                       int MaxSegment = 0)
    {
        bool boolean_value, ok = true;
        var a_counter = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var ex = new Explorer(S, ShapeType.Edge);

        while (ex.More())
        {
            if (a_counter.Add(ex.Current()))
            {
                boolean_value = BuildCurve3d(ShapeCasts.Edge(ex.Current()), Tolerance, Continuity, MaxDegree, MaxSegment);
                ok = ok && boolean_value;
            }
            ex.Next();
        }
        return ok;
    }

    //=================================================================================================

    /// <summary>
    /// If the edge's tolerance is below <paramref name="MaxToleranceToCheck"/>, sets it to the radius of the pipe
    /// around its reference curve holding all its curve representations. True if the tolerance was set. Writes into
    /// the edge.
    /// </summary>
    internal static bool UpdateEdgeTol(in Edge AnEdge,
                                       double MinToleranceRequested,
                                       double MaxToleranceToCheck)
    {

        int curve_on_surface_index, curve_index, jj, ii,
          geom_reference_curve_flag = 0, max_sampling_points = 90, min_sampling_points = 30;
        bool not_done, has_closed_curve, has_curve;
        double factor = 100.0e0,
               //     sampling_array[2],
               safe_factor = 1.4e0, current_last, current_first, max_distance, coded_edge_tolerance,
               edge_tolerance = 0.0e0;
        NCollection_HArray1<double> parameters_ptr;
        GCurve? geometric_representation_ptr;

        if (Tool.Degenerated(AnEdge))
        {
            return false;
        }
        coded_edge_tolerance = Tool.Tolerance(AnEdge);
        if (coded_edge_tolerance > MaxToleranceToCheck)
        {
            return false;
        }

        BrepTEdge TE = (BrepTEdge)AnEdge.TShape()!;
        NCollection_List<CurveRepresentation> list_curve_rep = TE.ChangeCurves();
        NCollection_List<CurveRepresentation>.Iterator an_iterator = new(list_curve_rep),
                                                            second_iterator = new(list_curve_rep);
        Curve2D? curve2d_ptr = null;
        Surface? surface_ptr = null;
        Placement local_location;
        var a_sampler = new QuasiUniformDeflection();
        var geom_reference_curve = new CurveAdaptor();
        var curve_on_surface_reference = new CurveOnSurface();
        Curve3D? C = Tool.Curve(AnEdge, out local_location, out current_first, out current_last);
        curve_on_surface_index = -1;
        if (C is not null)
        {
            if (!local_location.IsIdentity)
            {
                C = (Curve3D)C.Transformed(local_location.Transformation());
            }
            geom_reference_curve.Load(C);
            geom_reference_curve_flag = 1;
            a_sampler.Initialize(geom_reference_curve,
                                 MinToleranceRequested * factor,
                                 current_first,
                                 current_last);
        }
        else
        {
            not_done = true;
            curve_on_surface_index = 0;

            // OCCT loops on an_iterator.More() but reads second_iterator and advances neither: the loop looks only
            // at the first representation, never ends if it isn't a curve on a surface, and leaves
            // curve_on_surface_index at 1 (the second representation), not 0.
            while (not_done && an_iterator.More())
            {
                geometric_representation_ptr = second_iterator.Value() as GCurve;
                if (geometric_representation_ptr is not null
                    && geometric_representation_ptr.IsCurveOnSurface())
                {
                    curve2d_ptr = geometric_representation_ptr.PCurve();
                    local_location = geometric_representation_ptr.Location();
                    current_first = geometric_representation_ptr.First();
                    // first = geometric_representation_ptr->First();
                    current_last = geometric_representation_ptr.Last();
                    // must be inverted
                    //
                    if (!local_location.IsIdentity)
                    {
                        surface_ptr = (Surface)geometric_representation_ptr.Surface()!.Transformed(local_location.Transformation());
                    }
                    else
                    {
                        surface_ptr = geometric_representation_ptr.Surface();
                    }
                    not_done = false;
                }
                curve_on_surface_index += 1; // OCCT (see above)
            }
            var AnAdaptor3dCurve2dPtr = new CurveAdaptor2D(curve2d_ptr!);
            var AnAdaptor3dSurfacePtr = new SurfaceAdaptor(surface_ptr!);
            curve_on_surface_reference.Load(AnAdaptor3dCurve2dPtr, AnAdaptor3dSurfacePtr);
            a_sampler.Initialize(curve_on_surface_reference,
                                 MinToleranceRequested * factor,
                                 current_first,
                                 current_last);
        }
        var sampling_parameters = new NCollection_Array1<double>(1, a_sampler.PointCount);
        for (ii = 1; ii <= a_sampler.PointCount; ii++)
        {
            sampling_parameters[ii] = a_sampler.Parameter(ii);
        }
        if (a_sampler.PointCount < min_sampling_points)
        {
            global::Kerf.Engine.Geometry.Library.GeomLib.DensifyArray1OfReal(min_sampling_points, sampling_parameters, out parameters_ptr);
        }
        else if (a_sampler.PointCount > max_sampling_points)
        {
            global::Kerf.Engine.Geometry.Library.GeomLib.RemovePointsFromArray(max_sampling_points, sampling_parameters, out parameters_ptr);
        }
        else
        {
            jj = 1;
            parameters_ptr = new NCollection_HArray1<double>(1, sampling_parameters.Length());
            for (ii = sampling_parameters.Lower(); ii <= sampling_parameters.Upper(); ii++)
            {
                parameters_ptr.ChangeArray1()[jj] = sampling_parameters[ii];
                jj += 1;
            }
        }

        curve_index = 0;

        while (second_iterator.More())
        {
            geometric_representation_ptr = second_iterator.Value() as GCurve;
            if (geometric_representation_ptr is not null && curve_index != curve_on_surface_index)
            {
                has_closed_curve = has_curve = false;
                //	first = geometric_representation_ptr->First();
                //	last =  geometric_representation_ptr->Last();
                local_location = geometric_representation_ptr.Location();
                if (geometric_representation_ptr.IsCurveOnSurface())
                {
                    curve2d_ptr = geometric_representation_ptr.PCurve();
                    has_curve = true;
                }
                if (geometric_representation_ptr.IsCurveOnClosedSurface)
                {
                    curve2d_ptr = geometric_representation_ptr.PCurve2();
                    has_closed_curve = true;
                }

                if (has_curve || has_closed_curve)
                {
                    if (!local_location.IsIdentity)
                    {
                        surface_ptr = (Surface)geometric_representation_ptr.Surface()!.Transformed(local_location.Transformation());
                    }
                    else
                    {
                        surface_ptr = geometric_representation_ptr.Surface();
                    }
                    var an_adaptor_curve2d_ptr = new CurveAdaptor2D(curve2d_ptr!);
                    var an_adaptor_surface_ptr = new SurfaceAdaptor(surface_ptr!);
                    var a_curve_on_surface = new CurveOnSurface(an_adaptor_curve2d_ptr, an_adaptor_surface_ptr);

                    // OCCT: an edge without 3D curve flagged SameParameter is measured against geom_reference_curve, which
                    // was never loaded; OCCT dereferences its null curve, the port throws a NullReferenceException.
                    if (Tool.SameParameter(AnEdge))
                    {

                        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxParametricDistance(a_curve_on_surface,
                                                          geom_reference_curve,
                                                          MinToleranceRequested,
                                                          parameters_ptr.Array1(),
                                                          out max_distance);
                    }
                    else if (geom_reference_curve_flag != 0)
                    {
                        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxDistanceAlongParameter(a_curve_on_surface,
                                                              geom_reference_curve,
                                                              MinToleranceRequested,
                                                              parameters_ptr.Array1(),
                                                              out max_distance);
                    }
                    else
                    {

                        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxDistanceAlongParameter(a_curve_on_surface,
                                                              curve_on_surface_reference,
                                                              MinToleranceRequested,
                                                              parameters_ptr.Array1(),
                                                              out max_distance);
                    }
                    max_distance *= safe_factor;
                    edge_tolerance = Max(max_distance, edge_tolerance);
                }
            }
            curve_index += 1;
            second_iterator.Next();
        }

        TE.Tolerance(edge_tolerance);
        return true;
    }

    //=================================================================================================

    /// <summary>
    /// <see cref="UpdateEdgeTol"/> on every edge of the shape; true if at least one edge was updated. Writes into
    /// the edges.
    /// </summary>
    internal static bool UpdateEdgeTolerance(in Shape S,
                                             double MinToleranceRequested,
                                             double MaxToleranceToCheck)
    {
        var ex = new Explorer(S, ShapeType.Edge);
        var a_counter = new NCollection_Map<Shape>(ShapeHasher.Instance);

        bool return_status = false, local_flag;

        while (ex.More())
        {
            if (a_counter.Add(ex.Current()))
            {
                local_flag = UpdateEdgeTol(ShapeCasts.Edge(ex.Current()),
                                           MinToleranceRequested,
                                           MaxToleranceToCheck);
                if (local_flag && !return_status)
                {
                    return_status = true;
                }
            }
            ex.Next();
        }
        return return_status;
    }

    //=================================================================================================

    private static void GetEdgeTol(in Edge theEdge, in Face theFace, ref double theEdTol)
    {
        Surface? S = Tool.Surface(theFace, out Placement L);
        Placement l = L.Predivided(theEdge.Location());

        BrepTEdge TE = (BrepTEdge)theEdge.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(S, l))
            {
                return;
            }
            itcr.Next();
        }

        PlaneSurface? GP;
        TrimmedSurface? GRTS;
        GRTS = S as TrimmedSurface;
        if (GRTS is not null)
        {
            GP = GRTS.BasisSurface() as PlaneSurface;
        }
        else
        {
            GP = S as PlaneSurface;
        }

        var HC = new CurveAdaptor();
        var HS = new SurfaceAdaptor();

        Placement LC;
        double First, Last;
        HC.Load(Tool.Curve(theEdge, out LC, out First, out Last)!);
        LC = L.Predivided(LC);

        if (!LC.IsIdentity)
        {
            GP = (PlaneSurface)GP!.Transformed(LC.Transformation());
        }
        SurfaceAdaptor GAS = HS;
        GAS.Load(GP!);

        var Proj = new ProjectedCurve(HS, HC);
        Curve2D pc = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(Proj);

        Plane3D pln = GAS.Plane();
        double d2 = 0.0;
        int nn = 23;
        double unsurnn = 1.0 / nn;
        for (int i = 0; i <= nn; i++)
        {
            double t = unsurnn * i;
            double u = First * (1.0 - t) + Last * t;
            Point3D Pc3d = HC.Value(u);
            Point2D p2d = pc.Value(u);
            Point3D Pcons = global::Kerf.Engine.Geometry.ElSLib.Value(p2d.X(), p2d.Y(), pln);
            double eps = Max(Pc3d.XYZ().SquareModulus(), Pcons.XYZ().SquareModulus());
            eps = Epsilon(eps);
            double temp = Pc3d.SquareDistance(Pcons);
            if (temp <= eps)
            {
                temp = 0.0;
            }
            if (temp > d2)
            {
                d2 = temp;
            }
        }
        d2 = 1.05 * Math.Sqrt(d2);
        theEdTol = d2;
    }

    //=======================================================================
    // function : UpdTolMap
    // purpose  : Update map ShToTol (shape to tolerance)
    //=======================================================================
    private static void UpdTolMap(in Shape theSh,
                                  double theNewTol,
                                  NCollection_DataMap<Shape, double> theShToTol)
    {
        ShapeType aSt = theSh.ShapeType();
        double aShTol;
        if (aSt == ShapeType.Vertex)
        {
            aShTol = Tool.Tolerance(ShapeCasts.Vertex(theSh));
        }
        else if (aSt == ShapeType.Edge)
        {
            aShTol = Tool.Tolerance(ShapeCasts.Edge(theSh));
        }
        else
        {
            return;
        }
        //
        if (theNewTol > aShTol)
        {
            if (!theShToTol.Find(theSh, out double anOldtol))
            {
                theShToTol.Bind(theSh, theNewTol);
            }
            else
            {
                theShToTol.ChangeFind(theSh) = Max(anOldtol, theNewTol);
            }
        }
    }

    //=======================================================================
    // function : UpdShTol
    // purpose  : Update vertices/edges/faces according to ShToTol map (create copies of necessary)
    //=======================================================================
    private static void UpdShTol(NCollection_DataMap<Shape, double> theShToTol,
                                 bool IsMutableInput,
                                 ReShape theReshaper,
                                 bool theVForceUpdate)
    {
        var aB = new BrepBuilder();
        foreach (KeyValuePair<Shape, double> SHToTolit in theShToTol)
        {
            Shape aSh = SHToTolit.Key;
            double aTol = SHToTolit.Value;
            //
            Shape aNsh;
            // The reshaper's internal form: its replacements are this algorithm's own, still unfrozen (design 4.2).
            Shape aVsh = theReshaper.InternalValue(aSh);
            bool UseOldSh = IsMutableInput || theReshaper.IsNewShape(aSh) || !aVsh.IsSame(aSh);
            if (UseOldSh)
            {
                aNsh = aVsh;
            }
            else
            {
                aNsh = aSh.EmptyCopied();
                // add subshapes from the original shape
                var sit = new Iterator(aSh);
                for (; sit.More(); sit.Next())
                {
                    aB.Add(aNsh, sit.Value());
                }
                //
                aNsh.Free(aSh.Free());
                aNsh.Checked(aSh.Checked());
                aNsh.Orientable(aSh.Orientable());
                aNsh.Closed(aSh.Closed());
                aNsh.Infinite(aSh.Infinite());
                aNsh.Convex(aSh.Convex());
                //
            }
            //
            switch (aSh.ShapeType())
            {
                case ShapeType.Face:
                    {
                        aB.UpdateFace(ShapeCasts.Face(aNsh), aTol);
                        break;
                    }
                case ShapeType.Edge:
                    {
                        aB.UpdateEdge(ShapeCasts.Edge(aNsh), aTol);
                        break;
                    }
                case ShapeType.Vertex:
                    {
                        BrepTVertex aTV = (BrepTVertex)aNsh.TShape()!;
                        if (theVForceUpdate)
                        {
                            aTV.Tolerance(aTol);
                        }
                        else
                        {
                            aTV.UpdateTolerance(aTol);
                        }
                        aTV.Modified(true);
                        break;
                    }
                default:
                    break;
            }
            //
            if (!UseOldSh)
            {
                theReshaper.Replace(aSh, aNsh);
            }
        }
    }

    //=================================================================================================

    private static void InternalSameParameter(in Shape theSh,
                                              ReShape theReshaper,
                                              double theTol,
                                              bool IsForced,
                                              bool IsMutableInput)
    {
        var ex = new Explorer(theSh, ShapeType.Edge);
        var Done = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aB = new BrepBuilder();
        var aShToTol = new NCollection_DataMap<Shape, double>(ShapeHasher.Instance);

        while (ex.More())
        {
            Edge aCE = ShapeCasts.Edge(ex.Current());
            if (Done.Add(aCE))
            {
                Edge aNE = ShapeCasts.Edge(theReshaper.InternalValue(aCE));
                bool UseOldEdge = IsMutableInput || theReshaper.IsNewShape(aCE) || !aNE.IsSame(aCE);
                if (IsForced && (Tool.SameRange(aCE) || Tool.SameParameter(aCE)))
                {
                    if (!UseOldEdge)
                    {
                        aNE = ShapeCasts.Edge(aCE.EmptyCopied());
                        var sit = new Iterator(aCE);
                        for (; sit.More(); sit.Next())
                        {
                            aB.Add(aNE, sit.Value());
                        }
                        theReshaper.Replace(aCE, aNE);
                        UseOldEdge = true;
                    }
                    aB.SameRange(aNE, false);
                    aB.SameParameter(aNE, false);
                }
                double aNewTol = -1;
                Edge aResEdge = SameParameter(aNE, theTol, ref aNewTol, UseOldEdge);
                if (!UseOldEdge && !aResEdge.IsNull)
                {
                    // NE have been empty-copied
                    theReshaper.Replace(aNE, aResEdge);
                }
                if (aNewTol > 0)
                {
                    Exploration.Vertices(aCE, out Vertex aV1, out Vertex aV2);
                    if (!aV1.IsNull)
                    {
                        UpdTolMap(aV1, aNewTol, aShToTol);
                    }
                    if (!aV2.IsNull)
                    {
                        UpdTolMap(aV2, aNewTol, aShToTol);
                    }
                }
            }
            ex.Next();
        }

        Done.Clear();
        var BS = new FaceSurfaceAdaptor();
        for (ex.Init(theSh, ShapeType.Face); ex.More(); ex.Next())
        {
            Face curface = ShapeCasts.Face(ex.Current());
            if (!Done.Add(curface))
            {
                continue;
            }
            BS.Initialize(curface);
            if (BS.GetType() != global::Kerf.Geometry.SurfaceKind.Plane)
            {
                continue;
            }
            var ex2 = new Explorer();
            for (ex2.Init(curface, ShapeType.Edge); ex2.More(); ex2.Next())
            {
                Edge E = ShapeCasts.Edge(ex2.Current());
                if (Tool.Degenerated(E))
                {
                    continue;
                }

                Shape aNe = theReshaper.InternalValue(E);
                double aNewEtol = -1;
                GetEdgeTol(ShapeCasts.Edge(aNe), curface, ref aNewEtol);
                if (aNewEtol >= 0)
                { // not equal to -1
                    UpdTolMap(E, aNewEtol, aShToTol);
                }
            }
        }

        //
        UpdShTol(aShToTol, IsMutableInput, theReshaper, false);

        InternalUpdateTolerances(theSh, false, IsMutableInput, theReshaper);
    }

    //================================================================
    // function : SameParameter
    // WARNING  : New spec DUB LBO 9/9/97.
    //  Recode in the edge the best tolerance found,
    //  for vertex extremities it is required to find something else
    //================================================================

    /// <summary>
    /// Computes new pcurves for all the edges of <paramref name="S"/> so that they have the same parameter as the
    /// 3D curve (skipping edges flagged SameParameter unless <paramref name="forced"/>), then harmonizes the
    /// tolerances. Writes into the shape.
    /// </summary>
    internal static void SameParameter(in Shape S, double Tolerance = 1.0e-5, bool forced = false)
    {
        var reshaper = new ReShape();
        InternalSameParameter(S, reshaper, Tolerance, forced, true);
    }

    //=================================================================================================

    /// <summary>
    /// <see cref="SameParameter(in Shape, double, bool)"/> without changing <paramref name="S"/>: the
    /// changed edges, vertices and faces are copies recorded as replacements in <paramref name="theReshaper"/>, whose
    /// <see cref="ReShape.Apply"/> gives the result.
    /// </summary>
    public static void SameParameter(in Shape S,
                                     ReShape theReshaper,
                                     double Tolerance = 1.0e-5,
                                     bool forced = false)
    {
        InternalSameParameter(S, theReshaper, Tolerance, forced, false);
    }

    //=================================================================================================

    private static bool EvalTol(Curve2D pc,
                                Surface s,
                                CurveAdaptor gac,
                                double tol,
                                out double tolbail)
    {
        int ok = 0;
        double f = gac.FirstParameter();
        double l = gac.LastParameter();
        var Projector = new LocateExtPC();
        Projector.Initialize(gac, f, l, tol);
        double u, v;
        Point3D p;
        tolbail = tol;
        for (int i = 1; i <= 5; i++)
        {
            double t = i / 6.0;
            t = (1.0 - t) * f + t * l;
            pc.Value(t).Coord(out u, out v);
            p = s.Value(u, v);
            Projector.Perform(p, t);
            if (Projector.IsDone)
            {
                double dist2 = Projector.SquareDistance();
                if (dist2 > tolbail * tolbail)
                {
                    tolbail = Math.Sqrt(dist2);
                }
                ok++;
            }
        }
        return (ok > 2);
    }

    //=================================================================================================

    private static double ComputeTol(CurveAdaptorBase c3d,
                                     Curve2DAdaptorBase c2d,
                                     SurfaceAdaptorBase surf,
                                     int nbp)

    {

        var dist = new NCollection_Array1<double>(1, nbp + 10);
        dist.Init(-1.0);

        // Adaptor3d_CurveOnSurface  cons(c2d,surf);
        double uf = surf.FirstUParameter(), ul = surf.LastUParameter(), vf = surf.FirstVParameter(),
               vl = surf.LastVParameter();
        double du = 0.01 * (ul - uf), dv = 0.01 * (vl - vf);
        bool isUPeriodic = surf.IsUPeriodic, isVPeriodic = surf.IsVPeriodic;
        double DSdu = 1.0 / surf.UResolution(1.0), DSdv = 1.0 / surf.VResolution(1.0);
        double d2 = 0.0;
        double first = c3d.FirstParameter();
        double last = c3d.LastParameter();
        double dapp = -1.0;
        for (int i = 0; i <= nbp; ++i)
        {
            double t = IntToReal(i) / IntToReal(nbp);
            double u = first * (1.0 - t) + last * t;
            Point3D Pc3d = c3d.Value(u);
            Point2D Puv = c2d.Value(u);
            if (!isUPeriodic)
            {
                if (Puv.X() < uf - du)
                {
                    dapp = Max(dapp, DSdu * (uf - Puv.X()));
                    continue;
                }
                else if (Puv.X() > ul + du)
                {
                    dapp = Max(dapp, DSdu * (Puv.X() - ul));
                    continue;
                }
            }
            if (!isVPeriodic)
            {
                if (Puv.Y() < vf - dv)
                {
                    dapp = Max(dapp, DSdv * (vf - Puv.Y()));
                    continue;
                }
                else if (Puv.Y() > vl + dv)
                {
                    dapp = Max(dapp, DSdv * (Puv.Y() - vl));
                    continue;
                }
            }
            Point3D Pcons = surf.Value(Puv.X(), Puv.Y());
            if (Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.X()) || Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.Y())
                || Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.Z()))
            {
                d2 = Kerf.Engine.Foundation.Precision.Infinite();
                break;
            }
            double temp = Pc3d.SquareDistance(Pcons);

            dist[i + 1] = temp;

            d2 = Max(d2, temp);
        }

        if (Kerf.Engine.Foundation.Precision.IsInfinite(d2))
        {
            return d2;
        }

        d2 = Math.Sqrt(d2);
        if (dapp > d2)
        {
            return dapp;
        }

        bool ana = false;
        double D2 = 0;
        int N1 = 0;
        int N2 = 0;
        int N3 = 0;

        for (int i = 1; i <= nbp + 10; ++i)
        {
            if (dist[i] > 0)
            {
                if (dist[i] < 1.0)
                {
                    ++N1;
                }
                else
                {
                    ++N2;
                }
            }
        }

        if (N1 > N2 && N2 != 0)
        {
            N3 = 100 * N2 / (N1 + N2);
        }
        if (N3 < 10 && N3 != 0)
        {
            ana = true;
            for (int i = 1; i <= nbp + 10; ++i)
            {
                if (dist[i] > 0 && dist[i] < 1.0)
                {
                    D2 = Max(D2, dist[i]);
                }
            }
        }

        // d2 = 1.5*sqrt(d2);
        d2 = (!ana) ? 1.5 * d2 : 1.5 * Math.Sqrt(D2);
        d2 = Max(d2, 1.0e-7);
        return d2;
    }

    //=================================================================================================

    // OCCT copies the edge's list of representations (`theCList = aTE->ChangeCurves()`); the representations are the
    // edge's own either way, and the loop that follows changes them, not the list, so the port returns the list.
    // It is read through Curves() (OCCT.NET: frozen): the ReShape form calls this on the caller's edge.
    private static void GetCurve3d(in Edge theEdge,
                                   ref Curve3D? theC3d,
                                   ref double theF3d,
                                   ref double theL3d,
                                   ref Placement theLoc3d,
                                   out NCollection_List<CurveRepresentation> theCList)
    {
        BrepTEdge aTE = (BrepTEdge)theEdge.TShape()!;
        theCList = aTE.Curves();
        var anIt = new NCollection_List<CurveRepresentation>.Iterator(theCList);
        bool NotDone = true;
        while (NotDone && anIt.More())
        {
            if (anIt.Value() is GCurve GCurve && GCurve.IsCurve3D)
            {
                theC3d = GCurve.Curve3D();
                theF3d = GCurve.First();
                theL3d = GCurve.Last();
                theLoc3d = GCurve.Location();
                NotDone = false;
            }
            anIt.Next();
        }
    }

    //=================================================================================================

    private static void UpdateVTol(in Vertex theV1, in Vertex theV2, double theTol)
    {
        var aB = new BrepBuilder();
        if (!theV1.IsNull && !isUnchangedFrozen(theV1, theTol))
        {
            aB.UpdateVertex(theV1, theTol);
        }
        if (!theV2.IsNull && !isUnchangedFrozen(theV2, theTol))
        {
            aB.UpdateVertex(theV2, theTol);
        }
    }

    // OCCT.NET: frozen. UpdateVertex(V, Tol) only raises the tolerance, so on a frozen vertex already at least as
    // tolerant it would change nothing but the Modified cache bit: skipped, rather than throwing.
    private static bool isUnchangedFrozen(in Vertex theV, double theTol)
    {
        return theV.TShape()!.IsFrozen && theTol <= Tool.Tolerance(theV);
    }

    //=================================================================================================

    /// <summary>
    /// Computes new pcurves for the edge so that they have the same parameter as its 3D curve, unless the edge is
    /// flagged SameParameter, and enlarges its vertices. Writes into the edge.
    /// </summary>
    internal static void SameParameter(in Edge theEdge, double Tolerance = 1.0e-5)
    {
        double aNewTol = -1;
        SameParameter(theEdge, Tolerance, ref aNewTol, true);
        if (aNewTol > 0)
        {
            Exploration.Vertices(theEdge, out Vertex aV1, out Vertex aV2);
            UpdateVTol(aV1, aV2, aNewTol);
        }
    }

    //=================================================================================================

    /// <summary>
    /// The SameParameter computation of one edge, on the edge itself (<paramref name="IsUseOldEdge"/>) or on an
    /// empty copy of it, which is returned (unfrozen). <paramref name="theNewTol"/> receives the tolerance its
    /// vertices need (not applied); it is left unchanged when nothing was computed. Returns a null edge when the edge
    /// is SameParameter or has no 3D curve.
    /// </summary>
    internal static Edge SameParameter(in Edge theEdge,
                                              double theTolerance,
                                              ref double theNewTol,
                                              bool IsUseOldEdge)
    {
        if (Tool.SameParameter(theEdge))
        {
            return new Edge();
        }
        double f3d = 0.0, l3d = 0.0;
        Placement L3d = new();
        Curve3D? C3d = null;
        NCollection_List<CurveRepresentation> CList;
        GetCurve3d(theEdge, ref C3d, ref f3d, ref l3d, ref L3d, out CList);
        if (C3d is null)
        {
            return new Edge();
        }

        var B = new BrepBuilder();
        Edge aNE;
        BrepTEdge aNTE;
        if (IsUseOldEdge)
        {
            aNE = theEdge;
            aNTE = (BrepTEdge)theEdge.TShape()!;
        }
        else
        {
            aNE = ShapeCasts.Edge(theEdge.EmptyCopied()); //will be modified a little bit later, so copy anyway
            GetCurve3d(aNE, ref C3d, ref f3d, ref l3d, ref L3d, out CList); //C3d pointer and CList will be differ after copying
            aNTE = (BrepTEdge)aNE.TShape()!;
            var sit = new Iterator(theEdge);
            for (; sit.More(); sit.Next())
            { // add vertices from old edge to the new ones
                B.Add(aNE, sit.Value());
            }
        }

        var It = new NCollection_List<CurveRepresentation>.Iterator(CList);

        const int NCONTROL = 22;

        var HC = new CurveAdaptor();
        var HC2d = new CurveAdaptor2D();
        var HS = new SurfaceAdaptor();
        CurveAdaptor GAC = HC;
        CurveAdaptor2D GAC2d = HC2d;
        SurfaceAdaptor GAS = HS;

        // modified by NIZHNY-OCC486  Tue Aug 27 17:15:13 2002 :
        bool m_TrimmedPeriodical = false;
        if (C3d!.GetType() == typeof(TrimmedCurve3D))
        {
            Curve3D gtC = ((TrimmedCurve3D)C3d).BasisCurve();
            m_TrimmedPeriodical = gtC.IsPeriodic;
        }
        // modified by NIZHNY-OCC486  Tue Aug 27 17:15:17 2002 .

        if (!C3d.IsPeriodic)
        {
            double Udeb = C3d.FirstParameter();
            double Ufin = C3d.LastParameter();
            // modified by NIZHNY-OCC486  Tue Aug 27 17:17:14 2002 :
            // if (Udeb > f3d) f3d = Udeb;
            // if (l3d > Ufin) l3d = Ufin;
            if (!m_TrimmedPeriodical)
            {
                if (Udeb > f3d)
                {
                    f3d = Udeb;
                }
                if (l3d > Ufin)
                {
                    l3d = Ufin;
                }
            }
            // modified by NIZHNY-OCC486  Tue Aug 27 17:17:55 2002 .
        }
        if (!L3d.IsIdentity)
        {
            C3d = (Curve3D)C3d.Transformed(L3d.Transformation());
        }
        GAC.Load(C3d, f3d, l3d);

        double Prec_C3d = BRepCheck.PrecCurve(GAC);

        bool IsSameP = true;
        double maxdist = 0.0;

        //  Modified by skv - Thu Jun  3 12:39:19 2004 OCC5898 Begin
        double anEdgeTol = Tool.Tolerance(aNE);
        //  Modified by skv - Thu Jun  3 12:39:20 2004 OCC5898 End
        bool SameRange = Tool.SameRange(aNE);
        bool YaPCu = false;
        const double BigError = 1.0e10;
        It.Initialize(CList);

        while (It.More())
        {

            bool isANA = false;
            bool isBSP = false;
            GCurve? GCurve = It.Value() as GCurve;
            var PC = new Curve2D?[2];
            Surface? S;
            if (GCurve is not null && GCurve.IsCurveOnSurface())
            {
                YaPCu = true;
                PC[0] = GCurve.PCurve();
                Placement PCLoc = GCurve.Location();
                S = GCurve.Surface();
                if (!PCLoc.IsIdentity)
                {
                    S = (Surface)S!.Transformed(PCLoc.Transformation());
                }

                GAS.Load(S!);
                if (GCurve.IsCurveOnClosedSurface)
                {
                    PC[1] = GCurve.PCurve2();
                }

                // Eval tol2d to compute SameRange
                double TolSameRange = Max(GAC.Resolution(theTolerance), Kerf.Engine.Foundation.Precision.PConfusion());
                for (int i = 0; i < 2; i++)
                {
                    Curve2D? curPC = PC[i];
                    bool updatepc = false;
                    if (curPC is null)
                    {
                        break;
                    }
                    if (!SameRange)
                    {
                        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(TolSameRange, PC[i], GCurve.First(), GCurve.Last(), f3d, l3d, ref curPC);

                        updatepc = !ReferenceEquals(curPC, PC[i]);
                    }
                    bool goodpc = true;
                    GAC2d.Load(curPC!, f3d, l3d);

                    double error = ComputeTol(HC, HC2d, HS, NCONTROL);

                    if (error > BigError)
                    {
                        maxdist = error;
                        break;
                    }

                    if (GAC2d.GetType() == BSplineCurve && GAC2d.Continuity() == C0)
                    {
                        double UResol = GAS.UResolution(theTolerance);
                        double VResol = GAS.VResolution(theTolerance);
                        double TolConf2d = Min(UResol, VResol);
                        TolConf2d = Max(TolConf2d, Kerf.Engine.Foundation.Precision.PConfusion());
                        BSplineCurve2D bs2d = GAC2d.BSpline();
                        BSplineCurve2D bs2dsov = bs2d;
                        double fC0 = bs2d.FirstParameter(), lC0 = bs2d.LastParameter();
                        bool repar = true;
                        Point2D OriginPoint;
                        bs2d.D0(fC0, out OriginPoint);
                        Geom2dConvert2D.C0BSplineToC1BSplineCurve(ref bs2d, TolConf2d);
                        isBSP = true;

                        if (bs2d.IsPeriodic)
                        { // -------- IFV, Jan 2000
                            bs2d.D0(bs2d.FirstParameter(), out Point2D NewOriginPoint);
                            if (Math.Abs(OriginPoint.X() - NewOriginPoint.X()) > Kerf.Engine.Foundation.Precision.PConfusion()
                                || Math.Abs(OriginPoint.Y() - NewOriginPoint.Y()) > Kerf.Engine.Foundation.Precision.PConfusion())
                            {

                                PackedList<double> Knotbs2d = bs2d.Knots();

                                for (int Index = 1; Index <= bs2d.KnotCount; Index++)
                                {
                                    bs2d.D0(Knotbs2d[Index - 1], out NewOriginPoint);
                                    if (Math.Abs(OriginPoint.X() - NewOriginPoint.X()) > Kerf.Engine.Foundation.Precision.PConfusion()
                                        || Math.Abs(OriginPoint.Y() - NewOriginPoint.Y()) > Kerf.Engine.Foundation.Precision.PConfusion())
                                    {
                                        continue;
                                    }

                                    bs2d.SetOrigin(Index);
                                    break;
                                }
                            }
                        }

                        if (bs2d.Continuity() == C0)
                        {
                            if (EvalTol(curPC!, S!, GAC, theTolerance, out double tolbail))
                            {
                                bs2d = bs2dsov;
                                double UResbail = GAS.UResolution(tolbail);
                                double VResbail = GAS.VResolution(tolbail);
                                double Tol2dbail = Min(UResbail, VResbail);
                                bs2d.D0(bs2d.FirstParameter(), out OriginPoint);

                                int nbp = bs2d.PoleCount;
                                Point2D p = bs2d.Pole(1), p1;
                                double d = Kerf.Engine.Foundation.Precision.Infinite();
                                for (int ip = 2; ip <= nbp; ip++)
                                {
                                    p1 = bs2d.Pole(ip);
                                    d = Min(d, p.SquareDistance(p1));
                                    p = p1;
                                }
                                d = Math.Sqrt(d) * 0.1;

                                Tol2dbail = Max(Min(Tol2dbail, d), TolConf2d);

                                Geom2dConvert2D.C0BSplineToC1BSplineCurve(ref bs2d, Tol2dbail);

                                if (bs2d.IsPeriodic)
                                { // -------- IFV, Jan 2000
                                    bs2d.D0(bs2d.FirstParameter(), out Point2D NewOriginPoint);
                                    if (Math.Abs(OriginPoint.X() - NewOriginPoint.X()) > Kerf.Engine.Foundation.Precision.PConfusion()
                                        || Math.Abs(OriginPoint.Y() - NewOriginPoint.Y()) > Kerf.Engine.Foundation.Precision.PConfusion())
                                    {

                                        PackedList<double> Knotbs2d = bs2d.Knots();

                                        for (int Index = 1; Index <= bs2d.KnotCount; Index++)
                                        {
                                            bs2d.D0(Knotbs2d[Index - 1], out NewOriginPoint);
                                            if (Math.Abs(OriginPoint.X() - NewOriginPoint.X()) > Kerf.Engine.Foundation.Precision.PConfusion()
                                                || Math.Abs(OriginPoint.Y() - NewOriginPoint.Y()) > Kerf.Engine.Foundation.Precision.PConfusion())
                                            {
                                                continue;
                                            }

                                            bs2d.SetOrigin(Index);
                                            break;
                                        }
                                    }
                                }

                                if (bs2d.Continuity() == C0)
                                {
                                    goodpc = true;
                                    bs2d = bs2dsov;
                                    repar = false;
                                }
                            }
                            else
                            {
                                goodpc = false;
                            }
                        }

                        if (goodpc)
                        {
                            if (repar)
                            {
                                var Knots = new NCollection_Array1<double>(1, bs2d.KnotCount);
                                bs2d.Knots(Knots);
                                //	    BSplCLib::Reparametrize(f3d,l3d,Knots);
                                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(fC0, lC0, Knots);
                                bs2d.SetKnots(Knots);
                                GAC2d.Load(bs2d, f3d, l3d);
                                curPC = bs2d;
                                bool updatepcsov = updatepc;
                                updatepc = true;

                                double error1 = ComputeTol(HC, HC2d, HS, NCONTROL);
                                if (error1 > error)
                                {
                                    bs2d = bs2dsov;
                                    GAC2d.Load(bs2d, f3d, l3d);
                                    curPC = bs2d;
                                    updatepc = updatepcsov;
                                    isANA = true;
                                }
                                else
                                {
                                    error = error1;
                                }
                            }

                            // check, if new BSpline "good" or not --------- IFV, Jan of 2000
                            Continuity cont = bs2d.Continuity();
                            bool IsBad = false;

                            if (cont > C0 && error > Max(1.0e-3, theTolerance))
                            {
                                int NbKnots = bs2d.KnotCount;
                                double critratio = 10.0;
                                double dtprev = bs2d.Knot(2) - bs2d.Knot(1), dtratio = 1.0;
                                double dtmin = dtprev;
                                double dtcur = 0.0;
                                for (int j = 2; j < NbKnots; j++)
                                {
                                    dtcur = bs2d.Knot(j + 1) - bs2d.Knot(j);
                                    dtmin = Min(dtmin, dtcur);

                                    if (IsBad)
                                    {
                                        continue;
                                    }

                                    if (dtcur > dtprev)
                                    {
                                        dtratio = dtcur / dtprev;
                                    }
                                    else
                                    {
                                        dtratio = dtprev / dtcur;
                                    }
                                    if (dtratio > critratio)
                                    {
                                        IsBad = true;
                                    }
                                    dtprev = dtcur;
                                }
                                if (IsBad)
                                {
                                    // To avoid failures in Approx_CurvilinearParameter
                                    bs2d.Resolution(Max(1.0e-3, theTolerance), out dtcur);
                                    if (dtmin < dtcur)
                                    {
                                        IsBad = false;
                                    }
                                }
                            }

                            if (IsBad)
                            { // if BSpline "bad", try to reparametrize it
                              // by its curve length

                                //	      GeomAbs_Shape cont = bs2d->Continuity();
                                if (cont > C2)
                                {
                                    cont = C2;
                                }
                                int maxdeg = bs2d.Degree();
                                if (maxdeg == 1)
                                {
                                    maxdeg = 14;
                                }
                                var AppCurPar = new CurvilinearParameter(HC2d,
                                                                                HS,
                                                                                Max(1.0e-3, theTolerance),
                                                                                cont,
                                                                                maxdeg,
                                                                                10);
                                if (AppCurPar.IsDone || AppCurPar.HasResult)
                                {
                                    // The internal form: the curve is re-knotted below before the edge takes it.
                                    bs2d = AppCurPar.InternalCurve2d1()!;
                                    GAC2d.Load(bs2d, f3d, l3d);
                                    curPC = bs2d;

                                    if (Math.Abs(bs2d.FirstParameter() - fC0) > TolSameRange
                                        || Math.Abs(bs2d.LastParameter() - lC0) > TolSameRange)
                                    {
                                        var Knots = new NCollection_Array1<double>(1, bs2d.KnotCount);
                                        bs2d.Knots(Knots);
                                        //		  BSplCLib::Reparametrize(f3d,l3d,Knots);
                                        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(fC0, lC0, Knots);
                                        bs2d.SetKnots(Knots);
                                        GAC2d.Load(bs2d, f3d, l3d);
                                        curPC = bs2d;
                                    }
                                }
                            }
                        }
                    }

                    if (goodpc)
                    {
                        //	  Approx_SameParameter SameP(HC,HC2d,HS,Tolerance);
                        double aTol = (isANA && isBSP) ? 1.0e-7 : theTolerance;
                        var SameP = new SameParameter(HC, HC2d, HS, aTol);

                        if (SameP.IsSameParameter)
                        {
                            maxdist = Max(maxdist, SameP.TolReached());
                            if (updatepc)
                            {
                                if (i == 0)
                                {
                                    GCurve.PCurve(curPC);
                                }
                                else
                                {
                                    GCurve.PCurve2(curPC);
                                }
                            }
                        }
                        else if (SameP.IsDone)
                        {
                            double tolreached = SameP.TolReached();
                            if (tolreached <= error)
                            {
                                curPC = SameP.InternalCurve2d();
                                updatepc = true;
                                maxdist = Max(maxdist, tolreached);
                            }
                            else
                            {
                                maxdist = Max(maxdist, error);
                            }
                            if (updatepc)
                            {
                                if (i == 0)
                                {
                                    GCurve.PCurve(curPC);
                                }
                                else
                                {
                                    GCurve.PCurve2(curPC);
                                }
                            }
                        }
                        else
                        {
                            // Approx_SameParameter has failed.
                            // Consequently, the situation might be,
                            // when 3D and 2D-curve do not have same-range.
                            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(TolSameRange,
                                              PC[i],
                                              GCurve.First(),
                                              GCurve.Last(),
                                              f3d,
                                              l3d,
                                              ref curPC);

                            if (i == 0)
                            {
                                GCurve.PCurve(curPC);
                            }
                            else
                            {
                                GCurve.PCurve2(curPC);
                            }

                            IsSameP = false;
                        }
                    }
                    else
                    {
                        IsSameP = false;
                    }

                    //  Modified by skv - Thu Jun  3 12:39:19 2004 OCC5898 Begin
                    if (!IsSameP)
                    {
                        double Prec_Surf = BRepCheck.PrecSurface(HS);
                        double CurTol = anEdgeTol + Max(Prec_C3d, Prec_Surf);
                        if (CurTol >= error)
                        {
                            maxdist = Max(maxdist, anEdgeTol);
                            IsSameP = true;
                        }
                    }
                    //  Modified by skv - Thu Jun  3 12:39:20 2004 OCC5898 End
                }
            }
            It.Next();
        }
        B.Range(aNE, f3d, l3d);
        B.SameRange(aNE, true);
        if (IsSameP)
        {
            // Reduce eventually the tolerance of the edge, as
            // all its representations are processed (except for some associated
            // to planes and not stored in the edge !)
            // The same cannot be done with vertices that cannot be enlarged
            // or left as is.
            if (YaPCu)
            {
                // Avoid setting too small tolerances.
                maxdist = Max(maxdist, Kerf.Engine.Foundation.Precision.Confusion());
                theNewTol = maxdist;
                aNTE.Modified(true);
                aNTE.Tolerance(maxdist);
            }
            B.SameParameter(aNE, true);
        }

        return aNE;
    }

    //=================================================================================================

    private static void InternalUpdateTolerances(in Shape theOldShape,
                                                 bool IsVerifyTolerance,
                                                 bool IsMutableInput,
                                                 ReShape theReshaper)
    {
        var aShToTol = new NCollection_DataMap<Shape, double>(ShapeHasher.Instance);
        // Harmonize tolerances
        // with rule Tolerance(VERTEX)>=Tolerance(EDGE)>=Tolerance(FACE)
        double tol = 0;
        if (IsVerifyTolerance)
        {
            // Set tolerance to its minimum value
            Surface? S;
            var ex = new Explorer();
            var aB = new Box();
            double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax, dMax;
            for (ex.Init(theOldShape, ShapeType.Face); ex.More(); ex.Next())
            {
                Face curf = ShapeCasts.Face(ex.Current());
                S = Tool.Surface(curf, out _);
                if (S is not null)
                {
                    aB.SetVoid();
                    BRepBndLib.Add(curf, ref aB);
                    if (S.GetType() == typeof(TrimmedSurface))
                    {
                        S = ((TrimmedSurface)S).BasisSurface();
                    }
                    var AS = new SurfaceAdaptor(S);
                    switch (AS.GetType())
                    {
                        case global::Kerf.Geometry.SurfaceKind.Plane:
                        case SurfaceKind.Cylinder:
                        case SurfaceKind.Cone:
                            {
                                tol = Kerf.Engine.Foundation.Precision.Confusion();
                                break;
                            }
                        case SurfaceKind.Sphere:
                        case SurfaceKind.Torus:
                            {
                                tol = Kerf.Engine.Foundation.Precision.Confusion() * 2;
                                break;
                            }
                        default:
                            tol = Kerf.Engine.Foundation.Precision.Confusion() * 4;
                            break;
                    }
                    if (!aB.IsWhole)
                    {
                        aB.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);
                        dMax = 1.0;
                        if (!aB.IsOpenXmin && !aB.IsOpenXmax)
                        {
                            dMax = aXmax - aXmin;
                        }
                        if (!aB.IsOpenYmin && !aB.IsOpenYmax)
                        {
                            aYmin = aYmax - aYmin;
                        }
                        if (!aB.IsOpenZmin && !aB.IsOpenZmax)
                        {
                            aZmin = aZmax - aZmin;
                        }
                        if (aYmin > dMax)
                        {
                            dMax = aYmin;
                        }
                        if (aZmin > dMax)
                        {
                            dMax = aZmin;
                        }
                        tol = tol * dMax;
                        // Do not process tolerances > 1.
                        if (tol > 1.0)
                        {
                            tol = 0.99;
                        }
                    }
                    aShToTol.Bind(curf, tol);
                }
            }
        }

        // Process edges
        var parents = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theOldShape, ShapeType.Edge, ShapeType.Face, parents);
        var lConx = new NCollection_List<Shape>.Iterator();
        int iCur;
        for (iCur = 1; iCur <= parents.Extent(); iCur++)
        {
            tol = 0;
            for (lConx.Initialize(parents[iCur]); lConx.More(); lConx.Next())
            {
                Face FF = ShapeCasts.Face(lConx.Value());
                double Ftol;
                if (IsVerifyTolerance && aShToTol.IsBound(FF))
                { // first condition for speed-up
                    Ftol = aShToTol.Find(FF);
                }
                else
                {
                    Ftol = Tool.Tolerance(FF); // tolerance have not been updated
                }
                tol = Max(tol, Ftol);
            }
            // Update can only increase tolerance, so if the edge has a greater
            //  tolerance than its faces it is not concerned
            Edge EK = ShapeCasts.Edge(parents.FindKey(iCur));
            if (tol > Tool.Tolerance(EK))
            {
                aShToTol.Bind(EK, tol);
            }
        }

        // Vertices are processed
        const double BigTol = 1.0e10;
        parents.Clear();

        Exploration.MapShapesAndUniqueAncestors(theOldShape, ShapeType.Vertex, ShapeType.Edge, parents);
        // OCCT's map of handles: the vertex TShapes by reference.
        var Initialized = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        int nbV = parents.Extent();
        for (iCur = 1; iCur <= nbV; iCur++)
        {
            tol = 0;
            Vertex V = ShapeCasts.Vertex(parents.FindKey(iCur));
            Point3D aPV = Tool.Pnt(V);
            double aMaxDist = 0.0;
            Point3D p3d;
            for (lConx.Initialize(parents[iCur]); lConx.More(); lConx.Next())
            {
                Edge E = ShapeCasts.Edge(lConx.Value());
                tol = Max(tol, aShToTol.Find(E, out double aNtol) ? aNtol : Tool.Tolerance(E));
                if (tol > BigTol)
                {
                    continue;
                }
                if (!Tool.SameRange(E))
                {
                    continue;
                }
                double par = Tool.Parameter(V, E);
                BrepTEdge TE = (BrepTEdge)E.TShape()!;
                var itcr = new NCollection_List<CurveRepresentation>.Iterator(TE.Curves());
                Placement Eloc = E.Location();
                while (itcr.More())
                {
                    // For each CurveRepresentation, check the provided parameter
                    CurveRepresentation cr = itcr.Value();
                    Placement loc = cr.Location();
                    Placement L = (Eloc * loc);
                    if (cr.IsCurve3D)
                    {
                        Curve3D? C = cr.Curve3D();
                        if (C is not null)
                        { // edge non degenerated
                            p3d = C.Value(par);
                            p3d.Transform(L.Transformation());
                            double aDist = p3d.SquareDistance(aPV);
                            if (aDist > aMaxDist)
                            {
                                aMaxDist = aDist;
                            }
                        }
                    }
                    else if (cr.IsCurveOnSurface())
                    {
                        Surface Su = cr.Surface()!;
                        Curve2D PC = cr.PCurve()!;
                        Curve2D? PC2 = null;
                        if (cr.IsCurveOnClosedSurface)
                        {
                            PC2 = cr.PCurve2();
                        }
                        Point2D p2d = PC.Value(par);
                        p3d = Su.Value(p2d.X(), p2d.Y());
                        p3d.Transform(L.Transformation());
                        double aDist = p3d.SquareDistance(aPV);
                        if (aDist > aMaxDist)
                        {
                            aMaxDist = aDist;
                        }
                        if (PC2 is not null)
                        {
                            p2d = PC2.Value(par);
                            p3d = Su.Value(p2d.X(), p2d.Y());
                            p3d.Transform(L.Transformation());
                            aDist = p3d.SquareDistance(aPV);
                            if (aDist > aMaxDist)
                            {
                                aMaxDist = aDist;
                            }
                        }
                    }
                    itcr.Next();
                }
            }
            tol = Max(tol, Math.Sqrt(aMaxDist));
            tol += 2.0 * Epsilon(tol);
            //
            double aVTol = Tool.Tolerance(V);
            bool anUpdTol = tol > aVTol;
            TShape aTV = V.TShape()!;
            bool toAdd = false;
            if (IsVerifyTolerance)
            {
                // ASet minimum value of the tolerance
                // Attention to sharing of the vertex by other shapes
                toAdd = Initialized.Add(aTV) && aVTol != tol; // if Vtol == tol => no need to update toler
            }
            //'Initialized' map is not used anywhere outside this block
            if (anUpdTol || toAdd)
            {
                aShToTol.Bind(V, tol);
            }
        }

        UpdShTol(aShToTol, IsMutableInput, theReshaper, true);
    }

    //=================================================================================================

    /// <summary>
    /// Raises the tolerances of the edges and vertices of <paramref name="S"/> to the rule
    /// Tolerance(VERTEX) &gt;= Tolerance(EDGE) &gt;= Tolerance(FACE); with <paramref name="verifyFaceTolerance"/>,
    /// first sets the faces' tolerances to a minimum by surface type and size (and lowers vertices to what they need).
    /// Writes into the shape.
    /// </summary>
    internal static void UpdateTolerances(in Shape S, bool verifyFaceTolerance = false)
    {
        var aReshaper = new ReShape();
        InternalUpdateTolerances(S, verifyFaceTolerance, true, aReshaper);
    }

    //=================================================================================================

    /// <summary>
    /// <see cref="UpdateTolerances(in Shape, bool)"/> without changing <paramref name="S"/>: the changed
    /// faces, edges and vertices are copies recorded as replacements in <paramref name="theReshaper"/>, whose
    /// <see cref="ReShape.Apply"/> gives the result.
    /// </summary>
    public static void UpdateTolerances(in Shape S,
                                        ReShape theReshaper,
                                        bool verifyFaceTolerance = false)
    {
        InternalUpdateTolerances(S, verifyFaceTolerance, false, theReshaper);
    }

    //=================================================================================================

    /// <summary>
    /// Checks the tolerances of the edges (inner points included) and vertices of the shape and raises them to
    /// satisfy the SameParameter condition. Writes into the shape.
    /// </summary>
    internal static void UpdateInnerTolerances(in Shape aShape)
    {
        var EFmap = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aShape, ShapeType.Edge, ShapeType.Face, EFmap);
        var BB = new BrepBuilder();
        for (int i = 1; i <= EFmap.Extent(); i++)
        {
            Edge anEdge = ShapeCasts.Edge(EFmap.FindKey(i));

            if (!Tool.IsGeometric(anEdge))
            {
                continue;
            }

            Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
            Tool.Range(anEdge, out double fpar, out double lpar);
            double TolEdge = Tool.Tolerance(anEdge);
            Point3D Pnt1 = new(), Pnt2 = new();
            var anHCurve = new EdgeCurveAdaptor();
            anHCurve.Initialize(anEdge);
            if (!V1.IsNull)
            {
                Pnt1 = Tool.Pnt(V1);
            }
            if (!V2.IsNull)
            {
                Pnt2 = Tool.Pnt(V2);
            }

            if (!Tool.Degenerated(anEdge) && EFmap[i].Extent() > 0)
            {
                var theRep = new NCollection_Sequence<CurveAdaptorBase>();
                theRep.Append(anHCurve);
                var itl = new NCollection_List<Shape>.Iterator(EFmap[i]);
                for (; itl.More(); itl.Next())
                {
                    Face aFace = ShapeCasts.Face(itl.Value());
                    var anHCurvOnSurf = new EdgeCurveAdaptor();
                    anHCurvOnSurf.Initialize(anEdge, aFace);
                    theRep.Append(anHCurvOnSurf);
                }

                int NbSamples = (Tool.SameParameter(anEdge)) ? 23 : 2;
                double delta = (lpar - fpar) / (NbSamples - 1);
                double MaxDist = 0.0;
                for (int j = 2; j <= theRep.Length(); j++)
                {
                    for (int k = 0; k <= NbSamples; k++)
                    {
                        double ParamOnCenter = (k == NbSamples) ? lpar : fpar + k * delta;
                        Point3D Center = theRep[1].Value(ParamOnCenter);
                        double ParamOnCurve =
                          (Tool.SameParameter(anEdge))
                            ? ParamOnCenter
                            : ((k == 0) ? theRep[j].FirstParameter() : theRep[j].LastParameter());
                        Point3D aPoint = theRep[j].Value(ParamOnCurve);
                        double aDist = Center.Distance(aPoint);
                        // aDist *= 1.1;
                        aDist += 2.0 * Epsilon(aDist);
                        if (aDist > MaxDist)
                        {
                            MaxDist = aDist;
                        }

                        // Update tolerances of vertices
                        if (k == 0 && !V1.IsNull)
                        {
                            double aDist1 = Pnt1.Distance(aPoint);
                            aDist1 += 2.0 * Epsilon(aDist1);
                            BB.UpdateVertex(V1, aDist1);
                        }
                        if (k == NbSamples && !V2.IsNull)
                        {
                            double aDist2 = Pnt2.Distance(aPoint);
                            aDist2 += 2.0 * Epsilon(aDist2);
                            BB.UpdateVertex(V2, aDist2);
                        }
                    }
                }
                BB.UpdateEdge(anEdge, MaxDist);
            }
            TolEdge = Tool.Tolerance(anEdge);
            if (!V1.IsNull)
            {
                Point3D End1 = anHCurve.Value(fpar);
                double dist1 = Pnt1.Distance(End1);
                dist1 += 2.0 * Epsilon(dist1);
                BB.UpdateVertex(V1, Max(dist1, TolEdge));
            }
            if (!V2.IsNull)
            {
                Point3D End2 = anHCurve.Value(lpar);
                double dist2 = Pnt2.Distance(End2);
                dist2 += 2.0 * Epsilon(dist2);
                BB.UpdateVertex(V2, Max(dist2, TolEdge));
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Orients the solid so that matter is inside it (reversing the value when the infinite point classifies IN).
    /// False if the solid is not orientable (open or incoherent). Changes only the value's orientation.
    /// </summary>
    internal static bool OrientClosedSolid(ref Solid solid)
    {
        // Set material inside the solid
        var where = new BRepClass3d_SolidClassifier(solid);
        where.PerformInfinitePoint(Kerf.Engine.Foundation.Precision.Confusion());
        if (where.State() == In)
        {
            solid.Reverse();
        }
        else if (where.State() == On || where.State() == Unknown)
        {
            return false;
        }

        return true;
    }

    // Structure for calculation of properties, necessary for decision about continuity
    private sealed class SurfaceProperties
    {
        private readonly SLProps mySurfaceProps; // properties calculator
        private readonly Transform3D mySurfaceTrsf;
        private readonly Curve2D myCurve2d;
        private readonly bool myIsReversed; // the face based on the surface is reversed

        // tangent vector to Pcurve in UV
        private Vector2D myCurveTangent;

        public SurfaceProperties(Surface theSurface, in Transform3D theSurfaceTrsf, Curve2D theCurve2D, bool theReversed)
        {
            mySurfaceProps = new SLProps(theSurface, 2, Kerf.Engine.Foundation.Precision.Confusion());
            mySurfaceTrsf = theSurfaceTrsf;
            myCurve2d = theCurve2D;
            myIsReversed = theReversed;
            myCurveTangent = new Vector2D();
        }

        // Calculate derivatives on surface related to the point on curve
        public void Calculate(double theParamOnCurve)
        {
            myCurve2d.D1(theParamOnCurve, out Point2D aUV, out myCurveTangent);
            mySurfaceProps.SetParameters(aUV.X(), aUV.Y());
        }

        // Returns point just calculated
        public Point3D Value() => mySurfaceProps.Value().Transformed(mySurfaceTrsf);

        // Calculate a derivative orthogonal to curve's tangent vector
        public Vector3D Derivative()
        {
            var aDeriv = new Vector3D();
            // direction orthogonal to tangent vector of the curve
            var anOrtho = new Vector2D(-myCurveTangent.Y(), myCurveTangent.X());
            double aLen = anOrtho.Magnitude();
            if (aLen < Kerf.Engine.Foundation.Precision.Confusion())
            {
                return aDeriv;
            }
            anOrtho /= aLen;
            if (myIsReversed)
            {
                anOrtho.Reverse();
            }

            aDeriv.SetLinearForm(anOrtho.X(), mySurfaceProps.D1U(), anOrtho.Y(), mySurfaceProps.D1V());
            return aDeriv.Transformed(mySurfaceTrsf);
        }

        public Direction3D Normal()
        {
            Direction3D aNormal = mySurfaceProps.Normal();
            return aNormal.Transformed(mySurfaceTrsf);
        }

        // Calculate principal curvatures, which consist of minimal and maximal normal curvatures and
        // the directions on the tangent plane (principal direction) where the extremums are reached
        public void Curvature(out Direction3D thePrincipalDir1, out double theCurvature1, out Direction3D thePrincipalDir2, out double theCurvature2)
        {
            mySurfaceProps.CurvatureDirections(out thePrincipalDir1, out thePrincipalDir2);
            theCurvature1 = mySurfaceProps.MaxCurvature();
            theCurvature2 = mySurfaceProps.MinCurvature();
            if (myIsReversed)
            {
                theCurvature1 = -theCurvature1;
                theCurvature2 = -theCurvature2;
            }
            if (mySurfaceTrsf.IsNegative)
            {
                theCurvature1 = -theCurvature1;
                theCurvature2 = -theCurvature2;
            }

            thePrincipalDir1.Transform(mySurfaceTrsf);
            thePrincipalDir2.Transform(mySurfaceTrsf);
        }
    }

    /// <summary>
    /// The order of continuity between two faces connected by an edge (the angular tolerance in radians). A seam
    /// edge (both faces the same) on an elementary surface is CN.
    /// </summary>
    public static Continuity ContinuityOfFaces(in Edge theEdge,
                                                  in Face theFace1,
                                                  in Face theFace2,
                                                  double theAngleTol)
    {
        bool isSeam = theFace1.IsEqual(theFace2);

        Edge anEdgeInFace1 = new(), anEdgeInFace2 = new();
        Curve2D? aCurve1, aCurve2;

        double aFirst, aLast;

        if (!theFace1.IsSame(theFace2) && Tool.IsClosed(theEdge, theFace1)
            && Tool.IsClosed(theEdge, theFace2))
        {
            // Find the edge in the face 1: this edge will have correct orientation
            Face aFace1 = theFace1;
            aFace1.Orientation(Forward);
            var anExplo = new Explorer(aFace1, ShapeType.Edge);
            for (; anExplo.More(); anExplo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anExplo.Current());
                if (anEdge.IsSame(theEdge))
                {
                    anEdgeInFace1 = anEdge;
                    break;
                }
            }
            if (anEdgeInFace1.IsNull)
            {
                return C0;
            }

            aCurve1 = Tool.CurveOnSurface(anEdgeInFace1, aFace1, out aFirst, out aLast);
            Face aFace2 = theFace2;
            aFace2.Orientation(Forward);
            anEdgeInFace2 = anEdgeInFace1;
            anEdgeInFace2.Reverse();
            aCurve2 = Tool.CurveOnSurface(anEdgeInFace2, aFace2, out aFirst, out aLast);
        }
        else
        {
            // Obtaining of pcurves of edge on two faces.
            anEdgeInFace1 = anEdgeInFace2 = theEdge;
            aCurve1 = Tool.CurveOnSurface(anEdgeInFace1, theFace1, out aFirst, out aLast);
            // For the case of seam edge
            if (theFace1.IsSame(theFace2))
            {
                anEdgeInFace2.Reverse();
            }
            aCurve2 = Tool.CurveOnSurface(anEdgeInFace2, theFace2, out aFirst, out aLast);
        }

        if (aCurve1 is null || aCurve2 is null)
        {
            return C0;
        }

        Surface aSurface1 = Tool.Surface(theFace1, out Placement aLoc1)!;
        Transform3D aSurf1Trsf = aLoc1.Transformation();
        Surface aSurface2 = Tool.Surface(theFace2, out Placement aLoc2)!;
        Transform3D aSurf2Trsf = aLoc2.Transformation();

        if (aSurface1 is TrimmedSurface aTS1)
        {
            aSurface1 = aTS1.BasisSurface();
        }
        if (aSurface2 is TrimmedSurface aTS2)
        {
            aSurface2 = aTS2.BasisSurface();
        }

        // seam edge on elementary surface is always CN
        bool isElementary = (aSurface1 is ElementarySurface && aSurface2 is ElementarySurface);
        if (isSeam && isElementary)
        {
            return CN;
        }

        var aSP1 = new SurfaceProperties(aSurface1, aSurf1Trsf, aCurve1, theFace1.Orientation() == Reversed);
        var aSP2 = new SurfaceProperties(aSurface2, aSurf2Trsf, aCurve2, theFace2.Orientation() == Reversed);

        double f, l, eps;
        Tool.Range(theEdge, out f, out l);
        var ext = new LocateExtPC();
        EdgeCurveAdaptor? aHC2 = null;

        eps = (l - f) / 100.0;
        f += eps; // to avoid calculations on
        l -= eps; // points of pointed squares.

        double anAngleTol2 = theAngleTol * theAngleTol;

        Vector3D aDer1, aDer2;
        double aSqLen1, aSqLen2;
        var aCrvDir1 = new Direction3D[2];
        var aCrvDir2 = new Direction3D[2];
        Span<double> aCrvLen1 = stackalloc double[2];
        Span<double> aCrvLen2 = stackalloc double[2];

        Continuity aCont = (isElementary ? CN : C2);
        Continuity aCurCont;
        double u;
        for (int i = 0; i <= 20 && aCont > C0; i++)
        {
            // First suppose that this is sameParameter
            u = f + (l - f) * i / 20;

            // Check conditions for G1 and C1 continuity:
            // * calculate a derivative in tangent plane of each surface
            //   orthogonal to curve's tangent vector
            // * continuity is C1 if the vectors are equal
            // * continuity is G1 if the vectors are just parallel
            aCurCont = C0;

            aSP1.Calculate(u);
            aSP2.Calculate(u);

            aDer1 = aSP1.Derivative();
            aSqLen1 = aDer1.SquareMagnitude();
            aDer2 = aSP2.Derivative();
            aSqLen2 = aDer2.SquareMagnitude();
            bool isSmoothSuspect = (aDer1.CrossSquareMagnitude(aDer2) <= anAngleTol2 * aSqLen1 * aSqLen2);
            if (isSmoothSuspect)
            {
                Direction3D aNormal1 = aSP1.Normal();
                if (theFace1.Orientation() == Reversed)
                {
                    aNormal1.Reverse();
                }
                Direction3D aNormal2 = aSP2.Normal();
                if (theFace2.Orientation() == Reversed)
                {
                    aNormal2.Reverse();
                }

                if (aNormal1 * aNormal2 < 0.0)
                {
                    return C0;
                }
            }

            if (!isSmoothSuspect)
            {
                // Refine by projection
                if (aHC2 is null)
                {
                    // adaptor for pcurve on the second surface
                    aHC2 = new EdgeCurveAdaptor(anEdgeInFace2, theFace2);
                    ext.Initialize(aHC2, f, l, Kerf.Engine.Foundation.Precision.PConfusion());
                }
                ext.Perform(aSP1.Value(), u);
                if (ext.IsDone && ext.IsMin())
                {
                    POnCurv poc = ext.Point();
                    aSP2.Calculate(poc.Parameter());
                    aDer2 = aSP2.Derivative();
                    aSqLen2 = aDer2.SquareMagnitude();
                }
                isSmoothSuspect = (aDer1.CrossSquareMagnitude(aDer2) <= anAngleTol2 * aSqLen1 * aSqLen2);
            }
            if (isSmoothSuspect)
            {
                aCurCont = G1;
                if (Math.Abs(Math.Sqrt(aSqLen1) - Math.Sqrt(aSqLen2)) < Kerf.Engine.Foundation.Precision.Confusion()
                    && aDer1.Dot(aDer2) > Kerf.Engine.Foundation.Precision.SquareConfusion())
                { // <= check vectors are codirectional
                    aCurCont = C1;
                }
            }
            else
            {
                return C0;
            }

            if (aCont < G2)
            {
                continue; // no need further processing, because maximal continuity is less than G2
            }

            // Check conditions for G2 and C2 continuity:
            // * calculate principal curvatures on each surface
            // * continuity is C2 if directions of principal curvatures are equal on different surfaces
            // * continuity is G2 if directions of principal curvatures are just parallel
            //   and values of curvatures are the same
            aSP1.Curvature(out aCrvDir1[0], out aCrvLen1[0], out aCrvDir1[1], out aCrvLen1[1]);
            aSP2.Curvature(out aCrvDir2[0], out aCrvLen2[0], out aCrvDir2[1], out aCrvLen2[1]);
            for (int aStep = 0; aStep <= 1; ++aStep)
            {
                if (aCrvDir1[0].XYZ().CrossSquareMagnitude(aCrvDir2[aStep].XYZ()) <= Kerf.Engine.Foundation.Precision.SquareConfusion()
                    && Math.Abs(aCrvLen1[0] - aCrvLen2[aStep]) < Kerf.Engine.Foundation.Precision.Confusion()
                    && aCrvDir1[1].XYZ().CrossSquareMagnitude(aCrvDir2[1 - aStep].XYZ()) <= Kerf.Engine.Foundation.Precision.SquareConfusion()
                    && Math.Abs(aCrvLen1[1] - aCrvLen2[1 - aStep]) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    if (aCurCont == C1 && aCrvDir1[0].Dot(aCrvDir2[aStep]) > Kerf.Engine.Foundation.Precision.Confusion()
                        && aCrvDir1[1].Dot(aCrvDir2[1 - aStep]) > Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        aCurCont = C2;
                    }
                    else
                    {
                        aCurCont = G2;
                    }
                    break;
                }
            }

            if (aCurCont < aCont)
            {
                aCont = aCurCont;
            }
        }

        // according to the list of supported elementary surfaces,
        // if the continuity is C2, than it is totally CN
        if (isElementary && aCont == C2)
        {
            aCont = CN;
        }
        return aCont;
    }

    //=======================================================================
    // function : EncodeRegularity
    // purpose  : Code the regularities on all edges of the shape, boundary of
    //            two faces that do not have it.
    //            Takes into account that compound may consists of same solid
    //            placed with different transformations
    //=======================================================================
    private static void encodeRegularity(in Shape theShape,
                                         double theTolAng,
                                         NCollection_Map<Shape> theMap,
                                         NCollection_Map<Shape>? theEdgesToEncode = null)
    {
        Shape aShape = theShape;
        var aNullLoc = new Placement();
        aShape.Location(aNullLoc); // nullify location
        if (!theMap.Add(aShape))
        {
            return; // do not need to process shape twice
        }

        if (aShape.ShapeType() == ShapeType.Compound || aShape.ShapeType() == ShapeType.CompSolid)
        {
            for (var it = new Iterator(aShape); it.More(); it.Next())
            {
                encodeRegularity(it.Value(), theTolAng, theMap, theEdgesToEncode);
            }
            return;
        }

        try
        {
            var M = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(aShape, ShapeType.Edge, ShapeType.Face, M);
            var It = new NCollection_List<Shape>.Iterator();
            var Ex = new Explorer();
            Face F1 = new(), F2 = new();
            for (int i = 1; i <= M.Extent(); i++)
            {
                Edge E = ShapeCasts.Edge(M.FindKey(i));
                if (theEdgesToEncode is not null && !theEdgesToEncode.IsEmpty())
                {
                    // process only the edges from the list to update their regularity
                    Shape aPureEdge = E.Located(aNullLoc);
                    aPureEdge.Orientation(Forward);
                    if (!theEdgesToEncode.Contains(aPureEdge))
                    {
                        continue;
                    }
                }

                bool found = false;
                F1.Nullify();
                for (It.Initialize(M.FindFromIndex(i)); It.More() && !found; It.Next())
                {
                    if (F1.IsNull)
                    {
                        F1 = ShapeCasts.Face(It.Value());
                    }
                    else
                    {
                        Face aTmpF2 = ShapeCasts.Face(It.Value());
                        if (!F1.IsSame(aTmpF2))
                        {
                            found = true;
                            F2 = aTmpF2;
                        }
                    }
                }
                if (!found && !F1.IsNull)
                { // is it a sewing edge?
                    Orientation orE = E.Orientation();
                    Edge curE;
                    for (Ex.Init(F1, ShapeType.Edge); Ex.More() && !found; Ex.Next())
                    {
                        curE = ShapeCasts.Edge(Ex.Current());
                        if (E.IsSame(curE) && orE != curE.Orientation())
                        {
                            found = true;
                            F2 = F1;
                        }
                    }
                }
                if (found)
                {
                    EncodeRegularity(E, F1, F2, theTolAng);
                }
            }
        }
        catch (FailureException anException) when (anException is not LockedShapeException) // OCCT.NET: frozen
        {
        }
    }

    /// <summary>
    /// Codes the regularity of every edge of the shape bounding two faces that hasn't one yet (in place: internal; the
    /// angular tolerance in radians). A compound holding the same solid at several locations is coded once.
    /// </summary>
    internal static void EncodeRegularity(in Shape S, double TolAng = 1.0e-10)
    {
        var aMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        encodeRegularity(S, TolAng, aMap);
    }

    /// <summary>
    /// Codes the regularity of the edges of <paramref name="LE"/> bounding two faces of the shape that haven't one yet
    /// (in place: internal).
    /// </summary>
    internal static void EncodeRegularity(in Shape S, NCollection_List<Shape> LE, double TolAng = 1.0e-10)
    {
        // Collect edges without location and orientation
        var aPureEdges = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var aNullLoc = new Placement();
        var anEdgeIt = new NCollection_List<Shape>.Iterator(LE);
        for (; anEdgeIt.More(); anEdgeIt.Next())
        {
            Shape anEdge = anEdgeIt.Value();
            anEdge.Location(aNullLoc);
            anEdge.Orientation(Forward);
            aPureEdges.Add(anEdge);
        }

        var aMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        encodeRegularity(S, TolAng, aMap, aPureEdges);
    }

    /// <summary>
    /// Codes the regularity between <paramref name="F1"/> and <paramref name="F2"/> on <paramref name="E"/>, unless
    /// already coded (in place: internal).
    /// </summary>
    internal static void EncodeRegularity(in Edge E, in Face F1, in Face F2, double TolAng = 1.0e-10)
    {
        var B = new BrepBuilder();
        if (Tool.Continuity(E, F1, F2) <= C0)
        {
            try
            {
                Continuity aCont = ContinuityOfFaces(E, F1, F2, TolAng);
                B.Continuity(E, F1, F2, aCont);
            }
            // OCCT.NET: frozen. OCCT swallows the TopoDS_LockedShape of a locked edge; a frozen one is a write into a
            // shape the caller holds, which must not pass silently.
            catch (FailureException) when (!E.TShape()!.IsFrozen)
            {
            }
        }
    }

    /// <summary>
    /// The writable copies of the frozen triangulations an in-place static changes, attached once it is done.
    /// </summary>
    /// <remarks>
    /// OCCT.NET: frozen. OCCT writes normals and deflections into the triangulations attached to the faces, which the
    /// port freezes when they are attached (design 4.1). The port writes into a copy of each (one copy per
    /// triangulation, so faces sharing one share its copy, as they would see OCCT's change) and, at the end, puts the
    /// copy in place of the original on each face of the shape that has it active (keeping the face's list of
    /// triangulations, with the copy active) and on the edges' polygons on it.
    /// </remarks>
    private sealed class WritableTriangulations
    {
        private readonly Dictionary<Triangulation, Triangulation> myCopies = new(ReferenceEqualityComparer.Instance);

        // The triangulation to read: the copy if one was made.
        public Triangulation Read(Triangulation theT)
            => myCopies.TryGetValue(theT, out Triangulation? aCopy) ? aCopy : theT;

        // The triangulation to write into: a copy of a frozen one.
        public Triangulation Write(Triangulation theT)
        {
            if (myCopies.TryGetValue(theT, out Triangulation? aCopy))
            {
                return aCopy;
            }
            if (!theT.IsFrozen)
            {
                return theT;
            }
            aCopy = theT.Copy();
            myCopies.Add(theT, aCopy);
            return aCopy;
        }

        public void Attach(in Shape theShape)
        {
            if (myCopies.Count == 0)
            {
                return;
            }
            var aB = new BrepBuilder();
            var aDone = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
            for (var anExpFace = new Explorer(theShape, ShapeType.Face); anExpFace.More(); anExpFace.Next())
            {
                Face aFace = ShapeCasts.Face(anExpFace.Current());
                if (!aDone.Add(aFace.TShape()!))
                {
                    continue;
                }
                var aTF = (BrepTFace)aFace.TShape()!;
                Triangulation? anOld = aTF.ActiveTriangulation();
                if (anOld is null || !myCopies.TryGetValue(anOld, out Triangulation? aNew))
                {
                    continue;
                }
                aB.UpdateFace(aFace, aNew, false);
                for (var anExpEdge = new Explorer(aFace, ShapeType.Edge); anExpEdge.More(); anExpEdge.Next())
                {
                    var aTE = (BrepTEdge)anExpEdge.Current().TShape()!;
                    if (!aDone.Add(aTE))
                    {
                        continue;
                    }
                    for (var anIt = new NCollection_List<CurveRepresentation>.Iterator(aTE.ChangeCurves()); anIt.More(); anIt.Next())
                    {
                        CurveRepresentation aCR = anIt.Value();
                        if (!aCR.IsPolygonOnTriangulation() || !myCopies.TryGetValue(aCR.Triangulation()!, out Triangulation? aNewT))
                        {
                            continue;
                        }
                        anIt.ChangeValue() = aCR.IsPolygonOnClosedTriangulation
                            ? new PolygonOnClosedTriangulation(aCR.PolygonOnTriangulation(), aCR.PolygonOnTriangulation2(), aNewT, aCR.Location())
                            : new PolygonOnTriangulationRepresentation(aCR.PolygonOnTriangulation(), aNewT, aCR.Location());
                    }
                    anExpEdge.Current().TShape()!.Modified(true);
                }
            }
        }
    }

    /// <summary>
    /// Corrects the normals of the faces' triangulations so that nodes along smooth edges have the same normal on
    /// both sides (computing missing normals first). Returns true if anything was corrected. In place: internal; the
    /// triangulations, frozen once attached, are replaced by corrected copies (see the remarks of the port).
    /// </summary>
    internal static bool EnsureNormalConsistency(in Shape theShape, double theAngTol = 0.001, bool theForceComputeNormals = false)
    {
        double aThresDot = Math.Cos(theAngTol);

        bool aRetVal = false, isNormalsFound = false;
        var aWritable = new WritableTriangulations(); // OCCT.NET: frozen

        // compute normals if they are absent
        var anExpFace = new Explorer(theShape, ShapeType.Face);
        for (; anExpFace.More(); anExpFace.Next())
        {
            Face aFace = ShapeCasts.Face(anExpFace.Current());
            Surface? aSurf = Tool.Surface(aFace);
            if (aSurf is null)
            {
                continue;
            }
            Triangulation? anAttached = Tool.Triangulation(aFace, out Placement aLoc);
            if (anAttached is null)
            {
                continue;
            }
            Triangulation aPT = aWritable.Read(anAttached);
            if (!theForceComputeNormals && aPT.HasNormals)
            {
                isNormalsFound = true;
                continue;
            }

            aPT = aWritable.Write(anAttached);
            aPT.AddNormals();
            var aSLP = new SLProps(aSurf, 2, Kerf.Engine.Foundation.Precision.Confusion());
            for (int i = 1; i <= aPT.NodeCount; i++)
            {
                Point2D aP2d = aPT.UVNode(i);
                aSLP.SetParameters(aP2d.X(), aP2d.Y());

                if (!aSLP.IsNormalDefined)
                {
                    aPT.SetNormal(i, new NCollection_Vec3<float>(0.0f));
                }
                else
                {
                    Direction3D aNorm = aSLP.Normal();
                    if (aFace.Orientation() == Reversed)
                    {
                        aNorm.Reverse();
                    }
                    aPT.SetNormal(i, aNorm);
                }
            }

            aRetVal = true;
            isNormalsFound = true;
        }

        if (!isNormalsFound)
        {
            aWritable.Attach(theShape);
            return aRetVal;
        }

        // loop by edges
        var aMapEF = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theShape, ShapeType.Edge, ShapeType.Face, aMapEF);
        for (int anInd = 1; anInd <= aMapEF.Extent(); anInd++)
        {
            Edge anEdg = ShapeCasts.Edge(aMapEF.FindKey(anInd));
            NCollection_List<Shape> anEdgList = aMapEF.FindFromIndex(anInd);
            if (anEdgList.Extent() != 2)
            {
                continue;
            }
            var anItF = new NCollection_List<Shape>.Iterator(anEdgList);
            Face aFace1 = ShapeCasts.Face(anItF.Value());
            anItF.Next();
            Face aFace2 = ShapeCasts.Face(anItF.Value());
            Triangulation? anAttached1 = Tool.Triangulation(aFace1, out Placement aLoc1);
            Triangulation? anAttached2 = Tool.Triangulation(aFace2, out Placement aLoc2);

            if (anAttached1 is null || anAttached2 is null)
            {
                continue;
            }
            Triangulation aPT1 = aWritable.Read(anAttached1);
            Triangulation aPT2 = aWritable.Read(anAttached2);

            if (!aPT1.HasNormals || !aPT2.HasNormals)
            {
                continue;
            }

            // the edges still refer to the attached triangulations. OCCT dereferences the polygons without a null check
            // (occt-issues.md): an edge with no polygon on one of the triangulations throws here.
            PolygonOnTriangulation aPTEF1 = Tool.PolygonOnTriangulation(anEdg, anAttached1, aLoc1)!;
            PolygonOnTriangulation aPTEF2 = Tool.PolygonOnTriangulation(anEdg, anAttached2, aLoc2)!;

            if (aPTEF1.Nodes().Lower() != aPTEF2.Nodes().Lower()
                || aPTEF1.Nodes().Upper() != aPTEF2.Nodes().Upper())
            {
                continue;
            }

            for (int anEdgNode = aPTEF1.Nodes().Lower(); anEdgNode <= aPTEF1.Nodes().Upper(); anEdgNode++)
            {
                // Number of node
                int aFNodF1 = aPTEF1.Nodes().Value(anEdgNode);
                int aFNodF2 = aPTEF2.Nodes().Value(anEdgNode);

                aPT1 = aWritable.Read(anAttached1);
                aPT2 = aWritable.Read(anAttached2);
                aPT1.Normal(aFNodF1, out NCollection_Vec3<float> aNorm1f);
                aPT2.Normal(aFNodF2, out NCollection_Vec3<float> aNorm2f);
                var aNorm1 = new Xyz(aNorm1f.X(), aNorm1f.Y(), aNorm1f.Z());
                var aNorm2 = new Xyz(aNorm2f.X(), aNorm2f.Y(), aNorm2f.Z());
                double aDot = aNorm1 * aNorm2;
                if (aDot > aThresDot)
                {
                    Xyz aNewNorm = (aNorm1 + aNorm2).Normalized();
                    aWritable.Write(anAttached1).SetNormal(aFNodF1, new Direction3D(aNewNorm));
                    aWritable.Write(anAttached2).SetNormal(aFNodF2, new Direction3D(aNewNorm));
                    aRetVal = true;
                }
            }
        }

        aWritable.Attach(theShape);
        return aRetVal;
    }

    // Tool to estimate deflection of the given UV point with regard to its representation in 3D space.
    private sealed class EvalDeflection
    {
        private readonly FaceSurfaceAdaptor Surface;

        // Initializes tool with the given face.
        public EvalDeflection(in Face theFace)
        {
            Surface = new FaceSurfaceAdaptor(theFace);
        }

        // Evaluates deflection of the given 2d point from its 3d representation.
        public double Eval(in Point2D thePoint2d, in Point3D thePoint3d)
        {
            Surface.D0(thePoint2d.X(), thePoint2d.Y(), out Point3D aPnt);
            return (thePoint3d.XYZ() - aPnt.XYZ()).SquareModulus();
        }
    }

    // Represents link of triangulation: equal whatever the order of its nodes.
    private readonly struct Link : IEquatable<Link>
    {
        private readonly int myNode0;
        private readonly int myNode1;

        public Link(int theNode1, int theNode2)
        {
            myNode0 = theNode1;
            myNode1 = theNode2;
        }

        public bool Equals(Link theOther)
            => (myNode0 == theOther.myNode0 && myNode1 == theOther.myNode1)
               || (myNode0 == theOther.myNode1 && myNode1 == theOther.myNode0);

        public override bool Equals(object? obj) => obj is Link aLink && Equals(aLink);

        public override int GetHashCode() => HashCode.Combine(Math.Min(myNode0, myNode1), Math.Max(myNode0, myNode1));
    }

    /// <summary>
    /// Sets the deflection of each face's triangulation to the maximum deviation measured on it (triangle and inner
    /// link midpoints against the surface). In place: internal; the triangulations, frozen once attached, are replaced
    /// by copies with the new deflection.
    /// </summary>
    internal static void UpdateDeflection(in Shape theShape)
    {
        var aWritable = new WritableTriangulations(); // OCCT.NET: frozen
        Span<int> aNode = stackalloc int[3];
        var anExpFace = new Explorer(theShape, ShapeType.Face);
        for (; anExpFace.More(); anExpFace.Next())
        {
            Face aFace = ShapeCasts.Face(anExpFace.Current());
            Surface? aSurf = Tool.Surface(aFace);
            if (aSurf is null)
            {
                continue;
            }

            Triangulation? anAttached = Tool.Triangulation(aFace, out Placement aLoc);
            if (anAttached is null)
            {
                continue;
            }
            Triangulation aPT = aWritable.Read(anAttached);
            if (!aPT.HasUVNodes)
            {
                continue;
            }

            // Collect all nodes of degenerative edges and skip elements
            // build upon them due to huge distortions introduced by passage
            // from UV space to 3D.
            var aDegNodes = new NCollection_Map<int>();
            var anExpEdge = new Explorer(aFace, ShapeType.Edge);
            for (; anExpEdge.More(); anExpEdge.Next())
            {
                Edge aEdge = ShapeCasts.Edge(anExpEdge.Current());
                if (Tool.Degenerated(aEdge))
                {
                    PolygonOnTriangulation? aPolygon = Tool.PolygonOnTriangulation(aEdge, anAttached, aLoc);
                    if (aPolygon is null)
                    {
                        continue;
                    }

                    for (int aNodeIt = aPolygon.Nodes().Lower(); aNodeIt <= aPolygon.Nodes().Upper(); ++aNodeIt)
                    {
                        aDegNodes.Add(aPolygon.Node(aNodeIt));
                    }
                }
            }

            var aTool = new EvalDeflection(aFace);
            var aLinks = new NCollection_Map<Link>();
            double aSqDeflection = 0.0;
            Transform3D aTrsf = aLoc.Transformation();
            var aP3d = new Point3D[3];
            var aP2d = new Point2D[3];
            for (int aTriIt = 1; aTriIt <= aPT.TriangleCount; ++aTriIt)
            {
                Triangle1 aTriangle = aPT.Triangle(aTriIt);

                aTriangle.Get(out aNode[0], out aNode[1], out aNode[2]);
                if (aDegNodes.Contains(aNode[0]) || aDegNodes.Contains(aNode[1]) || aDegNodes.Contains(aNode[2]))
                {
                    continue;
                }

                aP3d[0] = aPT.Node(aNode[0]).Transformed(aTrsf);
                aP3d[1] = aPT.Node(aNode[1]).Transformed(aTrsf);
                aP3d[2] = aPT.Node(aNode[2]).Transformed(aTrsf);

                aP2d[0] = aPT.UVNode(aNode[0]);
                aP2d[1] = aPT.UVNode(aNode[1]);
                aP2d[2] = aPT.UVNode(aNode[2]);

                // Check midpoint of triangle.
                var aMid3d_t = new Point3D((aP3d[0].XYZ() + aP3d[1].XYZ() + aP3d[2].XYZ()) / 3.0);
                var aMid2d_t = new Point2D((aP2d[0].XY() + aP2d[1].XY() + aP2d[2].XY()) / 3.0);

                aSqDeflection = Max(aSqDeflection, aTool.Eval(aMid2d_t, aMid3d_t));

                for (int i = 0; i < 3; ++i)
                {
                    int j = (i + 1) % 3;
                    var aLink = new Link(aNode[i], aNode[j]);
                    if (!aLinks.Add(aLink))
                    {
                        // Do not estimate boundary links due to high distortions at the edge.
                        Point3D aP3d1 = aP3d[i];
                        Point3D aP3d2 = aP3d[j];

                        Point2D aP2d1 = aP2d[i];
                        Point2D aP2d2 = aP2d[j];

                        var aMid3d_l = new Point3D((aP3d1.XYZ() + aP3d2.XYZ()) / 2.0);
                        var aMid2d_l = new Point2D((aP2d1.XY() + aP2d2.XY()) / 2.0);

                        aSqDeflection = Max(aSqDeflection, aTool.Eval(aMid2d_l, aMid3d_l));
                    }
                }
            }

            aWritable.Write(anAttached).Deflection(Math.Sqrt(aSqDeflection));
        }
        aWritable.Attach(theShape);
    }

    /// <summary>
    /// Fills <paramref name="LF"/> with the faces of <paramref name="Sh"/> sorted by the complexity of their surfaces:
    /// planes, cylinders, cones, spheres, tori, others, then faces without a surface.
    /// </summary>
    public static void SortFaces(in Shape Sh, NCollection_List<Shape> LF)
    {
        LF.Clear();
        NCollection_List<Shape> LTri = new(), LPlan = new(), LCyl = new(), LCon = new(), LSphere = new(), LTor = new(), LOther = new();
        var exp = new Explorer(Sh, ShapeType.Face);
        Surface? S;

        for (; exp.More(); exp.Next())
        {
            Face F = ShapeCasts.Face(exp.Current());
            S = Tool.Surface(F, out _);
            if (S is not null)
            {
                if (S.GetType() == typeof(TrimmedSurface))
                {
                    S = ((TrimmedSurface)S).BasisSurface();
                }
                var AS = new SurfaceAdaptor(S);
                switch (AS.GetType())
                {
                    case global::Kerf.Geometry.SurfaceKind.Plane:
                        LPlan.Append(F);
                        break;
                    case SurfaceKind.Cylinder:
                        LCyl.Append(F);
                        break;
                    case SurfaceKind.Cone:
                        LCon.Append(F);
                        break;
                    case SurfaceKind.Sphere:
                        LSphere.Append(F);
                        break;
                    case SurfaceKind.Torus:
                        LTor.Append(F);
                        break;
                    default:
                        LOther.Append(F);
                        break;
                }
            }
            else
            {
                LTri.Append(F);
            }
        }
        LF.Append(LPlan);
        LF.Append(LCyl);
        LF.Append(LCon);
        LF.Append(LSphere);
        LF.Append(LTor);
        LF.Append(LOther);
        LF.Append(LTri);
    }

    /// <summary>The faces of <paramref name="Sh"/> sorted as <see cref="SortFaces(in Shape, NCollection_List{Shape})"/> does.</summary>
    public static ISchematiqList<Shape> SortFaces(in Shape Sh)
    {
        var LF = new NCollection_List<Shape>();
        SortFaces(Sh, LF);
        return toList(LF);
    }

    /// <summary>
    /// Fills <paramref name="LF"/> with the faces of <paramref name="Sh"/> in the reverse order of complexity: faces
    /// without a surface, others, tori, spheres, cones, cylinders, planes.
    /// </summary>
    public static void ReverseSortFaces(in Shape Sh, NCollection_List<Shape> LF)
    {
        LF.Clear();
        NCollection_List<Shape> LTri = new(), LPlan = new(), LCyl = new(), LCon = new(), LSphere = new(), LTor = new(), LOther = new();
        var exp = new Explorer(Sh, ShapeType.Face);

        for (; exp.More(); exp.Next())
        {
            Face F = ShapeCasts.Face(exp.Current());
            Surface? S = Tool.Surface(F, out _);
            if (S is not null)
            {
                var AS = new SurfaceAdaptor(S);
                switch (AS.GetType())
                {
                    case global::Kerf.Geometry.SurfaceKind.Plane:
                        LPlan.Append(F);
                        break;
                    case SurfaceKind.Cylinder:
                        LCyl.Append(F);
                        break;
                    case SurfaceKind.Cone:
                        LCon.Append(F);
                        break;
                    case SurfaceKind.Sphere:
                        LSphere.Append(F);
                        break;
                    case SurfaceKind.Torus:
                        LTor.Append(F);
                        break;
                    default:
                        LOther.Append(F);
                        break;
                }
            }
            else
            {
                LTri.Append(F);
            }
        }
        LF.Append(LTri);
        LF.Append(LOther);
        LF.Append(LTor);
        LF.Append(LSphere);
        LF.Append(LCon);
        LF.Append(LCyl);
        LF.Append(LPlan);
    }

    /// <summary>The faces of <paramref name="Sh"/> sorted as <see cref="ReverseSortFaces(in Shape, NCollection_List{Shape})"/> does.</summary>
    public static ISchematiqList<Shape> ReverseSortFaces(in Shape Sh)
    {
        var LF = new NCollection_List<Shape>();
        ReverseSortFaces(Sh, LF);
        return toList(LF);
    }

    private static ISchematiqList<Shape> toList(NCollection_List<Shape> theList)
    {
        var anArray = new Shape[theList.Extent()];
        int i = 0;
        for (var anIt = new NCollection_List<Shape>.Iterator(theList); anIt.More(); anIt.Next())
        {
            anArray[i++] = anIt.Value();
        }
        return PackedList.Own(anArray);
    }

    /// <summary>
    /// The bounding sphere of the vertices of <paramref name="theLV"/>: its centre and radius, for a new vertex
    /// covering them all. With fewer than two vertices the outputs are left unchanged, as in OCCT.
    /// </summary>
    public static void BoundingVertex(NCollection_List<Shape> theLV, ref Point3D theNewCenter, ref double theNewTol)
    {
        int aNb;
        //
        aNb = theLV.Extent();
        if (aNb < 2)
        {
            return;
        }
        //
        else if (aNb == 2)
        {
            int m, n;
            double dR, aD, aEps;
            Span<double> aR = stackalloc double[2];
            var aV = new Vertex[2];
            var aP = new Point3D[2];
            //
            aEps = RealEpsilon();
            for (m = 0; m < aNb; ++m)
            {
                aV[m] = (m == 0) ? ShapeCasts.Vertex(theLV.First()) : ShapeCasts.Vertex(theLV.Last());
                aP[m] = Tool.Pnt(aV[m]);
                aR[m] = Tool.Tolerance(aV[m]);
            }
            //
            m = 0; // max R
            n = 1; // min R
            if (aR[0] < aR[1])
            {
                m = 1;
                n = 0;
            }
            //
            dR = aR[m] - aR[n]; // dR >= 0.
            var aVD = new Vector3D(aP[m], aP[n]);
            aD = aVD.Magnitude();
            //
            if (aD <= dR || aD < aEps)
            {
                theNewCenter = aP[m];
                theNewTol = aR[m];
            }
            else
            {
                double aRr;
                Xyz aXYZr;
                var aPr = new Point3D();
                //
                aRr = 0.5 * (aR[m] + aR[n] + aD);
                aXYZr = 0.5 * (aP[m].XYZ() + aP[n].XYZ() - aVD.XYZ() * (dR / aD));
                aPr.SetXYZ(aXYZr);
                //
                theNewCenter = aPr;
                theNewTol = aRr;
            }
            return;
        }
        //
        else
        { // if (aNb>2)
            // compute the point
            //
            // issue 0027540 - sum of doubles may depend on the order
            // of addition, thus sort the coordinates for stable result
            int i;
            var aPoints = new Point3D[aNb];
            NCollection_List<Shape>.Iterator aIt = new(theLV);
            for (i = 0; aIt.More(); aIt.Next(), ++i)
            {
                Vertex aVi = ShapeCasts.Vertex(aIt.Value());
                Point3D aPi = Tool.Pnt(aVi);
                aPoints[i] = aPi;
            }
            //
            Array.Sort(aPoints, comparePoints);
            //
            var aXYZ = new Xyz(0.0, 0.0, 0.0);
            for (i = 0; i < aNb; ++i)
            {
                aXYZ += aPoints[i].XYZ();
            }
            aXYZ.Divide((double)aNb);
            //
            var aP = new Point3D(aXYZ);
            //
            // compute the tolerance for the new vertex
            double aTi, aDi, aDmax;
            //
            aDmax = -1.0;
            aIt.Initialize(theLV);
            for (; aIt.More(); aIt.Next())
            {
                Vertex aVi = ShapeCasts.Vertex(aIt.Value());
                Point3D aPi = Tool.Pnt(aVi);
                aTi = Tool.Tolerance(aVi);
                aDi = aP.SquareDistance(aPi);
                aDi = Math.Sqrt(aDi);
                aDi = aDi + aTi;
                if (aDi > aDmax)
                {
                    aDmax = aDi;
                }
            }
            //
            theNewCenter = aP;
            theNewTol = aDmax;
        }
    }

    // BRepLib_ComparePoints: lexicographic order of the coordinates (a strict weak order, as std::sort needs).
    private static int comparePoints(Point3D theP1, Point3D theP2)
    {
        for (int i = 1; i <= 3; ++i)
        {
            if (theP1.Coord(i) < theP2.Coord(i))
            {
                return -1;
            }
            else if (theP1.Coord(i) > theP2.Coord(i))
            {
                return 1;
            }
        }
        return 0;
    }

    /// <summary>
    /// The face <paramref name="theF"/> enlarged by <paramref name="theExtVal"/> in the chosen directions, frozen. An
    /// analytical surface is extended within its period and natural bounds; a bounded one by
    /// <c>GeomLib.ExtendSurfByLength</c>; another face is returned as it is.
    /// </summary>
    public static void ExtendFace(in Face theF,
                                  double theExtVal,
                                  bool theExtUMin,
                                  bool theExtUMax,
                                  bool theExtVMin,
                                  bool theExtVMax,
                                  out Face theFExtended)
    {
        InternalExtendFace(theF, theExtVal, theExtUMin, theExtUMax, theExtVMin, theExtVMax, out theFExtended);
        theFExtended.TShape()?.FreezeGraph();
    }

    /// <summary><see cref="ExtendFace"/> without freezing the new face, for ported code that goes on building on it.</summary>
    internal static void InternalExtendFace(in Face theF,
                                            double theExtVal,
                                            bool theExtUMin,
                                            bool theExtUMax,
                                            bool theExtVMin,
                                            bool theExtVMax,
                                            out Face theFExtended)
    {
        // Get face bounds
        var aBAS = new FaceSurfaceAdaptor(theF);
        double aFUMin = aBAS.FirstUParameter(), aFUMax = aBAS.LastUParameter(),
               aFVMin = aBAS.FirstVParameter(), aFVMax = aBAS.LastVParameter();
        double aTol = Tool.Tolerance(theF);

        // Surface to build the face
        Surface aS;

        SurfaceKind aType = aBAS.GetType();
        // treat analytical surfaces first
        if (aType == global::Kerf.Geometry.SurfaceKind.Plane || aType == SurfaceKind.Sphere || aType == SurfaceKind.Cylinder
            || aType == SurfaceKind.Torus || aType == SurfaceKind.Cone)
        {
            // Get basis transformed basis surface
            Surface aSurf = aBAS.GeomSurfaceTransformed()!;

            // Get bounds of the basis surface
            aSurf.Bounds(out double aSUMin, out double aSUMax, out double aSVMin, out double aSVMax);

            bool isUPeriodic = aBAS.IsUPeriodic;
            double anUPeriod = isUPeriodic ? aBAS.UPeriod() : 0.0;
            if (isUPeriodic)
            {
                // Adjust face bounds to first period
                double aDelta = aFUMax - aFUMin;
                aFUMin = Max(aSUMin, aFUMin + anUPeriod * Math.Ceiling((aSUMin - aFUMin) / anUPeriod));
                aFUMax = aFUMin + aDelta;
            }

            bool isVPeriodic = aBAS.IsVPeriodic;
            double aVPeriod = isVPeriodic ? aBAS.VPeriod() : 0.0;
            if (isVPeriodic)
            {
                // Adjust face bounds to first period
                double aDelta = aFVMax - aFVMin;
                aFVMin = Max(aSVMin, aFVMin + aVPeriod * Math.Ceiling((aSVMin - aFVMin) / aVPeriod));
                aFVMax = aFVMin + aDelta;
            }

            // Enlarge the face
            double anURes = 0.0, aVRes = 0.0;
            if (theExtUMin || theExtUMax)
            {
                anURes = aBAS.UResolution(theExtVal);
            }
            if (theExtVMin || theExtVMax)
            {
                aVRes = aBAS.VResolution(theExtVal);
            }

            if (theExtUMin)
            {
                aFUMin = Max(aSUMin, aFUMin - anURes);
            }
            if (theExtUMax)
            {
                aFUMax = Min(isUPeriodic ? aFUMin + anUPeriod : aSUMax, aFUMax + anURes);
            }
            if (theExtVMin)
            {
                aFVMin = Max(aSVMin, aFVMin - aVRes);
            }
            if (theExtVMax)
            {
                aFVMax = Min(isVPeriodic ? aFVMin + aVPeriod : aSVMax, aFVMax + aVRes);
            }

            // Check if the periodic surface should become closed.
            // In this case, use the basis surface with basis bounds.
            double anEps = Kerf.Engine.Foundation.Precision.PConfusion();
            if (isUPeriodic && Math.Abs(aFUMax - aFUMin - anUPeriod) < anEps)
            {
                aFUMin = aSUMin;
                aFUMax = aSUMax;
            }
            if (isVPeriodic && Math.Abs(aFVMax - aFVMin - aVPeriod) < anEps)
            {
                aFVMin = aSVMin;
                aFVMax = aSVMax;
            }

            aS = aSurf;
        }
        else
        {
            // General case

            // OCCT takes the face's surface without its location here, so the extended face of a located face
            // loses the location (occt-issues.md). OCCT also extends that surface handle in place below, changing the
            // input face and every face sharing the surface (occt-issues.md); the port's GeomLib.ExtendSurfByLength
            // replaces aSB by an extended copy, so the input is unchanged and the extended face is OCCT's.
            if (Tool.Surface(theF) is not BoundedSurface aSB)
            {
                theFExtended = theF;
                return;
            }

            // Get surfaces bounds
            aSB.Bounds(out double aSUMin, out double aSUMax, out double aSVMin, out double aSVMax);

            bool isUClosed = aSB.IsUClosed;
            bool isVClosed = aSB.IsVClosed;

            // Check if the extension in necessary directions is done
            bool isExtUMin = false, isExtUMax = false, isExtVMin = false, isExtVMax = false;

            // UMin
            if (theExtUMin && !isUClosed && !Kerf.Engine.Foundation.Precision.IsInfinite(aSUMin))
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSB, theExtVal, 1, true, false);
                isExtUMin = true;
            }
            // UMax
            if (theExtUMax && !isUClosed && !Kerf.Engine.Foundation.Precision.IsInfinite(aSUMax))
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSB, theExtVal, 1, true, true);
                isExtUMax = true;
            }
            // VMin
            if (theExtVMin && !isVClosed && !Kerf.Engine.Foundation.Precision.IsInfinite(aSVMax)) // OCCT tests aSVMax where aSVMin is meant (occt-issues.md)
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSB, theExtVal, 1, false, false);
                isExtVMin = true;
            }
            // VMax
            if (theExtVMax && !isVClosed && !Kerf.Engine.Foundation.Precision.IsInfinite(aSVMax))
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSB, theExtVal, 1, false, true);
                isExtVMax = true;
            }

            aS = aSB;

            // Get new bounds
            aS.Bounds(out aSUMin, out aSUMax, out aSVMin, out aSVMax);
            if (isExtUMin)
            {
                aFUMin = aSUMin;
            }
            if (isExtUMax)
            {
                aFUMax = aSUMax;
            }
            if (isExtVMin)
            {
                aFVMin = aSVMin;
            }
            if (isExtVMax)
            {
                aFVMax = aSVMax;
            }
        }

        var aMF = new BRepLib_MakeFace(aS, aFUMin, aFUMax, aFVMin, aFVMax, aTol);
        theFExtended = ShapeCasts.Face(aMF.InternalShape());
        if (theF.Orientation() == Reversed)
        {
            theFExtended.Reverse();
        }
    }
}
