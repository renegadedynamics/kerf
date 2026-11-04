// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// General utilities for BRep: the default precision and plane of the makers, the computation of missing 3D curves
/// and pcurves, SameParameter and SameRange, tolerances, regularity, orientation of closed solids.
/// </summary>
internal static partial class BrepLibrary
{
    // OCCT's file statics, process-wide in OCCT: settings of the operation context (unit 10I), whose setters are
    // internal (Draw-lite and tests), so callers see the defaults unless they set them in their flow
    internal static readonly ContextKey<double> PrecisionSetting =
        new("BRepLib.Precision", Kerf.Engine.Foundation.Precision.Confusion());

    internal static readonly ContextKey<PlaneSurface> PlaneSetting = new("BRepLib.Plane", MakeDefaultPlane());

    private static PlaneSurface MakeDefaultPlane()
    {
        var plane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        plane.Freeze();
        return plane;
    }

    /// <summary>Sets the default precision for the rest of the current flow of execution (internal).</summary>
    internal static void Precision(double p) => OperationContext.Set(PrecisionSetting, p);

    /// <summary>Returns the default precision of the makers.</summary>
    public static double Precision() => PrecisionSetting.Value;

    /// <summary>Sets the current plane for the rest of the current flow of execution (internal).</summary>
    internal static void Plane(PlaneSurface p)
    {
        p.Freeze();
        OperationContext.Set(PlaneSetting, p);
    }

    /// <summary>Returns the current plane of the 2D makers (frozen).</summary>
    public static PlaneSurface Plane() => PlaneSetting.Value;

    //=================================================================================================

    /// <summary>
    /// True if all the curve representations of the edge have the same range within <paramref name="confusion"/>,
    /// ignoring the edge's SameRange flag.
    /// </summary>
    public static bool CheckSameRange(in Edge anEdge, double confusion = 1.0e-12)
    {
        double tolerance = confusion;
        bool isSameRange = true, first_time_in = true;

        // OCCT.NET: frozen. OCCT iterates ChangeCurves() to read them; Curves() is the same list.
        var an_Iterator = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)anEdge.TShape()!).Curves());

        double first, last;
        double current_first = 0.0, current_last = 0.0;
        GCurve? geometric_representation_ptr;

        while (isSameRange && an_Iterator.More())
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
                    isSameRange = (Math.Abs(current_first - first) <= tolerance)
                                  && (Math.Abs(current_last - last) <= tolerance);
                }
            }
            an_Iterator.Next();
        }
        return isSameRange;
    }

    //=================================================================================================

    /// <summary>
    /// Gives all the curve representations of the edge the same range: the 3D curve's if there is one, else the
    /// first representation's; then sets SameRange. Ignores the SameRange flag. Writes into the edge.
    /// </summary>
    internal static void SameRange(in Edge anEdge, double tolerance = 1.0e-5)
    {
        var an_Iterator = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)anEdge.TShape()!).ChangeCurves());

        Curve2D? curve2dPtr = null, curve2dPtr2 = null, newCurve2dPtr = null, newCurve2dPtr2 = null;

        bool first_time_in = true, has_curve, has_closed_curve;
        GCurve? geometric_representation_ptr;
        double first, current_first, last, current_last;

        Curve3D? c = Tool.Curve(anEdge, out _, out current_first, out current_last);
        if (c is not null)
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
                    curve2dPtr = geometric_representation_ptr.PCurve();
                    has_curve = true;
                }
                if (geometric_representation_ptr.IsCurveOnClosedSurface)
                {
                    curve2dPtr2 = geometric_representation_ptr.PCurve2();
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
                            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(tolerance,
                                              curve2dPtr,
                                              geometric_representation_ptr.First(),
                                              geometric_representation_ptr.Last(),
                                              current_first,
                                              current_last,
                                              ref newCurve2dPtr);
                            geometric_representation_ptr.PCurve(newCurve2dPtr);
                        }
                        if (has_closed_curve)
                        {
                            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(tolerance,
                                              curve2dPtr2,
                                              geometric_representation_ptr.First(),
                                              geometric_representation_ptr.Last(),
                                              current_first,
                                              current_last,
                                              ref newCurve2dPtr2);
                            geometric_representation_ptr.PCurve2(newCurve2dPtr2);
                        }
                    }
                }
            }
            an_Iterator.Next();
        }
        var b = new BrepBuilder();
        b.Range(anEdge, current_first, current_last);

        b.SameRange(anEdge, true);
    }

    //=======================================================================
    // function : EvaluateMaxSegment
    // purpose  : return MaxSegment to pass in approximation, if MaxSegment==0 provided
    //=======================================================================

    private static int EvaluateMaxSegment(int aMaxSegment, CurveOnSurface aCurveOnSurface)
    {
        if (aMaxSegment != 0)
        {
            return aMaxSegment;
        }

        SurfaceAdaptorBase surf = aCurveOnSurface.GetSurface()!;
        Curve2DAdaptorBase curv2d = aCurveOnSurface.GetCurve()!;

        double nbSKnots = 0, nbC2dKnots = 0;

        if (surf.GetType() == SurfaceKind.BSplineSurface)
        {
            global::Kerf.Geometry.BSplineSurface bSpline = surf.BSpline();
            nbSKnots = Max(bSpline.UKnotCount, bSpline.VKnotCount);
        }
        if (curv2d.GetType() == BSplineCurve)
        {
            nbC2dKnots = curv2d.KnotCount;
        }
        int aReturn = (int)(30 + Max(nbSKnots, nbC2dKnots));
        return aReturn;
    }

    //=================================================================================================

    /// <summary>
    /// Computes the 3D curve of the edge if it has none: from its pcurve on a plane, else by approximation of its
    /// first curve on a surface. True if the curve was computed or existed. Writes into the edge.
    /// </summary>
    /// <remarks><paramref name="maxSegment"/> 0 means 30 plus the number of knots of the surface or pcurve.</remarks>
    internal static bool BuildCurve3d(in Edge anEdge,
                                      double Tolerance = 1.0e-5,
                                      Continuity continuity = C1,
                                      int maxDegree = 14,
                                      int maxSegment = 0)
    {
        int ii, jj;

        Placement localLoc = new(), LC;
        var L = new Placement[] { new(), new() };
        double f, l, fc, lc, tolerance, max_deviation;
        var first = new double[2];
        var last = new double[2];
        Curve2D? curve2dPtr = null;
        var curve2dArray = new Curve2D?[2];
        Surface? surfacePtr = null;
        var surfaceArray = new Surface?[2];

        bool not_done;
        // if the edge has a 3d curve returns true

        Curve3D? c = Tool.Curve(anEdge, out localLoc, out f, out l);
        if (c is not null)
        {
            return true;
        }
        //
        // this should not exists but UpdateEdge makes funny things
        // if the edge is not same range
        //
        if (!CheckSameRange(anEdge, Kerf.Engine.Foundation.Precision.Confusion()))
        {
            SameRange(anEdge, Tolerance);
        }

        // search a curve on a plane
        Surface? s = null;
        Curve2D? pc = null;
        int i = 0;
        PlaneSurface? p = null;
        not_done = true;

        while (not_done)
        {
            i++;
            Tool.CurveOnSurface(anEdge, ref pc, ref s, ref localLoc, ref f, ref l, i);
            if (s is not TrimmedSurface rt)
            {
                p = s as PlaneSurface;
            }
            else
            {
                p = rt.BasisSurface as PlaneSurface;
            }
            not_done = s is not null && p is null;
        }
        if (p is not null)
        {
            // compute the 3d curve
            Frame3D axes = p.Position.Frame;
            Curve3D? c3d = global::Kerf.Engine.Geometry.Library.GeomLib.To3d(axes, pc!);
            if (c3d is null)
            {
                return false;
            }
            // update the edge
            double First, Last;

            var b = new BrepBuilder();
            b.UpdateEdge(anEdge, c3d, localLoc, 0.0e0);
            LC = new Placement();
            Tool.Range(anEdge, s, LC, out First, out Last);
            b.Range(anEdge, First, Last); // Do not forget 3D range.(PRO6412)
        }
        else
        {
            //
            // compute the 3d curve using existing surface
            //
            fc = f;
            lc = l;
            if (!Tool.Degenerated(anEdge))
            {
                jj = 0;
                // BRep_Tool::CurveOnSurface expects Index >= 1, so start from 1.
                for (ii = 1; ii < 3; ii++)
                {
                    Tool.CurveOnSurface(anEdge, ref curve2dPtr, ref surfacePtr, ref localLoc, ref fc, ref lc, ii);

                    if (curve2dPtr is not null && jj < 2)
                    {
                        curve2dArray[jj] = curve2dPtr;
                        surfaceArray[jj] = surfacePtr;
                        L[jj] = localLoc;
                        first[jj] = fc;
                        last[jj] = lc;
                        jj += 1;
                    }
                }
                f = first[0];
                l = last[0];
                curve2dPtr = curve2dArray[0];
                surfacePtr = surfaceArray[0];

                // OCCT copies a local adaptor into a handle; the port makes the handle directly (decision 331).
                var anAdaptor3dCurve2dPtr = new CurveAdaptor2D(curve2dPtr!, f, l);
                var anAdaptor3dSurfacePtr = new SurfaceAdaptor(surfacePtr!);
                var curveOnSurface = new CurveOnSurface(anAdaptor3dCurve2dPtr, anAdaptor3dSurfacePtr);

                Curve3D? newCurvePtr = null;

                global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(Tolerance,
                                     curveOnSurface,
                                     f,
                                     l,
                                     ref newCurvePtr,
                                     out max_deviation,
                                     out _,
                                     continuity,
                                     maxDegree,
                                     EvaluateMaxSegment(maxSegment, curveOnSurface));
                var b = new BrepBuilder();
                tolerance = Tool.Tolerance(anEdge);
                // Patch
                // max_deviation = std::max(tolerance, max_deviation) ;
                max_deviation = Max(tolerance, Tolerance);
                if (newCurvePtr is null)
                {
                    return false;
                }
                b.UpdateEdge(anEdge, newCurvePtr, L[0], max_deviation);
                if (jj == 1)
                {
                    //
                    // if there is only one curve on surface attached to the edge
                    // than it can be qualified sameparameter
                    //
                    b.SameParameter(anEdge, true);
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

    /// <summary>Computes the 3D curves of the edges of <paramref name="s"/> with a tolerance of 1e-5.</summary>
    internal static bool BuildCurves3d(in Shape s)

    {
        return BuildCurves3d(s, 1.0e-5);
    }

    //=================================================================================================

    /// <summary>
    /// Computes the missing 3D curves of all the edges of <paramref name="s"/>; false if one of the computations
    /// failed. Writes into the edges.
    /// </summary>
    internal static bool BuildCurves3d(in Shape s,
                                       double tolerance,
                                       Continuity continuity = C1,
                                       int maxDegree = 14,
                                       int maxSegment = 0)
    {
        bool boolean_value, ok = true;
        var a_counter = new HashSet<Shape>(ShapeHasher.Instance);
        var ex = new Explorer(s, ShapeType.Edge);

        while (ex.More())
        {
            if (a_counter.Add(ex.Current()))
            {
                boolean_value = BuildCurve3d(ShapeCasts.Edge(ex.Current()), tolerance, continuity, maxDegree, maxSegment);
                ok = ok && boolean_value;
            }
            ex.Next();
        }
        return ok;
    }

    //=================================================================================================

    /// <summary>
    /// If the edge's tolerance is below <paramref name="maxToleranceToCheck"/>, sets it to the radius of the pipe
    /// around its reference curve holding all its curve representations. True if the tolerance was set. Writes into
    /// the edge.
    /// </summary>
    internal static bool UpdateEdgeTol(in Edge anEdge,
                                       double minToleranceRequested,
                                       double maxToleranceToCheck)
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

        if (Tool.Degenerated(anEdge))
        {
            return false;
        }
        coded_edge_tolerance = Tool.Tolerance(anEdge);
        if (coded_edge_tolerance > maxToleranceToCheck)
        {
            return false;
        }

        BrepTEdge te = (BrepTEdge)anEdge.TShape()!;
        NCollection_List<CurveRepresentation> list_curve_rep = te.ChangeCurves();
        NCollection_List<CurveRepresentation>.Iterator an_iterator = new(list_curve_rep),
                                                            second_iterator = new(list_curve_rep);
        Curve2D? curve2d_ptr = null;
        Surface? surface_ptr = null;
        Placement local_location;
        var a_sampler = new QuasiUniformDeflection();
        var geom_reference_curve = new CurveAdaptor();
        var curve_on_surface_reference = new CurveOnSurface();
        Curve3D? c = Tool.Curve(anEdge, out local_location, out current_first, out current_last);
        curve_on_surface_index = -1;
        if (c is not null)
        {
            if (!local_location.IsIdentity)
            {
                c = (Curve3D)c.Transformed(local_location.Transformation);
            }
            geom_reference_curve.Load(c);
            geom_reference_curve_flag = 1;
            a_sampler.Initialize(geom_reference_curve,
                                 minToleranceRequested * factor,
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
                        surface_ptr = (Surface)geometric_representation_ptr.Surface()!.Transformed(local_location.Transformation);
                    }
                    else
                    {
                        surface_ptr = geometric_representation_ptr.Surface();
                    }
                    not_done = false;
                }
                curve_on_surface_index += 1; // OCCT (see above)
            }
            var anAdaptor3dCurve2dPtr = new CurveAdaptor2D(curve2d_ptr!);
            var anAdaptor3dSurfacePtr = new SurfaceAdaptor(surface_ptr!);
            curve_on_surface_reference.Load(anAdaptor3dCurve2dPtr, anAdaptor3dSurfacePtr);
            a_sampler.Initialize(curve_on_surface_reference,
                                 minToleranceRequested * factor,
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
                        surface_ptr = (Surface)geometric_representation_ptr.Surface()!.Transformed(local_location.Transformation);
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
                    if (Tool.SameParameter(anEdge))
                    {

                        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxParametricDistance(a_curve_on_surface,
                                                          geom_reference_curve,
                                                          minToleranceRequested,
                                                          parameters_ptr.Array1(),
                                                          out max_distance);
                    }
                    else if (geom_reference_curve_flag != 0)
                    {
                        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxDistanceAlongParameter(a_curve_on_surface,
                                                              geom_reference_curve,
                                                              minToleranceRequested,
                                                              parameters_ptr.Array1(),
                                                              out max_distance);
                    }
                    else
                    {

                        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxDistanceAlongParameter(a_curve_on_surface,
                                                              curve_on_surface_reference,
                                                              minToleranceRequested,
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

        te.Tolerance(edge_tolerance);
        return true;
    }

    //=================================================================================================

    /// <summary>
    /// <see cref="UpdateEdgeTol"/> on every edge of the shape; true if at least one edge was updated. Writes into
    /// the edges.
    /// </summary>
    internal static bool UpdateEdgeTolerance(in Shape s,
                                             double minToleranceRequested,
                                             double maxToleranceToCheck)
    {
        var ex = new Explorer(s, ShapeType.Edge);
        var a_counter = new HashSet<Shape>(ShapeHasher.Instance);

        bool return_status = false, local_flag;

        while (ex.More())
        {
            if (a_counter.Add(ex.Current()))
            {
                local_flag = UpdateEdgeTol(ShapeCasts.Edge(ex.Current()),
                                           minToleranceRequested,
                                           maxToleranceToCheck);
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

    private static void GetEdgeTol(in Edge edge, in Face face, ref double edTol)
    {
        Surface? s = Tool.Surface(face, out Placement L);
        Placement l = L.Predivided(edge.Placement);

        BrepTEdge te = (BrepTEdge)edge.TShape()!;
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(s, l))
            {
                return;
            }
            itcr.Next();
        }

        PlaneSurface? gp;
        TrimmedSurface? grts;
        grts = s as TrimmedSurface;
        if (grts is not null)
        {
            gp = grts.BasisSurface as PlaneSurface;
        }
        else
        {
            gp = s as PlaneSurface;
        }

        var hc = new CurveAdaptor();
        var hs = new SurfaceAdaptor();

        Placement lc;
        double first, last;
        hc.Load(Tool.Curve(edge, out lc, out first, out last)!);
        lc = L.Predivided(lc);

        if (!lc.IsIdentity)
        {
            gp = (PlaneSurface)gp!.Transformed(lc.Transformation);
        }
        SurfaceAdaptor gas = hs;
        gas.Load(gp!);

        var proj = new ProjectedCurve(hs, hc);
        Curve2D pc = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(proj);

        Plane3D pln = gas.Plane();
        double d2 = 0.0;
        int nn = 23;
        double unsurnn = 1.0 / nn;
        for (int i = 0; i <= nn; i++)
        {
            double t = unsurnn * i;
            double u = first * (1.0 - t) + last * t;
            Point3D pc3d = hc.Value(u);
            Point2D p2d = pc.Value(u);
            Point3D pcons = global::Kerf.Engine.Geometry.ElSLib.Value(p2d.X, p2d.Y, pln);
            double eps = Max(pc3d.XYZ().SquareModulus(), pcons.XYZ().SquareModulus());
            eps = Epsilon(eps);
            double temp = pc3d.SquareDistanceTo(pcons);
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
        edTol = d2;
    }

    //=======================================================================
    // function : UpdTolMap
    // purpose  : Update map ShToTol (shape to tolerance)
    //=======================================================================
    private static void UpdTolMap(in Shape sh,
                                  double newTol,
                                  Dictionary<Shape, double> shToTol)
    {
        ShapeType st = sh.Type;
        double shTol;
        if (st == ShapeType.Vertex)
        {
            shTol = Tool.Tolerance(ShapeCasts.Vertex(sh));
        }
        else if (st == ShapeType.Edge)
        {
            shTol = Tool.Tolerance(ShapeCasts.Edge(sh));
        }
        else
        {
            return;
        }
        //
        if (newTol > shTol)
        {
            if (!shToTol.TryGetValue(sh, out double oldtol))
            {
                shToTol[sh] = newTol;
            }
            else
            {
                shToTol.GetValueRef(sh) = Max(oldtol, newTol);
            }
        }
    }

    //=======================================================================
    // function : UpdShTol
    // purpose  : Update vertices/edges/faces according to ShToTol map (create copies of necessary)
    //=======================================================================
    private static void UpdShTol(Dictionary<Shape, double> shToTol,
                                 bool isMutableInput,
                                 ReShape reshaper,
                                 bool vForceUpdate)
    {
        var b = new BrepBuilder();
        foreach (KeyValuePair<Shape, double> shToTolit in shToTol)
        {
            Shape sh = shToTolit.Key;
            double tol = shToTolit.Value;
            //
            Shape nsh;
            // The reshaper's internal form: its replacements are this algorithm's own, still unfrozen (design 4.2).
            Shape vsh = reshaper.InternalValue(sh);
            bool useOldSh = isMutableInput || reshaper.IsNewShape(sh) || !vsh.IsSame(sh);
            if (useOldSh)
            {
                nsh = vsh;
            }
            else
            {
                nsh = sh.EmptyCopied();
                // add subshapes from the original shape
                var sit = new Iterator(sh);
                for (; sit.More(); sit.Next())
                {
                    b.Add(nsh, sit.Value());
                }
                //
                nsh.Free(sh.Free());
                nsh.Checked(sh.Checked());
                nsh.Orientable(sh.IsOrientable);
                nsh.Closed(sh.IsClosed);
                nsh.Infinite(sh.IsInfinite);
                nsh.Convex(sh.IsConvex);
                //
            }
            //
            switch (sh.Type)
            {
                case ShapeType.Face:
                    {
                        b.UpdateFace(ShapeCasts.Face(nsh), tol);
                        break;
                    }
                case ShapeType.Edge:
                    {
                        b.UpdateEdge(ShapeCasts.Edge(nsh), tol);
                        break;
                    }
                case ShapeType.Vertex:
                    {
                        BrepTVertex tv = (BrepTVertex)nsh.TShape()!;
                        if (vForceUpdate)
                        {
                            tv.Tolerance(tol);
                        }
                        else
                        {
                            tv.UpdateTolerance(tol);
                        }
                        tv.Modified(true);
                        break;
                    }
                default:
                    break;
            }
            //
            if (!useOldSh)
            {
                reshaper.Replace(sh, nsh);
            }
        }
    }

    //=================================================================================================

    private static void InternalSameParameter(in Shape sh,
                                              ReShape reshaper,
                                              double tol,
                                              bool isForced,
                                              bool isMutableInput)
    {
        var ex = new Explorer(sh, ShapeType.Edge);
        var done = new HashSet<Shape>(ShapeHasher.Instance);
        var b = new BrepBuilder();
        var shToTol = new Dictionary<Shape, double>(ShapeHasher.Instance);

        while (ex.More())
        {
            Edge ce = ShapeCasts.Edge(ex.Current());
            if (done.Add(ce))
            {
                Edge aNE = ShapeCasts.Edge(reshaper.InternalValue(ce));
                bool useOldEdge = isMutableInput || reshaper.IsNewShape(ce) || !aNE.IsSame(ce);
                if (isForced && (Tool.SameRange(ce) || Tool.SameParameter(ce)))
                {
                    if (!useOldEdge)
                    {
                        aNE = ShapeCasts.Edge(ce.EmptyCopied());
                        var sit = new Iterator(ce);
                        for (; sit.More(); sit.Next())
                        {
                            b.Add(aNE, sit.Value());
                        }
                        reshaper.Replace(ce, aNE);
                        useOldEdge = true;
                    }
                    b.SameRange(aNE, false);
                    b.SameParameter(aNE, false);
                }
                double newTol = -1;
                Edge resEdge = SameParameter(aNE, tol, ref newTol, useOldEdge);
                if (!useOldEdge && !resEdge.IsNull)
                {
                    // NE have been empty-copied
                    reshaper.Replace(aNE, resEdge);
                }
                if (newTol > 0)
                {
                    Exploration.Vertices(ce, out Vertex v1, out Vertex v2);
                    if (!v1.IsNull)
                    {
                        UpdTolMap(v1, newTol, shToTol);
                    }
                    if (!v2.IsNull)
                    {
                        UpdTolMap(v2, newTol, shToTol);
                    }
                }
            }
            ex.Next();
        }

        done.Clear();
        var bs = new FaceSurfaceAdaptor();
        for (ex.Init(sh, ShapeType.Face); ex.More(); ex.Next())
        {
            Face curface = ShapeCasts.Face(ex.Current());
            if (!done.Add(curface))
            {
                continue;
            }
            bs.Initialize(curface);
            if (bs.GetType() != global::Kerf.Geometry.SurfaceKind.Plane)
            {
                continue;
            }
            var ex2 = new Explorer();
            for (ex2.Init(curface, ShapeType.Edge); ex2.More(); ex2.Next())
            {
                Edge e = ShapeCasts.Edge(ex2.Current());
                if (Tool.Degenerated(e))
                {
                    continue;
                }

                Shape ne = reshaper.InternalValue(e);
                double newEtol = -1;
                GetEdgeTol(ShapeCasts.Edge(ne), curface, ref newEtol);
                if (newEtol >= 0)
                { // not equal to -1
                    UpdTolMap(e, newEtol, shToTol);
                }
            }
        }

        //
        UpdShTol(shToTol, isMutableInput, reshaper, false);

        InternalUpdateTolerances(sh, false, isMutableInput, reshaper);
    }

    //================================================================
    // function : SameParameter
    // WARNING  : New spec DUB LBO 9/9/97.
    //  Recode in the edge the best tolerance found,
    //  for vertex extremities it is required to find something else
    //================================================================

    /// <summary>
    /// Computes new pcurves for all the edges of <paramref name="s"/> so that they have the same parameter as the
    /// 3D curve (skipping edges flagged SameParameter unless <paramref name="forced"/>), then harmonizes the
    /// tolerances. Writes into the shape.
    /// </summary>
    internal static void SameParameter(in Shape s, double tolerance = 1.0e-5, bool forced = false)
    {
        var reshaper = new ReShape();
        InternalSameParameter(s, reshaper, tolerance, forced, true);
    }

    //=================================================================================================

    /// <summary>
    /// <see cref="SameParameter(in Shape, double, bool)"/> without changing <paramref name="s"/>: the
    /// changed edges, vertices and faces are copies recorded as replacements in <paramref name="reshaper"/>, whose
    /// <see cref="ReShape.Apply"/> gives the result.
    /// </summary>
    public static void SameParameter(in Shape s,
                                     ReShape reshaper,
                                     double tolerance = 1.0e-5,
                                     bool forced = false)
    {
        InternalSameParameter(s, reshaper, tolerance, forced, false);
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
        var projector = new LocateExtPC();
        projector.Initialize(gac, f, l, tol);
        double u, v;
        Point3D p;
        tolbail = tol;
        for (int i = 1; i <= 5; i++)
        {
            double t = i / 6.0;
            t = (1.0 - t) * f + t * l;
            pc.Value(t).Coord(out u, out v);
            p = s.Value(u, v);
            projector.Perform(p, t);
            if (projector.IsDone)
            {
                double dist2 = projector.SquareDistance();
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
        double dSdu = 1.0 / surf.UResolution(1.0), dSdv = 1.0 / surf.VResolution(1.0);
        double d2 = 0.0;
        double first = c3d.FirstParameter();
        double last = c3d.LastParameter();
        double dapp = -1.0;
        for (int i = 0; i <= nbp; ++i)
        {
            double t = IntToReal(i) / IntToReal(nbp);
            double u = first * (1.0 - t) + last * t;
            Point3D pc3d = c3d.Value(u);
            Point2D puv = c2d.Value(u);
            if (!isUPeriodic)
            {
                if (puv.X < uf - du)
                {
                    dapp = Max(dapp, dSdu * (uf - puv.X));
                    continue;
                }
                else if (puv.X > ul + du)
                {
                    dapp = Max(dapp, dSdu * (puv.X - ul));
                    continue;
                }
            }
            if (!isVPeriodic)
            {
                if (puv.Y < vf - dv)
                {
                    dapp = Max(dapp, dSdv * (vf - puv.Y));
                    continue;
                }
                else if (puv.Y > vl + dv)
                {
                    dapp = Max(dapp, dSdv * (puv.Y - vl));
                    continue;
                }
            }
            Point3D pcons = surf.Value(puv.X, puv.Y);
            if (Kerf.Engine.Foundation.Precision.IsInfinite(pcons.X) || Kerf.Engine.Foundation.Precision.IsInfinite(pcons.Y)
                || Kerf.Engine.Foundation.Precision.IsInfinite(pcons.Z))
            {
                d2 = Kerf.Engine.Foundation.Precision.Infinite();
                break;
            }
            double temp = pc3d.SquareDistanceTo(pcons);

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
        int n1 = 0;
        int n2 = 0;
        int n3 = 0;

        for (int i = 1; i <= nbp + 10; ++i)
        {
            if (dist[i] > 0)
            {
                if (dist[i] < 1.0)
                {
                    ++n1;
                }
                else
                {
                    ++n2;
                }
            }
        }

        if (n1 > n2 && n2 != 0)
        {
            n3 = 100 * n2 / (n1 + n2);
        }
        if (n3 < 10 && n3 != 0)
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
    // It is read through Curves() (OCCT.NET: frozen): the ReShape form calls this on the caller's edge. Kerf (unit
    // 10H4): when the representations are to be changed (forWrite) and a working overlay takes writes into the edge,
    // they are its working copy's, through ChangeCurves(), as OCCT's are the edge's own.
    private static void GetCurve3d(in Edge edge,
                                   ref Curve3D? c3d,
                                   ref double f3d,
                                   ref double l3d,
                                   ref Placement loc3d,
                                   out NCollection_List<CurveRepresentation> cList,
                                   bool forWrite = false)
    {
        BrepTEdge te = (BrepTEdge)edge.TShape()!;
        cList = forWrite && te.IsFrozen && !WorkingOverlay.IsReadOnly(te) ? te.ChangeCurves() : te.Curves();
        var it = new NCollection_List<CurveRepresentation>.Iterator(cList);
        bool notDone = true;
        while (notDone && it.More())
        {
            if (it.Value() is GCurve gCurve && gCurve.IsCurve3D)
            {
                c3d = gCurve.Curve3D();
                f3d = gCurve.First();
                l3d = gCurve.Last();
                loc3d = gCurve.Location();
                notDone = false;
            }
            it.Next();
        }
    }

    //=================================================================================================

    private static void UpdateVTol(in Vertex v1, in Vertex v2, double tol)
    {
        var b = new BrepBuilder();
        if (!v1.IsNull && !IsUnchangedFrozen(v1, tol))
        {
            b.UpdateVertex(v1, tol);
        }
        if (!v2.IsNull && !IsUnchangedFrozen(v2, tol))
        {
            b.UpdateVertex(v2, tol);
        }
    }

    // OCCT.NET: frozen. UpdateVertex(V, Tol) only raises the tolerance, so on a frozen vertex already at least as
    // tolerant it would change nothing but the Modified cache bit: skipped, rather than throwing.
    private static bool IsUnchangedFrozen(in Vertex v, double tol)
    {
        return v.TShape()!.IsFrozen && tol <= Tool.Tolerance(v);
    }

    //=================================================================================================

    /// <summary>
    /// Computes new pcurves for the edge so that they have the same parameter as its 3D curve, unless the edge is
    /// flagged SameParameter, and enlarges its vertices. Writes into the edge.
    /// </summary>
    internal static void SameParameter(in Edge edge, double tolerance = 1.0e-5)
    {
        double newTol = -1;
        SameParameter(edge, tolerance, ref newTol, true);
        if (newTol > 0)
        {
            Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
            UpdateVTol(v1, v2, newTol);
        }
    }

    //=================================================================================================

    /// <summary>
    /// The SameParameter computation of one edge, on the edge itself (<paramref name="isUseOldEdge"/>) or on an
    /// empty copy of it, which is returned (unfrozen). <paramref name="newTol"/> receives the tolerance its
    /// vertices need (not applied); it is left unchanged when nothing was computed. Returns a null edge when the edge
    /// is SameParameter or has no 3D curve.
    /// </summary>
    internal static Edge SameParameter(in Edge edge,
                                              double tolerance,
                                              ref double newTol,
                                              bool isUseOldEdge)
    {
        if (Tool.SameParameter(edge))
        {
            return new Edge();
        }
        double f3d = 0.0, l3d = 0.0;
        Placement L3d = new();
        Curve3D? c3d = null;
        NCollection_List<CurveRepresentation> cList;
        GetCurve3d(edge, ref c3d, ref f3d, ref l3d, ref L3d, out cList, isUseOldEdge);
        if (c3d is null)
        {
            return new Edge();
        }

        var b = new BrepBuilder();
        Edge ne;
        BrepTEdge nte;
        if (isUseOldEdge)
        {
            ne = edge;
            nte = (BrepTEdge)edge.TShape()!;
            if (nte.IsFrozen && !WorkingOverlay.IsLocked(nte))
            {
                // Kerf: the caller's edge in an operation's working overlay (unit 10H3): the loop below edits its
                // representations, which are the working copy's
                cList = nte.ChangeCurves();
            }
        }
        else
        {
            ne = ShapeCasts.Edge(edge.EmptyCopied()); //will be modified a little bit later, so copy anyway
            GetCurve3d(ne, ref c3d, ref f3d, ref l3d, ref L3d, out cList); //C3d pointer and CList will be differ after copying
            nte = (BrepTEdge)ne.TShape()!;
            var sit = new Iterator(edge);
            for (; sit.More(); sit.Next())
            { // add vertices from old edge to the new ones
                b.Add(ne, sit.Value());
            }
        }

        var it = new NCollection_List<CurveRepresentation>.Iterator(cList);

        const int ncontrol = 22;

        var hc = new CurveAdaptor();
        var hc2d = new CurveAdaptor2D();
        var hs = new SurfaceAdaptor();
        CurveAdaptor gac = hc;
        CurveAdaptor2D gac2d = hc2d;
        SurfaceAdaptor gas = hs;

        // modified by NIZHNY-OCC486  Tue Aug 27 17:15:13 2002 :
        bool m_TrimmedPeriodical = false;
        if (c3d!.GetType() == typeof(TrimmedCurve3D))
        {
            Curve3D gtC = ((TrimmedCurve3D)c3d).BasisCurve;
            m_TrimmedPeriodical = gtC.IsPeriodic;
        }
        // modified by NIZHNY-OCC486  Tue Aug 27 17:15:17 2002 .

        if (!c3d.IsPeriodic)
        {
            double udeb = c3d.FirstParameter;
            double ufin = c3d.LastParameter;
            // modified by NIZHNY-OCC486  Tue Aug 27 17:17:14 2002 :
            // if (Udeb > f3d) f3d = Udeb;
            // if (l3d > Ufin) l3d = Ufin;
            if (!m_TrimmedPeriodical)
            {
                if (udeb > f3d)
                {
                    f3d = udeb;
                }
                if (l3d > ufin)
                {
                    l3d = ufin;
                }
            }
            // modified by NIZHNY-OCC486  Tue Aug 27 17:17:55 2002 .
        }
        if (!L3d.IsIdentity)
        {
            c3d = (Curve3D)c3d.Transformed(L3d.Transformation);
        }
        gac.Load(c3d, f3d, l3d);

        double Prec_C3d = BRepCheckCheck.PrecCurve(gac);

        bool isSameP = true;
        double maxdist = 0.0;

        //  Modified by skv - Thu Jun  3 12:39:19 2004 OCC5898 Begin
        double edgeTol = Tool.Tolerance(ne);
        //  Modified by skv - Thu Jun  3 12:39:20 2004 OCC5898 End
        bool sameRange = Tool.SameRange(ne);
        bool yaPCu = false;
        const double bigError = 1.0e10;
        it.Initialize(cList);

        while (it.More())
        {

            bool isANA = false;
            bool isBSP = false;
            GCurve? gCurve = it.Value() as GCurve;
            var pc = new Curve2D?[2];
            Surface? s;
            if (gCurve is not null && gCurve.IsCurveOnSurface())
            {
                yaPCu = true;
                pc[0] = gCurve.PCurve();
                Placement pcLoc = gCurve.Location();
                s = gCurve.Surface();
                if (!pcLoc.IsIdentity)
                {
                    s = (Surface)s!.Transformed(pcLoc.Transformation);
                }

                gas.Load(s!);
                if (gCurve.IsCurveOnClosedSurface)
                {
                    pc[1] = gCurve.PCurve2();
                }

                // Eval tol2d to compute SameRange
                double tolSameRange = Max(gac.Resolution(tolerance), Kerf.Engine.Foundation.Precision.PConfusion());
                for (int i = 0; i < 2; i++)
                {
                    Curve2D? curPC = pc[i];
                    bool updatepc = false;
                    if (curPC is null)
                    {
                        break;
                    }
                    if (!sameRange)
                    {
                        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(tolSameRange, pc[i], gCurve.First(), gCurve.Last(), f3d, l3d, ref curPC);

                        updatepc = !ReferenceEquals(curPC, pc[i]);
                    }
                    bool goodpc = true;
                    gac2d.Load(curPC!, f3d, l3d);

                    double error = ComputeTol(hc, hc2d, hs, ncontrol);

                    if (error > bigError)
                    {
                        maxdist = error;
                        break;
                    }

                    if (gac2d.GetType() == BSplineCurve && gac2d.Continuity() == C0)
                    {
                        double uResol = gas.UResolution(tolerance);
                        double vResol = gas.VResolution(tolerance);
                        double tolConf2d = Min(uResol, vResol);
                        tolConf2d = Max(tolConf2d, Kerf.Engine.Foundation.Precision.PConfusion());
                        BSplineCurve2D bs2d = gac2d.BSpline();
                        BSplineCurve2D bs2dsov = bs2d;
                        double fC0 = bs2d.FirstParameter, lC0 = bs2d.LastParameter;
                        bool repar = true;
                        Point2D originPoint;
                        bs2d.D0(fC0, out originPoint);
                        Geom2dConvert2D.C0BSplineToC1BSplineCurve(ref bs2d, tolConf2d);
                        isBSP = true;

                        if (bs2d.IsPeriodic)
                        { // -------- IFV, Jan 2000
                            bs2d.D0(bs2d.FirstParameter, out Point2D newOriginPoint);
                            if (Math.Abs(originPoint.X - newOriginPoint.X) > Kerf.Engine.Foundation.Precision.PConfusion()
                                || Math.Abs(originPoint.Y - newOriginPoint.Y) > Kerf.Engine.Foundation.Precision.PConfusion())
                            {

                                IReadOnlyList<double> knotbs2d = bs2d.Knots;

                                for (int index = 1; index <= bs2d.KnotCount; index++)
                                {
                                    bs2d.D0(knotbs2d[index - 1], out newOriginPoint);
                                    if (Math.Abs(originPoint.X - newOriginPoint.X) > Kerf.Engine.Foundation.Precision.PConfusion()
                                        || Math.Abs(originPoint.Y - newOriginPoint.Y) > Kerf.Engine.Foundation.Precision.PConfusion())
                                    {
                                        continue;
                                    }

                                    bs2d.SetOrigin(index);
                                    break;
                                }
                            }
                        }

                        if (bs2d.Continuity == C0)
                        {
                            if (EvalTol(curPC!, s!, gac, tolerance, out double tolbail))
                            {
                                bs2d = bs2dsov;
                                double uResbail = gas.UResolution(tolbail);
                                double vResbail = gas.VResolution(tolbail);
                                double tol2dbail = Min(uResbail, vResbail);
                                bs2d.D0(bs2d.FirstParameter, out originPoint);

                                int nbp = bs2d.PoleCount;
                                Point2D p = bs2d.Pole(1), p1;
                                double d = Kerf.Engine.Foundation.Precision.Infinite();
                                for (int ip = 2; ip <= nbp; ip++)
                                {
                                    p1 = bs2d.Pole(ip);
                                    d = Min(d, p.SquareDistanceTo(p1));
                                    p = p1;
                                }
                                d = Math.Sqrt(d) * 0.1;

                                tol2dbail = Max(Min(tol2dbail, d), tolConf2d);

                                Geom2dConvert2D.C0BSplineToC1BSplineCurve(ref bs2d, tol2dbail);

                                if (bs2d.IsPeriodic)
                                { // -------- IFV, Jan 2000
                                    bs2d.D0(bs2d.FirstParameter, out Point2D newOriginPoint);
                                    if (Math.Abs(originPoint.X - newOriginPoint.X) > Kerf.Engine.Foundation.Precision.PConfusion()
                                        || Math.Abs(originPoint.Y - newOriginPoint.Y) > Kerf.Engine.Foundation.Precision.PConfusion())
                                    {

                                        IReadOnlyList<double> knotbs2d = bs2d.Knots;

                                        for (int index = 1; index <= bs2d.KnotCount; index++)
                                        {
                                            bs2d.D0(knotbs2d[index - 1], out newOriginPoint);
                                            if (Math.Abs(originPoint.X - newOriginPoint.X) > Kerf.Engine.Foundation.Precision.PConfusion()
                                                || Math.Abs(originPoint.Y - newOriginPoint.Y) > Kerf.Engine.Foundation.Precision.PConfusion())
                                            {
                                                continue;
                                            }

                                            bs2d.SetOrigin(index);
                                            break;
                                        }
                                    }
                                }

                                if (bs2d.Continuity == C0)
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
                                var knots = new NCollection_Array1<double>(1, bs2d.KnotCount);
                                bs2d.CopyKnotsTo(knots);
                                //	    BSplCLib::Reparametrize(f3d,l3d,Knots);
                                global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(fC0, lC0, knots);
                                bs2d.SetKnots(knots);
                                gac2d.Load(bs2d, f3d, l3d);
                                curPC = bs2d;
                                bool updatepcsov = updatepc;
                                updatepc = true;

                                double error1 = ComputeTol(hc, hc2d, hs, ncontrol);
                                if (error1 > error)
                                {
                                    bs2d = bs2dsov;
                                    gac2d.Load(bs2d, f3d, l3d);
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
                            Continuity cont = bs2d.Continuity;
                            bool isBad = false;

                            if (cont > C0 && error > Max(1.0e-3, tolerance))
                            {
                                int nbKnots = bs2d.KnotCount;
                                double critratio = 10.0;
                                double dtprev = bs2d.Knot(2) - bs2d.Knot(1), dtratio = 1.0;
                                double dtmin = dtprev;
                                double dtcur = 0.0;
                                for (int j = 2; j < nbKnots; j++)
                                {
                                    dtcur = bs2d.Knot(j + 1) - bs2d.Knot(j);
                                    dtmin = Min(dtmin, dtcur);

                                    if (isBad)
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
                                        isBad = true;
                                    }
                                    dtprev = dtcur;
                                }
                                if (isBad)
                                {
                                    // To avoid failures in Approx_CurvilinearParameter
                                    bs2d.Resolution(Max(1.0e-3, tolerance), out dtcur);
                                    if (dtmin < dtcur)
                                    {
                                        isBad = false;
                                    }
                                }
                            }

                            if (isBad)
                            { // if BSpline "bad", try to reparametrize it
                              // by its curve length

                                //	      GeomAbs_Shape cont = bs2d->Continuity();
                                if (cont > C2)
                                {
                                    cont = C2;
                                }
                                int maxdeg = bs2d.Degree;
                                if (maxdeg == 1)
                                {
                                    maxdeg = 14;
                                }
                                var appCurPar = new CurvilinearParameter(hc2d,
                                                                                hs,
                                                                                Max(1.0e-3, tolerance),
                                                                                cont,
                                                                                maxdeg,
                                                                                10);
                                if (appCurPar.IsDone || appCurPar.HasResult)
                                {
                                    // The internal form: the curve is re-knotted below before the edge takes it.
                                    bs2d = appCurPar.InternalCurve2d1()!;
                                    gac2d.Load(bs2d, f3d, l3d);
                                    curPC = bs2d;

                                    if (Math.Abs(bs2d.FirstParameter - fC0) > tolSameRange
                                        || Math.Abs(bs2d.LastParameter - lC0) > tolSameRange)
                                    {
                                        var knots = new NCollection_Array1<double>(1, bs2d.KnotCount);
                                        bs2d.CopyKnotsTo(knots);
                                        //		  BSplCLib::Reparametrize(f3d,l3d,Knots);
                                        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(fC0, lC0, knots);
                                        bs2d.SetKnots(knots);
                                        gac2d.Load(bs2d, f3d, l3d);
                                        curPC = bs2d;
                                    }
                                }
                            }
                        }
                    }

                    if (goodpc)
                    {
                        //	  Approx_SameParameter SameP(HC,HC2d,HS,Tolerance);
                        double tol = (isANA && isBSP) ? 1.0e-7 : tolerance;
                        var sameP = new SameParameter(hc, hc2d, hs, tol);

                        if (sameP.IsSameParameter)
                        {
                            maxdist = Max(maxdist, sameP.TolReached());
                            if (updatepc)
                            {
                                if (i == 0)
                                {
                                    gCurve.PCurve(curPC);
                                }
                                else
                                {
                                    gCurve.PCurve2(curPC);
                                }
                            }
                        }
                        else if (sameP.IsDone)
                        {
                            double tolreached = sameP.TolReached();
                            if (tolreached <= error)
                            {
                                curPC = sameP.InternalCurve2d();
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
                                    gCurve.PCurve(curPC);
                                }
                                else
                                {
                                    gCurve.PCurve2(curPC);
                                }
                            }
                        }
                        else
                        {
                            // Approx_SameParameter has failed.
                            // Consequently, the situation might be,
                            // when 3D and 2D-curve do not have same-range.
                            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(tolSameRange,
                                              pc[i],
                                              gCurve.First(),
                                              gCurve.Last(),
                                              f3d,
                                              l3d,
                                              ref curPC);

                            if (i == 0)
                            {
                                gCurve.PCurve(curPC);
                            }
                            else
                            {
                                gCurve.PCurve2(curPC);
                            }

                            isSameP = false;
                        }
                    }
                    else
                    {
                        isSameP = false;
                    }

                    //  Modified by skv - Thu Jun  3 12:39:19 2004 OCC5898 Begin
                    if (!isSameP)
                    {
                        double Prec_Surf = BRepCheckCheck.PrecSurface(hs);
                        double curTol = edgeTol + Max(Prec_C3d, Prec_Surf);
                        if (curTol >= error)
                        {
                            maxdist = Max(maxdist, edgeTol);
                            isSameP = true;
                        }
                    }
                    //  Modified by skv - Thu Jun  3 12:39:20 2004 OCC5898 End
                }
            }
            it.Next();
        }
        b.Range(ne, f3d, l3d);
        b.SameRange(ne, true);
        if (isSameP)
        {
            // Reduce eventually the tolerance of the edge, as
            // all its representations are processed (except for some associated
            // to planes and not stored in the edge !)
            // The same cannot be done with vertices that cannot be enlarged
            // or left as is.
            if (yaPCu)
            {
                // Avoid setting too small tolerances.
                maxdist = Max(maxdist, Kerf.Engine.Foundation.Precision.Confusion());
                newTol = maxdist;
                nte.Modified(true);
                nte.Tolerance(maxdist);
            }
            b.SameParameter(ne, true);
        }

        return ne;
    }

    //=================================================================================================

    private static void InternalUpdateTolerances(in Shape oldShape,
                                                 bool isVerifyTolerance,
                                                 bool isMutableInput,
                                                 ReShape reshaper)
    {
        var shToTol = new Dictionary<Shape, double>(ShapeHasher.Instance);
        // Harmonize tolerances
        // with rule Tolerance(VERTEX)>=Tolerance(EDGE)>=Tolerance(FACE)
        double tol = 0;
        if (isVerifyTolerance)
        {
            // Set tolerance to its minimum value
            Surface? s;
            var ex = new Explorer();
            var b = new Box();
            double xmin, ymin, zmin, xmax, ymax, zmax, dMax;
            for (ex.Init(oldShape, ShapeType.Face); ex.More(); ex.Next())
            {
                Face curf = ShapeCasts.Face(ex.Current());
                s = Tool.Surface(curf, out _);
                if (s is not null)
                {
                    b.SetVoid();
                    Bounds.Add(curf, ref b);
                    if (s.GetType() == typeof(TrimmedSurface))
                    {
                        s = ((TrimmedSurface)s).BasisSurface;
                    }
                    var AS = new SurfaceAdaptor(s);
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
                    if (!b.IsWhole)
                    {
                        b.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);
                        dMax = 1.0;
                        if (!b.IsOpenXmin && !b.IsOpenXmax)
                        {
                            dMax = xmax - xmin;
                        }
                        if (!b.IsOpenYmin && !b.IsOpenYmax)
                        {
                            ymin = ymax - ymin;
                        }
                        if (!b.IsOpenZmin && !b.IsOpenZmax)
                        {
                            zmin = zmax - zmin;
                        }
                        if (ymin > dMax)
                        {
                            dMax = ymin;
                        }
                        if (zmin > dMax)
                        {
                            dMax = zmin;
                        }
                        tol = tol * dMax;
                        // Do not process tolerances > 1.
                        if (tol > 1.0)
                        {
                            tol = 0.99;
                        }
                    }
                    shToTol[curf] = tol;
                }
            }
        }

        // Process edges
        var parents = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(oldShape, ShapeType.Edge, ShapeType.Face, parents);
        var lConx = new NCollection_List<Shape>.Iterator();
        int iCur;
        for (iCur = 1; iCur <= parents.Count; iCur++)
        {
            tol = 0;
            for (lConx.Initialize(parents.ValueAt(iCur - 1)); lConx.More(); lConx.Next())
            {
                Face ff = ShapeCasts.Face(lConx.Value());
                double ftol;
                if (isVerifyTolerance && shToTol.ContainsKey(ff))
                { // first condition for speed-up
                    ftol = shToTol.GetValue(ff);
                }
                else
                {
                    ftol = Tool.Tolerance(ff); // tolerance have not been updated
                }
                tol = Max(tol, ftol);
            }
            // Update can only increase tolerance, so if the edge has a greater
            //  tolerance than its faces it is not concerned
            Edge ek = ShapeCasts.Edge(parents.KeyAt(iCur - 1));
            if (tol > Tool.Tolerance(ek))
            {
                shToTol[ek] = tol;
            }
        }

        // Vertices are processed
        const double bigTol = 1.0e10;
        parents.Clear();

        Exploration.MapShapesAndUniqueAncestors(oldShape, ShapeType.Vertex, ShapeType.Edge, parents);
        // OCCT's map of handles: the vertex TShapes by reference.
        var initialized = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        int nbV = parents.Count;
        for (iCur = 1; iCur <= nbV; iCur++)
        {
            tol = 0;
            Vertex v = ShapeCasts.Vertex(parents.KeyAt(iCur - 1));
            Point3D pv = Tool.Pnt(v);
            double maxDist = 0.0;
            Point3D p3d;
            for (lConx.Initialize(parents.ValueAt(iCur - 1)); lConx.More(); lConx.Next())
            {
                Edge e = ShapeCasts.Edge(lConx.Value());
                tol = Max(tol, shToTol.TryGetValue(e, out double ntol) ? ntol : Tool.Tolerance(e));
                if (tol > bigTol)
                {
                    continue;
                }
                if (!Tool.SameRange(e))
                {
                    continue;
                }
                double par = Tool.Parameter(v, e);
                BrepTEdge te = (BrepTEdge)e.TShape()!;
                var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
                Placement eloc = e.Placement;
                while (itcr.More())
                {
                    // For each CurveRepresentation, check the provided parameter
                    CurveRepresentation cr = itcr.Value();
                    Placement loc = cr.Location();
                    Placement l = (eloc * loc);
                    if (cr.IsCurve3D)
                    {
                        Curve3D? c = cr.Curve3D();
                        if (c is not null)
                        { // edge non degenerated
                            p3d = c.Value(par);
                            p3d = p3d.Transformed(l.Transformation);
                            double dist = p3d.SquareDistanceTo(pv);
                            if (dist > maxDist)
                            {
                                maxDist = dist;
                            }
                        }
                    }
                    else if (cr.IsCurveOnSurface())
                    {
                        Surface su = cr.Surface()!;
                        Curve2D pc = cr.PCurve()!;
                        Curve2D? pc2 = null;
                        if (cr.IsCurveOnClosedSurface)
                        {
                            pc2 = cr.PCurve2();
                        }
                        Point2D p2d = pc.Value(par);
                        p3d = su.Value(p2d.X, p2d.Y);
                        p3d = p3d.Transformed(l.Transformation);
                        double dist = p3d.SquareDistanceTo(pv);
                        if (dist > maxDist)
                        {
                            maxDist = dist;
                        }
                        if (pc2 is not null)
                        {
                            p2d = pc2.Value(par);
                            p3d = su.Value(p2d.X, p2d.Y);
                            p3d = p3d.Transformed(l.Transformation);
                            dist = p3d.SquareDistanceTo(pv);
                            if (dist > maxDist)
                            {
                                maxDist = dist;
                            }
                        }
                    }
                    itcr.Next();
                }
            }
            tol = Max(tol, Math.Sqrt(maxDist));
            tol += 2.0 * Epsilon(tol);
            //
            double vTol = Tool.Tolerance(v);
            bool updTol = tol > vTol;
            TShape tv = v.TShape()!;
            bool toAdd = false;
            if (isVerifyTolerance)
            {
                // ASet minimum value of the tolerance
                // Attention to sharing of the vertex by other shapes
                toAdd = initialized.Add(tv) && vTol != tol; // if Vtol == tol => no need to update toler
            }
            //'Initialized' map is not used anywhere outside this block
            if (updTol || toAdd)
            {
                shToTol[v] = tol;
            }
        }

        UpdShTol(shToTol, isMutableInput, reshaper, true);
    }

    //=================================================================================================

    /// <summary>
    /// Raises the tolerances of the edges and vertices of <paramref name="s"/> to the rule
    /// Tolerance(VERTEX) &gt;= Tolerance(EDGE) &gt;= Tolerance(FACE); with <paramref name="verifyFaceTolerance"/>,
    /// first sets the faces' tolerances to a minimum by surface type and size (and lowers vertices to what they need).
    /// Writes into the shape.
    /// </summary>
    internal static void UpdateTolerances(in Shape s, bool verifyFaceTolerance = false)
    {
        var reshaper = new ReShape();
        InternalUpdateTolerances(s, verifyFaceTolerance, true, reshaper);
    }

    //=================================================================================================

    /// <summary>
    /// <see cref="UpdateTolerances(in Shape, bool)"/> without changing <paramref name="s"/>: the changed
    /// faces, edges and vertices are copies recorded as replacements in <paramref name="reshaper"/>, whose
    /// <see cref="ReShape.Apply"/> gives the result.
    /// </summary>
    public static void UpdateTolerances(in Shape s,
                                        ReShape reshaper,
                                        bool verifyFaceTolerance = false)
    {
        InternalUpdateTolerances(s, verifyFaceTolerance, false, reshaper);
    }

    //=================================================================================================

    /// <summary>
    /// Checks the tolerances of the edges (inner points included) and vertices of the shape and raises them to
    /// satisfy the SameParameter condition. Writes into the shape.
    /// </summary>
    internal static void UpdateInnerTolerances(in Shape aShape)
    {
        var eFmap = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aShape, ShapeType.Edge, ShapeType.Face, eFmap);
        var bb = new BrepBuilder();
        for (int i = 0; i < eFmap.Count; i++)
        {
            Edge edge = ShapeCasts.Edge(eFmap.KeyAt(i));

            if (!Tool.IsGeometric(edge))
            {
                continue;
            }

            Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
            Tool.Range(edge, out double fpar, out double lpar);
            double tolEdge = Tool.Tolerance(edge);
            Point3D pnt1 = new(), pnt2 = new();
            var hCurve = new EdgeCurveAdaptor();
            hCurve.Initialize(edge);
            if (!v1.IsNull)
            {
                pnt1 = Tool.Pnt(v1);
            }
            if (!v2.IsNull)
            {
                pnt2 = Tool.Pnt(v2);
            }

            if (!Tool.Degenerated(edge) && eFmap.ValueAt(i).Extent() > 0)
            {
                var theRep = new NCollection_Sequence<CurveAdaptorBase>();
                theRep.Append(hCurve);
                var itl = new NCollection_List<Shape>.Iterator(eFmap.ValueAt(i));
                for (; itl.More(); itl.Next())
                {
                    Face face = ShapeCasts.Face(itl.Value());
                    var hCurvOnSurf = new EdgeCurveAdaptor();
                    hCurvOnSurf.Initialize(edge, face);
                    theRep.Append(hCurvOnSurf);
                }

                int nbSamples = (Tool.SameParameter(edge)) ? 23 : 2;
                double delta = (lpar - fpar) / (nbSamples - 1);
                double maxDist = 0.0;
                for (int j = 2; j <= theRep.Length(); j++)
                {
                    for (int k = 0; k <= nbSamples; k++)
                    {
                        double paramOnCenter = (k == nbSamples) ? lpar : fpar + k * delta;
                        Point3D center = theRep[1].Value(paramOnCenter);
                        double paramOnCurve =
                          (Tool.SameParameter(edge))
                            ? paramOnCenter
                            : ((k == 0) ? theRep[j].FirstParameter() : theRep[j].LastParameter());
                        Point3D point = theRep[j].Value(paramOnCurve);
                        double dist = center.DistanceTo(point);
                        // aDist *= 1.1;
                        dist += 2.0 * Epsilon(dist);
                        if (dist > maxDist)
                        {
                            maxDist = dist;
                        }

                        // Update tolerances of vertices
                        if (k == 0 && !v1.IsNull)
                        {
                            double aDist1 = pnt1.DistanceTo(point);
                            aDist1 += 2.0 * Epsilon(aDist1);
                            bb.UpdateVertex(v1, aDist1);
                        }
                        if (k == nbSamples && !v2.IsNull)
                        {
                            double aDist2 = pnt2.DistanceTo(point);
                            aDist2 += 2.0 * Epsilon(aDist2);
                            bb.UpdateVertex(v2, aDist2);
                        }
                    }
                }
                bb.UpdateEdge(edge, maxDist);
            }
            tolEdge = Tool.Tolerance(edge);
            if (!v1.IsNull)
            {
                Point3D end1 = hCurve.Value(fpar);
                double dist1 = pnt1.DistanceTo(end1);
                dist1 += 2.0 * Epsilon(dist1);
                bb.UpdateVertex(v1, Max(dist1, tolEdge));
            }
            if (!v2.IsNull)
            {
                Point3D end2 = hCurve.Value(lpar);
                double dist2 = pnt2.DistanceTo(end2);
                dist2 += 2.0 * Epsilon(dist2);
                bb.UpdateVertex(v2, Max(dist2, tolEdge));
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
        var where = new SolidClassifier(solid);
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
        private readonly SLProps _surfaceProps; // properties calculator
        private readonly Transform3D _surfaceTrsf;
        private readonly Curve2D _curve2d;
        private readonly bool _isReversed; // the face based on the surface is reversed

        // tangent vector to Pcurve in UV
        private Vector2D _curveTangent;

        public SurfaceProperties(Surface surface, in Transform3D surfaceTrsf, Curve2D curve2D, bool reversed)
        {
            _surfaceProps = new SLProps(surface, 2, Kerf.Engine.Foundation.Precision.Confusion());
            _surfaceTrsf = surfaceTrsf;
            _curve2d = curve2D;
            _isReversed = reversed;
            _curveTangent = new Vector2D();
        }

        // Calculate derivatives on surface related to the point on curve
        public void Calculate(double paramOnCurve)
        {
            _curve2d.D1(paramOnCurve, out Point2D uv, out _curveTangent);
            _surfaceProps.SetParameters(uv.X, uv.Y);
        }

        // Returns point just calculated
        public Point3D Value() => _surfaceProps.Value().Transformed(_surfaceTrsf);

        // Calculate a derivative orthogonal to curve's tangent vector
        public Vector3D Derivative()
        {
            var deriv = new Vector3D();
            // direction orthogonal to tangent vector of the curve
            var ortho = new Vector2D(-_curveTangent.Y, _curveTangent.X);
            double len = ortho.Magnitude;
            if (len < Kerf.Engine.Foundation.Precision.Confusion())
            {
                return deriv;
            }
            ortho /= len;
            if (_isReversed)
            {
                ortho = ortho.Reversed();
            }

            deriv = Vector3D.LinearForm(ortho.X, _surfaceProps.D1U(), ortho.Y, _surfaceProps.D1V());
            return deriv.Transformed(_surfaceTrsf);
        }

        public Direction3D Normal()
        {
            Direction3D normal = _surfaceProps.Normal();
            return normal.Transformed(_surfaceTrsf);
        }

        // Calculate principal curvatures, which consist of minimal and maximal normal curvatures and
        // the directions on the tangent plane (principal direction) where the extremums are reached
        public void Curvature(out Direction3D principalDir1, out double curvature1, out Direction3D principalDir2, out double curvature2)
        {
            _surfaceProps.CurvatureDirections(out principalDir1, out principalDir2);
            curvature1 = _surfaceProps.MaxCurvature();
            curvature2 = _surfaceProps.MinCurvature();
            if (_isReversed)
            {
                curvature1 = -curvature1;
                curvature2 = -curvature2;
            }
            if (_surfaceTrsf.IsNegative)
            {
                curvature1 = -curvature1;
                curvature2 = -curvature2;
            }

            principalDir1 = principalDir1.Transformed(_surfaceTrsf);
            principalDir2 = principalDir2.Transformed(_surfaceTrsf);
        }
    }

    /// <summary>
    /// The order of continuity between two faces connected by an edge (the angular tolerance in radians). A seam
    /// edge (both faces the same) on an elementary surface is CN.
    /// </summary>
    public static Continuity ContinuityOfFaces(in Edge edge,
                                                  in Face face1,
                                                  in Face face2,
                                                  double angleTol)
    {
        bool isSeam = face1.IsEqual(face2);

        Edge edgeInFace1 = new(), edgeInFace2 = new();
        Curve2D? curve1, curve2;

        double first, last;

        if (!face1.IsSame(face2) && Tool.IsClosed(edge, face1)
            && Tool.IsClosed(edge, face2))
        {
            // Find the edge in the face 1: this edge will have correct orientation
            Face aFace1 = face1;
            aFace1.Orient(Forward);
            var explo = new Explorer(aFace1, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(explo.Current());
                if (anEdge.IsSame(edge))
                {
                    edgeInFace1 = anEdge;
                    break;
                }
            }
            if (edgeInFace1.IsNull)
            {
                return C0;
            }

            curve1 = Tool.CurveOnSurface(edgeInFace1, aFace1, out first, out last);
            Face aFace2 = face2;
            aFace2.Orient(Forward);
            edgeInFace2 = edgeInFace1;
            edgeInFace2.Reverse();
            curve2 = Tool.CurveOnSurface(edgeInFace2, aFace2, out first, out last);
        }
        else
        {
            // Obtaining of pcurves of edge on two faces.
            edgeInFace1 = edgeInFace2 = edge;
            curve1 = Tool.CurveOnSurface(edgeInFace1, face1, out first, out last);
            // For the case of seam edge
            if (face1.IsSame(face2))
            {
                edgeInFace2.Reverse();
            }
            curve2 = Tool.CurveOnSurface(edgeInFace2, face2, out first, out last);
        }

        if (curve1 is null || curve2 is null)
        {
            return C0;
        }

        Surface surface1 = Tool.Surface(face1, out Placement loc1)!;
        Transform3D surf1Trsf = loc1.Transformation;
        Surface surface2 = Tool.Surface(face2, out Placement loc2)!;
        Transform3D surf2Trsf = loc2.Transformation;

        if (surface1 is TrimmedSurface ts1)
        {
            surface1 = ts1.BasisSurface;
        }
        if (surface2 is TrimmedSurface ts2)
        {
            surface2 = ts2.BasisSurface;
        }

        // seam edge on elementary surface is always CN
        bool isElementary = (surface1 is ElementarySurface && surface2 is ElementarySurface);
        if (isSeam && isElementary)
        {
            return CN;
        }

        var sp1 = new SurfaceProperties(surface1, surf1Trsf, curve1, face1.Orientation == Reversed);
        var sp2 = new SurfaceProperties(surface2, surf2Trsf, curve2, face2.Orientation == Reversed);

        double f, l, eps;
        Tool.Range(edge, out f, out l);
        var ext = new LocateExtPC();
        EdgeCurveAdaptor? hc2 = null;

        eps = (l - f) / 100.0;
        f += eps; // to avoid calculations on
        l -= eps; // points of pointed squares.

        double angleTol2 = angleTol * angleTol;

        Vector3D der1, der2;
        double sqLen1, sqLen2;
        var crvDir1 = new Direction3D[2];
        var crvDir2 = new Direction3D[2];
        Span<double> crvLen1 = stackalloc double[2];
        Span<double> crvLen2 = stackalloc double[2];

        Continuity cont = (isElementary ? CN : C2);
        Continuity curCont;
        double u;
        for (int i = 0; i <= 20 && cont > C0; i++)
        {
            // First suppose that this is sameParameter
            u = f + (l - f) * i / 20;

            // Check conditions for G1 and C1 continuity:
            // * calculate a derivative in tangent plane of each surface
            //   orthogonal to curve's tangent vector
            // * continuity is C1 if the vectors are equal
            // * continuity is G1 if the vectors are just parallel
            curCont = C0;

            sp1.Calculate(u);
            sp2.Calculate(u);

            der1 = sp1.Derivative();
            sqLen1 = der1.SquareMagnitude;
            der2 = sp2.Derivative();
            sqLen2 = der2.SquareMagnitude;
            bool isSmoothSuspect = (der1.CrossSquareMagnitude(der2) <= angleTol2 * sqLen1 * sqLen2);
            if (isSmoothSuspect)
            {
                Direction3D normal1 = sp1.Normal();
                if (face1.Orientation == Reversed)
                {
                    normal1 = normal1.Reversed();
                }
                Direction3D normal2 = sp2.Normal();
                if (face2.Orientation == Reversed)
                {
                    normal2 = normal2.Reversed();
                }

                if (normal1 * normal2 < 0.0)
                {
                    return C0;
                }
            }

            if (!isSmoothSuspect)
            {
                // Refine by projection
                if (hc2 is null)
                {
                    // adaptor for pcurve on the second surface
                    hc2 = new EdgeCurveAdaptor(edgeInFace2, face2);
                    ext.Initialize(hc2, f, l, Kerf.Engine.Foundation.Precision.PConfusion());
                }
                ext.Perform(sp1.Value(), u);
                if (ext.IsDone && ext.IsMin())
                {
                    POnCurv poc = ext.Point();
                    sp2.Calculate(poc.Parameter());
                    der2 = sp2.Derivative();
                    sqLen2 = der2.SquareMagnitude;
                }
                isSmoothSuspect = (der1.CrossSquareMagnitude(der2) <= angleTol2 * sqLen1 * sqLen2);
            }
            if (isSmoothSuspect)
            {
                curCont = G1;
                if (Math.Abs(Math.Sqrt(sqLen1) - Math.Sqrt(sqLen2)) < Kerf.Engine.Foundation.Precision.Confusion()
                    && der1.Dot(der2) > Kerf.Engine.Foundation.Precision.SquareConfusion())
                { // <= check vectors are codirectional
                    curCont = C1;
                }
            }
            else
            {
                return C0;
            }

            if (cont < G2)
            {
                continue; // no need further processing, because maximal continuity is less than G2
            }

            // Check conditions for G2 and C2 continuity:
            // * calculate principal curvatures on each surface
            // * continuity is C2 if directions of principal curvatures are equal on different surfaces
            // * continuity is G2 if directions of principal curvatures are just parallel
            //   and values of curvatures are the same
            sp1.Curvature(out crvDir1[0], out crvLen1[0], out crvDir1[1], out crvLen1[1]);
            sp2.Curvature(out crvDir2[0], out crvLen2[0], out crvDir2[1], out crvLen2[1]);
            for (int step = 0; step <= 1; ++step)
            {
                if (crvDir1[0].XYZ().CrossSquareMagnitude(crvDir2[step].XYZ()) <= Kerf.Engine.Foundation.Precision.SquareConfusion()
                    && Math.Abs(crvLen1[0] - crvLen2[step]) < Kerf.Engine.Foundation.Precision.Confusion()
                    && crvDir1[1].XYZ().CrossSquareMagnitude(crvDir2[1 - step].XYZ()) <= Kerf.Engine.Foundation.Precision.SquareConfusion()
                    && Math.Abs(crvLen1[1] - crvLen2[1 - step]) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    if (curCont == C1 && crvDir1[0].Dot(crvDir2[step]) > Kerf.Engine.Foundation.Precision.Confusion()
                        && crvDir1[1].Dot(crvDir2[1 - step]) > Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        curCont = C2;
                    }
                    else
                    {
                        curCont = G2;
                    }
                    break;
                }
            }

            if (curCont < cont)
            {
                cont = curCont;
            }
        }

        // according to the list of supported elementary surfaces,
        // if the continuity is C2, than it is totally CN
        if (isElementary && cont == C2)
        {
            cont = CN;
        }
        return cont;
    }

    //=======================================================================
    // function : EncodeRegularity
    // purpose  : Code the regularities on all edges of the shape, boundary of
    //            two faces that do not have it.
    //            Takes into account that compound may consists of same solid
    //            placed with different transformations
    //=======================================================================
    private static void EncodeRegularityCore(in Shape shape,
                                         double tolAng,
                                         HashSet<Shape> map,
                                         HashSet<Shape>? edgesToEncode = null)
    {
        Shape aShape = shape;
        var nullLoc = new Placement();
        aShape.Location(nullLoc); // nullify location
        if (!map.Add(aShape))
        {
            return; // do not need to process shape twice
        }

        if (aShape.Type == ShapeType.Compound || aShape.Type == ShapeType.CompSolid)
        {
            for (var it = new Iterator(aShape); it.More(); it.Next())
            {
                EncodeRegularityCore(it.Value(), tolAng, map, edgesToEncode);
            }
            return;
        }

        try
        {
            var m = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(aShape, ShapeType.Edge, ShapeType.Face, m);
            var It = new NCollection_List<Shape>.Iterator();
            var ex = new Explorer();
            Face f1 = new(), f2 = new();
            for (int i = 0; i < m.Count; i++)
            {
                Edge e = ShapeCasts.Edge(m.KeyAt(i));
                if (edgesToEncode is not null && edgesToEncode.Count != 0)
                {
                    // process only the edges from the list to update their regularity
                    Shape pureEdge = e.Located(nullLoc);
                    pureEdge.Orient(Forward);
                    if (!edgesToEncode.Contains(pureEdge))
                    {
                        continue;
                    }
                }

                bool found = false;
                f1.Nullify();
                for (It.Initialize(m.ValueAt(i)); It.More() && !found; It.Next())
                {
                    if (f1.IsNull)
                    {
                        f1 = ShapeCasts.Face(It.Value());
                    }
                    else
                    {
                        Face tmpF2 = ShapeCasts.Face(It.Value());
                        if (!f1.IsSame(tmpF2))
                        {
                            found = true;
                            f2 = tmpF2;
                        }
                    }
                }
                if (!found && !f1.IsNull)
                { // is it a sewing edge?
                    Orientation orE = e.Orientation;
                    Edge curE;
                    for (ex.Init(f1, ShapeType.Edge); ex.More() && !found; ex.Next())
                    {
                        curE = ShapeCasts.Edge(ex.Current());
                        if (e.IsSame(curE) && orE != curE.Orientation)
                        {
                            found = true;
                            f2 = f1;
                        }
                    }
                }
                if (found)
                {
                    EncodeRegularity(e, f1, f2, tolAng);
                }
            }
        }
        catch (FailureException exception) when (exception is not LockedShapeException) // OCCT.NET: frozen
        {
        }
    }

    /// <summary>
    /// Codes the regularity of every edge of the shape bounding two faces that hasn't one yet (in place: internal; the
    /// angular tolerance in radians). A compound holding the same solid at several locations is coded once.
    /// </summary>
    internal static void EncodeRegularity(in Shape s, double tolAng = 1.0e-10)
    {
        var map = new HashSet<Shape>(ShapeHasher.Instance);
        EncodeRegularityCore(s, tolAng, map);
    }

    /// <summary>
    /// Codes the regularity of the edges of <paramref name="le"/> bounding two faces of the shape that haven't one yet
    /// (in place: internal).
    /// </summary>
    internal static void EncodeRegularity(in Shape s, NCollection_List<Shape> le, double tolAng = 1.0e-10)
    {
        // Collect edges without location and orientation
        var pureEdges = new HashSet<Shape>(ShapeHasher.Instance);
        var nullLoc = new Placement();
        var edgeIt = new NCollection_List<Shape>.Iterator(le);
        for (; edgeIt.More(); edgeIt.Next())
        {
            Shape edge = edgeIt.Value();
            edge.Location(nullLoc);
            edge.Orient(Forward);
            pureEdges.Add(edge);
        }

        var map = new HashSet<Shape>(ShapeHasher.Instance);
        EncodeRegularityCore(s, tolAng, map, pureEdges);
    }

    /// <summary>
    /// Codes the regularity between <paramref name="f1"/> and <paramref name="f2"/> on <paramref name="e"/>, unless
    /// already coded (in place: internal).
    /// </summary>
    internal static void EncodeRegularity(in Edge e, in Face f1, in Face f2, double tolAng = 1.0e-10)
    {
        var b = new BrepBuilder();
        if (Tool.Continuity(e, f1, f2) <= C0)
        {
            try
            {
                Continuity cont = ContinuityOfFaces(e, f1, f2, tolAng);
                b.Continuity(e, f1, f2, cont);
            }
            // OCCT.NET: frozen. OCCT swallows the TopoDS_LockedShape of a locked edge; a frozen one is a write into a
            // shape the caller holds, which must not pass silently.
            catch (FailureException) when (!e.TShape()!.IsFrozen)
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
        private readonly Dictionary<Triangulation, Triangulation> _copies = new(ReferenceEqualityComparer.Instance);

        // The triangulation to read: the copy if one was made.
        public Triangulation Read(Triangulation t)
            => _copies.TryGetValue(t, out Triangulation? copy) ? copy : t;

        // The triangulation to write into: a copy of a frozen one.
        public Triangulation Write(Triangulation t)
        {
            if (_copies.TryGetValue(t, out Triangulation? copy))
            {
                return copy;
            }
            if (!t.IsFrozen)
            {
                return t;
            }
            copy = t.Copy();
            _copies.Add(t, copy);
            return copy;
        }

        public void Attach(in Shape shape)
        {
            if (_copies.Count == 0)
            {
                return;
            }
            var b = new BrepBuilder();
            var done = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
            for (var expFace = new Explorer(shape, ShapeType.Face); expFace.More(); expFace.Next())
            {
                Face face = ShapeCasts.Face(expFace.Current());
                if (!done.Add(face.TShape()!))
                {
                    continue;
                }
                var tf = (BrepTFace)face.TShape()!;
                Triangulation? old = tf.ActiveTriangulation();
                if (old is null || !_copies.TryGetValue(old, out Triangulation? aNew))
                {
                    continue;
                }
                b.UpdateFace(face, aNew, false);
                for (var expEdge = new Explorer(face, ShapeType.Edge); expEdge.More(); expEdge.Next())
                {
                    var te = (BrepTEdge)expEdge.Current().TShape()!;
                    if (!done.Add(te))
                    {
                        continue;
                    }
                    for (var it = new NCollection_List<CurveRepresentation>.Iterator(te.ChangeCurves()); it.More(); it.Next())
                    {
                        CurveRepresentation cr = it.Value();
                        if (!cr.IsPolygonOnTriangulation() || !_copies.TryGetValue(cr.Triangulation()!, out Triangulation? newT))
                        {
                            continue;
                        }
                        it.ChangeValue() = cr.IsPolygonOnClosedTriangulation
                            ? new PolygonOnClosedTriangulation(cr.PolygonOnTriangulation(), cr.PolygonOnTriangulation2(), newT, cr.Location())
                            : new PolygonOnTriangulationRepresentation(cr.PolygonOnTriangulation(), newT, cr.Location());
                    }
                    expEdge.Current().TShape()!.Modified(true);
                }
            }
        }
    }

    /// <summary>
    /// Corrects the normals of the faces' triangulations so that nodes along smooth edges have the same normal on
    /// both sides (computing missing normals first). Returns true if anything was corrected. In place: internal; the
    /// triangulations, frozen once attached, are replaced by corrected copies (see the remarks of the port).
    /// </summary>
    internal static bool EnsureNormalConsistency(in Shape shape, double angTol = 0.001, bool forceComputeNormals = false)
    {
        double thresDot = Math.Cos(angTol);

        bool retVal = false, isNormalsFound = false;
        var writable = new WritableTriangulations(); // OCCT.NET: frozen

        // compute normals if they are absent
        var expFace = new Explorer(shape, ShapeType.Face);
        for (; expFace.More(); expFace.Next())
        {
            Face face = ShapeCasts.Face(expFace.Current());
            Surface? surf = Tool.Surface(face);
            if (surf is null)
            {
                continue;
            }
            Triangulation? attached = Tool.Triangulation(face, out Placement loc);
            if (attached is null)
            {
                continue;
            }
            Triangulation pt = writable.Read(attached);
            if (!forceComputeNormals && pt.HasNormals)
            {
                isNormalsFound = true;
                continue;
            }

            pt = writable.Write(attached);
            pt.AddNormals();
            var slp = new SLProps(surf, 2, Kerf.Engine.Foundation.Precision.Confusion());
            for (int i = 1; i <= pt.NodeCount; i++)
            {
                Point2D p2d = pt.UVNode(i);
                slp.SetParameters(p2d.X, p2d.Y);

                if (!slp.IsNormalDefined)
                {
                    pt.SetNormal(i, new NCollection_Vec3<float>(0.0f));
                }
                else
                {
                    Direction3D norm = slp.Normal();
                    if (face.Orientation == Reversed)
                    {
                        norm = norm.Reversed();
                    }
                    pt.SetNormal(i, norm);
                }
            }

            retVal = true;
            isNormalsFound = true;
        }

        if (!isNormalsFound)
        {
            writable.Attach(shape);
            return retVal;
        }

        // loop by edges
        var mapEF = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(shape, ShapeType.Edge, ShapeType.Face, mapEF);
        for (int ind = 0; ind < mapEF.Count; ind++)
        {
            Edge edg = ShapeCasts.Edge(mapEF.KeyAt(ind));
            NCollection_List<Shape> edgList = mapEF.ValueAt(ind);
            if (edgList.Extent() != 2)
            {
                continue;
            }
            var itF = new NCollection_List<Shape>.Iterator(edgList);
            Face face1 = ShapeCasts.Face(itF.Value());
            itF.Next();
            Face face2 = ShapeCasts.Face(itF.Value());
            Triangulation? attached1 = Tool.Triangulation(face1, out Placement loc1);
            Triangulation? attached2 = Tool.Triangulation(face2, out Placement loc2);

            if (attached1 is null || attached2 is null)
            {
                continue;
            }
            Triangulation pt1 = writable.Read(attached1);
            Triangulation pt2 = writable.Read(attached2);

            if (!pt1.HasNormals || !pt2.HasNormals)
            {
                continue;
            }

            // the edges still refer to the attached triangulations. OCCT dereferences the polygons without a null check
            // (occt-issues.md): an edge with no polygon on one of the triangulations throws here.
            PolygonOnTriangulation ptef1 = Tool.PolygonOnTriangulation(edg, attached1, loc1)!;
            PolygonOnTriangulation ptef2 = Tool.PolygonOnTriangulation(edg, attached2, loc2)!;

            if (ptef1.Nodes().Lower() != ptef2.Nodes().Lower()
                || ptef1.Nodes().Upper() != ptef2.Nodes().Upper())
            {
                continue;
            }

            for (int edgNode = ptef1.Nodes().Lower(); edgNode <= ptef1.Nodes().Upper(); edgNode++)
            {
                // Number of node
                int fNodF1 = ptef1.Nodes().Value(edgNode);
                int fNodF2 = ptef2.Nodes().Value(edgNode);

                pt1 = writable.Read(attached1);
                pt2 = writable.Read(attached2);
                pt1.Normal(fNodF1, out NCollection_Vec3<float> norm1f);
                pt2.Normal(fNodF2, out NCollection_Vec3<float> norm2f);
                var norm1 = new Xyz(norm1f.X(), norm1f.Y(), norm1f.Z());
                var norm2 = new Xyz(norm2f.X(), norm2f.Y(), norm2f.Z());
                double dot = norm1 * norm2;
                if (dot > thresDot)
                {
                    Xyz newNorm = (norm1 + norm2).Normalized();
                    writable.Write(attached1).SetNormal(fNodF1, new Direction3D(newNorm));
                    writable.Write(attached2).SetNormal(fNodF2, new Direction3D(newNorm));
                    retVal = true;
                }
            }
        }

        writable.Attach(shape);
        return retVal;
    }

    // Tool to estimate deflection of the given UV point with regard to its representation in 3D space.
    private sealed class EvalDeflection
    {
        private readonly FaceSurfaceAdaptor _surface;

        // Initializes tool with the given face.
        public EvalDeflection(in Face face)
        {
            _surface = new FaceSurfaceAdaptor(face);
        }

        // Evaluates deflection of the given 2d point from its 3d representation.
        public double Eval(in Point2D point2d, in Point3D point3d)
        {
            _surface.D0(point2d.X, point2d.Y, out Point3D pnt);
            return (point3d.XYZ() - pnt.XYZ()).SquareModulus();
        }
    }

    // Represents link of triangulation: equal whatever the order of its nodes.
    private readonly struct Link : IEquatable<Link>
    {
        private readonly int _node0;
        private readonly int _node1;

        public Link(int node1, int node2)
        {
            _node0 = node1;
            _node1 = node2;
        }

        public bool Equals(Link other)
            => (_node0 == other._node0 && _node1 == other._node1)
               || (_node0 == other._node1 && _node1 == other._node0);

        public override bool Equals(object? obj) => obj is Link link && Equals(link);

        public override int GetHashCode() => HashCode.Combine(Math.Min(_node0, _node1), Math.Max(_node0, _node1));
    }

    /// <summary>
    /// Sets the deflection of each face's triangulation to the maximum deviation measured on it (triangle and inner
    /// link midpoints against the surface). In place: internal; the triangulations, frozen once attached, are replaced
    /// by copies with the new deflection.
    /// </summary>
    internal static void UpdateDeflection(in Shape shape)
    {
        var writable = new WritableTriangulations(); // OCCT.NET: frozen
        Span<int> node = stackalloc int[3];
        var expFace = new Explorer(shape, ShapeType.Face);
        for (; expFace.More(); expFace.Next())
        {
            Face face = ShapeCasts.Face(expFace.Current());
            Surface? surf = Tool.Surface(face);
            if (surf is null)
            {
                continue;
            }

            Triangulation? attached = Tool.Triangulation(face, out Placement loc);
            if (attached is null)
            {
                continue;
            }
            Triangulation pt = writable.Read(attached);
            if (!pt.HasUVNodes)
            {
                continue;
            }

            // Collect all nodes of degenerative edges and skip elements
            // build upon them due to huge distortions introduced by passage
            // from UV space to 3D.
            var degNodes = new HashSet<int>();
            var expEdge = new Explorer(face, ShapeType.Edge);
            for (; expEdge.More(); expEdge.Next())
            {
                Edge edge = ShapeCasts.Edge(expEdge.Current());
                if (Tool.Degenerated(edge))
                {
                    PolygonOnTriangulation? polygon = Tool.PolygonOnTriangulation(edge, attached, loc);
                    if (polygon is null)
                    {
                        continue;
                    }

                    for (int nodeIt = polygon.Nodes().Lower(); nodeIt <= polygon.Nodes().Upper(); ++nodeIt)
                    {
                        degNodes.Add(polygon.Node(nodeIt));
                    }
                }
            }

            var tool = new EvalDeflection(face);
            var links = new HashSet<Link>();
            double sqDeflection = 0.0;
            Transform3D trsf = loc.Transformation;
            var p3d = new Point3D[3];
            var p2d = new Point2D[3];
            for (int triIt = 1; triIt <= pt.TriangleCount; ++triIt)
            {
                Triangle1 triangle = pt.Triangle(triIt);

                triangle.Get(out node[0], out node[1], out node[2]);
                if (degNodes.Contains(node[0]) || degNodes.Contains(node[1]) || degNodes.Contains(node[2]))
                {
                    continue;
                }

                p3d[0] = pt.Node(node[0]).Transformed(trsf);
                p3d[1] = pt.Node(node[1]).Transformed(trsf);
                p3d[2] = pt.Node(node[2]).Transformed(trsf);

                p2d[0] = pt.UVNode(node[0]);
                p2d[1] = pt.UVNode(node[1]);
                p2d[2] = pt.UVNode(node[2]);

                // Check midpoint of triangle.
                var mid3d_t = new Point3D((p3d[0].XYZ() + p3d[1].XYZ() + p3d[2].XYZ()) / 3.0);
                var mid2d_t = new Point2D((p2d[0].XY() + p2d[1].XY() + p2d[2].XY()) / 3.0);

                sqDeflection = Max(sqDeflection, tool.Eval(mid2d_t, mid3d_t));

                for (int i = 0; i < 3; ++i)
                {
                    int j = (i + 1) % 3;
                    var link = new Link(node[i], node[j]);
                    if (!links.Add(link))
                    {
                        // Do not estimate boundary links due to high distortions at the edge.
                        Point3D p3d1 = p3d[i];
                        Point3D p3d2 = p3d[j];

                        Point2D p2d1 = p2d[i];
                        Point2D p2d2 = p2d[j];

                        var mid3d_l = new Point3D((p3d1.XYZ() + p3d2.XYZ()) / 2.0);
                        var mid2d_l = new Point2D((p2d1.XY() + p2d2.XY()) / 2.0);

                        sqDeflection = Max(sqDeflection, tool.Eval(mid2d_l, mid3d_l));
                    }
                }
            }

            writable.Write(attached).Deflection(Math.Sqrt(sqDeflection));
        }
        writable.Attach(shape);
    }

    /// <summary>
    /// Fills <paramref name="lf"/> with the faces of <paramref name="sh"/> sorted by the complexity of their surfaces:
    /// planes, cylinders, cones, spheres, tori, others, then faces without a surface.
    /// </summary>
    public static void SortFaces(in Shape sh, NCollection_List<Shape> lf)
    {
        lf.Clear();
        NCollection_List<Shape> lTri = new(), lPlan = new(), lCyl = new(), lCon = new(), lSphere = new(), lTor = new(), lOther = new();
        var exp = new Explorer(sh, ShapeType.Face);
        Surface? s;

        for (; exp.More(); exp.Next())
        {
            Face f = ShapeCasts.Face(exp.Current());
            s = Tool.Surface(f, out _);
            if (s is not null)
            {
                if (s.GetType() == typeof(TrimmedSurface))
                {
                    s = ((TrimmedSurface)s).BasisSurface;
                }
                var AS = new SurfaceAdaptor(s);
                switch (AS.GetType())
                {
                    case global::Kerf.Geometry.SurfaceKind.Plane:
                        lPlan.Append(f);
                        break;
                    case SurfaceKind.Cylinder:
                        lCyl.Append(f);
                        break;
                    case SurfaceKind.Cone:
                        lCon.Append(f);
                        break;
                    case SurfaceKind.Sphere:
                        lSphere.Append(f);
                        break;
                    case SurfaceKind.Torus:
                        lTor.Append(f);
                        break;
                    default:
                        lOther.Append(f);
                        break;
                }
            }
            else
            {
                lTri.Append(f);
            }
        }
        lf.Append(lPlan);
        lf.Append(lCyl);
        lf.Append(lCon);
        lf.Append(lSphere);
        lf.Append(lTor);
        lf.Append(lOther);
        lf.Append(lTri);
    }

    /// <summary>The faces of <paramref name="sh"/> sorted as <see cref="SortFaces(in Shape, NCollection_List{Shape})"/> does.</summary>
    public static ISchematiqList<Shape> SortFaces(in Shape sh)
    {
        var lf = new NCollection_List<Shape>();
        SortFaces(sh, lf);
        return ToList(lf);
    }

    /// <summary>
    /// Fills <paramref name="lf"/> with the faces of <paramref name="sh"/> in the reverse order of complexity: faces
    /// without a surface, others, tori, spheres, cones, cylinders, planes.
    /// </summary>
    public static void ReverseSortFaces(in Shape sh, NCollection_List<Shape> lf)
    {
        lf.Clear();
        NCollection_List<Shape> lTri = new(), lPlan = new(), lCyl = new(), lCon = new(), lSphere = new(), lTor = new(), lOther = new();
        var exp = new Explorer(sh, ShapeType.Face);

        for (; exp.More(); exp.Next())
        {
            Face f = ShapeCasts.Face(exp.Current());
            Surface? s = Tool.Surface(f, out _);
            if (s is not null)
            {
                var AS = new SurfaceAdaptor(s);
                switch (AS.GetType())
                {
                    case global::Kerf.Geometry.SurfaceKind.Plane:
                        lPlan.Append(f);
                        break;
                    case SurfaceKind.Cylinder:
                        lCyl.Append(f);
                        break;
                    case SurfaceKind.Cone:
                        lCon.Append(f);
                        break;
                    case SurfaceKind.Sphere:
                        lSphere.Append(f);
                        break;
                    case SurfaceKind.Torus:
                        lTor.Append(f);
                        break;
                    default:
                        lOther.Append(f);
                        break;
                }
            }
            else
            {
                lTri.Append(f);
            }
        }
        lf.Append(lTri);
        lf.Append(lOther);
        lf.Append(lTor);
        lf.Append(lSphere);
        lf.Append(lCon);
        lf.Append(lCyl);
        lf.Append(lPlan);
    }

    /// <summary>The faces of <paramref name="sh"/> sorted as <see cref="ReverseSortFaces(in Shape, NCollection_List{Shape})"/> does.</summary>
    public static ISchematiqList<Shape> ReverseSortFaces(in Shape sh)
    {
        var lf = new NCollection_List<Shape>();
        ReverseSortFaces(sh, lf);
        return ToList(lf);
    }

    private static ISchematiqList<Shape> ToList(NCollection_List<Shape> list)
    {
        var array = new Shape[list.Extent()];
        int i = 0;
        for (var it = new NCollection_List<Shape>.Iterator(list); it.More(); it.Next())
        {
            array[i++] = it.Value();
        }
        return PackedList.Own(array);
    }

    /// <summary>
    /// The bounding sphere of the vertices of <paramref name="lv"/>: its centre and radius, for a new vertex
    /// covering them all. With fewer than two vertices the outputs are left unchanged, as in OCCT.
    /// </summary>
    public static void BoundingVertex(NCollection_List<Shape> lv, ref Point3D newCenter, ref double newTol)
    {
        int nb;
        //
        nb = lv.Extent();
        if (nb < 2)
        {
            return;
        }
        //
        else if (nb == 2)
        {
            int m, n;
            double dR, d, eps;
            Span<double> r = stackalloc double[2];
            var v = new Vertex[2];
            var p = new Point3D[2];
            //
            eps = RealEpsilon();
            for (m = 0; m < nb; ++m)
            {
                v[m] = (m == 0) ? ShapeCasts.Vertex(lv.First()) : ShapeCasts.Vertex(lv.Last());
                p[m] = Tool.Pnt(v[m]);
                r[m] = Tool.Tolerance(v[m]);
            }
            //
            m = 0; // max R
            n = 1; // min R
            if (r[0] < r[1])
            {
                m = 1;
                n = 0;
            }
            //
            dR = r[m] - r[n]; // dR >= 0.
            var vd = new Vector3D(p[m], p[n]);
            d = vd.Magnitude;
            //
            if (d <= dR || d < eps)
            {
                newCenter = p[m];
                newTol = r[m];
            }
            else
            {
                double rr;
                Xyz xyZr;
                var pr = new Point3D();
                //
                rr = 0.5 * (r[m] + r[n] + d);
                xyZr = 0.5 * (p[m].XYZ() + p[n].XYZ() - vd.XYZ() * (dR / d));
                pr = pr.WithXYZ(xyZr);
                //
                newCenter = pr;
                newTol = rr;
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
            var points = new Point3D[nb];
            NCollection_List<Shape>.Iterator it = new(lv);
            for (i = 0; it.More(); it.Next(), ++i)
            {
                Vertex vi = ShapeCasts.Vertex(it.Value());
                Point3D pi = Tool.Pnt(vi);
                points[i] = pi;
            }
            //
            Array.Sort(points, ComparePoints);
            //
            var xyz = new Xyz(0.0, 0.0, 0.0);
            for (i = 0; i < nb; ++i)
            {
                xyz += points[i].XYZ();
            }
            xyz.Divide((double)nb);
            //
            var p = new Point3D(xyz);
            //
            // compute the tolerance for the new vertex
            double ti, di, dmax;
            //
            dmax = -1.0;
            it.Initialize(lv);
            for (; it.More(); it.Next())
            {
                Vertex vi = ShapeCasts.Vertex(it.Value());
                Point3D pi = Tool.Pnt(vi);
                ti = Tool.Tolerance(vi);
                di = p.SquareDistanceTo(pi);
                di = Math.Sqrt(di);
                di = di + ti;
                if (di > dmax)
                {
                    dmax = di;
                }
            }
            //
            newCenter = p;
            newTol = dmax;
        }
    }

    // BRepLib_ComparePoints: lexicographic order of the coordinates (a strict weak order, as std::sort needs).
    private static int ComparePoints(Point3D p1, Point3D p2)
    {
        for (int i = 1; i <= 3; ++i)
        {
            if (p1.Coord(i) < p2.Coord(i))
            {
                return -1;
            }
            else if (p1.Coord(i) > p2.Coord(i))
            {
                return 1;
            }
        }
        return 0;
    }

    /// <summary>
    /// The face <paramref name="f"/> enlarged by <paramref name="extVal"/> in the chosen directions, frozen. An
    /// analytical surface is extended within its period and natural bounds; a bounded one by
    /// <c>GeomLib.ExtendSurfByLength</c>; another face is returned as it is.
    /// </summary>
    public static void ExtendFace(in Face f,
                                  double extVal,
                                  bool extUMin,
                                  bool extUMax,
                                  bool extVMin,
                                  bool extVMax,
                                  out Face fExtended)
    {
        InternalExtendFace(f, extVal, extUMin, extUMax, extVMin, extVMax, out fExtended);
        fExtended.TShape()?.FreezeGraph();
    }

    /// <summary><see cref="ExtendFace"/> without freezing the new face, for ported code that goes on building on it.</summary>
    internal static void InternalExtendFace(in Face f,
                                            double extVal,
                                            bool extUMin,
                                            bool extUMax,
                                            bool extVMin,
                                            bool extVMax,
                                            out Face fExtended)
    {
        // Get face bounds
        var bas = new FaceSurfaceAdaptor(f);
        double fuMin = bas.FirstUParameter(), fuMax = bas.LastUParameter(),
               fvMin = bas.FirstVParameter(), fvMax = bas.LastVParameter();
        double tol = Tool.Tolerance(f);

        // Surface to build the face
        Surface s;

        SurfaceKind type = bas.GetType();
        // treat analytical surfaces first
        if (type == global::Kerf.Geometry.SurfaceKind.Plane || type == SurfaceKind.Sphere || type == SurfaceKind.Cylinder
            || type == SurfaceKind.Torus || type == SurfaceKind.Cone)
        {
            // Get basis transformed basis surface
            Surface surf = bas.GeomSurfaceTransformed()!;

            // Get bounds of the basis surface
            surf.ParameterBounds(out double suMin, out double suMax, out double svMin, out double svMax);

            bool isUPeriodic = bas.IsUPeriodic;
            double uPeriod = isUPeriodic ? bas.UPeriod() : 0.0;
            if (isUPeriodic)
            {
                // Adjust face bounds to first period
                double delta = fuMax - fuMin;
                fuMin = Max(suMin, fuMin + uPeriod * Math.Ceiling((suMin - fuMin) / uPeriod));
                fuMax = fuMin + delta;
            }

            bool isVPeriodic = bas.IsVPeriodic;
            double vPeriod = isVPeriodic ? bas.VPeriod() : 0.0;
            if (isVPeriodic)
            {
                // Adjust face bounds to first period
                double delta = fvMax - fvMin;
                fvMin = Max(svMin, fvMin + vPeriod * Math.Ceiling((svMin - fvMin) / vPeriod));
                fvMax = fvMin + delta;
            }

            // Enlarge the face
            double uRes = 0.0, vRes = 0.0;
            if (extUMin || extUMax)
            {
                uRes = bas.UResolution(extVal);
            }
            if (extVMin || extVMax)
            {
                vRes = bas.VResolution(extVal);
            }

            if (extUMin)
            {
                fuMin = Max(suMin, fuMin - uRes);
            }
            if (extUMax)
            {
                fuMax = Min(isUPeriodic ? fuMin + uPeriod : suMax, fuMax + uRes);
            }
            if (extVMin)
            {
                fvMin = Max(svMin, fvMin - vRes);
            }
            if (extVMax)
            {
                fvMax = Min(isVPeriodic ? fvMin + vPeriod : svMax, fvMax + vRes);
            }

            // Check if the periodic surface should become closed.
            // In this case, use the basis surface with basis bounds.
            double eps = Kerf.Engine.Foundation.Precision.PConfusion();
            if (isUPeriodic && Math.Abs(fuMax - fuMin - uPeriod) < eps)
            {
                fuMin = suMin;
                fuMax = suMax;
            }
            if (isVPeriodic && Math.Abs(fvMax - fvMin - vPeriod) < eps)
            {
                fvMin = svMin;
                fvMax = svMax;
            }

            s = surf;
        }
        else
        {
            // General case

            // The face's surface, placed (BRep_Tool::Surface(F) transforms it by the face's location: decision 1477).
            // OCCT extends that surface handle in place below, changing the input face and every face sharing the
            // surface (occt-issues.md); the port's GeomLib.ExtendSurfByLength replaces aSB by an extended copy, so the
            // input is unchanged and the extended face is OCCT's.
            if (Tool.Surface(f) is not BoundedSurface sb)
            {
                fExtended = f;
                return;
            }

            // Get surfaces bounds
            sb.ParameterBounds(out double suMin, out double suMax, out double svMin, out double svMax);

            bool isUClosed = sb.IsUClosed;
            bool isVClosed = sb.IsVClosed;

            // Check if the extension in necessary directions is done
            bool isExtUMin = false, isExtUMax = false, isExtVMin = false, isExtVMax = false;

            // UMin
            if (extUMin && !isUClosed && !Kerf.Engine.Foundation.Precision.IsInfinite(suMin))
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref sb, extVal, 1, true, false);
                isExtUMin = true;
            }
            // UMax
            if (extUMax && !isUClosed && !Kerf.Engine.Foundation.Precision.IsInfinite(suMax))
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref sb, extVal, 1, true, true);
                isExtUMax = true;
            }
            // VMin
            if (extVMin && !isVClosed && !Kerf.Engine.Foundation.Precision.IsInfinite(svMin)) // OCCT tested aSVMax (decision 1478)
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref sb, extVal, 1, false, false);
                isExtVMin = true;
            }
            // VMax
            if (extVMax && !isVClosed && !Kerf.Engine.Foundation.Precision.IsInfinite(svMax))
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref sb, extVal, 1, false, true);
                isExtVMax = true;
            }

            s = sb;

            // Get new bounds
            s.ParameterBounds(out suMin, out suMax, out svMin, out svMax);
            if (isExtUMin)
            {
                fuMin = suMin;
            }
            if (isExtUMax)
            {
                fuMax = suMax;
            }
            if (isExtVMin)
            {
                fvMin = svMin;
            }
            if (isExtVMax)
            {
                fvMax = svMax;
            }
        }

        var mf = new LibraryMakeFace(s, fuMin, fuMax, fvMin, fvMax, tol);
        fExtended = ShapeCasts.Face(mf.InternalShape());
        if (f.Orientation == Reversed)
        {
            fExtended.Reverse();
        }
    }
}
