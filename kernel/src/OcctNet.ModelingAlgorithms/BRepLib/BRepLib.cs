// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// General utilities for BRep: the default precision and plane of the makers, the computation of missing 3D curves
/// and pcurves, SameParameter and SameRange, tolerances, regularity, orientation of closed solids.
/// </summary>
public static partial class BRepLib
{
    // OCCT's file statics. Process-wide as in OCCT; their setters are internal, so callers always see the defaults.
    private static double thePrecision = Occt.Precision.Confusion();
    private static Geom_Plane? thePlane;

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
    internal static void Plane(Geom_Plane P)
    {
        P.Freeze();
        thePlane = P;
    }

    /// <summary>Returns the current plane of the 2D makers (frozen).</summary>
    public static Geom_Plane Plane()
    {
        if (thePlane is null)
        {
            var aPlane = new Geom_Plane(new gp_Ax3(gp.XOY()));
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
    public static bool CheckSameRange(in TopoDS_Edge AnEdge, double Confusion = 1.0e-12)
    {
        double Tolerance = Confusion;
        bool IsSameRange = true, first_time_in = true;

        // OCCT.NET: frozen. OCCT iterates ChangeCurves() to read them; Curves() is the same list.
        var an_Iterator = new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)AnEdge.TShape()!).Curves());

        double first, last;
        double current_first = 0.0, current_last = 0.0;
        BRep_GCurve? geometric_representation_ptr;

        while (IsSameRange && an_Iterator.More())
        {
            geometric_representation_ptr = an_Iterator.Value() as BRep_GCurve;
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
    internal static void SameRange(in TopoDS_Edge AnEdge, double Tolerance = 1.0e-5)
    {
        var an_Iterator = new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)AnEdge.TShape()!).ChangeCurves());

        Geom2d_Curve? Curve2dPtr = null, Curve2dPtr2 = null, NewCurve2dPtr = null, NewCurve2dPtr2 = null;

        bool first_time_in = true, has_curve, has_closed_curve;
        BRep_GCurve? geometric_representation_ptr;
        double first, current_first, last, current_last;

        Geom_Curve? C = BRep_Tool.Curve(AnEdge, out _, out current_first, out current_last);
        if (C is not null)
        {
            first_time_in = false;
        }

        while (an_Iterator.More())
        {
            geometric_representation_ptr = an_Iterator.Value() as BRep_GCurve;
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
                if (geometric_representation_ptr.IsCurveOnClosedSurface())
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

                    if (Math.Abs(first - current_first) > Occt.Precision.Confusion()
                        || Math.Abs(last - current_last) > Occt.Precision.Confusion())
                    {
                        if (has_curve)
                        {
                            GeomLib.SameRange(Tolerance,
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
                            GeomLib.SameRange(Tolerance,
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
        var B = new BRep_Builder();
        B.Range(AnEdge, current_first, current_last);

        B.SameRange(AnEdge, true);
    }

    //=======================================================================
    // function : EvaluateMaxSegment
    // purpose  : return MaxSegment to pass in approximation, if MaxSegment==0 provided
    //=======================================================================

    private static int evaluateMaxSegment(int aMaxSegment, Adaptor3d_CurveOnSurface aCurveOnSurface)
    {
        if (aMaxSegment != 0)
        {
            return aMaxSegment;
        }

        Adaptor3d_Surface aSurf = aCurveOnSurface.GetSurface()!;
        Adaptor2d_Curve2d aCurv2d = aCurveOnSurface.GetCurve()!;

        double aNbSKnots = 0, aNbC2dKnots = 0;

        if (aSurf.GetType() == GeomAbs_BSplineSurface)
        {
            Geom_BSplineSurface aBSpline = aSurf.BSpline();
            aNbSKnots = Max(aBSpline.NbUKnots(), aBSpline.NbVKnots());
        }
        if (aCurv2d.GetType() == GeomAbs_BSplineCurve)
        {
            aNbC2dKnots = aCurv2d.NbKnots();
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
    internal static bool BuildCurve3d(in TopoDS_Edge AnEdge,
                                      double Tolerance = 1.0e-5,
                                      GeomAbs_Shape Continuity = GeomAbs_C1,
                                      int MaxDegree = 14,
                                      int MaxSegment = 0)
    {
        int ii, jj;

        TopLoc_Location LocalLoc = new(), LC;
        var L = new TopLoc_Location[] { new(), new() };
        double f, l, fc, lc, tolerance, max_deviation;
        var first = new double[2];
        var last = new double[2];
        Geom2d_Curve? Curve2dPtr = null;
        var Curve2dArray = new Geom2d_Curve?[2];
        Geom_Surface? SurfacePtr = null;
        var SurfaceArray = new Geom_Surface?[2];

        bool not_done;
        // if the edge has a 3d curve returns true

        Geom_Curve? C = BRep_Tool.Curve(AnEdge, out LocalLoc, out f, out l);
        if (C is not null)
        {
            return true;
        }
        //
        // this should not exists but UpdateEdge makes funny things
        // if the edge is not same range
        //
        if (!CheckSameRange(AnEdge, Occt.Precision.Confusion()))
        {
            SameRange(AnEdge, Tolerance);
        }

        // search a curve on a plane
        Geom_Surface? S = null;
        Geom2d_Curve? PC = null;
        int i = 0;
        Geom_Plane? P = null;
        not_done = true;

        while (not_done)
        {
            i++;
            BRep_Tool.CurveOnSurface(AnEdge, ref PC, ref S, ref LocalLoc, ref f, ref l, i);
            if (S is not Geom_RectangularTrimmedSurface RT)
            {
                P = S as Geom_Plane;
            }
            else
            {
                P = RT.BasisSurface() as Geom_Plane;
            }
            not_done = S is not null && P is null;
        }
        if (P is not null)
        {
            // compute the 3d curve
            gp_Ax2 axes = P.Position().Ax2();
            Geom_Curve? C3d = GeomLib.To3d(axes, PC!);
            if (C3d is null)
            {
                return false;
            }
            // update the edge
            double First, Last;

            var B = new BRep_Builder();
            B.UpdateEdge(AnEdge, C3d, LocalLoc, 0.0e0);
            LC = new TopLoc_Location();
            BRep_Tool.Range(AnEdge, S, LC, out First, out Last);
            B.Range(AnEdge, First, Last); // Do not forget 3D range.(PRO6412)
        }
        else
        {
            //
            // compute the 3d curve using existing surface
            //
            fc = f;
            lc = l;
            if (!BRep_Tool.Degenerated(AnEdge))
            {
                jj = 0;
                // BRep_Tool::CurveOnSurface expects Index >= 1, so start from 1.
                for (ii = 1; ii < 3; ii++)
                {
                    BRep_Tool.CurveOnSurface(AnEdge, ref Curve2dPtr, ref SurfacePtr, ref LocalLoc, ref fc, ref lc, ii);

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
                var AnAdaptor3dCurve2dPtr = new Geom2dAdaptor_Curve(Curve2dPtr!, f, l);
                var AnAdaptor3dSurfacePtr = new GeomAdaptor_Surface(SurfacePtr!);
                var CurveOnSurface = new Adaptor3d_CurveOnSurface(AnAdaptor3dCurve2dPtr, AnAdaptor3dSurfacePtr);

                Geom_Curve? NewCurvePtr = null;

                GeomLib.BuildCurve3d(Tolerance,
                                     CurveOnSurface,
                                     f,
                                     l,
                                     ref NewCurvePtr,
                                     out max_deviation,
                                     out _,
                                     Continuity,
                                     MaxDegree,
                                     evaluateMaxSegment(MaxSegment, CurveOnSurface));
                var B = new BRep_Builder();
                tolerance = BRep_Tool.Tolerance(AnEdge);
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
    internal static bool BuildCurves3d(in TopoDS_Shape S)

    {
        return BuildCurves3d(S, 1.0e-5);
    }

    //=================================================================================================

    /// <summary>
    /// Computes the missing 3D curves of all the edges of <paramref name="S"/>; false if one of the computations
    /// failed. Writes into the edges.
    /// </summary>
    internal static bool BuildCurves3d(in TopoDS_Shape S,
                                       double Tolerance,
                                       GeomAbs_Shape Continuity = GeomAbs_C1,
                                       int MaxDegree = 14,
                                       int MaxSegment = 0)
    {
        bool boolean_value, ok = true;
        var a_counter = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var ex = new TopExp_Explorer(S, TopAbs_EDGE);

        while (ex.More())
        {
            if (a_counter.Add(ex.Current()))
            {
                boolean_value = BuildCurve3d(TopoDS.Edge(ex.Current()), Tolerance, Continuity, MaxDegree, MaxSegment);
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
    internal static bool UpdateEdgeTol(in TopoDS_Edge AnEdge,
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
        BRep_GCurve? geometric_representation_ptr;

        if (BRep_Tool.Degenerated(AnEdge))
        {
            return false;
        }
        coded_edge_tolerance = BRep_Tool.Tolerance(AnEdge);
        if (coded_edge_tolerance > MaxToleranceToCheck)
        {
            return false;
        }

        BRep_TEdge TE = (BRep_TEdge)AnEdge.TShape()!;
        NCollection_List<BRep_CurveRepresentation> list_curve_rep = TE.ChangeCurves();
        NCollection_List<BRep_CurveRepresentation>.Iterator an_iterator = new(list_curve_rep),
                                                            second_iterator = new(list_curve_rep);
        Geom2d_Curve? curve2d_ptr = null;
        Geom_Surface? surface_ptr = null;
        TopLoc_Location local_location;
        var a_sampler = new GCPnts_QuasiUniformDeflection();
        var geom_reference_curve = new GeomAdaptor_Curve();
        var curve_on_surface_reference = new Adaptor3d_CurveOnSurface();
        Geom_Curve? C = BRep_Tool.Curve(AnEdge, out local_location, out current_first, out current_last);
        curve_on_surface_index = -1;
        if (C is not null)
        {
            if (!local_location.IsIdentity())
            {
                C = (Geom_Curve)C.Transformed(local_location.Transformation());
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
                geometric_representation_ptr = second_iterator.Value() as BRep_GCurve;
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
                    if (!local_location.IsIdentity())
                    {
                        surface_ptr = (Geom_Surface)geometric_representation_ptr.Surface()!.Transformed(local_location.Transformation());
                    }
                    else
                    {
                        surface_ptr = geometric_representation_ptr.Surface();
                    }
                    not_done = false;
                }
                curve_on_surface_index += 1; // OCCT (see above)
            }
            var AnAdaptor3dCurve2dPtr = new Geom2dAdaptor_Curve(curve2d_ptr!);
            var AnAdaptor3dSurfacePtr = new GeomAdaptor_Surface(surface_ptr!);
            curve_on_surface_reference.Load(AnAdaptor3dCurve2dPtr, AnAdaptor3dSurfacePtr);
            a_sampler.Initialize(curve_on_surface_reference,
                                 MinToleranceRequested * factor,
                                 current_first,
                                 current_last);
        }
        var sampling_parameters = new NCollection_Array1<double>(1, a_sampler.NbPoints());
        for (ii = 1; ii <= a_sampler.NbPoints(); ii++)
        {
            sampling_parameters[ii] = a_sampler.Parameter(ii);
        }
        if (a_sampler.NbPoints() < min_sampling_points)
        {
            GeomLib.DensifyArray1OfReal(min_sampling_points, sampling_parameters, out parameters_ptr);
        }
        else if (a_sampler.NbPoints() > max_sampling_points)
        {
            GeomLib.RemovePointsFromArray(max_sampling_points, sampling_parameters, out parameters_ptr);
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
            geometric_representation_ptr = second_iterator.Value() as BRep_GCurve;
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
                if (geometric_representation_ptr.IsCurveOnClosedSurface())
                {
                    curve2d_ptr = geometric_representation_ptr.PCurve2();
                    has_closed_curve = true;
                }

                if (has_curve || has_closed_curve)
                {
                    if (!local_location.IsIdentity())
                    {
                        surface_ptr = (Geom_Surface)geometric_representation_ptr.Surface()!.Transformed(local_location.Transformation());
                    }
                    else
                    {
                        surface_ptr = geometric_representation_ptr.Surface();
                    }
                    var an_adaptor_curve2d_ptr = new Geom2dAdaptor_Curve(curve2d_ptr!);
                    var an_adaptor_surface_ptr = new GeomAdaptor_Surface(surface_ptr!);
                    var a_curve_on_surface = new Adaptor3d_CurveOnSurface(an_adaptor_curve2d_ptr, an_adaptor_surface_ptr);

                    // OCCT: an edge without 3D curve flagged SameParameter is measured against geom_reference_curve, which
                    // was never loaded; OCCT dereferences its null curve, the port throws a NullReferenceException.
                    if (BRep_Tool.SameParameter(AnEdge))
                    {

                        GeomLib.EvalMaxParametricDistance(a_curve_on_surface,
                                                          geom_reference_curve,
                                                          MinToleranceRequested,
                                                          parameters_ptr.Array1(),
                                                          out max_distance);
                    }
                    else if (geom_reference_curve_flag != 0)
                    {
                        GeomLib.EvalMaxDistanceAlongParameter(a_curve_on_surface,
                                                              geom_reference_curve,
                                                              MinToleranceRequested,
                                                              parameters_ptr.Array1(),
                                                              out max_distance);
                    }
                    else
                    {

                        GeomLib.EvalMaxDistanceAlongParameter(a_curve_on_surface,
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
    internal static bool UpdateEdgeTolerance(in TopoDS_Shape S,
                                             double MinToleranceRequested,
                                             double MaxToleranceToCheck)
    {
        var ex = new TopExp_Explorer(S, TopAbs_EDGE);
        var a_counter = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        bool return_status = false, local_flag;

        while (ex.More())
        {
            if (a_counter.Add(ex.Current()))
            {
                local_flag = UpdateEdgeTol(TopoDS.Edge(ex.Current()),
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

    private static void GetEdgeTol(in TopoDS_Edge theEdge, in TopoDS_Face theFace, ref double theEdTol)
    {
        Geom_Surface? S = BRep_Tool.Surface(theFace, out TopLoc_Location L);
        TopLoc_Location l = L.Predivided(theEdge.Location());

        BRep_TEdge TE = (BRep_TEdge)theEdge.TShape()!;
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface(S, l))
            {
                return;
            }
            itcr.Next();
        }

        Geom_Plane? GP;
        Geom_RectangularTrimmedSurface? GRTS;
        GRTS = S as Geom_RectangularTrimmedSurface;
        if (GRTS is not null)
        {
            GP = GRTS.BasisSurface() as Geom_Plane;
        }
        else
        {
            GP = S as Geom_Plane;
        }

        var HC = new GeomAdaptor_Curve();
        var HS = new GeomAdaptor_Surface();

        TopLoc_Location LC;
        double First, Last;
        HC.Load(BRep_Tool.Curve(theEdge, out LC, out First, out Last)!);
        LC = L.Predivided(LC);

        if (!LC.IsIdentity())
        {
            GP = (Geom_Plane)GP!.Transformed(LC.Transformation());
        }
        GeomAdaptor_Surface GAS = HS;
        GAS.Load(GP!);

        var Proj = new ProjLib_ProjectedCurve(HS, HC);
        Geom2d_Curve pc = Geom2dAdaptor.MakeCurve(Proj);

        gp_Pln pln = GAS.Plane();
        double d2 = 0.0;
        int nn = 23;
        double unsurnn = 1.0 / nn;
        for (int i = 0; i <= nn; i++)
        {
            double t = unsurnn * i;
            double u = First * (1.0 - t) + Last * t;
            gp_Pnt Pc3d = HC.Value(u);
            gp_Pnt2d p2d = pc.Value(u);
            gp_Pnt Pcons = ElSLib.Value(p2d.X(), p2d.Y(), pln);
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
    private static void UpdTolMap(in TopoDS_Shape theSh,
                                  double theNewTol,
                                  NCollection_DataMap<TopoDS_Shape, double> theShToTol)
    {
        TopAbs_ShapeEnum aSt = theSh.ShapeType();
        double aShTol;
        if (aSt == TopAbs_VERTEX)
        {
            aShTol = BRep_Tool.Tolerance(TopoDS.Vertex(theSh));
        }
        else if (aSt == TopAbs_EDGE)
        {
            aShTol = BRep_Tool.Tolerance(TopoDS.Edge(theSh));
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
    private static void UpdShTol(NCollection_DataMap<TopoDS_Shape, double> theShToTol,
                                 bool IsMutableInput,
                                 BRepTools_ReShape theReshaper,
                                 bool theVForceUpdate)
    {
        var aB = new BRep_Builder();
        foreach (KeyValuePair<TopoDS_Shape, double> SHToTolit in theShToTol)
        {
            TopoDS_Shape aSh = SHToTolit.Key;
            double aTol = SHToTolit.Value;
            //
            TopoDS_Shape aNsh;
            // The reshaper's internal form: its replacements are this algorithm's own, still unfrozen (design 4.2).
            TopoDS_Shape aVsh = theReshaper.InternalValue(aSh);
            bool UseOldSh = IsMutableInput || theReshaper.IsNewShape(aSh) || !aVsh.IsSame(aSh);
            if (UseOldSh)
            {
                aNsh = aVsh;
            }
            else
            {
                aNsh = aSh.EmptyCopied();
                // add subshapes from the original shape
                var sit = new TopoDS_Iterator(aSh);
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
                case TopAbs_FACE:
                    {
                        aB.UpdateFace(TopoDS.Face(aNsh), aTol);
                        break;
                    }
                case TopAbs_EDGE:
                    {
                        aB.UpdateEdge(TopoDS.Edge(aNsh), aTol);
                        break;
                    }
                case TopAbs_VERTEX:
                    {
                        BRep_TVertex aTV = (BRep_TVertex)aNsh.TShape()!;
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

    private static void InternalSameParameter(in TopoDS_Shape theSh,
                                              BRepTools_ReShape theReshaper,
                                              double theTol,
                                              bool IsForced,
                                              bool IsMutableInput)
    {
        var ex = new TopExp_Explorer(theSh, TopAbs_EDGE);
        var Done = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aB = new BRep_Builder();
        var aShToTol = new NCollection_DataMap<TopoDS_Shape, double>(TopTools_ShapeMapHasher.Instance);

        while (ex.More())
        {
            TopoDS_Edge aCE = TopoDS.Edge(ex.Current());
            if (Done.Add(aCE))
            {
                TopoDS_Edge aNE = TopoDS.Edge(theReshaper.InternalValue(aCE));
                bool UseOldEdge = IsMutableInput || theReshaper.IsNewShape(aCE) || !aNE.IsSame(aCE);
                if (IsForced && (BRep_Tool.SameRange(aCE) || BRep_Tool.SameParameter(aCE)))
                {
                    if (!UseOldEdge)
                    {
                        aNE = TopoDS.Edge(aCE.EmptyCopied());
                        var sit = new TopoDS_Iterator(aCE);
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
                TopoDS_Edge aResEdge = SameParameter(aNE, theTol, ref aNewTol, UseOldEdge);
                if (!UseOldEdge && !aResEdge.IsNull())
                {
                    // NE have been empty-copied
                    theReshaper.Replace(aNE, aResEdge);
                }
                if (aNewTol > 0)
                {
                    TopExp.Vertices(aCE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                    if (!aV1.IsNull())
                    {
                        UpdTolMap(aV1, aNewTol, aShToTol);
                    }
                    if (!aV2.IsNull())
                    {
                        UpdTolMap(aV2, aNewTol, aShToTol);
                    }
                }
            }
            ex.Next();
        }

        Done.Clear();
        var BS = new BRepAdaptor_Surface();
        for (ex.Init(theSh, TopAbs_FACE); ex.More(); ex.Next())
        {
            TopoDS_Face curface = TopoDS.Face(ex.Current());
            if (!Done.Add(curface))
            {
                continue;
            }
            BS.Initialize(curface);
            if (BS.GetType() != GeomAbs_Plane)
            {
                continue;
            }
            var ex2 = new TopExp_Explorer();
            for (ex2.Init(curface, TopAbs_EDGE); ex2.More(); ex2.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(ex2.Current());
                if (BRep_Tool.Degenerated(E))
                {
                    continue;
                }

                TopoDS_Shape aNe = theReshaper.InternalValue(E);
                double aNewEtol = -1;
                GetEdgeTol(TopoDS.Edge(aNe), curface, ref aNewEtol);
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
    internal static void SameParameter(in TopoDS_Shape S, double Tolerance = 1.0e-5, bool forced = false)
    {
        var reshaper = new BRepTools_ReShape();
        InternalSameParameter(S, reshaper, Tolerance, forced, true);
    }

    //=================================================================================================

    /// <summary>
    /// <see cref="SameParameter(in TopoDS_Shape, double, bool)"/> without changing <paramref name="S"/>: the
    /// changed edges, vertices and faces are copies recorded as replacements in <paramref name="theReshaper"/>, whose
    /// <see cref="BRepTools_ReShape.Apply"/> gives the result.
    /// </summary>
    public static void SameParameter(in TopoDS_Shape S,
                                     BRepTools_ReShape theReshaper,
                                     double Tolerance = 1.0e-5,
                                     bool forced = false)
    {
        InternalSameParameter(S, theReshaper, Tolerance, forced, false);
    }

    //=================================================================================================

    private static bool EvalTol(Geom2d_Curve pc,
                                Geom_Surface s,
                                GeomAdaptor_Curve gac,
                                double tol,
                                out double tolbail)
    {
        int ok = 0;
        double f = gac.FirstParameter();
        double l = gac.LastParameter();
        var Projector = new Extrema_LocateExtPC();
        Projector.Initialize(gac, f, l, tol);
        double u, v;
        gp_Pnt p;
        tolbail = tol;
        for (int i = 1; i <= 5; i++)
        {
            double t = i / 6.0;
            t = (1.0 - t) * f + t * l;
            pc.Value(t).Coord(out u, out v);
            p = s.Value(u, v);
            Projector.Perform(p, t);
            if (Projector.IsDone())
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

    private static double ComputeTol(Adaptor3d_Curve c3d,
                                     Adaptor2d_Curve2d c2d,
                                     Adaptor3d_Surface surf,
                                     int nbp)

    {

        var dist = new NCollection_Array1<double>(1, nbp + 10);
        dist.Init(-1.0);

        // Adaptor3d_CurveOnSurface  cons(c2d,surf);
        double uf = surf.FirstUParameter(), ul = surf.LastUParameter(), vf = surf.FirstVParameter(),
               vl = surf.LastVParameter();
        double du = 0.01 * (ul - uf), dv = 0.01 * (vl - vf);
        bool isUPeriodic = surf.IsUPeriodic(), isVPeriodic = surf.IsVPeriodic();
        double DSdu = 1.0 / surf.UResolution(1.0), DSdv = 1.0 / surf.VResolution(1.0);
        double d2 = 0.0;
        double first = c3d.FirstParameter();
        double last = c3d.LastParameter();
        double dapp = -1.0;
        for (int i = 0; i <= nbp; ++i)
        {
            double t = IntToReal(i) / IntToReal(nbp);
            double u = first * (1.0 - t) + last * t;
            gp_Pnt Pc3d = c3d.Value(u);
            gp_Pnt2d Puv = c2d.Value(u);
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
            gp_Pnt Pcons = surf.Value(Puv.X(), Puv.Y());
            if (Occt.Precision.IsInfinite(Pcons.X()) || Occt.Precision.IsInfinite(Pcons.Y())
                || Occt.Precision.IsInfinite(Pcons.Z()))
            {
                d2 = Occt.Precision.Infinite();
                break;
            }
            double temp = Pc3d.SquareDistance(Pcons);

            dist[i + 1] = temp;

            d2 = Max(d2, temp);
        }

        if (Occt.Precision.IsInfinite(d2))
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
    private static void GetCurve3d(in TopoDS_Edge theEdge,
                                   ref Geom_Curve? theC3d,
                                   ref double theF3d,
                                   ref double theL3d,
                                   ref TopLoc_Location theLoc3d,
                                   out NCollection_List<BRep_CurveRepresentation> theCList)
    {
        BRep_TEdge aTE = (BRep_TEdge)theEdge.TShape()!;
        theCList = aTE.Curves();
        var anIt = new NCollection_List<BRep_CurveRepresentation>.Iterator(theCList);
        bool NotDone = true;
        while (NotDone && anIt.More())
        {
            if (anIt.Value() is BRep_GCurve GCurve && GCurve.IsCurve3D())
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

    private static void UpdateVTol(in TopoDS_Vertex theV1, in TopoDS_Vertex theV2, double theTol)
    {
        var aB = new BRep_Builder();
        if (!theV1.IsNull())
        {
            aB.UpdateVertex(theV1, theTol);
        }
        if (!theV2.IsNull())
        {
            aB.UpdateVertex(theV2, theTol);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Computes new pcurves for the edge so that they have the same parameter as its 3D curve, unless the edge is
    /// flagged SameParameter, and enlarges its vertices. Writes into the edge.
    /// </summary>
    internal static void SameParameter(in TopoDS_Edge theEdge, double Tolerance = 1.0e-5)
    {
        double aNewTol = -1;
        SameParameter(theEdge, Tolerance, ref aNewTol, true);
        if (aNewTol > 0)
        {
            TopExp.Vertices(theEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
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
    internal static TopoDS_Edge SameParameter(in TopoDS_Edge theEdge,
                                              double theTolerance,
                                              ref double theNewTol,
                                              bool IsUseOldEdge)
    {
        if (BRep_Tool.SameParameter(theEdge))
        {
            return new TopoDS_Edge();
        }
        double f3d = 0.0, l3d = 0.0;
        TopLoc_Location L3d = new();
        Geom_Curve? C3d = null;
        NCollection_List<BRep_CurveRepresentation> CList;
        GetCurve3d(theEdge, ref C3d, ref f3d, ref l3d, ref L3d, out CList);
        if (C3d is null)
        {
            return new TopoDS_Edge();
        }

        var B = new BRep_Builder();
        TopoDS_Edge aNE;
        BRep_TEdge aNTE;
        if (IsUseOldEdge)
        {
            aNE = theEdge;
            aNTE = (BRep_TEdge)theEdge.TShape()!;
        }
        else
        {
            aNE = TopoDS.Edge(theEdge.EmptyCopied()); //will be modified a little bit later, so copy anyway
            GetCurve3d(aNE, ref C3d, ref f3d, ref l3d, ref L3d, out CList); //C3d pointer and CList will be differ after copying
            aNTE = (BRep_TEdge)aNE.TShape()!;
            var sit = new TopoDS_Iterator(theEdge);
            for (; sit.More(); sit.Next())
            { // add vertices from old edge to the new ones
                B.Add(aNE, sit.Value());
            }
        }

        var It = new NCollection_List<BRep_CurveRepresentation>.Iterator(CList);

        const int NCONTROL = 22;

        var HC = new GeomAdaptor_Curve();
        var HC2d = new Geom2dAdaptor_Curve();
        var HS = new GeomAdaptor_Surface();
        GeomAdaptor_Curve GAC = HC;
        Geom2dAdaptor_Curve GAC2d = HC2d;
        GeomAdaptor_Surface GAS = HS;

        // modified by NIZHNY-OCC486  Tue Aug 27 17:15:13 2002 :
        bool m_TrimmedPeriodical = false;
        if (C3d!.GetType() == typeof(Geom_TrimmedCurve))
        {
            Geom_Curve gtC = ((Geom_TrimmedCurve)C3d).BasisCurve();
            m_TrimmedPeriodical = gtC.IsPeriodic();
        }
        // modified by NIZHNY-OCC486  Tue Aug 27 17:15:17 2002 .

        if (!C3d.IsPeriodic())
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
        if (!L3d.IsIdentity())
        {
            C3d = (Geom_Curve)C3d.Transformed(L3d.Transformation());
        }
        GAC.Load(C3d, f3d, l3d);

        double Prec_C3d = BRepCheck.PrecCurve(GAC);

        bool IsSameP = true;
        double maxdist = 0.0;

        //  Modified by skv - Thu Jun  3 12:39:19 2004 OCC5898 Begin
        double anEdgeTol = BRep_Tool.Tolerance(aNE);
        //  Modified by skv - Thu Jun  3 12:39:20 2004 OCC5898 End
        bool SameRange = BRep_Tool.SameRange(aNE);
        bool YaPCu = false;
        const double BigError = 1.0e10;
        It.Initialize(CList);

        while (It.More())
        {

            bool isANA = false;
            bool isBSP = false;
            BRep_GCurve? GCurve = It.Value() as BRep_GCurve;
            var PC = new Geom2d_Curve?[2];
            Geom_Surface? S;
            if (GCurve is not null && GCurve.IsCurveOnSurface())
            {
                YaPCu = true;
                PC[0] = GCurve.PCurve();
                TopLoc_Location PCLoc = GCurve.Location();
                S = GCurve.Surface();
                if (!PCLoc.IsIdentity())
                {
                    S = (Geom_Surface)S!.Transformed(PCLoc.Transformation());
                }

                GAS.Load(S!);
                if (GCurve.IsCurveOnClosedSurface())
                {
                    PC[1] = GCurve.PCurve2();
                }

                // Eval tol2d to compute SameRange
                double TolSameRange = Max(GAC.Resolution(theTolerance), Occt.Precision.PConfusion());
                for (int i = 0; i < 2; i++)
                {
                    Geom2d_Curve? curPC = PC[i];
                    bool updatepc = false;
                    if (curPC is null)
                    {
                        break;
                    }
                    if (!SameRange)
                    {
                        GeomLib.SameRange(TolSameRange, PC[i], GCurve.First(), GCurve.Last(), f3d, l3d, ref curPC);

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

                    if (GAC2d.GetType() == GeomAbs_BSplineCurve && GAC2d.Continuity() == GeomAbs_C0)
                    {
                        double UResol = GAS.UResolution(theTolerance);
                        double VResol = GAS.VResolution(theTolerance);
                        double TolConf2d = Min(UResol, VResol);
                        TolConf2d = Max(TolConf2d, Occt.Precision.PConfusion());
                        Geom2d_BSplineCurve bs2d = GAC2d.BSpline();
                        Geom2d_BSplineCurve bs2dsov = bs2d;
                        double fC0 = bs2d.FirstParameter(), lC0 = bs2d.LastParameter();
                        bool repar = true;
                        gp_Pnt2d OriginPoint;
                        bs2d.D0(fC0, out OriginPoint);
                        Geom2dConvert.C0BSplineToC1BSplineCurve(ref bs2d, TolConf2d);
                        isBSP = true;

                        if (bs2d.IsPeriodic())
                        { // -------- IFV, Jan 2000
                            bs2d.D0(bs2d.FirstParameter(), out gp_Pnt2d NewOriginPoint);
                            if (Math.Abs(OriginPoint.X() - NewOriginPoint.X()) > Occt.Precision.PConfusion()
                                || Math.Abs(OriginPoint.Y() - NewOriginPoint.Y()) > Occt.Precision.PConfusion())
                            {

                                PackedList<double> Knotbs2d = bs2d.Knots();

                                for (int Index = 1; Index <= bs2d.NbKnots(); Index++)
                                {
                                    bs2d.D0(Knotbs2d[Index - 1], out NewOriginPoint);
                                    if (Math.Abs(OriginPoint.X() - NewOriginPoint.X()) > Occt.Precision.PConfusion()
                                        || Math.Abs(OriginPoint.Y() - NewOriginPoint.Y()) > Occt.Precision.PConfusion())
                                    {
                                        continue;
                                    }

                                    bs2d.SetOrigin(Index);
                                    break;
                                }
                            }
                        }

                        if (bs2d.Continuity() == GeomAbs_C0)
                        {
                            if (EvalTol(curPC!, S!, GAC, theTolerance, out double tolbail))
                            {
                                bs2d = bs2dsov;
                                double UResbail = GAS.UResolution(tolbail);
                                double VResbail = GAS.VResolution(tolbail);
                                double Tol2dbail = Min(UResbail, VResbail);
                                bs2d.D0(bs2d.FirstParameter(), out OriginPoint);

                                int nbp = bs2d.NbPoles();
                                gp_Pnt2d p = bs2d.Pole(1), p1;
                                double d = Occt.Precision.Infinite();
                                for (int ip = 2; ip <= nbp; ip++)
                                {
                                    p1 = bs2d.Pole(ip);
                                    d = Min(d, p.SquareDistance(p1));
                                    p = p1;
                                }
                                d = Math.Sqrt(d) * 0.1;

                                Tol2dbail = Max(Min(Tol2dbail, d), TolConf2d);

                                Geom2dConvert.C0BSplineToC1BSplineCurve(ref bs2d, Tol2dbail);

                                if (bs2d.IsPeriodic())
                                { // -------- IFV, Jan 2000
                                    bs2d.D0(bs2d.FirstParameter(), out gp_Pnt2d NewOriginPoint);
                                    if (Math.Abs(OriginPoint.X() - NewOriginPoint.X()) > Occt.Precision.PConfusion()
                                        || Math.Abs(OriginPoint.Y() - NewOriginPoint.Y()) > Occt.Precision.PConfusion())
                                    {

                                        PackedList<double> Knotbs2d = bs2d.Knots();

                                        for (int Index = 1; Index <= bs2d.NbKnots(); Index++)
                                        {
                                            bs2d.D0(Knotbs2d[Index - 1], out NewOriginPoint);
                                            if (Math.Abs(OriginPoint.X() - NewOriginPoint.X()) > Occt.Precision.PConfusion()
                                                || Math.Abs(OriginPoint.Y() - NewOriginPoint.Y()) > Occt.Precision.PConfusion())
                                            {
                                                continue;
                                            }

                                            bs2d.SetOrigin(Index);
                                            break;
                                        }
                                    }
                                }

                                if (bs2d.Continuity() == GeomAbs_C0)
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
                                var Knots = new NCollection_Array1<double>(1, bs2d.NbKnots());
                                bs2d.Knots(Knots);
                                //	    BSplCLib::Reparametrize(f3d,l3d,Knots);
                                BSplCLib.Reparametrize(fC0, lC0, Knots);
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
                            GeomAbs_Shape cont = bs2d.Continuity();
                            bool IsBad = false;

                            if (cont > GeomAbs_C0 && error > Max(1.0e-3, theTolerance))
                            {
                                int NbKnots = bs2d.NbKnots();
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
                                if (cont > GeomAbs_C2)
                                {
                                    cont = GeomAbs_C2;
                                }
                                int maxdeg = bs2d.Degree();
                                if (maxdeg == 1)
                                {
                                    maxdeg = 14;
                                }
                                var AppCurPar = new Approx_CurvilinearParameter(HC2d,
                                                                                HS,
                                                                                Max(1.0e-3, theTolerance),
                                                                                cont,
                                                                                maxdeg,
                                                                                10);
                                if (AppCurPar.IsDone() || AppCurPar.HasResult())
                                {
                                    // The internal form: the curve is re-knotted below before the edge takes it.
                                    bs2d = AppCurPar.InternalCurve2d1()!;
                                    GAC2d.Load(bs2d, f3d, l3d);
                                    curPC = bs2d;

                                    if (Math.Abs(bs2d.FirstParameter() - fC0) > TolSameRange
                                        || Math.Abs(bs2d.LastParameter() - lC0) > TolSameRange)
                                    {
                                        var Knots = new NCollection_Array1<double>(1, bs2d.NbKnots());
                                        bs2d.Knots(Knots);
                                        //		  BSplCLib::Reparametrize(f3d,l3d,Knots);
                                        BSplCLib.Reparametrize(fC0, lC0, Knots);
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
                        var SameP = new Approx_SameParameter(HC, HC2d, HS, aTol);

                        if (SameP.IsSameParameter())
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
                        else if (SameP.IsDone())
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
                            GeomLib.SameRange(TolSameRange,
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
                maxdist = Max(maxdist, Occt.Precision.Confusion());
                theNewTol = maxdist;
                aNTE.Modified(true);
                aNTE.Tolerance(maxdist);
            }
            B.SameParameter(aNE, true);
        }

        return aNE;
    }

    //=================================================================================================

    private static void InternalUpdateTolerances(in TopoDS_Shape theOldShape,
                                                 bool IsVerifyTolerance,
                                                 bool IsMutableInput,
                                                 BRepTools_ReShape theReshaper)
    {
        var aShToTol = new NCollection_DataMap<TopoDS_Shape, double>(TopTools_ShapeMapHasher.Instance);
        // Harmonize tolerances
        // with rule Tolerance(VERTEX)>=Tolerance(EDGE)>=Tolerance(FACE)
        double tol = 0;
        if (IsVerifyTolerance)
        {
            // Set tolerance to its minimum value
            Geom_Surface? S;
            var ex = new TopExp_Explorer();
            var aB = new Bnd_Box();
            double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax, dMax;
            for (ex.Init(theOldShape, TopAbs_FACE); ex.More(); ex.Next())
            {
                TopoDS_Face curf = TopoDS.Face(ex.Current());
                S = BRep_Tool.Surface(curf, out _);
                if (S is not null)
                {
                    aB.SetVoid();
                    BRepBndLib.Add(curf, ref aB);
                    if (S.GetType() == typeof(Geom_RectangularTrimmedSurface))
                    {
                        S = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
                    }
                    var AS = new GeomAdaptor_Surface(S);
                    switch (AS.GetType())
                    {
                        case GeomAbs_Plane:
                        case GeomAbs_Cylinder:
                        case GeomAbs_Cone:
                            {
                                tol = Occt.Precision.Confusion();
                                break;
                            }
                        case GeomAbs_Sphere:
                        case GeomAbs_Torus:
                            {
                                tol = Occt.Precision.Confusion() * 2;
                                break;
                            }
                        default:
                            tol = Occt.Precision.Confusion() * 4;
                            break;
                    }
                    if (!aB.IsWhole())
                    {
                        aB.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);
                        dMax = 1.0;
                        if (!aB.IsOpenXmin() && !aB.IsOpenXmax())
                        {
                            dMax = aXmax - aXmin;
                        }
                        if (!aB.IsOpenYmin() && !aB.IsOpenYmax())
                        {
                            aYmin = aYmax - aYmin;
                        }
                        if (!aB.IsOpenZmin() && !aB.IsOpenZmax())
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
        var parents = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theOldShape, TopAbs_EDGE, TopAbs_FACE, parents);
        var lConx = new NCollection_List<TopoDS_Shape>.Iterator();
        int iCur;
        for (iCur = 1; iCur <= parents.Extent(); iCur++)
        {
            tol = 0;
            for (lConx.Initialize(parents[iCur]); lConx.More(); lConx.Next())
            {
                TopoDS_Face FF = TopoDS.Face(lConx.Value());
                double Ftol;
                if (IsVerifyTolerance && aShToTol.IsBound(FF))
                { // first condition for speed-up
                    Ftol = aShToTol.Find(FF);
                }
                else
                {
                    Ftol = BRep_Tool.Tolerance(FF); // tolerance have not been updated
                }
                tol = Max(tol, Ftol);
            }
            // Update can only increase tolerance, so if the edge has a greater
            //  tolerance than its faces it is not concerned
            TopoDS_Edge EK = TopoDS.Edge(parents.FindKey(iCur));
            if (tol > BRep_Tool.Tolerance(EK))
            {
                aShToTol.Bind(EK, tol);
            }
        }

        // Vertices are processed
        const double BigTol = 1.0e10;
        parents.Clear();

        TopExp.MapShapesAndUniqueAncestors(theOldShape, TopAbs_VERTEX, TopAbs_EDGE, parents);
        // OCCT's map of handles: the vertex TShapes by reference.
        var Initialized = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        int nbV = parents.Extent();
        for (iCur = 1; iCur <= nbV; iCur++)
        {
            tol = 0;
            TopoDS_Vertex V = TopoDS.Vertex(parents.FindKey(iCur));
            gp_Pnt aPV = BRep_Tool.Pnt(V);
            double aMaxDist = 0.0;
            gp_Pnt p3d;
            for (lConx.Initialize(parents[iCur]); lConx.More(); lConx.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(lConx.Value());
                tol = Max(tol, aShToTol.Find(E, out double aNtol) ? aNtol : BRep_Tool.Tolerance(E));
                if (tol > BigTol)
                {
                    continue;
                }
                if (!BRep_Tool.SameRange(E))
                {
                    continue;
                }
                double par = BRep_Tool.Parameter(V, E);
                BRep_TEdge TE = (BRep_TEdge)E.TShape()!;
                var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
                TopLoc_Location Eloc = E.Location();
                while (itcr.More())
                {
                    // For each CurveRepresentation, check the provided parameter
                    BRep_CurveRepresentation cr = itcr.Value();
                    TopLoc_Location loc = cr.Location();
                    TopLoc_Location L = (Eloc * loc);
                    if (cr.IsCurve3D())
                    {
                        Geom_Curve? C = cr.Curve3D();
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
                        Geom_Surface Su = cr.Surface()!;
                        Geom2d_Curve PC = cr.PCurve()!;
                        Geom2d_Curve? PC2 = null;
                        if (cr.IsCurveOnClosedSurface())
                        {
                            PC2 = cr.PCurve2();
                        }
                        gp_Pnt2d p2d = PC.Value(par);
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
            double aVTol = BRep_Tool.Tolerance(V);
            bool anUpdTol = tol > aVTol;
            TopoDS_TShape aTV = V.TShape()!;
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
    internal static void UpdateTolerances(in TopoDS_Shape S, bool verifyFaceTolerance = false)
    {
        var aReshaper = new BRepTools_ReShape();
        InternalUpdateTolerances(S, verifyFaceTolerance, true, aReshaper);
    }

    //=================================================================================================

    /// <summary>
    /// <see cref="UpdateTolerances(in TopoDS_Shape, bool)"/> without changing <paramref name="S"/>: the changed
    /// faces, edges and vertices are copies recorded as replacements in <paramref name="theReshaper"/>, whose
    /// <see cref="BRepTools_ReShape.Apply"/> gives the result.
    /// </summary>
    public static void UpdateTolerances(in TopoDS_Shape S,
                                        BRepTools_ReShape theReshaper,
                                        bool verifyFaceTolerance = false)
    {
        InternalUpdateTolerances(S, verifyFaceTolerance, false, theReshaper);
    }

    //=================================================================================================

    /// <summary>
    /// Checks the tolerances of the edges (inner points included) and vertices of the shape and raises them to
    /// satisfy the SameParameter condition. Writes into the shape.
    /// </summary>
    internal static void UpdateInnerTolerances(in TopoDS_Shape aShape)
    {
        var EFmap = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aShape, TopAbs_EDGE, TopAbs_FACE, EFmap);
        var BB = new BRep_Builder();
        for (int i = 1; i <= EFmap.Extent(); i++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(EFmap.FindKey(i));

            if (!BRep_Tool.IsGeometric(anEdge))
            {
                continue;
            }

            TopExp.Vertices(anEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            BRep_Tool.Range(anEdge, out double fpar, out double lpar);
            double TolEdge = BRep_Tool.Tolerance(anEdge);
            gp_Pnt Pnt1 = new(), Pnt2 = new();
            var anHCurve = new BRepAdaptor_Curve();
            anHCurve.Initialize(anEdge);
            if (!V1.IsNull())
            {
                Pnt1 = BRep_Tool.Pnt(V1);
            }
            if (!V2.IsNull())
            {
                Pnt2 = BRep_Tool.Pnt(V2);
            }

            if (!BRep_Tool.Degenerated(anEdge) && EFmap[i].Extent() > 0)
            {
                var theRep = new NCollection_Sequence<Adaptor3d_Curve>();
                theRep.Append(anHCurve);
                var itl = new NCollection_List<TopoDS_Shape>.Iterator(EFmap[i]);
                for (; itl.More(); itl.Next())
                {
                    TopoDS_Face aFace = TopoDS.Face(itl.Value());
                    var anHCurvOnSurf = new BRepAdaptor_Curve();
                    anHCurvOnSurf.Initialize(anEdge, aFace);
                    theRep.Append(anHCurvOnSurf);
                }

                int NbSamples = (BRep_Tool.SameParameter(anEdge)) ? 23 : 2;
                double delta = (lpar - fpar) / (NbSamples - 1);
                double MaxDist = 0.0;
                for (int j = 2; j <= theRep.Length(); j++)
                {
                    for (int k = 0; k <= NbSamples; k++)
                    {
                        double ParamOnCenter = (k == NbSamples) ? lpar : fpar + k * delta;
                        gp_Pnt Center = theRep[1].Value(ParamOnCenter);
                        double ParamOnCurve =
                          (BRep_Tool.SameParameter(anEdge))
                            ? ParamOnCenter
                            : ((k == 0) ? theRep[j].FirstParameter() : theRep[j].LastParameter());
                        gp_Pnt aPoint = theRep[j].Value(ParamOnCurve);
                        double aDist = Center.Distance(aPoint);
                        // aDist *= 1.1;
                        aDist += 2.0 * Epsilon(aDist);
                        if (aDist > MaxDist)
                        {
                            MaxDist = aDist;
                        }

                        // Update tolerances of vertices
                        if (k == 0 && !V1.IsNull())
                        {
                            double aDist1 = Pnt1.Distance(aPoint);
                            aDist1 += 2.0 * Epsilon(aDist1);
                            BB.UpdateVertex(V1, aDist1);
                        }
                        if (k == NbSamples && !V2.IsNull())
                        {
                            double aDist2 = Pnt2.Distance(aPoint);
                            aDist2 += 2.0 * Epsilon(aDist2);
                            BB.UpdateVertex(V2, aDist2);
                        }
                    }
                }
                BB.UpdateEdge(anEdge, MaxDist);
            }
            TolEdge = BRep_Tool.Tolerance(anEdge);
            if (!V1.IsNull())
            {
                gp_Pnt End1 = anHCurve.Value(fpar);
                double dist1 = Pnt1.Distance(End1);
                dist1 += 2.0 * Epsilon(dist1);
                BB.UpdateVertex(V1, Max(dist1, TolEdge));
            }
            if (!V2.IsNull())
            {
                gp_Pnt End2 = anHCurve.Value(lpar);
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
    internal static bool OrientClosedSolid(ref TopoDS_Solid solid)
    {
        // Set material inside the solid
        var where = new BRepClass3d_SolidClassifier(solid);
        where.PerformInfinitePoint(Occt.Precision.Confusion());
        if (where.State() == TopAbs_IN)
        {
            solid.Reverse();
        }
        else if (where.State() == TopAbs_ON || where.State() == TopAbs_UNKNOWN)
        {
            return false;
        }

        return true;
    }

    /// <summary>
    /// The bounding sphere of the vertices of <paramref name="theLV"/>: its centre and radius, for a new vertex
    /// covering them all. With fewer than two vertices the outputs are left unchanged, as in OCCT.
    /// </summary>
    public static void BoundingVertex(NCollection_List<TopoDS_Shape> theLV, ref gp_Pnt theNewCenter, ref double theNewTol)
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
            var aV = new TopoDS_Vertex[2];
            var aP = new gp_Pnt[2];
            //
            aEps = RealEpsilon();
            for (m = 0; m < aNb; ++m)
            {
                aV[m] = (m == 0) ? TopoDS.Vertex(theLV.First()) : TopoDS.Vertex(theLV.Last());
                aP[m] = BRep_Tool.Pnt(aV[m]);
                aR[m] = BRep_Tool.Tolerance(aV[m]);
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
            var aVD = new gp_Vec(aP[m], aP[n]);
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
                gp_XYZ aXYZr;
                var aPr = new gp_Pnt();
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
            var aPoints = new gp_Pnt[aNb];
            NCollection_List<TopoDS_Shape>.Iterator aIt = new(theLV);
            for (i = 0; aIt.More(); aIt.Next(), ++i)
            {
                TopoDS_Vertex aVi = TopoDS.Vertex(aIt.Value());
                gp_Pnt aPi = BRep_Tool.Pnt(aVi);
                aPoints[i] = aPi;
            }
            //
            Array.Sort(aPoints, comparePoints);
            //
            var aXYZ = new gp_XYZ(0.0, 0.0, 0.0);
            for (i = 0; i < aNb; ++i)
            {
                aXYZ += aPoints[i].XYZ();
            }
            aXYZ.Divide((double)aNb);
            //
            var aP = new gp_Pnt(aXYZ);
            //
            // compute the tolerance for the new vertex
            double aTi, aDi, aDmax;
            //
            aDmax = -1.0;
            aIt.Initialize(theLV);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Vertex aVi = TopoDS.Vertex(aIt.Value());
                gp_Pnt aPi = BRep_Tool.Pnt(aVi);
                aTi = BRep_Tool.Tolerance(aVi);
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
    private static int comparePoints(gp_Pnt theP1, gp_Pnt theP2)
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
}
