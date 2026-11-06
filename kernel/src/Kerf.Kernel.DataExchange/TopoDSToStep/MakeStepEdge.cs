// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepEdge.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export.Geometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>
/// Maps a TopoDS_Edge (with its relative orientation in the wire) to a STEP edge_curve, its curve a surface_curve or a
/// seam_curve when pcurves are written.
/// </summary>
internal sealed class MakeStepEdge : ExportRoot
{
    private const int Nbpt = 21;

    private StepTopologicalRepresentationItem _result;
    private MakeEdgeError _error;

    public MakeStepEdge()
    {
        _error = MakeEdgeError.EdgeOther;
        _done = false;
    }

    public MakeStepEdge(in Edge e, ExportTool t, FinderProcess fp, in Factors localFactors)
    {
        _done = false;
        Init(e, t, fp, localFactors);
    }

    public MakeStepEdge(in Edge e, ExportTool t, FinderProcess fp)
        : this(e, t, fp, new Factors())
    {
    }

    public void Init(in Edge aEdge, ExportTool aTool, FinderProcess fp, in Factors localFactors)
    {
        // ------------------------------------------------------------------
        // The edge is given with its relative orientation (i.e. in the wire)
        // ------------------------------------------------------------------
        aTool.SetCurrentEdge(aEdge);

        // [BEGIN] Processing non-manifold topology (ssv; 11.11.2010)
        bool isNMMode = (fp.Model() as StepModel).InternalParameters.WriteNonmanifold;
        if (isNMMode)
        {
            StepEdgeCurve ec = null;
            ShapeMapper stepMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, aEdge);
            if (fp.FindTypedTransient(stepMapper, typeof(StepEdgeCurve), ref ec))
            {
                // Non-manifold topology detected
                _error = MakeEdgeError.EdgeDone;
                _result = ec;
                _done = true;
                return;
            }
        }
        // [END] Processing non-manifold topology (ssv; 11.11.2010)

        if (aTool.IsBound(aEdge))
        {
            _error = MakeEdgeError.EdgeDone;
            _done = true;
            _result = aTool.Find(aEdge);
            return;
        }

        int i;
        double u, u1, u2;
        Point3D p;

        bool isSeam = Tool.IsClosed(aEdge, aTool.CurrentFace());

        //: i4 abv 02 Sep 98: ProSTEP TR8 Motor.rle f3 & f62: check that edge
        // participates twice in the wires of the face before making it seam
        // (else it can have two pcurves on the same surface being shared by
        // two faces on that surface)
        // This fix is necessary because sharing of surfaces is not preserved when
        // writing faces to STEP (see TopoDSToSTEP_MakeStepFace)
        if (isSeam)
        {
            int count = 0;
            var exp = new Explorer(aTool.CurrentFace(), ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                if (aEdge.IsSame(exp.Current()))
                {
                    count++;
                }
            }

            if (count < 2)
            {
                isSeam = false;
            }
        }

        if (aEdge.Orientation == Internal || aEdge.Orientation == External)
        {
            var errShape = new ShapeMapper(aEdge);
            fp.AddWarning(errShape, " Edge(internal/external) from Non Manifold Topology");
            _error = MakeEdgeError.NonManifoldEdge;
            _done = false;
            return;
        }

        // Vertices

        StepVertex v1, v2;
        StepTopologicalRepresentationItem gpms2;
        Exploration.Vertices(aEdge, out Vertex vfirst, out Vertex vlast);

        var mkVertex = new MakeStepVertex();

        mkVertex.Init(vfirst, aTool, fp, localFactors);
        if (mkVertex.IsDone)
        {
            v1 = mkVertex.Value() as StepVertex;
        }
        else
        {
            var errShape = new ShapeMapper(aEdge);
            fp.AddWarning(errShape, " First Vertex of Edge not mapped");
            _error = MakeEdgeError.EdgeOther;
            _done = false;
            return;
        }

        mkVertex.Init(vlast, aTool, fp, localFactors);
        if (mkVertex.IsDone)
        {
            v2 = mkVertex.Value() as StepVertex;
        }
        else
        {
            var errShape = new ShapeMapper(aEdge);
            fp.AddWarning(errShape, " Last Vertex of Edge not mapped");
            _error = MakeEdgeError.EdgeOther;
            _done = false;
            return;
        }

        // ---------------------------------------
        // Translate 3D representation of the Edge
        // ---------------------------------------

        var ca = new EdgeCurveAdaptor(aEdge);
        StepCurve gpms;
        Curve3D c = ca.Curve().Curve();

        if (c is not null)
        {
            c = (Curve3D)c.Copy();
            Transform3D tr1 = ca.Trsf();
            c.Transform(tr1);
            // Special treatment is needed for very short edges based on periodic curves.
            // Since edge in STEP does not store its parametric range, parameters are computed
            // on import by projecting vertices on a curve, and for periodic curve this may
            // lead to use of wrong part of the curve if end vertices are too close to each other
            // (often whole curve is taken instead of its very small fragment).
            if (c.IsPeriodic)
            {
                double dpar = ca.LastParameter() - ca.FirstParameter();
                if (dpar <= 0)
                {
                    dpar += (Math.Ceiling(Math.Abs(dpar) / c.Period) * c.Period);
                }

                // if range obtained from projection of vertices contradicts with range
                // of the edge tnen vertices are swapped to keep results correct after import
                // (see test de step_5 A1)
                Point3D p1 = Tool.Pnt(vfirst);
                Point3D p2 = Tool.Pnt(vlast);
                var sac = new CurveAnalysis();
                sac.Project(c, p1, Tolerance(), out Point3D pproj, out u1, false);
                sac.Project(c, p2, Tolerance(), out pproj, out u2, false);
                double dU = u2 - u1;
                if (dU <= 0)
                {
                    dU += (Math.Ceiling(Math.Abs(dU) / c.Period) * c.Period);
                }

                if ((dU > global::Kerf.Engine.Foundation.Precision.PConfusion() && dU <= 0.1 * c.Period && dpar > 0.5 * c.Period)
                    || (dpar > global::Kerf.Engine.Foundation.Precision.PConfusion() && dpar <= 0.1 * c.Period && dU > 0.5 * c.Period))
                {
                    (v1, v2) = (v2, v1);
                }

                // If vertices overlap, we cut only needed part of the BSpline curve.
                // Note that this method cannot be used for canonic curves
                // (STEP does not support trimmed curves in AIC 514).
                if (c is BSplineCurve3D)
                {
                    double tolV1 = Tool.Tolerance(vfirst);
                    double tolV2 = Tool.Tolerance(vlast);
                    Point3D p11 = ca.Value(ca.FirstParameter());
                    Point3D p12 = ca.Value(ca.LastParameter());
                    Point3D pm = ca.Value((ca.FirstParameter() + ca.LastParameter()) * 0.5);
                    double dist11 = p11.DistanceTo(p12);
                    double dist1m = p11.DistanceTo(pm);
                    double dist2m = p12.DistanceTo(pm);
                    double distMax = Max(Max(dist1m, dist2m), dist11);
                    bool isSmallCurve = (distMax <= tolV1 || distMax <= tolV2);
                    if (BrepTools.Compare(vfirst, vlast) && isSmallCurve && dpar > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && dpar <= 0.1 * c.Period)
                    {
                        BSplineCurve3D bspl1 = (BSplineCurve3D)c.Copy();
                        bspl1.Segment(ca.FirstParameter(), ca.LastParameter());
                        c = bspl1;
                    }
                }
            }

            var mkCurve = new MakeCurve(c, localFactors);
            gpms = mkCurve.Value();
        }
        else
        {
            // -------------------------
            // a 3D Curve is constructed
            // -------------------------
            // OCCT_DEBUG output left out
            var sa = new FaceSurfaceAdaptor(aTool.CurrentFace());

            if ((sa.GetType() == Plane) && (ca.GetType() == Line))
            {
                u1 = ca.FirstParameter();
                u2 = ca.LastParameter();
                Vector3D v = new Vector3D(ca.Value(u1), ca.Value(u2));
                var l = new LineCurve3D(ca.Value(u1), new Direction3D(v));
                var mkLine = new GeometryMakeLine(l, localFactors);
                gpms = mkLine.Value();
            }
            else
            {
                // To Be Optimized : create an approximated BSpline
                //                   using GeomAPI_PointsToBSpline
                var points = new NCollection_Array1<Point3D>(1, Nbpt);
                var knots = new NCollection_Array1<double>(1, Nbpt);
                var mult = new NCollection_Array1<int>(1, Nbpt);
                u1 = ca.FirstParameter();
                u2 = ca.LastParameter();
                for (i = 1; i <= Nbpt; i++)
                {
                    u = u1 + (i - 1) * (u2 - u1) / (Nbpt - 1);
                    p = ca.Value(u);
                    points.SetValue(i, p);
                    knots.SetValue(i, u);
                    mult.SetValue(i, 1);
                }

                // Points.SetValue(1, BRep_Tool::Pnt(Vfirst));
                // Points.SetValue(Nbpt, BRep_Tool::Pnt(Vlast));
                mult.SetValue(1, 2);
                mult.SetValue(Nbpt, 2);
                Curve3D bs = new BSplineCurve3D(points, knots, mult, 1);
                var mkCurve = new MakeCurve(bs, localFactors);
                gpms = mkCurve.Value();
            }
        }

        // ---------------------------------------------------------
        // Warning : if the edge is connected aGeom->Length = 2
        //           otherwise = 1 ;
        //           and enumeration is pscrPcurveS2 or pscrPcurveS1
        // This is corrected in the Write File phases !
        // ---------------------------------------------------------

        //: abv 25.01.00 CAX-IF TRJ3
        // if PcurveMode is 1 (default), make surface_curve instead of simple 3d curve
        if (aTool.PCurveMode() != 0)
        {
            // OCCT's array default-constructs its two SELECT values (empty); the port's holds references
            var geom = new NCollection_HArray1<StepPcurveOrSurface>(1, 2);
            geom.SetValue(1, new StepPcurveOrSurface());
            geom.SetValue(2, new StepPcurveOrSurface());
            var name = new TCollection_HAsciiString("");

            if (!isSeam)
            {
                var surfaceCurve = new StepSurfaceCurve();
                surfaceCurve.Init(name, gpms, geom, StepPreferredSurfaceCurveRepresentation.PscrPcurveS1);
                gpms = surfaceCurve;
            }
            else
            {
                var seamCurve = new StepSeamCurve();
                seamCurve.Init(name, gpms, geom, StepPreferredSurfaceCurveRepresentation.PscrPcurveS1);
                gpms = seamCurve;
            }
        }

        // Edge curve
        var epms = new StepEdgeCurve();
        var name1 = new TCollection_HAsciiString("");
        epms.Init(name1, v1, v2, gpms, true);

        aTool.Bind(aEdge, epms);
        _error = MakeEdgeError.EdgeDone;
        _result = epms;
        _done = true;
    }

    public void Init(in Edge aEdge, ExportTool aTool, FinderProcess fp)
        => Init(aEdge, aTool, fp, new Factors());

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepTopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeStepEdge::Value() - no result");
        return _result;
    }

    public MakeEdgeError Error() => _error;
}
