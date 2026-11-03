// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepEdge.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings
#pragma warning disable CS0168 // locals OCCT declares and never uses

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.StepGeom_PreferredSurfaceCurveRepresentation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopoDSToStep_MakeEdgeError;

namespace Occt;

/// <summary>
/// Maps a TopoDS_Edge (with its relative orientation in the wire) to a STEP edge_curve, its curve a surface_curve or a
/// seam_curve when pcurves are written.
/// </summary>
public sealed class TopoDSToStep_MakeStepEdge : TopoDSToStep_Root
{
    private const int Nbpt = 21;

    private StepShape_TopologicalRepresentationItem myResult;
    private TopoDSToStep_MakeEdgeError myError;

    public TopoDSToStep_MakeStepEdge()
    {
        myError = TopoDSToStep_EdgeOther;
        done = false;
    }

    public TopoDSToStep_MakeStepEdge(in TopoDS_Edge E, TopoDSToStep_Tool T, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        done = false;
        Init(E, T, FP, theLocalFactors);
    }

    public TopoDSToStep_MakeStepEdge(in TopoDS_Edge E, TopoDSToStep_Tool T, Transfer_FinderProcess FP)
        : this(E, T, FP, new StepData_Factors())
    {
    }

    public void Init(in TopoDS_Edge aEdge, TopoDSToStep_Tool aTool, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        // ------------------------------------------------------------------
        // The edge is given with its relative orientation (i.e. in the wire)
        // ------------------------------------------------------------------
        aTool.SetCurrentEdge(aEdge);

        // [BEGIN] Processing non-manifold topology (ssv; 11.11.2010)
        bool isNMMode = (FP.Model() as StepData_StepModel).InternalParameters.WriteNonmanifold;
        if (isNMMode)
        {
            StepShape_EdgeCurve anEC = null;
            TransferBRep_ShapeMapper aSTEPMapper = TransferBRep.ShapeMapper(FP, aEdge);
            if (FP.FindTypedTransient(aSTEPMapper, typeof(StepShape_EdgeCurve), ref anEC))
            {
                // Non-manifold topology detected
                myError = TopoDSToStep_EdgeDone;
                myResult = anEC;
                done = true;
                return;
            }
        }
        // [END] Processing non-manifold topology (ssv; 11.11.2010)

        if (aTool.IsBound(aEdge))
        {
            myError = TopoDSToStep_EdgeDone;
            done = true;
            myResult = aTool.Find(aEdge);
            return;
        }

        int i;
        double U, U1, U2;
        Point3D P;

        bool isSeam = BRep_Tool.IsClosed(aEdge, aTool.CurrentFace());

        //: i4 abv 02 Sep 98: ProSTEP TR8 Motor.rle f3 & f62: check that edge
        // participates twice in the wires of the face before making it seam
        // (else it can have two pcurves on the same surface being shared by
        // two faces on that surface)
        // This fix is necessary because sharing of surfaces is not preserved when
        // writing faces to STEP (see TopoDSToSTEP_MakeStepFace)
        if (isSeam)
        {
            int count = 0;
            var exp = new TopExp_Explorer(aTool.CurrentFace(), TopAbs_EDGE);
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

        if (aEdge.Orientation() == TopAbs_INTERNAL || aEdge.Orientation() == TopAbs_EXTERNAL)
        {
            var errShape = new TransferBRep_ShapeMapper(aEdge);
            FP.AddWarning(errShape, " Edge(internal/external) from Non Manifold Topology");
            myError = TopoDSToStep_NonManifoldEdge;
            done = false;
            return;
        }

        // Vertices

        StepShape_Vertex V1, V2;
        StepShape_TopologicalRepresentationItem Gpms2;
        TopExp.Vertices(aEdge, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast);

        var MkVertex = new TopoDSToStep_MakeStepVertex();

        MkVertex.Init(Vfirst, aTool, FP, theLocalFactors);
        if (MkVertex.IsDone())
        {
            V1 = MkVertex.Value() as StepShape_Vertex;
        }
        else
        {
            var errShape = new TransferBRep_ShapeMapper(aEdge);
            FP.AddWarning(errShape, " First Vertex of Edge not mapped");
            myError = TopoDSToStep_EdgeOther;
            done = false;
            return;
        }

        MkVertex.Init(Vlast, aTool, FP, theLocalFactors);
        if (MkVertex.IsDone())
        {
            V2 = MkVertex.Value() as StepShape_Vertex;
        }
        else
        {
            var errShape = new TransferBRep_ShapeMapper(aEdge);
            FP.AddWarning(errShape, " Last Vertex of Edge not mapped");
            myError = TopoDSToStep_EdgeOther;
            done = false;
            return;
        }

        // ---------------------------------------
        // Translate 3D representation of the Edge
        // ---------------------------------------

        var CA = new BRepAdaptor_Curve(aEdge);
        StepGeom_Curve Gpms;
        Geom_Curve C = CA.Curve().Curve();

        if (C is not null)
        {
            C = (Geom_Curve)C.Copy();
            Transform3D Tr1 = CA.Trsf();
            C.Transform(Tr1);
            // Special treatment is needed for very short edges based on periodic curves.
            // Since edge in STEP does not store its parametric range, parameters are computed
            // on import by projecting vertices on a curve, and for periodic curve this may
            // lead to use of wrong part of the curve if end vertices are too close to each other
            // (often whole curve is taken instead of its very small fragment).
            if (C.IsPeriodic())
            {
                double dpar = CA.LastParameter() - CA.FirstParameter();
                if (dpar <= 0)
                {
                    dpar += (Math.Ceiling(Math.Abs(dpar) / C.Period()) * C.Period());
                }

                // if range obtained from projection of vertices contradicts with range
                // of the edge tnen vertices are swapped to keep results correct after import
                // (see test de step_5 A1)
                Point3D aP1 = BRep_Tool.Pnt(Vfirst);
                Point3D aP2 = BRep_Tool.Pnt(Vlast);
                var sac = new ShapeAnalysis_Curve();
                sac.Project(C, aP1, Tolerance(), out Point3D pproj, out U1, false);
                sac.Project(C, aP2, Tolerance(), out pproj, out U2, false);
                double dU = U2 - U1;
                if (dU <= 0)
                {
                    dU += (Math.Ceiling(Math.Abs(dU) / C.Period()) * C.Period());
                }

                if ((dU > global::Kerf.Engine.Foundation.Precision.PConfusion() && dU <= 0.1 * C.Period() && dpar > 0.5 * C.Period())
                    || (dpar > global::Kerf.Engine.Foundation.Precision.PConfusion() && dpar <= 0.1 * C.Period() && dU > 0.5 * C.Period()))
                {
                    (V1, V2) = (V2, V1);
                }

                // If vertices overlap, we cut only needed part of the BSpline curve.
                // Note that this method cannot be used for canonic curves
                // (STEP does not support trimmed curves in AIC 514).
                if (C is Geom_BSplineCurve)
                {
                    double aTolV1 = BRep_Tool.Tolerance(Vfirst);
                    double aTolV2 = BRep_Tool.Tolerance(Vlast);
                    Point3D aP11 = CA.Value(CA.FirstParameter());
                    Point3D aP12 = CA.Value(CA.LastParameter());
                    Point3D aPm = CA.Value((CA.FirstParameter() + CA.LastParameter()) * 0.5);
                    double aDist11 = aP11.Distance(aP12);
                    double aDist1m = aP11.Distance(aPm);
                    double aDist2m = aP12.Distance(aPm);
                    double aDistMax = Max(Max(aDist1m, aDist2m), aDist11);
                    bool isSmallCurve = (aDistMax <= aTolV1 || aDistMax <= aTolV2);
                    if (BRepTools.Compare(Vfirst, Vlast) && isSmallCurve && dpar > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && dpar <= 0.1 * C.Period())
                    {
                        Geom_BSplineCurve aBspl1 = (Geom_BSplineCurve)C.Copy();
                        aBspl1.Segment(CA.FirstParameter(), CA.LastParameter());
                        C = aBspl1;
                    }
                }
            }

            var MkCurve = new GeomToStep_MakeCurve(C, theLocalFactors);
            Gpms = MkCurve.Value();
        }
        else
        {
            // -------------------------
            // a 3D Curve is constructed
            // -------------------------
            // OCCT_DEBUG output left out
            var SA = new BRepAdaptor_Surface(aTool.CurrentFace());

            if ((SA.GetType() == Plane) && (CA.GetType() == Line))
            {
                U1 = CA.FirstParameter();
                U2 = CA.LastParameter();
                Vector3D V = new Vector3D(CA.Value(U1), CA.Value(U2));
                var L = new Geom_Line(CA.Value(U1), new Direction3D(V));
                var MkLine = new GeomToStep_MakeLine(L, theLocalFactors);
                Gpms = MkLine.Value();
            }
            else
            {
                // To Be Optimized : create an approximated BSpline
                //                   using GeomAPI_PointsToBSpline
                var Points = new NCollection_Array1<Point3D>(1, Nbpt);
                var Knots = new NCollection_Array1<double>(1, Nbpt);
                var Mult = new NCollection_Array1<int>(1, Nbpt);
                U1 = CA.FirstParameter();
                U2 = CA.LastParameter();
                for (i = 1; i <= Nbpt; i++)
                {
                    U = U1 + (i - 1) * (U2 - U1) / (Nbpt - 1);
                    P = CA.Value(U);
                    Points.SetValue(i, P);
                    Knots.SetValue(i, U);
                    Mult.SetValue(i, 1);
                }

                // Points.SetValue(1, BRep_Tool::Pnt(Vfirst));
                // Points.SetValue(Nbpt, BRep_Tool::Pnt(Vlast));
                Mult.SetValue(1, 2);
                Mult.SetValue(Nbpt, 2);
                Geom_Curve Bs = new Geom_BSplineCurve(Points, Knots, Mult, 1);
                var MkCurve = new GeomToStep_MakeCurve(Bs, theLocalFactors);
                Gpms = MkCurve.Value();
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
            var aGeom = new NCollection_HArray1<StepGeom_PcurveOrSurface>(1, 2);
            aGeom.SetValue(1, new StepGeom_PcurveOrSurface());
            aGeom.SetValue(2, new StepGeom_PcurveOrSurface());
            var aName = new TCollection_HAsciiString("");

            if (!isSeam)
            {
                var SurfaceCurve = new StepGeom_SurfaceCurve();
                SurfaceCurve.Init(aName, Gpms, aGeom, StepGeom_pscrPcurveS1);
                Gpms = SurfaceCurve;
            }
            else
            {
                var SeamCurve = new StepGeom_SeamCurve();
                SeamCurve.Init(aName, Gpms, aGeom, StepGeom_pscrPcurveS1);
                Gpms = SeamCurve;
            }
        }

        // Edge curve
        var Epms = new StepShape_EdgeCurve();
        var aName1 = new TCollection_HAsciiString("");
        Epms.Init(aName1, V1, V2, Gpms, true);

        aTool.Bind(aEdge, Epms);
        myError = TopoDSToStep_EdgeDone;
        myResult = Epms;
        done = true;
    }

    public void Init(in TopoDS_Edge aEdge, TopoDSToStep_Tool aTool, Transfer_FinderProcess FP)
        => Init(aEdge, aTool, FP, new StepData_Factors());

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepShape_TopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeStepEdge::Value() - no result");
        return myResult;
    }

    public TopoDSToStep_MakeEdgeError Error() => myError;
}
