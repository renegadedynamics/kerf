// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdge.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Construct;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Library.LibraryEdgeError;
using static Kerf.Engine.Exchange.Step.Import.TranslateEdgeError;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Translates a STEP edge (an edge_curve, or an oriented_edge of one) into a TopoDS_Edge.</summary>
/// <remarks>
/// The edge is always built FORWARD, along its curve. <see cref="Value"/> freezes the result; the translators use
/// <c>InternalValue</c> (decision 1247). The 3D curves are shared through the transient process
/// (<c>BindTransient</c>): where OCCT reverses or adjusts such a curve in place after an earlier edge took it, the port
/// works on a copy (decision 1248).
/// </remarks>
public sealed class TranslateEdge : ImportRoot
{
    private TranslateEdgeError _error;
    private Shape _result = new Shape();

    //: d8
    // Used in I-DEAS-like STP processing (ssv; 15.11.2010)

    private static void DecodeMakeEdgeError(LibraryMakeEdge me, Transient orig, ref Curve3D myCurve, in Vertex v1,
                                            in Vertex v2, double u1, double u2, ImportTool aTool,
                                            StepTopologicalRepresentationItem tobind)
    {
        _ = u1;
        _ = u2; // avoid compiler warning
        _ = tobind;
        TransientProcess tp = aTool.TransientProcess();

        // OCCT_DEBUG output left out
        switch (me.Error())
        {
            case LibraryEdgeError.EdgeDone:
                return;
            case LibraryEdgeError.PointProjectionFailed:
                tp.AddFail(orig, " Point Projection failed");
                break;
            case LibraryEdgeError.ParameterOutOfRange:
                tp.AddFail(orig, " Parameter Out Of Range");
                break;
            case LibraryEdgeError.DifferentPointsOnClosedCurve:
                tp.AddFail(orig, " Different Points on Closed Curve");
                break;
            case LibraryEdgeError.PointWithInfiniteParameter:
                tp.AddFail(orig, " Point with infinite Parameter");
                break;
            case LibraryEdgeError.DifferentsPointAndParameter:
                // OCCT.NET: the curve an earlier edge took is frozen; adjusted in a copy (decision 1248)
                if (myCurve.IsFrozen)
                {
                    myCurve = (Curve3D)myCurve.Copy();
                }

                if (!new ConstructCurve().AdjustCurve(myCurve, Tool.Pnt(v1), Tool.Pnt(v2), true, true))
                {
                    tp.AddFail(orig, " Different Points and Parameters");
                }
                else
                {
                    tp.AddWarning(orig, "Different Points and Parameters, adjusted");
                }

                break;
            case LibraryEdgeError.LineThroughIdenticPoints:
                tp.AddFail(orig, " Line through identic Points");
                break;
        }
    }

    private static Curve3D MakeCurve(StepCurve c1, TransientProcess tp, in Factors localFactors)
    {
        Curve3D c2 = tp.FindTransient(c1) as Curve3D;
        if (c2 is not null)
        {
            return c2;
        }

        c2 = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve(c1, localFactors);
        if (c2 is not null)
        {
            tp.BindTransient(c1, c2);
        }

        return c2;
    }

    private static Edge MakeEdge(Curve3D c3D, in Vertex v1, in Vertex v2, double u1, double u2)
    {
        var b = new BrepBuilder();
        Edge e = new Edge();
        b.MakeEdge(ref e, c3D, Kerf.Engine.Foundation.Precision.Confusion());
        b.Add(e, v1);
        b.Add(e, v2);
        b.UpdateVertex(v1, u1, e, 0.0);
        b.UpdateVertex(v2, u2, e, 0.0);
        return e;
    }

    public TranslateEdge()
    {
        _error = TranslateEdgeError.TranslateEdgeOther;
        _done = false;
    }

    public TranslateEdge(StepEdge e, ImportTool t, NMTool nmTool, in Factors localFactors)
    {
        Init(e, t, nmTool, localFactors);
    }

    public TranslateEdge(StepEdge e, ImportTool t, NMTool nmTool)
        : this(e, t, nmTool, new Factors())
    {
    }

    /// <summary>
    /// Translates the edge: an edge with two vertices and two parameters, always built FORWARD (BRepLib_MakeEdge).
    /// </summary>
    public void Init(StepEdge aEdge, ImportTool aTool, NMTool nmTool, in Factors localFactors)
    {
        TransientProcess tp = aTool.TransientProcess();

        StepOrientedEdge oe = aEdge as StepOrientedEdge;
        StepEdge wEdge = aEdge;
        if (oe is not null)
        {
            wEdge = oe.EdgeElement();
        }

        StepEdgeCurve ec = wEdge as StepEdgeCurve;

        if (aTool.IsBound(ec))
        {
            _result = aTool.Find(ec);
            if (Tool.Degenerated(ShapeCasts.Edge(_result)))
            {
                tp.AddWarning(ec, "Degenerated Edge in several faces : transferred for each face");
            }
            else
            {
                _error = TranslateEdgeError.TranslateEdgeDone;
                _done = true;
                //: S4136      B.SameRange(TopoDS::Edge(myResult), false);    //:a5 abv 11 Feb 98
                //: S4136      B.SameParameter(TopoDS::Edge(myResult), false);//:a5
                return;
            }
        }

        // [BEGIN] Proceed with non-manifold cases (ssv; 12.11.2010)
        if (nmTool.IsActive && nmTool.IsBound(ec))
        {
            Shape existingShape = nmTool.Find(ec);
            // Reverse shape's orientation if needed
            if (!oe.Orientation())
            {
                existingShape.Reverse();
            }

            _result = existingShape;
            _error = TranslateEdgeError.TranslateEdgeDone;
            _done = true;
            return;
        }
        // [END] Proceed with non-manifold cases (ssv; 12.11.2010)

        // [BEGIN] Proceed with I-DEAS-like STP (ssv; 15.11.2010)
        TCollection_HAsciiString ecName = ec.Name();
        if (nmTool.IsIDEASCase && ecName is not null && !ecName.IsEmpty()
            && nmTool.IsBound(ecName.String()))
        {
            Shape existingShape = nmTool.Find(ecName.String());
            // Reverse shape's orientation if needed
            if (!oe.Orientation())
            {
                existingShape.Reverse();
            }

            // Register Edge for final processing (I-DEAS case)
            nmTool.RegisterNMEdge(existingShape);
            _result = existingShape;
            _error = TranslateEdgeError.TranslateEdgeDone;
            _done = true;
            return;
        }
        // [END] Proceed with I-DEAS-like STP (ssv; 15.11.2010)

        var b = new BrepBuilder();

        StepCurve c = ec.EdgeGeometry();
        if (c is null)
        {
            tp.AddFail(ec, " Geom Curve in EdgeCurve is equal to 0");
            _error = TranslateEdgeError.TranslateEdgeOther;
            _done = false;
            return;
        }

        Edge e = new Edge();
        StepVertex vstart, vend;

        // -----------------------------------------------------------
        // Extract the start and end Vertices corresponding to FORWARD
        // (following the geometrical sense)
        // -----------------------------------------------------------

        bool edgeCurveSameSense = ec.SameSense();

        if (edgeCurveSameSense)
        {
            vstart = ec.EdgeStart();
            vend = ec.EdgeEnd();
        }
        else
        {
            vend = ec.EdgeStart();
            vstart = ec.EdgeEnd();
        }

        Vertex v1 = new Vertex(), v2 = new Vertex();

        var myTranVertex1 = new TranslateVertex(vstart, aTool, nmTool, localFactors);
        var myTranVertex2 = new TranslateVertex(vend, aTool, nmTool, localFactors);

        if (myTranVertex1.IsDone)
        {
            v1 = ShapeCasts.Vertex(myTranVertex1.InternalValue());
            v1.Orient(Forward);
        }

        if (vend == vstart)
        {
            v2 = v1;
            v2.Orient(Reversed);
        }
        else if (myTranVertex2.IsDone)
        {
            v2 = ShapeCasts.Vertex(myTranVertex2.InternalValue());
            v2.Orient(Reversed);
        }

        _done = true;

        // ----------------------------------------------------------
        // --- The EdgeCurve Geometry is of StepGeom_Curve Type
        // --- It can be : * a Pcurve : no 3D curve is constructed
        // ---             * a Surface Curve, Intersection Curve
        // ---               or a Seam Curve
        // ---             * a 3D Curve
        // ----------------------------------------------------------

        if (c is StepPcurve)
        {
            b.MakeEdge(ref e);
            //: S4136    B.UpdateEdge (E,preci);
            b.Add(e, v1);
            b.Add(e, v2);
        }
        else if (c is StepSurfaceCurve)
        {
            // For SeamCurve and IntersectionCurve types
            // --- The Edge Geometry is a Surface Curve ---
            // ---     (3d + 2 Pcurve Or Surface)       ---
            StepSurfaceCurve sc = c as StepSurfaceCurve;
            StepCurve c1 = sc.Curve3d();
            MakeFromCurve3D(c1, ec, vend, Precision(), ref e, ref v1, ref v2, aTool, localFactors);
        }
        else
        {
            // --- The Edge Geometry is a Single 3d Curve ---
            MakeFromCurve3D(c, ec, vend, Precision(), ref e, ref v1, ref v2, aTool, localFactors);
        }

        // Force set flags SameRange and SameParameter to false
        if (_done)
        {
            //: S4136    B.SameRange(E, false);
            //: S4136    B.SameParameter(E, false);
            aTool.Bind(ec, e);

            // Bind Edge in NM tool (ssv; 15.11.2010)
            if (nmTool.IsActive)
            {
                nmTool.Bind(ec, e);
                if (nmTool.IsIDEASCase && ecName is not null && !ecName.IsEmpty())
                {
                    nmTool.Bind(ecName.String(), e);
                }
            }

            _result = e;
            _error = TranslateEdgeError.TranslateEdgeDone;
        }
    }

    public void Init(StepEdge aEdge, ImportTool aTool, NMTool nmTool)
        => Init(aEdge, aTool, nmTool, new Factors());

    //: e6 abv 16 Apr 98: ProSTEP TR8, r0601_sy.stp, #14907
    private static void GetCartesianPoints(StepEdgeCurve ec, ref Point3D p1, ref Point3D p2, in Factors localFactors)
    {
        for (int i = 1; i <= 2; i++)
        {
            StepVertex v = ((i == 1) == ec.SameSense() ? ec.EdgeStart() : ec.EdgeEnd());
            StepVertexPoint vp = v as StepVertexPoint;
            if (vp is null)
            {
                continue;
            }

            StepCartesianPoint p = vp.VertexGeometry() as StepCartesianPoint;
            CartesianPoint cp = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(p, localFactors);
            if (i == 1)
            {
                p1 = cp.Pnt();
            }
            else
            {
                p2 = cp.Pnt();
            }
        }
    }

    /// <summary>The case of a 3D curve (alone or in a surface curve). The curve is assumed to be a 3D curve.</summary>
    public void MakeFromCurve3D(StepCurve c3D, StepEdgeCurve ec, StepVertex vend, double preci, ref Edge e,
                                ref Vertex v1, ref Vertex v2, ImportTool aTool, in Factors localFactors)
    {
        TransientProcess tp = aTool.TransientProcess();
        Curve3D c1 = MakeCurve(c3D, tp, localFactors);
        if (c1 is null)
        {
            tp.AddFail(c3D, " Make Geom_Curve (3D) failed");
            _error = TranslateEdgeError.TranslateEdgeOther;
            _done = false;
            return;
        }

        // -- Statistics -- -> No Warning message
        aTool.AddContinuity(c1);
        var b = new BrepBuilder();
        double temp1, temp2, u1, u2;
        Point3D pproj;
        Point3D pv1 = Tool.Pnt(v1);
        Point3D pv2 = Tool.Pnt(v2);

        //: e6 abv
        Point3D pnt1 = pv1, pnt2 = pv2;
        if (v1.IsSame(v2))
        {
            GetCartesianPoints(ec, ref pnt1, ref pnt2, localFactors);
        }

        var sac = new CurveAnalysis();
        temp1 = sac.Project(c1, pnt1, preci, out pproj, out u1, false);
        temp2 = sac.Project(c1, pnt2, preci, out pproj, out u2, false);

        // OCCT.NET: the curve an earlier edge took is frozen; reversed in a copy (decision 1248)
        if (!GeometricTool.UpdateParam3d(ref c1, ref u1, ref u2, preci, true))
        {
            tp.AddWarning(c3D, "Update of 3D-Parameters has failed");
        }

        //: d5: instead of AdjustCurve above which is incorrect if U1 and U2 are not ends
        var ca = new CurveAdaptor(c1);
        Point3D pU1 = ca.Value(u1), pU2 = ca.Value(u2);
        temp1 = pU1.DistanceTo(pv1);
        temp2 = pU2.DistanceTo(pv2);
        if (temp1 > preci || temp2 > preci)
        {
            // #25415: handling of special case found on some STEP files produced by FPX Expert 2013 (PCB
            // design system): edge curve is line displaced from its true position but with correct
            // direction; we can shift the line in this case so that it passes through vertices correctly
            if (Math.Abs(temp1 - temp2) < preci
                && Math.Abs(u2 - u1 - pnt1.DistanceTo(pnt2)) < Kerf.Engine.Foundation.Precision.Confusion()
                && c1 is LineCurve3D)
            {
                LineCurve3D line = c1 as LineCurve3D;
                Line3D lin = line.Line;
                Point3D origin = new Point3D(pnt1.XYZ() - lin.Position.Direction.XYZ() * u1);
                lin = lin.WithLocation(origin);
                c1 = new LineCurve3D(lin);
                tp.AddWarning(c3D, "Poor result from projection vertex / line 3d, line shifted");
            }
            else
            {
                tp.AddWarning(c3D, "Poor result from projection vertex / curve 3d");
            }
        }

        b.UpdateVertex(v1, 1.000001 * temp1); //: h6 abv 14 Jul 98: PRO8845 #2746: *=1.0001
        b.UpdateVertex(v2, 1.000001 * temp2); //: h6

        var me = new LibraryMakeEdge(c1, v1, v2, u1, u2);
        if (me.IsDone)
        {
            e = me.InternalEdge();
            b.Range(e, u1, u2); // abv 14 Mar 00: trj3_pm1-ug.stp #91739, edge 2
        }
        else
        {
            if (me.Error() == LibraryEdgeError.DifferentPointsOnClosedCurve)
            {
                // The Edge could be closed and trimmed by 2 Different vertices
                if (c1.IsClosed)
                {
                    // Attention : topology updating
                    aTool.Bind(vend, v1);
                    Shape localShape = v1.Reversed();
                    v2 = ShapeCasts.Vertex(localShape);
                    me.Init(c1, v1, v2, u1, u2);
                    if (me.IsDone)
                    {
                        tp.AddWarning(ec, "Wrong topology corrected : Closed Edge with TWO different Vertices");
                        e = me.InternalEdge();
                    }
                    else
                    {
                        DecodeMakeEdgeError(me, c3D, ref c1, v1, v2, u1, u2, aTool, ec);
                        e = MakeEdge(c1, v1, v2, u1, u2);
                        _error = TranslateEdgeError.TranslateEdgeDone;
                        _done = true;
                        //		return;
                    }
                }
                else
                {
                    // Then, this is should be coded as degenerated
                    // To be performed later !!!
                    _error = TranslateEdgeError.TranslateEdgeDone;
                    //  Bon, on la fait cette petite edge, mais faudra repasser
                    //  pour l enlever ET FUSIONNER LES VERTEX, pour tout le shell !
                    //  courbe trop petite pour etre mise -> fait planter
                    _done = true;
                    if (!v1.IsSame(v2))
                    {
                        tp.AddFail(ec, "This edge has null arc length");
                        Point3D p1 = Tool.Pnt(v1);
                        Point3D p2 = Tool.Pnt(v2);
                        Vector3D avec = new Vector3D(p1, p2);
                        Direction3D adir = new Direction3D(avec);
                        Line3D alin = new Line3D(p1, adir);
                        c1 = new LineCurve3D(alin);
                        u1 = 0.0;
                        u2 = p1.DistanceTo(p2);
                        e = MakeEdge(c1, v1, v2, u1, u2); //,preci
                    }
                    else
                    {
                        tp.AddFail(ec, "NULL EDGE, SKIPPED");
                        _result.Nullify();
                        return;
                    }
                }
            }
            else
            {
                DecodeMakeEdgeError(me, c3D, ref c1, v1, v2, u1, u2, aTool, ec);
                e = MakeEdge(c1, v1, v2, u1, u2);
                _error = TranslateEdgeError.TranslateEdgeDone;
                _done = true;
            }
        }
    }

    /// <summary>Computes an individual pcurve (a 2D curve) of the pcurve entity on the surface; null if it fails.</summary>
    public Curve2D MakePCurve(StepPcurve pcu, Surface convSurf, in Factors localFactors)
    {
        Curve2D c2d = null;
        DefinitionalRepresentation dri = pcu.ReferenceToCurve();
        if (dri is null)
        {
            return c2d;
        }

        StepCurve stepCurve = dri.ItemsValue(1) as StepCurve;
        try
        {
            c2d = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve2d(stepCurve, localFactors);
            if (c2d is not null)
            {
                // -- if the surface is a RectangularTrimmedSurface,
                // -- send the BasisSurface.
                c2d = Units.DegreeToRadian(c2d, convSurf, localFactors.LengthFactor(), localFactors.FactorDegreeRadian());
            }
        }
        catch (FailureException)
        {
            return c2d;
        }

        return c2d;
    }

    public Curve2D MakePCurve(StepPcurve pcu, Surface convSurf) => MakePCurve(pcu, convSurf, new Factors());

    /// <summary>The edge, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape shape = InternalValue();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_TranslateEdge::Value() - no result");
        return _result;
    }

    public TranslateEdgeError Error() => _error;
}
