// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Edge.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// abv 30 Dec 98: code optimizations
//: o1 abv 16.02.99: updating vertices tolerance when edge is updated
//    rln 03.03.99 S4135: removed unnecessary check for Geom_SphericalSurface (as not V-closed)
//: q8 abv 23.03.99: bm4_al_eye.stp #53710: avoid shifting pcurves for pseudo-seam
// #78 rln 12.03.99 S4135: checking spatial closure with prec
// #81 rln 15.03.99 S4135: for not SP edge chose the best result (either BRepLib or deviation only)
// #82 rln 16.03.99 S4135: avoiding setting input precision into the edge in FixAddPCurve
//: r4 abv 02.04.99 improving method FixSameParameter()
//: s5 abv 22.04.99 Adding debug printouts in catch {} blocks
//    abv 05.05.99 S4137: method CopyPCurves moved to ShapeBuild_Edge

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Construct;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Fixes the problems of one edge: removes a pcurve or a 3D curve inconsistent with the vertices, adds a missing
/// pcurve (by projection) or 3D curve, reverses a pcurve oriented against the 3D curve, makes the edge
/// SameParameter, and raises the tolerances of its vertices to cover the curves' ends.
/// </summary>
/// <remarks>
/// <para>A working object (decision 21). Each fix sets the status queried by <see cref="Status"/>.</para>
/// <para>
/// The fixes write into the edge they are given (its curves, ranges, flags, tolerances, and its vertices'), so OCCT's
/// in-place forms (<c>FixAddPCurve</c>, <c>FixSameParameter</c>, ...) are internal, for ported code fixing edges it
/// owns. Callers use the public twins (<see cref="AddPCurveFixed(in Edge, in Face, bool, out bool, double)"/>,
/// ...), which run the fix on a copy of a frozen edge, give back its unchanged parts and freeze the result
/// (decision 583). With a context, the vertex-tolerance fixes write only into new vertices recorded in the context,
/// as OCCT documents.
/// </para>
/// </remarks>
internal partial class EdgeFixer : Transient
{
    protected ReShapeBuilder?                  _context;
    protected int                                  _status;
    protected ProjectCurveOnSurface _projector;

    /// <summary>Empty constructor.</summary>
    public EdgeFixer()
    {
        _status    = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _projector = new ProjectCurveOnSurface();
    }

    /// <summary>Returns the projector used for recomputing missing pcurves (it may be tuned before the fixes).</summary>
    public ProjectCurveOnSurface Projector() => _projector;

    /// <summary>
    /// Removes the edge's pcurve on the face if its ends don't lie on the vertices. Writes into the edge.
    /// </summary>
    internal bool FixRemovePCurve(in Edge edge, in Face face)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return FixRemovePCurve(edge, s!, l);
    }

    /// <summary>
    /// Removes the edge's pcurve on the surface if its ends don't lie on the vertices. Writes into the edge.
    /// </summary>
    internal bool FixRemovePCurve(in Edge edge, Surface surface, in Placement location)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        var  ea     = new EdgeAnalysis();
        bool result = ea.CheckVerticesWithPCurve(edge, surface, location);
        if (result)
        {
            new BuildEdgeBuilder().RemovePCurve(edge, surface, location);
        }
        return result;
    }

    /// <summary>Removes the edge's 3D curve if its ends don't lie on the vertices. Writes into the edge.</summary>
    internal bool FixRemoveCurve3d(in Edge edge)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        var  ea     = new EdgeAnalysis();
        bool result = ea.CheckVerticesWithCurve3d(edge);
        if (result)
        {
            new BuildEdgeBuilder().RemoveCurve3d(edge);
        }
        return result;
    }

    /// <summary>
    /// Adds the edge's pcurve(s) on the face if missing (both for a seam), by projecting the 3D curve. Writes into
    /// the edge. Status: DONE1 pcurve added, DONE2 the projection was approximated, DONE3 the 3D curve was replaced;
    /// FAIL1 no 3D curve, FAIL2 the projection failed.
    /// </summary>
    internal bool FixAddPCurve(in Edge edge, in Face face, bool isSeam, double prec = 0.0)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return FixAddPCurve(edge, s!, l, isSeam, prec);
    }

    /// <summary>Adds the edge's pcurve(s) on the surface if missing; see the face form.</summary>
    internal bool FixAddPCurve(in Edge edge, Surface surface, in Placement location, bool isSeam,
                               double prec = 0.0)
    {
        Surface transSurf = surface;
        if (!location.IsIdentity)
        {
            Transform3D trsf = location.Transformation;
            transSurf = (Surface)surface.Transformed(trsf);
        }
        var sas = new SurfaceAnalysis(transSurf);
        return FixAddPCurve(edge, surface, location, isSeam, sas, prec);
    }

    /// <summary>Adds the edge's pcurve(s) on the face if missing, with an analyser of its surface.</summary>
    internal bool FixAddPCurve(in Edge edge, in Face face, bool isSeam, SurfaceAnalysis surfana,
                               double prec = 0.0)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return FixAddPCurve(edge, s!, l, isSeam, surfana, prec);
    }

    // #12 rln 17/03/98 making this method to be more general : if a curve is
    // parallel to one iso let us translate it parallely in the direction to another
    // iso (which is located farther from aC2d). Thus, the requirement for closeness
    // to the surface bounds may be avoided.
    // For example, instead of std::abs(theLoc.X()-uf) <= Tol) ... elseif (...-ul..)...
    // the comparison if (std::abs(theLoc.X()-uf) <= std::abs(theLoc.X()-ul)) .... can be used.

    // The reason for fix #12 is that seam is not certain to lie on the bound :
    // if a surface is periodic the whole contour may be shifted (e.g. ProSTEP,
    // file ug_exhaust-A.stp entity #284920)

    private static Curve2D TranslatePCurve(Surface aSurf, Curve2D aC2d, double aTol)
    {
        aSurf.ParameterBounds(out double uf, out double ul, out double vf, out double vl);

        // case of a line
        if (aC2d is LineCurve2D theL2d)
        {
            Point2D theLoc = theL2d.Location;
            Direction2D theDir = theL2d.Direction;

            var         newLoc    = new Point2D();
            LineCurve2D theNewL2d = theL2d;

            // case UClosed
            if (Math.Abs(theDir.X) <= aTol && Math.Abs(theDir.Y) >= aTol)
            {
                if (Math.Abs(theLoc.X - uf) < Math.Abs(theLoc.X - ul))
                {
                    newLoc = newLoc.WithCoord(theLoc.X + (ul - uf), theLoc.Y);
                }
                else
                {
                    newLoc = newLoc.WithCoord(theLoc.X - (ul - uf), theLoc.Y);
                }
                theNewL2d = new LineCurve2D(newLoc, theDir);
            }
            // case VClosed
            if (Math.Abs(theDir.X) >= aTol && Math.Abs(theDir.Y) <= aTol)
            {
                if (Math.Abs(theLoc.Y - vf) < Math.Abs(theLoc.Y - vl))
                {
                    newLoc = newLoc.WithCoord(theLoc.X, theLoc.Y + (vl - vf));
                }
                else
                {
                    newLoc = newLoc.WithCoord(theLoc.X, theLoc.Y - (vl - vf));
                }
                theNewL2d = new LineCurve2D(newLoc, theDir);
            }
            // TODO Other case not yet implemented
            return theNewL2d; //*theL2d;
        }
        else
        {
            // case of BSpline curve
            if (aC2d is not BSplineCurve2D bc)
            {
                return aC2d;
            }
            var      newC       = (BSplineCurve2D)bc.Copy();
            Point2D firstPoint = bc.StartPoint;
            Point2D lastPoint  = bc.EndPoint;
            var      theVector  = new Vector2D(firstPoint, lastPoint);
            Point2D p00        = new Point2D(uf, vf), p01 = new Point2D(uf, vl), p10 = new Point2D(ul, vf);
            var      vectIsoUF  = new Vector2D(p00, p01);
            var      vectIsoVF  = new Vector2D(p00, p10);

            var t = new Transform2D();
            if (theVector.IsParallel(vectIsoUF, aTol))
            {
                if (Math.Abs(firstPoint.X - uf) < Math.Abs(firstPoint.X - ul))
                {
                    t = t.WithTranslation(p00, p10);
                }
                else
                {
                    t = t.WithTranslation(p10, p00);
                }
                newC.Transform(t);
                return newC;
            }
            else if (theVector.IsParallel(vectIsoVF, aTol))
            {
                if (Math.Abs(firstPoint.Y - vf) < Math.Abs(firstPoint.Y - vl))
                {
                    t = t.WithTranslation(p00, p01);
                }
                else
                {
                    t = t.WithTranslation(p01, p00);
                }
                newC.Transform(t);
                return newC;
            }
        }
        // les courbes ne sont pas sur la couture
        return aC2d;
    }

    //: b0 abv 16 Feb 98: This is a copy of BRepLib::SameRange()
    // modified in order to be able to fix seam edges
    // NOTE: It is to be removed when is fixed either BRepLib::SameRange()
    // (concerning seam edges) or BRepLib::SameParameter() (concerning call
    // to GeomLib::SameRange() with 3d tolerance)

    private static void TempSameRange(in Edge anEdge, double tolerance)
    {
        var an_Iterator =
            new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)anEdge.TShape()!).ChangeCurves());

        Curve2D? curve2dPtr = null, newCurve2dPtr = null;
        Curve2D? curve2dPtr2 = null, newCurve2dPtr2 = null;

        // bool  IsSameRange = true //skl
        bool         first_time_in = true, has_curve, has_closed_curve;
        GCurve? geometric_representation_ptr;
        double       first, current_first, last, current_last;

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

                    if (Math.Abs(first - current_first) > Kerf.Engine.Foundation.Precision.PConfusion()
                        || //: b8 abv 20 Feb 98: Confusion -> PConfusion
                        Math.Abs(last - current_last) > Kerf.Engine.Foundation.Precision.PConfusion())
                    { //: b8
                        double oldFirst = 0.0, oldLast = 0.0; // skl
                        if (has_curve)
                        {
                            // pdn 20.05.99 Work around
                            oldFirst = geometric_representation_ptr.First();
                            oldLast = geometric_representation_ptr.Last();
                            // 15.11.2002 PTV OCC966
                            if (CurveAnalysis.IsPeriodic(curve2dPtr!))
                            {
                                var    tc    = new TrimmedCurve2D(curve2dPtr!, oldFirst, oldLast);
                                double shift = tc.FirstParameter - oldFirst;
                                oldFirst += shift;
                                oldLast += shift;
                            }
                            // pdn 30.06.2000 work around on beziers
                            double oldFirstCurve1 = oldFirst, oldLastCurve1 = oldLast;
                            if (curve2dPtr is BezierCurve2D)
                            {
                                double preci = Kerf.Engine.Foundation.Precision.PConfusion();
                                if (Math.Abs(oldFirst) > preci || Math.Abs(oldLast - 1) > preci)
                                {
                                    var bezier = (BezierCurve2D)curve2dPtr.Copy();
                                    bezier.Segment(oldFirst, oldLast);
                                    curve2dPtr = bezier;
                                }
                                oldFirstCurve1 = 0;
                                oldLastCurve1 = 1;
                            }

                            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(tolerance,
                                              curve2dPtr,
                                              oldFirstCurve1,
                                              oldLastCurve1,
                                              current_first,
                                              current_last,
                                              ref newCurve2dPtr);
                            geometric_representation_ptr.PCurve(newCurve2dPtr);
                        }
                        if (has_closed_curve)
                        {
                            double oldFirstCurve2 = oldFirst, oldLastCurve2 = oldLast;

                            if (curve2dPtr2 is BezierCurve2D)
                            {
                                double preci = Kerf.Engine.Foundation.Precision.PConfusion();
                                if (Math.Abs(oldFirst) > preci || Math.Abs(oldLast - 1) > preci)
                                {
                                    var bezier = (BezierCurve2D)curve2dPtr2.Copy();
                                    bezier.Segment(oldFirst, oldLast);
                                    curve2dPtr2 = bezier;
                                }
                                oldFirstCurve2 = 0;
                                oldLastCurve2 = 1;
                            }

                            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(tolerance,
                                              curve2dPtr2,
                                              oldFirstCurve2,
                                              oldLastCurve2,
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

    /// <summary>
    /// Adds the edge's pcurve(s) on the surface if missing, with an analyser of the surface; see the face form.
    /// </summary>
    internal bool FixAddPCurve(in Edge edge, Surface surface, in Placement location, bool isSeam,
                               SurfaceAnalysis surfana, double prec = 0.0)
    {
        Surface          surf = surface;
        SurfaceAnalysis sas  = surfana;
        var                   sae  = new EdgeAnalysis();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if ((!isSeam && sae.HasPCurve(edge, surf, location)) || (isSeam && sae.IsSeam(edge, surf, location)))
        {
            return false;
        }

        // PCurve on Plane not computed
        if (surf is PlaneSurface)
        {
            return false;
        }

        //  double step = 0;
        try
        {
            var b = new BrepBuilder();

            double      preci = (prec > 0.0 ? prec : Tool.Tolerance(edge));
            Curve3D? c3d   = Tool.Curve(edge, /*Loc,*/ out double first, out double last);
            if (c3d is null)
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                return false;
            }

            //  A present, on projette
            //  stat : 0 pas pu faire, 1 analytique, 2 approx
            Curve2D? c2d;
            double        a1, b1;
            if (!sae.HasPCurve(edge, surf, location))
            {
                double tolFirst = -1, tolLast = -1;
                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                if (!v1.IsNull)
                {
                    tolFirst = Tool.Tolerance(v1);
                }
                if (!v2.IsNull)
                {
                    tolLast = Tool.Tolerance(v2);
                }

                _projector.Init(sas, preci);
                _projector.InternalPerform(c3d, first, last, out c2d, tolFirst, tolLast);
                //  stat = 2 : reinterpoler la c3d ?
                if (_projector.Status(ExtendStatus.DONE4))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                }
                a1 = first;
                b1 = last;
            }
            else
            {
                sae.PCurve(edge, surf, location, out c2d, out a1, out b1, false);
            }

            if (isSeam)
            {
                // OCCT.NET: a null pcurve here is OCCT's null dereference, which OCC_CATCH_SIGNALS turns into a
                // Standard_Failure (FAIL2)
                if (c2d is null)
                {
                    throw new NullObjectException("ShapeFix_Edge::FixAddPCurve: no pcurve");
                }
                // On ne sait pas laquelle est Forward. Au PIF. La geometrie Forward
                // sera mise a jour dans ComputeWire
                var c2d2 = (Curve2D)c2d.Copy();
                //  ATTENTION : TranslatePCurve reconstruit une Line // bords, en
                //  intuitant U ou V ...
                //  Ici, on exploite les infos deja connues
                surf.ParameterBounds(out double uf, out double ul, out double vf, out double vl);
                // #4 rln 19/02/98 ProSTEP ug_exhaust-A.stp entity #284920 (toroidal surface)
                // #13 rln 17/03/98 (updating fix #4) call to TranslatePCurve in the case
                // when a surface is either u- and vclosed or neither u- nor vclosed
                // #78 rln 12.03.99 S4135: checking spatial closure with prec
                if (sas.IsUClosed(prec) && !sas.IsVClosed(prec)) // rln S4135 sphere is not considered as V-closed anymore
                { //: d9 abv 17 Mar 98: any sphere
                    var tranvec = new Vector2D(ul - uf, 0);
                    c2d2.Translate(tranvec);
                }
                else if (sas.IsVClosed(prec) && !sas.IsUClosed(prec))
                {
                    var tranvec = new Vector2D(0, vl - vf);
                    c2d2.Translate(tranvec);
                }
                else if (sas.IsUClosed() && sas.IsVClosed())
                { //: q8 abv 23 Mar 99: bm4_al_eye.stp #53710: avoid shifting pcurves for pseudo-seam
                    //      Doublement fermee (ex tore) : on lance la charge
                    c2d2 = TranslatePCurve(sas.Surface()!, c2d2, prec);
                }
                b.UpdateEdge(edge, c2d, c2d2, surf, location, 0.0); // #82 rln 16.03.99: preci
                b.Range(edge, surf, location, a1, b1);
            }
            else
            {
                b.UpdateEdge(edge, c2d, surf, location, 0.0); // #82 rln 16.03.99: preci
            }

            //  Conclusion
            if (_projector.Status(ExtendStatus.DONE3))
            {
                double g3dCFirst = c3d.FirstParameter;
                double g3dCLast  = c3d.LastParameter;
                b.UpdateEdge(edge, c3d, 0.0);
                b.Range(edge, g3dCFirst, g3dCLast, true);
            }
        } // end try
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }
        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        return true;
    }

    /// <summary>Adds the edge's 3D curve if missing (computed from a pcurve). Writes into the edge.</summary>
    internal bool FixAddCurve3d(in Edge edge)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        var ea = new EdgeAnalysis();
        if (Tool.Degenerated(edge) || ea.HasCurve3d(edge))
        {
            return false;
        }
        if (!Tool.SameRange(edge))
        {
            TempSameRange(edge, Kerf.Engine.Foundation.Precision.PConfusion());
        }

        if (!new BuildEdgeBuilder().BuildCurve3d(edge))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }
        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        return true;
    }

    /// <summary>
    /// Raises the tolerances of the edge's vertices to cover the ends of its 3D curve and of its pcurve on the face.
    /// With a context, the vertices are replaced by more tolerant copies recorded in it; otherwise they are written
    /// in place.
    /// </summary>
    internal bool FixVertexTolerance(in Edge edge, in Face face)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        Edge edgeCopy = edge;
        var         sae        = new EdgeAnalysis();
        if (Context() is not null)
        {
            Shape shape = Context()!.InternalApply(edge);
            if (shape.IsNull || shape.Type != ShapeType.Edge)
            {
                return false;
            }

            edgeCopy = ShapeCasts.Edge(shape);
        }

        if (!sae.CheckVertexTolerance(edgeCopy, face, out double toler1, out double toler2))
        {
            return false;
        }
        if (sae.Status(ExtendStatus.DONE1))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        if (sae.Status(ExtendStatus.DONE2))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        var           b  = new BrepBuilder();
        Vertex v1 = sae.FirstVertex(edgeCopy);
        Vertex v2 = sae.LastVertex(edgeCopy);
        if (Context() is not null)
        {
            Context()!.CopyVertex(v1, toler1);
            Context()!.CopyVertex(v2, toler2);
        }
        else
        {
            b.UpdateVertex(v1, toler1);
            b.UpdateVertex(v2, toler2);
        }
        return true;
    }

    /// <summary>
    /// Raises the tolerances of the edge's vertices to cover the ends of its curves; see the face form.
    /// </summary>
    internal bool FixVertexTolerance(in Edge edge)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        Edge edgeCopy = edge;
        var         sae        = new EdgeAnalysis();
        if (Context() is not null)
        {
            Shape shape = Context()!.InternalApply(edge);
            if (shape.IsNull || shape.Type != ShapeType.Edge)
            {
                return false;
            }

            edgeCopy = ShapeCasts.Edge(shape);
        }
        if (!sae.CheckVertexTolerance(edgeCopy, out double toler1, out double toler2))
        {
            return false;
        }
        if (sae.Status(ExtendStatus.DONE1))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        if (sae.Status(ExtendStatus.DONE2))
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        var           b  = new BrepBuilder();
        Vertex v1 = sae.FirstVertex(edgeCopy);
        Vertex v2 = sae.LastVertex(edgeCopy);
        if (Context() is not null)
        {
            Context()!.CopyVertex(v1, toler1);
            Context()!.CopyVertex(v2, toler2);
        }
        else
        {
            b.UpdateVertex(v1, toler1);
            b.UpdateVertex(v2, toler2);
        }
        return true;
    }

    /// <summary>Reverses the edge's pcurve on the face if it is oriented against the 3D curve. Writes into the edge.</summary>
    internal bool FixReversed2d(in Edge edge, in Face face)
    {
        Surface? s = Tool.Surface(face, out Placement l);
        return FixReversed2d(edge, s!, l);
    }

    /// <summary>
    /// Reverses the edge's pcurve on the surface if it is oriented against the 3D curve. Writes into the edge.
    /// </summary>
    internal bool FixReversed2d(in Edge edge, Surface surface, in Placement location)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        var ea = new EdgeAnalysis();
        ea.CheckCurve3dWithPCurve(edge, surface, location);
        if (ea.Status(ExtendStatus.FAIL1))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }
        if (ea.Status(ExtendStatus.FAIL2))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }
        if (!ea.Status(ExtendStatus.Done))
        {
            return false;
        }

        ea.PCurve(edge, surface, location, out Curve2D? c2d, out double f, out double l, false);
        // #46 rln 01.12.98 buc40130, entity 272 (4-th curve)
        double newf = c2d!.ReversedParameter(l), newl = c2d.ReversedParameter(f);
        // OCCT.NET: frozen. OCCT reverses the pcurve stored in the edge in place (any other edge sharing it is
        // reversed too); the port reverses a copy and puts it in the edge's representation in its place.
        Curve2D reversed = (Curve2D)c2d.Copy();
        reversed.Reverse();
        InternalReplacePCurveObject(edge, c2d, reversed);
        var b = new BrepBuilder();
        // will break seams!  B.UpdateEdge (edge, c2d, surface, location, Precision::Confusion());
        b.Range(edge, surface, location, newf, newl);
        // #51 rln 15.12.98 pro6562 entity 2788
        // Because of numerical accuracy the range on B-Splines (moreover, on any curve!)
        // the range is changed
        Tool.Range(edge, out double first, out double last);
        if (first != newf || last != newl)
        {
            b.SameRange(edge, false);
            b.SameParameter(edge, false);
        }
        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        return true;
    }

    // OCCT.NET: frozen. Puts theNew in place of the pcurve object theOld wherever the edge's representations hold it
    // (as OCCT's in-place change of theOld would be seen through them).
    internal static void InternalReplacePCurveObject(in Edge edge, Curve2D old, Curve2D theNew)
    {
        var it = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)edge.TShape()!).ChangeCurves());
        for (; it.More(); it.Next())
        {
            if (it.Value() is not GCurve gc)
            {
                continue;
            }
            if (gc.IsCurveOnSurface() && ReferenceEquals(gc.PCurve(), old))
            {
                gc.PCurve(theNew);
            }
            if (gc.IsCurveOnClosedSurface && ReferenceEquals(gc.PCurve2(), old))
            {
                gc.PCurve2(theNew);
            }
        }
    }

    /// <summary>
    /// Makes the edge SameParameter (and SameRange), choosing the better of BRepLib's result and the deviation of the
    /// original pcurves, and raises the tolerances to the deviation. Writes into the edge and its vertices. Status:
    /// DONE1 tolerance raised, DONE2 not SameParameter and BRepLib failed, DONE3 BRepLib's result compared, DONE5 its
    /// pcurves taken; FAIL1 deviation not computed, FAIL2 BRepLib failed.
    /// </summary>
    internal bool FixSameParameter(in Edge edge, double tolerance = 0.0)
    {
        var emptyFace = new Face();
        return FixSameParameter(edge, emptyFace, tolerance);
    }

    /// <summary>Makes the edge SameParameter, checking its pcurve on the face; see the other form.</summary>
    internal bool FixSameParameter(in Edge edge, in Face face, double tolerance = 0.0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        if (Tool.Degenerated(edge))
        {
            var bd = new BrepBuilder();
            if (!Tool.SameRange(edge))
            {
                TempSameRange(edge, Kerf.Engine.Foundation.Precision.PConfusion());
            }
            bd.SameParameter(edge, true);
            return false;
        }

        var sfst = new ShapeToleranceFixer();
        var sae  = new EdgeAnalysis();
        var b    = new BrepBuilder();

        var           copyedge = new Edge();
        Vertex v1       = sae.FirstVertex(edge);
        Vertex v2       = sae.LastVertex(edge);
        double        tolFV    = (v1.IsNull ? 0.0 : Tool.Tolerance(v1));
        double        tolLV    = (v2.IsNull ? 0.0 : Tool.Tolerance(v2));
        double        tol      = Tool.Tolerance(edge);

        bool wasSP = Tool.SameParameter(edge), sp = false;
        {
            try
            {
                if (!Tool.SameRange(edge))
                {
                    TempSameRange(edge, Kerf.Engine.Foundation.Precision.PConfusion());
                }
                // #81 rln 15.03.99 S4135: for not SP edge choose the best result (either BRepLib or deviation
                // only)
                if (!wasSP)
                {
                    // create copyedge as copy of edge with the same vertices and copy of pcurves on the same
                    // surface(s)
                    copyedge = new BuildEdgeBuilder().InternalCopy(edge, false);
                    b.SameParameter(copyedge, false);
                    // ShapeBuild_Edge::Copy() may change 3D curve range (if it's outside of its period).
                    // In this case pcurves in BRepLib::SameParameter() will be changed as well
                    // and later ShapeBuild_Edge::CopyPCurves() will copy pcurves keeping original range.
                    // To prevent this discrepancy we enforce original 3D range.
                    Tool.Range(edge, out double f, out double l);
                    b.Range(copyedge, f, l, true); // only 3D
                    BrepLibrary.SameParameter(copyedge, (tolerance >= Kerf.Engine.Foundation.Precision.Confusion() ? tolerance : tol));
                    sp = Tool.SameParameter(copyedge);
                    if (!sp)
                    {
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                    }
                }
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }
        }

        // compute deviation on the original pcurves
        double maxdev = 0.0;
        b.SameParameter(edge, true);

        // Should check all pcurves in case of non-sameparametrization input.
        Face aFace = face;
        if (!wasSP)
        {
            aFace = new Face();
        }

        sae.CheckSameParameter(edge, aFace, ref maxdev);
        if (sae.Status(ExtendStatus.FAIL2))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }

        // if BRepLib was OK, compare and select the best variant
        if (sp)
        {
            double brlTol = Tool.Tolerance(copyedge), brlDev = 0.0;
            sae.CheckSameParameter(copyedge, ref brlDev);
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            if (brlTol < brlDev)
            {
                brlTol = brlDev;
            }

            // chose the best result
            if (brlTol < maxdev)
            {
                if (sae.Status(ExtendStatus.FAIL2))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                }
                // copy pcurves and tolerances from copyedge
                new BuildEdgeBuilder().CopyPCurves(edge, copyedge);
                maxdev = brlTol;
                sfst.SetTolerance(edge, brlTol, ShapeType.Edge);
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
            }
        }

        // restore tolerances because they could be modified by BRepLib
        if (!v1.IsNull)
        {
            sfst.SetTolerance(v1, Max(maxdev, tolFV), ShapeType.Vertex);
        }
        if (!v2.IsNull)
        {
            sfst.SetTolerance(v2, Max(maxdev, tolLV), ShapeType.Vertex);
        }

        if (maxdev > tol)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            b.UpdateEdge(edge, maxdev);
            FixVertexTolerance(edge);
        }

        if (!wasSP && !sp)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        return Status(ExtendStatus.Done);
    }

    /// <summary>Returns the status (in the ShapeExtend_Status sense) of the last fix.</summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Returns the context.</summary>
    public ReShapeBuilder? Context() => _context;

    /// <summary>Sets the context.</summary>
    public void SetContext(ReShapeBuilder? context)
    {
        _context = context;
    }
}
