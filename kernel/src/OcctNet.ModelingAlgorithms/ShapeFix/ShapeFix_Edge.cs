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

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
/// owns. Callers use the public twins (<see cref="AddPCurveFixed(in TopoDS_Edge, in TopoDS_Face, bool, out bool, double)"/>,
/// ...), which run the fix on a copy of a frozen edge, give back its unchanged parts and freeze the result
/// (decision 583). With a context, the vertex-tolerance fixes write only into new vertices recorded in the context,
/// as OCCT documents.
/// </para>
/// </remarks>
public partial class ShapeFix_Edge : Standard_Transient
{
    protected ShapeBuild_ReShape?                  myContext;
    protected int                                  myStatus;
    protected ShapeConstruct_ProjectCurveOnSurface myProjector;

    /// <summary>Empty constructor.</summary>
    public ShapeFix_Edge()
    {
        myStatus    = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myProjector = new ShapeConstruct_ProjectCurveOnSurface();
    }

    /// <summary>Returns the projector used for recomputing missing pcurves (it may be tuned before the fixes).</summary>
    public ShapeConstruct_ProjectCurveOnSurface Projector() => myProjector;

    /// <summary>
    /// Removes the edge's pcurve on the face if its ends don't lie on the vertices. Writes into the edge.
    /// </summary>
    internal bool FixRemovePCurve(in TopoDS_Edge edge, in TopoDS_Face face)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return FixRemovePCurve(edge, S!, L);
    }

    /// <summary>
    /// Removes the edge's pcurve on the surface if its ends don't lie on the vertices. Writes into the edge.
    /// </summary>
    internal bool FixRemovePCurve(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        var  EA     = new ShapeAnalysis_Edge();
        bool result = EA.CheckVerticesWithPCurve(edge, surface, location);
        if (result)
        {
            new ShapeBuild_Edge().RemovePCurve(edge, surface, location);
        }
        return result;
    }

    /// <summary>Removes the edge's 3D curve if its ends don't lie on the vertices. Writes into the edge.</summary>
    internal bool FixRemoveCurve3d(in TopoDS_Edge edge)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        var  EA     = new ShapeAnalysis_Edge();
        bool result = EA.CheckVerticesWithCurve3d(edge);
        if (result)
        {
            new ShapeBuild_Edge().RemoveCurve3d(edge);
        }
        return result;
    }

    /// <summary>
    /// Adds the edge's pcurve(s) on the face if missing (both for a seam), by projecting the 3D curve. Writes into
    /// the edge. Status: DONE1 pcurve added, DONE2 the projection was approximated, DONE3 the 3D curve was replaced;
    /// FAIL1 no 3D curve, FAIL2 the projection failed.
    /// </summary>
    internal bool FixAddPCurve(in TopoDS_Edge edge, in TopoDS_Face face, bool isSeam, double prec = 0.0)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return FixAddPCurve(edge, S!, L, isSeam, prec);
    }

    /// <summary>Adds the edge's pcurve(s) on the surface if missing; see the face form.</summary>
    internal bool FixAddPCurve(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location, bool isSeam,
                               double prec = 0.0)
    {
        Geom_Surface aTransSurf = surface;
        if (!location.IsIdentity())
        {
            gp_Trsf aTrsf = location.Transformation();
            aTransSurf = (Geom_Surface)surface.Transformed(aTrsf);
        }
        var sas = new ShapeAnalysis_Surface(aTransSurf);
        return FixAddPCurve(edge, surface, location, isSeam, sas, prec);
    }

    /// <summary>Adds the edge's pcurve(s) on the face if missing, with an analyser of its surface.</summary>
    internal bool FixAddPCurve(in TopoDS_Edge edge, in TopoDS_Face face, bool isSeam, ShapeAnalysis_Surface surfana,
                               double prec = 0.0)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return FixAddPCurve(edge, S!, L, isSeam, surfana, prec);
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

    private static Geom2d_Curve TranslatePCurve(Geom_Surface aSurf, Geom2d_Curve aC2d, double aTol)
    {
        aSurf.Bounds(out double uf, out double ul, out double vf, out double vl);

        // case of a line
        if (aC2d is Geom2d_Line theL2d)
        {
            gp_Pnt2d theLoc = theL2d.Location();
            gp_Dir2d theDir = theL2d.Direction();

            var         newLoc    = new gp_Pnt2d();
            Geom2d_Line theNewL2d = theL2d;

            // case UClosed
            if (Math.Abs(theDir.X()) <= aTol && Math.Abs(theDir.Y()) >= aTol)
            {
                if (Math.Abs(theLoc.X() - uf) < Math.Abs(theLoc.X() - ul))
                {
                    newLoc.SetCoord(theLoc.X() + (ul - uf), theLoc.Y());
                }
                else
                {
                    newLoc.SetCoord(theLoc.X() - (ul - uf), theLoc.Y());
                }
                theNewL2d = new Geom2d_Line(newLoc, theDir);
            }
            // case VClosed
            if (Math.Abs(theDir.X()) >= aTol && Math.Abs(theDir.Y()) <= aTol)
            {
                if (Math.Abs(theLoc.Y() - vf) < Math.Abs(theLoc.Y() - vl))
                {
                    newLoc.SetCoord(theLoc.X(), theLoc.Y() + (vl - vf));
                }
                else
                {
                    newLoc.SetCoord(theLoc.X(), theLoc.Y() - (vl - vf));
                }
                theNewL2d = new Geom2d_Line(newLoc, theDir);
            }
            // TODO Other case not yet implemented
            return theNewL2d; //*theL2d;
        }
        else
        {
            // case of BSpline curve
            if (aC2d is not Geom2d_BSplineCurve aBC)
            {
                return aC2d;
            }
            var      newC       = (Geom2d_BSplineCurve)aBC.Copy();
            gp_Pnt2d FirstPoint = aBC.StartPoint();
            gp_Pnt2d LastPoint  = aBC.EndPoint();
            var      theVector  = new gp_Vec2d(FirstPoint, LastPoint);
            gp_Pnt2d p00        = new gp_Pnt2d(uf, vf), p01 = new gp_Pnt2d(uf, vl), p10 = new gp_Pnt2d(ul, vf);
            var      VectIsoUF  = new gp_Vec2d(p00, p01);
            var      VectIsoVF  = new gp_Vec2d(p00, p10);

            var T = new gp_Trsf2d();
            if (theVector.IsParallel(VectIsoUF, aTol))
            {
                if (Math.Abs(FirstPoint.X() - uf) < Math.Abs(FirstPoint.X() - ul))
                {
                    T.SetTranslation(p00, p10);
                }
                else
                {
                    T.SetTranslation(p10, p00);
                }
                newC.Transform(T);
                return newC;
            }
            else if (theVector.IsParallel(VectIsoVF, aTol))
            {
                if (Math.Abs(FirstPoint.Y() - vf) < Math.Abs(FirstPoint.Y() - vl))
                {
                    T.SetTranslation(p00, p01);
                }
                else
                {
                    T.SetTranslation(p01, p00);
                }
                newC.Transform(T);
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

    private static void TempSameRange(in TopoDS_Edge AnEdge, double Tolerance)
    {
        var an_Iterator =
            new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)AnEdge.TShape()!).ChangeCurves());

        Geom2d_Curve? Curve2dPtr = null, NewCurve2dPtr = null;
        Geom2d_Curve? Curve2dPtr2 = null, NewCurve2dPtr2 = null;

        // bool  IsSameRange = true //skl
        bool         first_time_in = true, has_curve, has_closed_curve;
        BRep_GCurve? geometric_representation_ptr;
        double       first, current_first, last, current_last;

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

                    if (Math.Abs(first - current_first) > Occt.Precision.PConfusion()
                        || //: b8 abv 20 Feb 98: Confusion -> PConfusion
                        Math.Abs(last - current_last) > Occt.Precision.PConfusion())
                    { //: b8
                        double oldFirst = 0.0, oldLast = 0.0; // skl
                        if (has_curve)
                        {
                            // pdn 20.05.99 Work around
                            oldFirst = geometric_representation_ptr.First();
                            oldLast = geometric_representation_ptr.Last();
                            // 15.11.2002 PTV OCC966
                            if (ShapeAnalysis_Curve.IsPeriodic(Curve2dPtr!))
                            {
                                var    tc    = new Geom2d_TrimmedCurve(Curve2dPtr!, oldFirst, oldLast);
                                double shift = tc.FirstParameter() - oldFirst;
                                oldFirst += shift;
                                oldLast += shift;
                            }
                            // pdn 30.06.2000 work around on beziers
                            double oldFirstCurve1 = oldFirst, oldLastCurve1 = oldLast;
                            if (Curve2dPtr is Geom2d_BezierCurve)
                            {
                                double preci = Occt.Precision.PConfusion();
                                if (Math.Abs(oldFirst) > preci || Math.Abs(oldLast - 1) > preci)
                                {
                                    var bezier = (Geom2d_BezierCurve)Curve2dPtr.Copy();
                                    bezier.Segment(oldFirst, oldLast);
                                    Curve2dPtr = bezier;
                                }
                                oldFirstCurve1 = 0;
                                oldLastCurve1 = 1;
                            }

                            GeomLib.SameRange(Tolerance,
                                              Curve2dPtr,
                                              oldFirstCurve1,
                                              oldLastCurve1,
                                              current_first,
                                              current_last,
                                              ref NewCurve2dPtr);
                            geometric_representation_ptr.PCurve(NewCurve2dPtr);
                        }
                        if (has_closed_curve)
                        {
                            double oldFirstCurve2 = oldFirst, oldLastCurve2 = oldLast;

                            if (Curve2dPtr2 is Geom2d_BezierCurve)
                            {
                                double preci = Occt.Precision.PConfusion();
                                if (Math.Abs(oldFirst) > preci || Math.Abs(oldLast - 1) > preci)
                                {
                                    var bezier = (Geom2d_BezierCurve)Curve2dPtr2.Copy();
                                    bezier.Segment(oldFirst, oldLast);
                                    Curve2dPtr2 = bezier;
                                }
                                oldFirstCurve2 = 0;
                                oldLastCurve2 = 1;
                            }

                            GeomLib.SameRange(Tolerance,
                                              Curve2dPtr2,
                                              oldFirstCurve2,
                                              oldLastCurve2,
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

    /// <summary>
    /// Adds the edge's pcurve(s) on the surface if missing, with an analyser of the surface; see the face form.
    /// </summary>
    internal bool FixAddPCurve(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location, bool isSeam,
                               ShapeAnalysis_Surface surfana, double prec = 0.0)
    {
        Geom_Surface          surf = surface;
        ShapeAnalysis_Surface sas  = surfana;
        var                   sae  = new ShapeAnalysis_Edge();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if ((!isSeam && sae.HasPCurve(edge, surf, location)) || (isSeam && sae.IsSeam(edge, surf, location)))
        {
            return false;
        }

        // PCurve on Plane not computed
        if (surf is Geom_Plane)
        {
            return false;
        }

        //  double step = 0;
        try
        {
            var B = new BRep_Builder();

            double      preci = (prec > 0.0 ? prec : BRep_Tool.Tolerance(edge));
            Geom_Curve? c3d   = BRep_Tool.Curve(edge, /*Loc,*/ out double First, out double Last);
            if (c3d is null)
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                return false;
            }

            //  A present, on projette
            //  stat : 0 pas pu faire, 1 analytique, 2 approx
            Geom2d_Curve? c2d;
            double        a1, b1;
            if (!sae.HasPCurve(edge, surf, location))
            {
                double TolFirst = -1, TolLast = -1;
                TopExp.Vertices(edge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (!V1.IsNull())
                {
                    TolFirst = BRep_Tool.Tolerance(V1);
                }
                if (!V2.IsNull())
                {
                    TolLast = BRep_Tool.Tolerance(V2);
                }

                myProjector.Init(sas, preci);
                myProjector.InternalPerform(c3d, First, Last, out c2d, TolFirst, TolLast);
                //  stat = 2 : reinterpoler la c3d ?
                if (myProjector.Status(ShapeExtend_DONE4))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                }
                a1 = First;
                b1 = Last;
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
                    throw new Standard_NullObject("ShapeFix_Edge::FixAddPCurve: no pcurve");
                }
                // On ne sait pas laquelle est Forward. Au PIF. La geometrie Forward
                // sera mise a jour dans ComputeWire
                var c2d2 = (Geom2d_Curve)c2d.Copy();
                //  ATTENTION : TranslatePCurve reconstruit une Line // bords, en
                //  intuitant U ou V ...
                //  Ici, on exploite les infos deja connues
                surf.Bounds(out double uf, out double ul, out double vf, out double vl);
                // #4 rln 19/02/98 ProSTEP ug_exhaust-A.stp entity #284920 (toroidal surface)
                // #13 rln 17/03/98 (updating fix #4) call to TranslatePCurve in the case
                // when a surface is either u- and vclosed or neither u- nor vclosed
                // #78 rln 12.03.99 S4135: checking spatial closure with prec
                if (sas.IsUClosed(prec) && !sas.IsVClosed(prec)) // rln S4135 sphere is not considered as V-closed anymore
                { //: d9 abv 17 Mar 98: any sphere
                    var tranvec = new gp_Vec2d(ul - uf, 0);
                    c2d2.Translate(tranvec);
                }
                else if (sas.IsVClosed(prec) && !sas.IsUClosed(prec))
                {
                    var tranvec = new gp_Vec2d(0, vl - vf);
                    c2d2.Translate(tranvec);
                }
                else if (sas.IsUClosed() && sas.IsVClosed())
                { //: q8 abv 23 Mar 99: bm4_al_eye.stp #53710: avoid shifting pcurves for pseudo-seam
                    //      Doublement fermee (ex tore) : on lance la charge
                    c2d2 = TranslatePCurve(sas.Surface()!, c2d2, prec);
                }
                B.UpdateEdge(edge, c2d, c2d2, surf, location, 0.0); // #82 rln 16.03.99: preci
                B.Range(edge, surf, location, a1, b1);
            }
            else
            {
                B.UpdateEdge(edge, c2d, surf, location, 0.0); // #82 rln 16.03.99: preci
            }

            //  Conclusion
            if (myProjector.Status(ShapeExtend_DONE3))
            {
                double G3dCFirst = c3d.FirstParameter();
                double G3dCLast  = c3d.LastParameter();
                B.UpdateEdge(edge, c3d, 0.0);
                B.Range(edge, G3dCFirst, G3dCLast, true);
            }
        } // end try
        catch (Standard_Failure)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }
        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        return true;
    }

    /// <summary>Adds the edge's 3D curve if missing (computed from a pcurve). Writes into the edge.</summary>
    internal bool FixAddCurve3d(in TopoDS_Edge edge)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        var EA = new ShapeAnalysis_Edge();
        if (BRep_Tool.Degenerated(edge) || EA.HasCurve3d(edge))
        {
            return false;
        }
        if (!BRep_Tool.SameRange(edge))
        {
            TempSameRange(edge, Occt.Precision.PConfusion());
        }

        if (!new ShapeBuild_Edge().BuildCurve3d(edge))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }
        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        return true;
    }

    /// <summary>
    /// Raises the tolerances of the edge's vertices to cover the ends of its 3D curve and of its pcurve on the face.
    /// With a context, the vertices are replaced by more tolerant copies recorded in it; otherwise they are written
    /// in place.
    /// </summary>
    internal bool FixVertexTolerance(in TopoDS_Edge edge, in TopoDS_Face face)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        TopoDS_Edge anEdgeCopy = edge;
        var         sae        = new ShapeAnalysis_Edge();
        if (Context() is not null)
        {
            TopoDS_Shape aShape = Context()!.InternalApply(edge);
            if (aShape.IsNull() || aShape.ShapeType() != TopAbs_EDGE)
            {
                return false;
            }

            anEdgeCopy = TopoDS.Edge(aShape);
        }

        if (!sae.CheckVertexTolerance(anEdgeCopy, face, out double toler1, out double toler2))
        {
            return false;
        }
        if (sae.Status(ShapeExtend_DONE1))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        if (sae.Status(ShapeExtend_DONE2))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        var           B  = new BRep_Builder();
        TopoDS_Vertex V1 = sae.FirstVertex(anEdgeCopy);
        TopoDS_Vertex V2 = sae.LastVertex(anEdgeCopy);
        if (Context() is not null)
        {
            Context()!.CopyVertex(V1, toler1);
            Context()!.CopyVertex(V2, toler2);
        }
        else
        {
            B.UpdateVertex(V1, toler1);
            B.UpdateVertex(V2, toler2);
        }
        return true;
    }

    /// <summary>
    /// Raises the tolerances of the edge's vertices to cover the ends of its curves; see the face form.
    /// </summary>
    internal bool FixVertexTolerance(in TopoDS_Edge edge)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        TopoDS_Edge anEdgeCopy = edge;
        var         sae        = new ShapeAnalysis_Edge();
        if (Context() is not null)
        {
            TopoDS_Shape aShape = Context()!.InternalApply(edge);
            if (aShape.IsNull() || aShape.ShapeType() != TopAbs_EDGE)
            {
                return false;
            }

            anEdgeCopy = TopoDS.Edge(aShape);
        }
        if (!sae.CheckVertexTolerance(anEdgeCopy, out double toler1, out double toler2))
        {
            return false;
        }
        if (sae.Status(ShapeExtend_DONE1))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        if (sae.Status(ShapeExtend_DONE2))
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        var           B  = new BRep_Builder();
        TopoDS_Vertex V1 = sae.FirstVertex(anEdgeCopy);
        TopoDS_Vertex V2 = sae.LastVertex(anEdgeCopy);
        if (Context() is not null)
        {
            Context()!.CopyVertex(V1, toler1);
            Context()!.CopyVertex(V2, toler2);
        }
        else
        {
            B.UpdateVertex(V1, toler1);
            B.UpdateVertex(V2, toler2);
        }
        return true;
    }

    /// <summary>Reverses the edge's pcurve on the face if it is oriented against the 3D curve. Writes into the edge.</summary>
    internal bool FixReversed2d(in TopoDS_Edge edge, in TopoDS_Face face)
    {
        Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
        return FixReversed2d(edge, S!, L);
    }

    /// <summary>
    /// Reverses the edge's pcurve on the surface if it is oriented against the 3D curve. Writes into the edge.
    /// </summary>
    internal bool FixReversed2d(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        var EA = new ShapeAnalysis_Edge();
        EA.CheckCurve3dWithPCurve(edge, surface, location);
        if (EA.Status(ShapeExtend_FAIL1))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (EA.Status(ShapeExtend_FAIL2))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }
        if (!EA.Status(ShapeExtend_DONE))
        {
            return false;
        }

        EA.PCurve(edge, surface, location, out Geom2d_Curve? c2d, out double f, out double l, false);
        // #46 rln 01.12.98 buc40130, entity 272 (4-th curve)
        double newf = c2d!.ReversedParameter(l), newl = c2d.ReversedParameter(f);
        // OCCT.NET: frozen. OCCT reverses the pcurve stored in the edge in place (any other edge sharing it is
        // reversed too); the port reverses a copy and puts it in the edge's representation in its place.
        Geom2d_Curve aReversed = (Geom2d_Curve)c2d.Copy();
        aReversed.Reverse();
        InternalReplacePCurveObject(edge, c2d, aReversed);
        var B = new BRep_Builder();
        // will break seams!  B.UpdateEdge (edge, c2d, surface, location, Precision::Confusion());
        B.Range(edge, surface, location, newf, newl);
        // #51 rln 15.12.98 pro6562 entity 2788
        // Because of numerical accuracy the range on B-Splines (moreover, on any curve!)
        // the range is changed
        BRep_Tool.Range(edge, out double first, out double last);
        if (first != newf || last != newl)
        {
            B.SameRange(edge, false);
            B.SameParameter(edge, false);
        }
        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        return true;
    }

    // OCCT.NET: frozen. Puts theNew in place of the pcurve object theOld wherever the edge's representations hold it
    // (as OCCT's in-place change of theOld would be seen through them).
    internal static void InternalReplacePCurveObject(in TopoDS_Edge edge, Geom2d_Curve theOld, Geom2d_Curve theNew)
    {
        var anIt = new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)edge.TShape()!).ChangeCurves());
        for (; anIt.More(); anIt.Next())
        {
            if (anIt.Value() is not BRep_GCurve aGC)
            {
                continue;
            }
            if (aGC.IsCurveOnSurface() && ReferenceEquals(aGC.PCurve(), theOld))
            {
                aGC.PCurve(theNew);
            }
            if (aGC.IsCurveOnClosedSurface() && ReferenceEquals(aGC.PCurve2(), theOld))
            {
                aGC.PCurve2(theNew);
            }
        }
    }

    /// <summary>
    /// Makes the edge SameParameter (and SameRange), choosing the better of BRepLib's result and the deviation of the
    /// original pcurves, and raises the tolerances to the deviation. Writes into the edge and its vertices. Status:
    /// DONE1 tolerance raised, DONE2 not SameParameter and BRepLib failed, DONE3 BRepLib's result compared, DONE5 its
    /// pcurves taken; FAIL1 deviation not computed, FAIL2 BRepLib failed.
    /// </summary>
    internal bool FixSameParameter(in TopoDS_Edge edge, double tolerance = 0.0)
    {
        var anEmptyFace = new TopoDS_Face();
        return FixSameParameter(edge, anEmptyFace, tolerance);
    }

    /// <summary>Makes the edge SameParameter, checking its pcurve on the face; see the other form.</summary>
    internal bool FixSameParameter(in TopoDS_Edge edge, in TopoDS_Face face, double tolerance = 0.0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        if (BRep_Tool.Degenerated(edge))
        {
            var BD = new BRep_Builder();
            if (!BRep_Tool.SameRange(edge))
            {
                TempSameRange(edge, Occt.Precision.PConfusion());
            }
            BD.SameParameter(edge, true);
            return false;
        }

        var SFST = new ShapeFix_ShapeTolerance();
        var sae  = new ShapeAnalysis_Edge();
        var B    = new BRep_Builder();

        var           copyedge = new TopoDS_Edge();
        TopoDS_Vertex V1       = sae.FirstVertex(edge);
        TopoDS_Vertex V2       = sae.LastVertex(edge);
        double        TolFV    = (V1.IsNull() ? 0.0 : BRep_Tool.Tolerance(V1));
        double        TolLV    = (V2.IsNull() ? 0.0 : BRep_Tool.Tolerance(V2));
        double        tol      = BRep_Tool.Tolerance(edge);

        bool wasSP = BRep_Tool.SameParameter(edge), SP = false;
        {
            try
            {
                if (!BRep_Tool.SameRange(edge))
                {
                    TempSameRange(edge, Occt.Precision.PConfusion());
                }
                // #81 rln 15.03.99 S4135: for not SP edge choose the best result (either BRepLib or deviation
                // only)
                if (!wasSP)
                {
                    // create copyedge as copy of edge with the same vertices and copy of pcurves on the same
                    // surface(s)
                    copyedge = new ShapeBuild_Edge().InternalCopy(edge, false);
                    B.SameParameter(copyedge, false);
                    // ShapeBuild_Edge::Copy() may change 3D curve range (if it's outside of its period).
                    // In this case pcurves in BRepLib::SameParameter() will be changed as well
                    // and later ShapeBuild_Edge::CopyPCurves() will copy pcurves keeping original range.
                    // To prevent this discrepancy we enforce original 3D range.
                    BRep_Tool.Range(edge, out double aF, out double aL);
                    B.Range(copyedge, aF, aL, true); // only 3D
                    BRepLib.SameParameter(copyedge, (tolerance >= Occt.Precision.Confusion() ? tolerance : tol));
                    SP = BRep_Tool.SameParameter(copyedge);
                    if (!SP)
                    {
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                    }
                }
            }
            catch (Standard_Failure)
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }
        }

        // compute deviation on the original pcurves
        double maxdev = 0.0;
        B.SameParameter(edge, true);

        // Should check all pcurves in case of non-sameparametrization input.
        TopoDS_Face aFace = face;
        if (!wasSP)
        {
            aFace = new TopoDS_Face();
        }

        sae.CheckSameParameter(edge, aFace, ref maxdev);
        if (sae.Status(ShapeExtend_FAIL2))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }

        // if BRepLib was OK, compare and select the best variant
        if (SP)
        {
            double BRLTol = BRep_Tool.Tolerance(copyedge), BRLDev = 0.0;
            sae.CheckSameParameter(copyedge, ref BRLDev);
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            if (BRLTol < BRLDev)
            {
                BRLTol = BRLDev;
            }

            // chose the best result
            if (BRLTol < maxdev)
            {
                if (sae.Status(ShapeExtend_FAIL2))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                }
                // copy pcurves and tolerances from copyedge
                new ShapeBuild_Edge().CopyPCurves(edge, copyedge);
                maxdev = BRLTol;
                SFST.SetTolerance(edge, BRLTol, TopAbs_EDGE);
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
            }
        }

        // restore tolerances because they could be modified by BRepLib
        if (!V1.IsNull())
        {
            SFST.SetTolerance(V1, Max(maxdev, TolFV), TopAbs_VERTEX);
        }
        if (!V2.IsNull())
        {
            SFST.SetTolerance(V2, Max(maxdev, TolLV), TopAbs_VERTEX);
        }

        if (maxdev > tol)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            B.UpdateEdge(edge, maxdev);
            FixVertexTolerance(edge);
        }

        if (!wasSP && !SP)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        return Status(ShapeExtend_DONE);
    }

    /// <summary>Returns the status (in the ShapeExtend_Status sense) of the last fix.</summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Returns the context.</summary>
    public ShapeBuild_ReShape? Context() => myContext;

    /// <summary>Sets the context.</summary>
    public void SetContext(ShapeBuild_ReShape? context)
    {
        myContext = context;
    }
}
