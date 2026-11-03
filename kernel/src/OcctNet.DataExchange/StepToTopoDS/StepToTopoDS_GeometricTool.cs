// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_GeometricTool.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_GeometricTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>Algorithmic services specific to the mapping of STEP to OCCT topology.</summary>
public static class StepToTopoDS_GeometricTool
{
    /// <summary>
    /// The index of the first pcurve of the surface curve, after <paramref name="last"/>, lying on the surface (0 if
    /// none); <paramref name="thePCurve"/> is that pcurve (null if none).
    /// </summary>
    public static int PCurve(StepGeom_SurfaceCurve SurfCurve, StepGeom_Surface BasisSurf, out StepGeom_Pcurve thePCurve, int last = 0)
    {
        int NbAssGeom = SurfCurve.NbAssociatedGeometry();
        thePCurve = null;
        for (int i = last + 1; i <= NbAssGeom; i++)
        {
            thePCurve = SurfCurve.AssociatedGeometryValue(i)?.Pcurve();
            if (thePCurve is not null)
            {
                if (thePCurve.BasisSurface() == BasisSurf)
                {
                    return i;
                }
            }
        }

        thePCurve = null;
        return 0;
    }

    /// <summary>
    /// True if the surface curve is a seam curve: a seam_curve, or two oriented edges of the same wire referencing the
    /// same edge, whose two pcurves lie on the surface.
    /// </summary>
    public static bool IsSeamCurve(StepGeom_SurfaceCurve SurfCurve, StepGeom_Surface Surf, StepShape_Edge StepEdge, StepShape_EdgeLoop EdgeLoop)
    {
        if (SurfCurve is StepGeom_SeamCurve)
        {
            return true;
        }

        if (SurfCurve.NbAssociatedGeometry() != 2)
        {
            return false;
        }

        StepGeom_Pcurve StepPCurve1 = SurfCurve.AssociatedGeometryValue(1)?.Pcurve();
        StepGeom_Pcurve StepPCurve2 = SurfCurve.AssociatedGeometryValue(2)?.Pcurve();

        // Do the two pcurves lye on the same surface ?
        if ((StepPCurve1 is not null && StepPCurve2 is not null) && (StepPCurve1.BasisSurface() == Surf)
            && (StepPCurve2.BasisSurface() == Surf))
        {
            int NbEdge = EdgeLoop.NbEdgeList();
            int nbOE = 0;

            StepShape_OrientedEdge OrEdge;
            for (int i = 1; i <= NbEdge; i++)
            {
                OrEdge = EdgeLoop.EdgeListValue(i);
                if (StepEdge == OrEdge.EdgeElement())
                {
                    nbOE++;
                }
            }

            // two oriented edges of the same wire share the same edge
            if (nbOE == 2)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// True if the two pcurves of the surface curve lie on the same surface but in different wires: typical of CATIA
    /// BRep, a cylinder coded as two faces on the same closed BSpline surface, not closed within gp_Resolution.
    /// </summary>
    public static bool IsLikeSeam(StepGeom_SurfaceCurve SurfCurve, StepGeom_Surface Surf, StepShape_Edge StepEdge, StepShape_EdgeLoop EdgeLoop)
    {
        if (SurfCurve.NbAssociatedGeometry() != 2)
        {
            return false;
        }

        StepGeom_Pcurve StepPCurve1 = SurfCurve.AssociatedGeometryValue(1)?.Pcurve();
        StepGeom_Pcurve StepPCurve2 = SurfCurve.AssociatedGeometryValue(2)?.Pcurve();

        // Do the two pcurves lye on the same surface ?
        if ((StepPCurve1 is not null && StepPCurve2 is not null) && (StepPCurve1.BasisSurface() == Surf)
            && (StepPCurve2.BasisSurface() == Surf))
        {
            int NbEdge = EdgeLoop.NbEdgeList();
            int nbOE = 0;

            StepShape_OrientedEdge OrEdge;
            for (int i = 1; i <= NbEdge; i++)
            {
                OrEdge = EdgeLoop.EdgeListValue(i);
                if (StepEdge == OrEdge.EdgeElement())
                {
                    nbOE++;
                }
            }

            // the two oriented edges are not in the same wire
            if (nbOE == 1)
            {
                // check if the two pcurves are not identical ?
                StepGeom_Line line1 = StepPCurve1.ReferenceToCurve().ItemsValue(1) as StepGeom_Line;
                StepGeom_Line line2 = StepPCurve2.ReferenceToCurve().ItemsValue(1) as StepGeom_Line;
                if (line1 is not null && line2 is not null)
                {
                    // Same Origin in X OR Y && Same Vector ??
                    // WITHIN A given tolerance !!!
                    double DeltaX = Math.Abs(line1.Pnt().CoordinatesValue(1) - line2.Pnt().CoordinatesValue(1));
                    double DeltaY = Math.Abs(line1.Pnt().CoordinatesValue(2) - line2.Pnt().CoordinatesValue(2));

                    double DeltaDirX = Math.Abs(line1.Dir().Orientation().DirectionRatiosValue(1)
                                                - line2.Dir().Orientation().DirectionRatiosValue(1));
                    double DeltaDirY = Math.Abs(line1.Dir().Orientation().DirectionRatiosValue(2)
                                                - line2.Dir().Orientation().DirectionRatiosValue(2));

                    double preci2d = global::Kerf.Engine.Foundation.Precision.PConfusion(); //:S4136: Parametric(BRepAPI::Precision(),10);

                    if ((DeltaX < preci2d) || (DeltaY < preci2d))
                    {
                        return ((DeltaDirX < preci2d) && (DeltaDirY < preci2d));
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }

            return false;
        }

        return false;
    }

    /// <summary>
    /// Updates the parameters of an edge on its 3D curve according to the type of curve (w1 &gt; w2 happens when an
    /// edge crosses the parametric origin); false if that failed.
    /// </summary>
    /// <remarks>May reverse <paramref name="theCurve"/> in place, as OCCT does: the curve must be a working one.</remarks>
    public static bool UpdateParam3d(Geom_Curve theCurve, ref double w1, ref double w2, double preci)
        => UpdateParam3d(ref theCurve, ref w1, ref w2, preci, false);

    /// <summary>
    /// As the public form; with <paramref name="theCopyFrozen"/>, a frozen curve (one an earlier edge took) is
    /// reversed in a copy that replaces it, where OCCT reverses the shared curve in place (decision 1248).
    /// </summary>
    internal static bool UpdateParam3d(ref Geom_Curve theCurve, ref double w1, ref double w2, double preci, bool theCopyFrozen)
    {
        // w1 et/ou w2 peuvent etre en dehors des bornes naturelles de la courbe.
        // On donnera alors la valeur en bout a w1 et/ou w2

        double cf = theCurve.FirstParameter();
        double cl = theCurve.LastParameter();

        if (theCurve is Geom_BoundedCurve && !theCurve.IsClosed())
        {
            if (w1 < cf)
            {
                w1 = cf;
            }
            else if (w1 > cl)
            {
                w1 = cl;
            }

            if (w2 < cf)
            {
                w2 = cf;
            }
            else if (w2 > cl)
            {
                w2 = cl;
            }
        }

        if (w1 < w2)
        {
            return true;
        }

        if (theCurve.IsPeriodic())
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref w1, ref w2); //:a7 abv 11 Feb 98: preci -> PConfusion()
        }
        else if (theCurve.IsClosed())
        {
            // l'un des points projecte se trouve sur l'origine du parametrage
            // de la courbe 3D. L algo a donne cl +- preci au lieu de cf ou vice-versa
            // DANGER precision 3d applique a une espace 1d

            // w2 = cf au lieu de w2 = cl
            if (Math.Abs(w2 - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
            {
                w2 = cl;
            }
            // w1 = cl au lieu de w1 = cf
            else if (Math.Abs(w1 - cl) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
            {
                w1 = cf;
            }
            // on se trouve dans un cas ou l origine est traversee
            // illegal sur une courbe fermee non periodique
            // on inverse quand meme les parametres !!!!!!
            else
            {
                //: S4136 abv 20 Apr 99: r0701_ug.stp #6230: add check in 3d
                if (theCurve.Value(w1).Distance(theCurve.Value(cf)) < preci)
                {
                    w1 = cf;
                }

                if (theCurve.Value(w2).Distance(theCurve.Value(cl)) < preci)
                {
                    w2 = cl;
                }

                if (Math.Abs(w2 - w1) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    w1 = cf;
                    w2 = cl;
                }
                else if (w1 > w2)
                {
                    double tmp = w1;
                    w1 = w2;
                    w2 = tmp;
                }
            }
        }
        // The curve is closed within the 3D tolerance
        else if (theCurve is Geom_BSplineCurve aBSpline)
        {
            if (aBSpline.StartPoint().Distance(aBSpline.EndPoint()) <= preci)
            {
                //: S4136	<= BRepAPI::Precision()) {
                // l'un des points projecte se trouve sur l'origine du parametrage
                // de la courbe 3D. L algo a donne cl +- preci au lieu de cf ou vice-versa
                // DANGER precision 3d applique a une espace 1d

                // w2 = cf au lieu de w2 = cl
                if (Math.Abs(w2 - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    w2 = cl;
                }
                // w1 = cl au lieu de w1 = cf
                else if (Math.Abs(w1 - cl) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    w1 = cf;
                }
                // on se trouve dans un cas ou l origine est traversee
                // illegal sur une courbe fermee non periodique
                // on inverse quand meme les parametres !!!!!!
                else
                {
                    double tmp = w1;
                    w1 = w2;
                    w2 = tmp;
                }
            }
            // abv 15.03.00 #72 bm1_pe_t4 protection of exceptions in draw
            else if (w1 > w2)
            {
                w1 = theCurve.ReversedParameter(w1);
                w2 = theCurve.ReversedParameter(w2);
                if (theCopyFrozen && theCurve.IsFrozen)
                {
                    theCurve = (Geom_Curve)theCurve.Copy(); // OCCT.NET: decision 1248
                }

                theCurve.Reverse();
            }

            //: j9 abv 11 Dec 98: PRO7747 #4875, after :j8:    else
            if (w1 == w2)
            { // gka 10.07.1998 file PRO7656 entity 33334
                w1 = cf;
                w2 = cl;
                return false;
            }
        }
        else
        {
            // OCCT_DEBUG output left out
            // abv 15.03.00 #72 bm1_pe_t4 protection of exceptions in draw
            if (w1 > w2)
            {
                w1 = theCurve.ReversedParameter(w1);
                w2 = theCurve.ReversedParameter(w2);
                if (theCopyFrozen && theCurve.IsFrozen)
                {
                    theCurve = (Geom_Curve)theCurve.Copy(); // OCCT.NET: decision 1248
                }

                theCurve.Reverse();
            }

            // pdn 11.01.99 #144 bm1_pe_t4 protection of exceptions in draw
            if (w1 == w2)
            {
                w1 -= global::Kerf.Engine.Foundation.Precision.PConfusion();
                w2 += global::Kerf.Engine.Foundation.Precision.PConfusion();
            }

            return false;
        }

        return true;
    }
}
