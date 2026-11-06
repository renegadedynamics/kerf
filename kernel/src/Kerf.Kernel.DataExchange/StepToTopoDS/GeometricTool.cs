// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_GeometricTool.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_GeometricTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Geometry;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Algorithmic services specific to the mapping of STEP to OCCT topology.</summary>
internal static class GeometricTool
{
    /// <summary>
    /// The index of the first pcurve of the surface curve, after <paramref name="last"/>, lying on the surface (0 if
    /// none); <paramref name="pCurve"/> is that pcurve (null if none).
    /// </summary>
    public static int PCurve(StepSurfaceCurve surfCurve, StepSurface basisSurf, out StepPcurve pCurve, int last = 0)
    {
        int nbAssGeom = surfCurve.AssociatedGeometryCount;
        pCurve = null;
        for (int i = last + 1; i <= nbAssGeom; i++)
        {
            pCurve = surfCurve.AssociatedGeometryValue(i)?.Pcurve();
            if (pCurve is not null)
            {
                if (pCurve.BasisSurface() == basisSurf)
                {
                    return i;
                }
            }
        }

        pCurve = null;
        return 0;
    }

    /// <summary>
    /// True if the surface curve is a seam curve: a seam_curve, or two oriented edges of the same wire referencing the
    /// same edge, whose two pcurves lie on the surface.
    /// </summary>
    public static bool IsSeamCurve(StepSurfaceCurve surfCurve, StepSurface surf, StepEdge stepEdge, StepEdgeLoop edgeLoop)
    {
        if (surfCurve is StepSeamCurve)
        {
            return true;
        }

        if (surfCurve.AssociatedGeometryCount != 2)
        {
            return false;
        }

        StepPcurve stepPCurve1 = surfCurve.AssociatedGeometryValue(1)?.Pcurve();
        StepPcurve stepPCurve2 = surfCurve.AssociatedGeometryValue(2)?.Pcurve();

        // Do the two pcurves lye on the same surface ?
        if ((stepPCurve1 is not null && stepPCurve2 is not null) && (stepPCurve1.BasisSurface() == surf)
            && (stepPCurve2.BasisSurface() == surf))
        {
            int nbEdge = edgeLoop.NbEdgeList();
            int nbOE = 0;

            StepOrientedEdge orEdge;
            for (int i = 1; i <= nbEdge; i++)
            {
                orEdge = edgeLoop.EdgeListValue(i);
                if (stepEdge == orEdge.EdgeElement())
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
    public static bool IsLikeSeam(StepSurfaceCurve surfCurve, StepSurface surf, StepEdge stepEdge, StepEdgeLoop edgeLoop)
    {
        if (surfCurve.AssociatedGeometryCount != 2)
        {
            return false;
        }

        StepPcurve stepPCurve1 = surfCurve.AssociatedGeometryValue(1)?.Pcurve();
        StepPcurve stepPCurve2 = surfCurve.AssociatedGeometryValue(2)?.Pcurve();

        // Do the two pcurves lye on the same surface ?
        if ((stepPCurve1 is not null && stepPCurve2 is not null) && (stepPCurve1.BasisSurface() == surf)
            && (stepPCurve2.BasisSurface() == surf))
        {
            int nbEdge = edgeLoop.NbEdgeList();
            int nbOE = 0;

            StepOrientedEdge orEdge;
            for (int i = 1; i <= nbEdge; i++)
            {
                orEdge = edgeLoop.EdgeListValue(i);
                if (stepEdge == orEdge.EdgeElement())
                {
                    nbOE++;
                }
            }

            // the two oriented edges are not in the same wire
            if (nbOE == 1)
            {
                // check if the two pcurves are not identical ?
                StepLine line1 = stepPCurve1.ReferenceToCurve().ItemsValue(1) as StepLine;
                StepLine line2 = stepPCurve2.ReferenceToCurve().ItemsValue(1) as StepLine;
                if (line1 is not null && line2 is not null)
                {
                    // Same Origin in X OR Y && Same Vector ??
                    // WITHIN A given tolerance !!!
                    double deltaX = Math.Abs(line1.Pnt().CoordinatesValue(1) - line2.Pnt().CoordinatesValue(1));
                    double deltaY = Math.Abs(line1.Pnt().CoordinatesValue(2) - line2.Pnt().CoordinatesValue(2));

                    double deltaDirX = Math.Abs(line1.Dir().Orientation().DirectionRatiosValue(1)
                                                - line2.Dir().Orientation().DirectionRatiosValue(1));
                    double deltaDirY = Math.Abs(line1.Dir().Orientation().DirectionRatiosValue(2)
                                                - line2.Dir().Orientation().DirectionRatiosValue(2));

                    double preci2d = global::Kerf.Engine.Foundation.Precision.PConfusion(); //:S4136: Parametric(BRepAPI::Precision(),10);

                    if ((deltaX < preci2d) || (deltaY < preci2d))
                    {
                        return ((deltaDirX < preci2d) && (deltaDirY < preci2d));
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
    /// <remarks>May reverse <paramref name="curve"/> in place, as OCCT does: the curve must be a working one.</remarks>
    public static bool UpdateParam3d(Curve3D curve, ref double w1, ref double w2, double preci)
        => UpdateParam3d(ref curve, ref w1, ref w2, preci, false);

    /// <summary>
    /// As the public form; with <paramref name="copyFrozen"/>, a frozen curve (one an earlier edge took) is
    /// reversed in a copy that replaces it, where OCCT reverses the shared curve in place (decision 1248).
    /// </summary>
    internal static bool UpdateParam3d(ref Curve3D curve, ref double w1, ref double w2, double preci, bool copyFrozen)
    {
        // w1 et/ou w2 peuvent etre en dehors des bornes naturelles de la courbe.
        // On donnera alors la valeur en bout a w1 et/ou w2

        double cf = curve.FirstParameter;
        double cl = curve.LastParameter;

        if (curve is BoundedCurve3D && !curve.IsClosed)
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

        if (curve.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref w1, ref w2); //:a7 abv 11 Feb 98: preci -> PConfusion()
        }
        else if (curve.IsClosed)
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
                if (curve.Value(w1).DistanceTo(curve.Value(cf)) < preci)
                {
                    w1 = cf;
                }

                if (curve.Value(w2).DistanceTo(curve.Value(cl)) < preci)
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
        else if (curve is BSplineCurve3D bSpline)
        {
            if (bSpline.StartPoint.DistanceTo(bSpline.EndPoint) <= preci)
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
                w1 = curve.ReversedParameter(w1);
                w2 = curve.ReversedParameter(w2);
                if (copyFrozen && curve.IsFrozen)
                {
                    curve = (Curve3D)curve.Copy(); // Kerf: decision 1248
                }

                curve.Reverse();
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
                w1 = curve.ReversedParameter(w1);
                w2 = curve.ReversedParameter(w2);
                if (copyFrozen && curve.IsFrozen)
                {
                    curve = (Curve3D)curve.Copy(); // Kerf: decision 1248
                }

                curve.Reverse();
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
