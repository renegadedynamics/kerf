// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_FacetedTool.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_FacetedTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Occt.TopoDSToStep_FacetedError;

namespace Occt;

/// <summary>Checks whether a shape can be written as a faceted brep: planar faces with linear pcurves.</summary>
public static class TopoDSToStep_FacetedTool
{
    public static TopoDSToStep_FacetedError CheckTopoDSShape(in Shape aShape)
    {
        Explorer FaceExp = new Explorer(), EdgeExp = new Explorer();
        FaceExp.Init(aShape, ShapeType.Face);
        while (FaceExp.More())
        {
            Face aFace = ShapeCasts.Face(FaceExp.Current());
            FaceExp.Next();

            Surface Su = Tool.Surface(aFace);

            if (Su is PlaneSurface)
            {
                // OK -> no further check
            }
            else if (Su is BSplineSurface)
            {
                BSplineSurface aBsplS = Su as BSplineSurface;
                int uDeg, vDeg, nUPol, nVPol;
                uDeg = aBsplS.UDegree();
                if (uDeg == 1)
                {
                    vDeg = aBsplS.VDegree();
                    if (vDeg == 1)
                    {
                        nUPol = aBsplS.UPoleCount;
                        nVPol = aBsplS.VPoleCount;
                        if (nUPol != 2 || nVPol != 2)
                        {
                            return TopoDSToStep_SurfaceNotPlane;
                        }
                    }
                    else
                    {
                        // Degree in v != 1
                        return TopoDSToStep_SurfaceNotPlane;
                    }
                }
                else
                {
                    // Degree in u != 1
                    return TopoDSToStep_SurfaceNotPlane;
                }
            }
            else if (Su is BezierSurface)
            {
                BezierSurface aBzS = Su as BezierSurface;
                int uDeg, vDeg, nUPol, nVPol;
                uDeg = aBzS.UDegree();
                if (uDeg == 1)
                {
                    vDeg = aBzS.VDegree();
                    if (vDeg == 1)
                    {
                        nUPol = aBzS.UPoleCount;
                        nVPol = aBzS.VPoleCount;
                        if (nUPol != 2 || nVPol != 2)
                        {
                            return TopoDSToStep_SurfaceNotPlane;
                        }
                    }
                    else
                    {
                        // Degree in v != 1
                        return TopoDSToStep_SurfaceNotPlane;
                    }
                }
                else
                {
                    // Degree in u != 1
                    return TopoDSToStep_SurfaceNotPlane;
                }
            }
            else
            {
                // the surface is neither a Plane nor a flat BSpline or Beziersurface
                return TopoDSToStep_SurfaceNotPlane;
            }

            // surface is flat; now check, if the PCurves are linear

            EdgeExp.Init(aFace, ShapeType.Edge);
            while (EdgeExp.More())
            {
                Edge anEdge = ShapeCasts.Edge(EdgeExp.Current());
                EdgeExp.Next();

                Curve2D C2d = Tool.CurveOnSurface(anEdge, aFace, out double cf, out double cl);

                if (C2d is LineCurve2D)
                {
                    return TopoDSToStep_FacetedDone; // OCCT: the first linear pcurve accepts the whole shape
                }
                else if (C2d is BSplineCurve2D)
                {
                    BSplineCurve2D aBspl2d = C2d as BSplineCurve2D;
                    if ((aBspl2d.Degree() != 1) || (aBspl2d.PoleCount != 2))
                    {
                        return TopoDSToStep_PCurveNotLinear;
                    }
                }
                else if (C2d is BezierCurve2D)
                {
                    BezierCurve2D aBzC2d = C2d as BezierCurve2D;
                    if ((aBzC2d.Degree() != 1) || (aBzC2d.PoleCount != 2))
                    {
                        return TopoDSToStep_PCurveNotLinear;
                    }
                }
                else
                {
                    return TopoDSToStep_PCurveNotLinear;
                }
            } // end while (EdgeExp.More())
        } // end while (FaceExp.More())

        return TopoDSToStep_FacetedDone;
    }
}
