// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_FacetedTool.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_FacetedTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Step.Export.FacetedError;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Checks whether a shape can be written as a faceted brep: planar faces with linear pcurves.</summary>
public static class FacetedTool
{
    public static FacetedError CheckTopoDSShape(in Shape aShape)
    {
        Explorer faceExp = new Explorer(), edgeExp = new Explorer();
        faceExp.Init(aShape, ShapeType.Face);
        while (faceExp.More())
        {
            Face face = ShapeCasts.Face(faceExp.Current());
            faceExp.Next();

            Surface su = Tool.Surface(face);

            if (su is PlaneSurface)
            {
                // OK -> no further check
            }
            else if (su is BSplineSurface)
            {
                BSplineSurface bsplS = su as BSplineSurface;
                int uDeg, vDeg, nUPol, nVPol;
                uDeg = bsplS.UDegree;
                if (uDeg == 1)
                {
                    vDeg = bsplS.VDegree;
                    if (vDeg == 1)
                    {
                        nUPol = bsplS.UPoleCount;
                        nVPol = bsplS.VPoleCount;
                        if (nUPol != 2 || nVPol != 2)
                        {
                            return FacetedError.SurfaceNotPlane;
                        }
                    }
                    else
                    {
                        // Degree in v != 1
                        return FacetedError.SurfaceNotPlane;
                    }
                }
                else
                {
                    // Degree in u != 1
                    return FacetedError.SurfaceNotPlane;
                }
            }
            else if (su is BezierSurface)
            {
                BezierSurface bzS = su as BezierSurface;
                int uDeg, vDeg, nUPol, nVPol;
                uDeg = bzS.UDegree;
                if (uDeg == 1)
                {
                    vDeg = bzS.VDegree;
                    if (vDeg == 1)
                    {
                        nUPol = bzS.UPoleCount;
                        nVPol = bzS.VPoleCount;
                        if (nUPol != 2 || nVPol != 2)
                        {
                            return FacetedError.SurfaceNotPlane;
                        }
                    }
                    else
                    {
                        // Degree in v != 1
                        return FacetedError.SurfaceNotPlane;
                    }
                }
                else
                {
                    // Degree in u != 1
                    return FacetedError.SurfaceNotPlane;
                }
            }
            else
            {
                // the surface is neither a Plane nor a flat BSpline or Beziersurface
                return FacetedError.SurfaceNotPlane;
            }

            // surface is flat; now check, if the PCurves are linear

            edgeExp.Init(face, ShapeType.Edge);
            while (edgeExp.More())
            {
                Edge edge = ShapeCasts.Edge(edgeExp.Current());
                edgeExp.Next();

                Curve2D c2d = Tool.CurveOnSurface(edge, face, out double cf, out double cl);

                if (c2d is LineCurve2D)
                {
                    return FacetedError.FacetedDone; // OCCT: the first linear pcurve accepts the whole shape
                }
                else if (c2d is BSplineCurve2D)
                {
                    BSplineCurve2D bspl2d = c2d as BSplineCurve2D;
                    if ((bspl2d.Degree != 1) || (bspl2d.PoleCount != 2))
                    {
                        return FacetedError.PCurveNotLinear;
                    }
                }
                else if (c2d is BezierCurve2D)
                {
                    BezierCurve2D bzC2d = c2d as BezierCurve2D;
                    if ((bzC2d.Degree != 1) || (bzC2d.PoleCount != 2))
                    {
                        return FacetedError.PCurveNotLinear;
                    }
                }
                else
                {
                    return FacetedError.PCurveNotLinear;
                }
            } // end while (EdgeExp.More())
        } // end while (FaceExp.More())

        return FacetedError.FacetedDone;
    }
}
