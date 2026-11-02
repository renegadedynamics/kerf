// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_FacetedTool.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_FacetedTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.TopAbs_ShapeEnum;
using static Occt.TopoDSToStep_FacetedError;

namespace Occt;

/// <summary>Checks whether a shape can be written as a faceted brep: planar faces with linear pcurves.</summary>
public static class TopoDSToStep_FacetedTool
{
    public static TopoDSToStep_FacetedError CheckTopoDSShape(in TopoDS_Shape aShape)
    {
        TopExp_Explorer FaceExp = new TopExp_Explorer(), EdgeExp = new TopExp_Explorer();
        FaceExp.Init(aShape, TopAbs_FACE);
        while (FaceExp.More())
        {
            TopoDS_Face aFace = TopoDS.Face(FaceExp.Current());
            FaceExp.Next();

            Geom_Surface Su = BRep_Tool.Surface(aFace);

            if (Su is Geom_Plane)
            {
                // OK -> no further check
            }
            else if (Su is Geom_BSplineSurface)
            {
                Geom_BSplineSurface aBsplS = Su as Geom_BSplineSurface;
                int uDeg, vDeg, nUPol, nVPol;
                uDeg = aBsplS.UDegree();
                if (uDeg == 1)
                {
                    vDeg = aBsplS.VDegree();
                    if (vDeg == 1)
                    {
                        nUPol = aBsplS.NbUPoles();
                        nVPol = aBsplS.NbVPoles();
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
            else if (Su is Geom_BezierSurface)
            {
                Geom_BezierSurface aBzS = Su as Geom_BezierSurface;
                int uDeg, vDeg, nUPol, nVPol;
                uDeg = aBzS.UDegree();
                if (uDeg == 1)
                {
                    vDeg = aBzS.VDegree();
                    if (vDeg == 1)
                    {
                        nUPol = aBzS.NbUPoles();
                        nVPol = aBzS.NbVPoles();
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

            EdgeExp.Init(aFace, TopAbs_EDGE);
            while (EdgeExp.More())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(EdgeExp.Current());
                EdgeExp.Next();

                Geom2d_Curve C2d = BRep_Tool.CurveOnSurface(anEdge, aFace, out double cf, out double cl);

                if (C2d is Geom2d_Line)
                {
                    return TopoDSToStep_FacetedDone; // OCCT: the first linear pcurve accepts the whole shape
                }
                else if (C2d is Geom2d_BSplineCurve)
                {
                    Geom2d_BSplineCurve aBspl2d = C2d as Geom2d_BSplineCurve;
                    if ((aBspl2d.Degree() != 1) || (aBspl2d.NbPoles() != 2))
                    {
                        return TopoDSToStep_PCurveNotLinear;
                    }
                }
                else if (C2d is Geom2d_BezierCurve)
                {
                    Geom2d_BezierCurve aBzC2d = C2d as Geom2d_BezierCurve;
                    if ((aBzC2d.Degree() != 1) || (aBzC2d.NbPoles() != 2))
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
