// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToRevolution.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The modification that converts the elementary periodic surfaces (spheres, tori, cylinders, cones), plain,
/// rectangular-trimmed or offset, to Geom_SurfaceOfRevolution, keeping the parametrisation (pcurves on spheres are
/// shifted by 2π in V).
/// </summary>
public class ShapeCustom_ConvertToRevolution : ShapeCustom_Modification
{
    /// <summary>An empty modification.</summary>
    public ShapeCustom_ConvertToRevolution()
    {
    }

    // Analyze surface: is it to be converted?
    private static bool IsToConvert(Geom_Surface? S, out Geom_ElementarySurface? ES)
    {
        ES = S as Geom_ElementarySurface;
        if (ES is null)
        {
            if (S is Geom_RectangularTrimmedSurface RTS)
            {
                ES = RTS.BasisSurface() as Geom_ElementarySurface;
            }
            else if (S is Geom_OffsetSurface OS)
            {
                ES = OS.BasisSurface() as Geom_ElementarySurface;
            }
            if (ES is null)
            {
                return false;
            }
        }

        return ES is Geom_SphericalSurface
               || ES is Geom_ToroidalSurface
               || ES is Geom_CylindricalSurface
               || ES is Geom_ConicalSurface;
    }

    /// <summary>
    /// True if the face lies on a sphere, torus, cylinder or cone (possibly trimmed or offset): then S is the surface
    /// of revolution of its meridian (trimmed or offset in the same way).
    /// </summary>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out TopLoc_Location L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        Tol = 0.0;
        RevWires = RevFace = false;
        S = BRep_Tool.Surface(F, out L);

        if (!IsToConvert(S, out Geom_ElementarySurface? ES))
        {
            return false;
        }

        // remove location if it contains inversion
        /*
          gp_Trsf t = L.Transformation();
          gp_Mat m = t.VectorialPart();
          bool neg = t.IsNegative();
          bool det = ( m.Determinant() <0 ? true : false );
          if ( neg != det ) {
            ES = Handle(Geom_ElementarySurface)::DownCast ( ES->Transformed(t) );
            L.Identity();
          }
        */

        gp_Ax3 Ax3 = ES!.Position();
        gp_Pnt pos = Ax3.Location();
        gp_Dir dir = Ax3.Direction();
        gp_Dir X = Ax3.XDirection();

        // create basis line to rotate
        Geom_Curve? BasisCurve = null;
        if (ES is Geom_SphericalSurface SS)
        {
            gp_Ax2 Ax2 = new gp_Ax2(pos, X.Crossed(dir), X);
            Geom_Circle Circ = new Geom_Circle(Ax2, SS.Radius());
            BasisCurve = new Geom_TrimmedCurve(Circ, -M_PI / 2.0, M_PI / 2.0);
        }
        else if (ES is Geom_ToroidalSurface TS)
        {
            gp_Ax2 Ax2 = new gp_Ax2(new gp_Pnt(pos.XYZ() + X.XYZ() * TS.MajorRadius()), X.Crossed(dir), X);
            BasisCurve = new Geom_Circle(Ax2, TS.MinorRadius());
        }
        else if (ES is Geom_CylindricalSurface CS)
        {
            gp_Ax1 Ax1 = new gp_Ax1(new gp_Pnt(pos.XYZ() + X.XYZ() * CS.Radius()), dir);
            BasisCurve = new Geom_Line(Ax1);
        }
        else if (ES is Geom_ConicalSurface CoS)
        {
            gp_Dir N = new gp_Dir(dir.XYZ() + X.XYZ() * Math.Tan(CoS.SemiAngle()));
            gp_Ax1 Ax1 = new gp_Ax1(new gp_Pnt(pos.XYZ() + X.XYZ() * CoS.RefRadius()), N);
            BasisCurve = new Geom_Line(Ax1);
        }

        // create revolution with proper U parametrization
        gp_Ax1 Axis = Ax3.Axis();

        // if the surface is indirect (taking into account locations), reverse dir

        /*
          gp_Trsf t = L.Transformation();
          gp_Mat m = t.VectorialPart();
          bool neg = t.IsNegative();
          bool det = ( m.Determinant() <0 ? true : false );
          bool isdir = Ax3.Direct();
          if ( ( neg != det ) == isdir ) Axis.Reverse();
        */
        if (!Ax3.Direct())
        {
            Axis.Reverse();
        }

        Geom_SurfaceOfRevolution Rev = new Geom_SurfaceOfRevolution(BasisCurve!, Axis);

        // set resulting surface and restore trimming or offsetting if necessary
        if (ReferenceEquals(ES, S))
        {
            S = Rev;
        }
        else
        {
            if (S is Geom_RectangularTrimmedSurface RTS)
            {
                RTS.Bounds(out double U1, out double U2, out double V1, out double V2);
                S = new Geom_RectangularTrimmedSurface(Rev, U1, U2, V1, V2);
            }
            else if (S is Geom_OffsetSurface OS)
            {
                S = new Geom_OffsetSurface(Rev, OS.Offset());
            }
            else
            {
                S = Rev;
            }
        }
        SendMsg(F, new Message_Msg("ConvertToRevolution.NewSurface.MSG0"));

        Tol = BRep_Tool.Tolerance(F);
        RevWires = false;
        RevFace = false;
        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on a surface to convert (so that the edge is copied): then C is a copy of its 3D
    /// curve.
    /// </summary>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out TopLoc_Location L, out double Tol)
    {
        C = null;
        L = new TopLoc_Location();
        Tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BRep_TEdge TE = (BRep_TEdge)E.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not BRep_GCurve GC || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Geom_Surface? S = GC.Surface();
            if (!IsToConvert(S, out _))
            {
                continue;
            }
            C = BRep_Tool.Curve(E, out L, out double f, out double l);
            if (C is not null)
            {
                C = (Geom_Curve)C.Copy();
            }
            Tol = BRep_Tool.Tolerance(E);
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in TopoDS_Vertex V, out gp_Pnt P, out double Tol)
    {
        P = new gp_Pnt();
        Tol = 0.0;
        // 3d points are never modified
        return false;
    }

    /// <summary>
    /// True if the face's surface is converted or the edge was copied: C is a copy of the pcurve, shifted by 2π in V
    /// on a sphere.
    /// </summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        Geom_Surface? S = BRep_Tool.Surface(F, out TopLoc_Location L);

        // just copy pcurve if either its surface is changing or edge was copied
        if (!IsToConvert(S, out Geom_ElementarySurface? ES) && E.IsSame(NewE))
        {
            return false;
        }

        C = BRep_Tool.CurveOnSurface(E, F, out double f, out double l);
        if (C is not null)
        {
            C = (Geom2d_Curve)C.Copy();

            // for spherical surface, surface of revolution since based on TrimmedCurve
            // has V parametrisation shifted on 2PI; translate pcurve accordingly
            if (ES is Geom_SphericalSurface)
            {
                gp_Vec2d shift = new gp_Vec2d(0.0, 2 * M_PI);
                C.Translate(shift);
            }
        }

        Tol = BRep_Tool.Tolerance(E);
        return true;
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override GeomAbs_Shape Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        return BRep_Tool.Continuity(E, F1, F2);
    }
}
