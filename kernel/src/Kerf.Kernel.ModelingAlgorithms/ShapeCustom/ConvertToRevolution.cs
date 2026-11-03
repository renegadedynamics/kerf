// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToRevolution.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_ConvertToRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// The modification that converts the elementary periodic surfaces (spheres, tori, cylinders, cones), plain,
/// rectangular-trimmed or offset, to Geom_SurfaceOfRevolution, keeping the parametrisation (pcurves on spheres are
/// shifted by 2π in V).
/// </summary>
public class ConvertToRevolution : CustomModification
{
    /// <summary>An empty modification.</summary>
    public ConvertToRevolution()
    {
    }

    // Analyze surface: is it to be converted?
    private static bool IsToConvert(Surface? s, out ElementarySurface? es)
    {
        es = s as ElementarySurface;
        if (es is null)
        {
            if (s is TrimmedSurface rts)
            {
                es = rts.BasisSurface() as ElementarySurface;
            }
            else if (s is OffsetSurface os)
            {
                es = os.BasisSurface() as ElementarySurface;
            }
            if (es is null)
            {
                return false;
            }
        }

        return es is SphericalSurface
               || es is ToroidalSurface
               || es is CylindricalSurface
               || es is ConicalSurface;
    }

    /// <summary>
    /// True if the face lies on a sphere, torus, cylinder or cone (possibly trimmed or offset): then S is the surface
    /// of revolution of its meridian (trimmed or offset in the same way).
    /// </summary>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        tol = 0.0;
        revWires = revFace = false;
        s = Tool.Surface(f, out l);

        if (!IsToConvert(s, out ElementarySurface? es))
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

        CoordinateSystem ax3 = es!.Position();
        Point3D pos = ax3.Location();
        Direction3D dir = ax3.Direction();
        Direction3D x = ax3.XDirection();

        // create basis line to rotate
        Curve3D? basisCurve = null;
        if (es is SphericalSurface ss)
        {
            Frame3D ax2 = new Frame3D(pos, x.Crossed(dir), x);
            CircleCurve3D circ = new CircleCurve3D(ax2, ss.Radius());
            basisCurve = new TrimmedCurve3D(circ, -M_PI / 2.0, M_PI / 2.0);
        }
        else if (es is ToroidalSurface ts)
        {
            Frame3D ax2 = new Frame3D(new Point3D(pos.XYZ() + x.XYZ() * ts.MajorRadius()), x.Crossed(dir), x);
            basisCurve = new CircleCurve3D(ax2, ts.MinorRadius());
        }
        else if (es is CylindricalSurface cs)
        {
            Axis3D ax1 = new Axis3D(new Point3D(pos.XYZ() + x.XYZ() * cs.Radius()), dir);
            basisCurve = new LineCurve3D(ax1);
        }
        else if (es is ConicalSurface coS)
        {
            Direction3D n = new Direction3D(dir.XYZ() + x.XYZ() * Math.Tan(coS.SemiAngle()));
            Axis3D ax1 = new Axis3D(new Point3D(pos.XYZ() + x.XYZ() * coS.RefRadius()), n);
            basisCurve = new LineCurve3D(ax1);
        }

        // create revolution with proper U parametrization
        Axis3D axis = ax3.Axis();

        // if the surface is indirect (taking into account locations), reverse dir

        /*
          gp_Trsf t = L.Transformation();
          gp_Mat m = t.VectorialPart();
          bool neg = t.IsNegative();
          bool det = ( m.Determinant() <0 ? true : false );
          bool isdir = Ax3.Direct();
          if ( ( neg != det ) == isdir ) Axis.Reverse();
        */
        if (!ax3.Direct())
        {
            axis.Reverse();
        }

        RevolutionSurface rev = new RevolutionSurface(basisCurve!, axis);

        // set resulting surface and restore trimming or offsetting if necessary
        if (ReferenceEquals(es, s))
        {
            s = rev;
        }
        else
        {
            if (s is TrimmedSurface rts)
            {
                rts.Bounds(out double u1, out double u2, out double v1, out double v2);
                s = new TrimmedSurface(rev, u1, u2, v1, v2);
            }
            else if (s is OffsetSurface os)
            {
                s = new OffsetSurface(rev, os.Offset());
            }
            else
            {
                s = rev;
            }
        }
        SendMsg(f, new Msg("ConvertToRevolution.NewSurface.MSG0"));

        tol = Tool.Tolerance(f);
        revWires = false;
        revFace = false;
        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on a surface to convert (so that the edge is copied): then C is a copy of its 3D
    /// curve.
    /// </summary>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement L, out double tol)
    {
        c = null;
        L = new Placement();
        tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BrepTEdge te = (BrepTEdge)e.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(te.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not GCurve gc || !gc.IsCurveOnSurface())
            {
                continue;
            }
            Surface? s = gc.Surface();
            if (!IsToConvert(s, out _))
            {
                continue;
            }
            c = Tool.Curve(e, out L, out double f, out double l);
            if (c is not null)
            {
                c = (Curve3D)c.Copy();
            }
            tol = Tool.Tolerance(e);
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        p = new Point3D();
        tol = 0.0;
        // 3d points are never modified
        return false;
    }

    /// <summary>
    /// True if the face's surface is converted or the edge was copied: C is a copy of the pcurve, shifted by 2π in V
    /// on a sphere.
    /// </summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face F,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        c = null;
        tol = 0.0;
        Surface? s = Tool.Surface(F, out Placement L);

        // just copy pcurve if either its surface is changing or edge was copied
        if (!IsToConvert(s, out ElementarySurface? es) && e.IsSame(newE))
        {
            return false;
        }

        c = Tool.CurveOnSurface(e, F, out double f, out double l);
        if (c is not null)
        {
            c = (Curve2D)c.Copy();

            // for spherical surface, surface of revolution since based on TrimmedCurve
            // has V parametrisation shifted on 2PI; translate pcurve accordingly
            if (es is SphericalSurface)
            {
                Vector2D shift = new Vector2D(0.0, 2 * M_PI);
                c.Translate(shift);
            }
        }

        tol = Tool.Tolerance(e);
        return true;
    }

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2)
    {
        return Tool.Continuity(e, f1, f2);
    }
}
