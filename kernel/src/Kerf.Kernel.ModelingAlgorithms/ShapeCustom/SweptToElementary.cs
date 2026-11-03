// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_SweptToElementary.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_SweptToElementary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// The modification that converts surfaces of revolution and of linear extrusion into elementary surfaces
/// (cylinders, spheres, cones, tori) where they are such, shifting the pcurves to the new parametrisation.
/// </summary>
public class SweptToElementary : CustomModification
{
    /// <summary>An empty modification.</summary>
    public SweptToElementary()
    {
    }

    // auxiliary (Analyze surface: is it to be converted?)
    private static bool IsToConvert(Surface? s, out SweptSurface? ss)
    {
        ss = null;
        Surface? stmp = null;

        if (s is SweptSurface swept)
        {
            ss = swept;
            return true;
        }
        if (s is TrimmedSurface rts)
        {
            stmp = rts.BasisSurface;
        }
        else if (s is global::Kerf.Geometry.OffsetSurface os)
        {
            stmp = os.BasisSurface;
        }
        if (stmp is null)
        {
            return false;
        }
        // OCCT: tests S, which isn't swept (tested above), instead of Stmp: trimmed and offset swept surfaces are never
        // converted (occt-issues.md)
        if (s is SweptSurface)
        {
            ss = stmp as SweptSurface;
            return true;
        }
        return false;
    }

    /// <summary>
    /// True if the face lies on a surface of revolution that is a cylinder, sphere, cone or torus, or on a surface of
    /// linear extrusion that is a cylinder: then S is that elementary surface.
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
        if (!IsToConvert(s, out SweptSurface? ss))
        {
            return false;
        }

        // case SurfaceOfRevolution
        if (ss is RevolutionSurface sr)
        {
            Curve3D bc = sr.BasisCurve;
            Axis3D ax1 = sr.Axis;
            CurveAdaptor hc = new CurveAdaptor();
            hc.Load(bc, bc.FirstParameter, bc.LastParameter);
            RevolutionSurfaceAdaptor AS = new RevolutionSurfaceAdaptor(hc, ax1);
            switch (AS.GetType())
            {
                // skl 18.12.2003 - plane not used, problems in PRO14665.igs
                // case GeomAbs_Plane : {
                //  occ::handle<Geom_Plane> Pl = new Geom_Plane(AS.Plane());
                //  S = Pl;
                //} break;
                case SurfaceKind.Cylinder:
                {
                    CylindricalSurface cy = new CylindricalSurface(AS.Cylinder());
                    s = cy;
                }
                    break;
                case SurfaceKind.Sphere:
                {
                    SphericalSurface sp = new SphericalSurface(AS.Sphere());
                    s = sp;
                }
                    break;
                case SurfaceKind.Cone:
                {
                    ConicalSurface co = new ConicalSurface(AS.Cone());
                    s = co;
                }
                    break;
                case SurfaceKind.Torus:
                {
                    ToroidalSurface to = new ToroidalSurface(AS.Torus());
                    s = to;
                }
                    break;
                default:
                    return false;
            }
        }
        // case SurfaceOfLinearExtrusion
        else if (ss is ExtrusionSurface sle)
        {
            Curve3D bc = sle.BasisCurve;
            Direction3D dir = sle.Direction;
            CurveAdaptor hc = new CurveAdaptor();
            hc.Load(bc, bc.FirstParameter, bc.LastParameter);
            SurfaceOfLinearExtrusion AS = new SurfaceOfLinearExtrusion(hc, dir);
            switch (AS.GetType())
            {
                // skl 18.12.2003 - plane not used, problems in ims013.igs
                // case GeomAbs_Plane : {
                //  occ::handle<Geom_Plane> Pl = new Geom_Plane(AS.Plane());
                //  S = Pl;
                //} break;
                case SurfaceKind.Cylinder:
                {
                    CylindricalSurface cy = new CylindricalSurface(AS.Cylinder());
                    s = cy;
                }
                    break;
                default:
                    return false;
            }
        }

        SendMsg(f, new Msg("SweptToElementary.NewSurface.MSG0"));

        tol = Tool.Tolerance(f);
        revWires = false;
        revFace = false;
        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on a swept surface (so that the edge is copied): then C is a copy of its 3D curve.
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
    /// True if the face's surface is swept or the edge was copied: C is a copy of the pcurve, shifted to the new
    /// torus's or sphere's parametrisation.
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
        if (!IsToConvert(s, out SweptSurface? ss) && e.IsSame(newE))
        {
            return false;
        }

        c = Tool.CurveOnSurface(e, F, out double f, out double l);
        if (c is not null)
        {
            c = (Curve2D)c.Copy();
            // The modifier asks with a null new face whether to copy an edge's vertices (CreateOtherVertices): no
            // surface, no shift. A face whose surface isn't swept keeps it: no shift either. OCCT dereferenced both
            // (decision 1437).
            Surface? ns = newF.IsNull ? null : Tool.Surface(newF, out L);
            if (ns is ToroidalSurface && ss is RevolutionSurface sr)
            {
                sr.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
                sr.D0(u1, v1, out Point3D p0);
                SurfaceAnalysis sas = new SurfaceAnalysis(ns);
                Point2D p2d = sas.ValueOfUV(p0, global::Kerf.Engine.Foundation.Precision.Confusion());
                Vector2D shift = new Vector2D(p2d.X - u1, p2d.Y - v1);
                c.Translate(shift);
            }
            if (ns is SphericalSurface sph && ss is RevolutionSurface sr2)
            {
                sph.ParameterBounds(out double us1, out double us2, out double vs1, out double vs2);
                sph.D0(us1, vs1, out Point3D ps);
                sr2.ParameterBounds(out double ur1, out double ur2, out double vr1, out double vr2);
                sr2.D0(ur1, vr1, out Point3D pr);
                Point3D p0 = sph.Location;
                Vector3D vs = new Vector3D(p0, ps);
                Vector3D vr = new Vector3D(p0, pr);
                double angle = vs.AngleTo(vr);
                Vector2D shift = new Vector2D(0, vs1 - vr1 + angle);
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
