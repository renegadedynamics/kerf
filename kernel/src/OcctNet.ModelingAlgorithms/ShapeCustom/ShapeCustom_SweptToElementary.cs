// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_SweptToElementary.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_SweptToElementary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The modification that converts surfaces of revolution and of linear extrusion into elementary surfaces
/// (cylinders, spheres, cones, tori) where they are such, shifting the pcurves to the new parametrisation.
/// </summary>
public class ShapeCustom_SweptToElementary : ShapeCustom_Modification
{
    /// <summary>An empty modification.</summary>
    public ShapeCustom_SweptToElementary()
    {
    }

    // auxiliary (Analyze surface: is it to be converted?)
    private static bool IsToConvert(Surface? S, out SweptSurface? SS)
    {
        SS = null;
        Surface? Stmp = null;

        if (S is SweptSurface aSwept)
        {
            SS = aSwept;
            return true;
        }
        if (S is TrimmedSurface RTS)
        {
            Stmp = RTS.BasisSurface();
        }
        else if (S is global::Kerf.Geometry.OffsetSurface OS)
        {
            Stmp = OS.BasisSurface();
        }
        if (Stmp is null)
        {
            return false;
        }
        // OCCT: tests S, which isn't swept (tested above), instead of Stmp: trimmed and offset swept surfaces are never
        // converted (occt-issues.md)
        if (S is SweptSurface)
        {
            SS = Stmp as SweptSurface;
            return true;
        }
        return false;
    }

    /// <summary>
    /// True if the face lies on a surface of revolution that is a cylinder, sphere, cone or torus, or on a surface of
    /// linear extrusion that is a cylinder: then S is that elementary surface.
    /// </summary>
    public override bool NewSurface(in Face F,
                                    out Surface? S,
                                    out Placement L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        Tol = 0.0;
        RevWires = RevFace = false;
        S = Tool.Surface(F, out L);
        if (!IsToConvert(S, out SweptSurface? SS))
        {
            return false;
        }

        // case SurfaceOfRevolution
        if (SS is RevolutionSurface SR)
        {
            Curve3D bc = SR.BasisCurve();
            Axis3D ax1 = SR.Axis();
            CurveAdaptor HC = new CurveAdaptor();
            HC.Load(bc, bc.FirstParameter(), bc.LastParameter());
            RevolutionSurfaceAdaptor AS = new RevolutionSurfaceAdaptor(HC, ax1);
            switch (AS.GetType())
            {
                // skl 18.12.2003 - plane not used, problems in PRO14665.igs
                // case GeomAbs_Plane : {
                //  occ::handle<Geom_Plane> Pl = new Geom_Plane(AS.Plane());
                //  S = Pl;
                //} break;
                case SurfaceKind.Cylinder:
                {
                    CylindricalSurface Cy = new CylindricalSurface(AS.Cylinder());
                    S = Cy;
                }
                    break;
                case SurfaceKind.Sphere:
                {
                    SphericalSurface Sp = new SphericalSurface(AS.Sphere());
                    S = Sp;
                }
                    break;
                case SurfaceKind.Cone:
                {
                    ConicalSurface Co = new ConicalSurface(AS.Cone());
                    S = Co;
                }
                    break;
                case SurfaceKind.Torus:
                {
                    ToroidalSurface To = new ToroidalSurface(AS.Torus());
                    S = To;
                }
                    break;
                default:
                    return false;
            }
        }
        // case SurfaceOfLinearExtrusion
        else if (SS is ExtrusionSurface SLE)
        {
            Curve3D bc = SLE.BasisCurve();
            Direction3D dir = SLE.Direction();
            CurveAdaptor HC = new CurveAdaptor();
            HC.Load(bc, bc.FirstParameter(), bc.LastParameter());
            SurfaceOfLinearExtrusion AS = new SurfaceOfLinearExtrusion(HC, dir);
            switch (AS.GetType())
            {
                // skl 18.12.2003 - plane not used, problems in ims013.igs
                // case GeomAbs_Plane : {
                //  occ::handle<Geom_Plane> Pl = new Geom_Plane(AS.Plane());
                //  S = Pl;
                //} break;
                case SurfaceKind.Cylinder:
                {
                    CylindricalSurface Cy = new CylindricalSurface(AS.Cylinder());
                    S = Cy;
                }
                    break;
                default:
                    return false;
            }
        }

        SendMsg(F, new Msg("SweptToElementary.NewSurface.MSG0"));

        Tol = Tool.Tolerance(F);
        RevWires = false;
        RevFace = false;
        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on a swept surface (so that the edge is copied): then C is a copy of its 3D curve.
    /// </summary>
    public override bool NewCurve(in Edge E, out Curve3D? C, out Placement L, out double Tol)
    {
        C = null;
        L = new Placement();
        Tol = 0.0;
        //: p5 abv 26 Feb 99: force copying of edge if any its pcurve will be replaced
        BrepTEdge TE = (BrepTEdge)E.TShape()!;

        // iterate on pcurves
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(TE.Curves());
        for (; itcr.More(); itcr.Next())
        {
            if (itcr.Value() is not GCurve GC || !GC.IsCurveOnSurface())
            {
                continue;
            }
            Surface? S = GC.Surface();
            if (!IsToConvert(S, out _))
            {
                continue;
            }
            C = Tool.Curve(E, out L, out double f, out double l);
            if (C is not null)
            {
                C = (Curve3D)C.Copy();
            }
            Tol = Tool.Tolerance(E);
            return true;
        }
        return false;
    }

    /// <summary>False: 3D points are never modified.</summary>
    public override bool NewPoint(in Vertex V, out Point3D P, out double Tol)
    {
        P = new Point3D();
        Tol = 0.0;
        // 3d points are never modified
        return false;
    }

    /// <summary>
    /// True if the face's surface is swept or the edge was copied: C is a copy of the pcurve, shifted to the new
    /// torus's or sphere's parametrisation.
    /// </summary>
    public override bool NewCurve2d(in Edge E,
                                    in Face F,
                                    in Edge NewE,
                                    in Face NewF,
                                    out Curve2D? C,
                                    out double Tol)
    {
        C = null;
        Tol = 0.0;
        Surface? S = Tool.Surface(F, out Placement L);

        // just copy pcurve if either its surface is changing or edge was copied
        if (!IsToConvert(S, out SweptSurface? SS) && E.IsSame(NewE))
        {
            return false;
        }

        C = Tool.CurveOnSurface(E, F, out double f, out double l);
        if (C is not null)
        {
            C = (Curve2D)C.Copy();
            // OCCT: dereferences the null face the modifier passes when it asks whether to copy the vertices
            // (BRepTools_Modifier::CreateOtherVertices), an access violation natively (occt-issues.md); the port's
            // BRep_Tool throws a NullReferenceException there
            Surface? NS = Tool.Surface(NewF, out L);
            if (NS is not null && NS is ToroidalSurface)
            {
                // OCCT: SS is null when the face's surface isn't swept and only the edge was copied, an access
                // violation natively, a NullReferenceException in the port (occt-issues.md, decision 1024)
                if ((SS ?? throw NullSweptSurface()) is RevolutionSurface SR)
                {
                    SR.Bounds(out double U1, out double U2, out double V1, out double V2);
                    SR.D0(U1, V1, out Point3D P0);
                    ShapeAnalysis_Surface sas = new ShapeAnalysis_Surface(NS);
                    Point2D p2d = sas.ValueOfUV(P0, global::Kerf.Engine.Foundation.Precision.Confusion());
                    Vector2D shift = new Vector2D(p2d.X() - U1, p2d.Y() - V1);
                    C.Translate(shift);
                }
            }
            if (NS is not null && NS is SphericalSurface)
            {
                // OCCT: as above
                if ((SS ?? throw NullSweptSurface()) is RevolutionSurface SR)
                {
                    SphericalSurface SPH = (SphericalSurface)NS;
                    SPH.Bounds(out double US1, out double US2, out double VS1, out double VS2);
                    SPH.D0(US1, VS1, out Point3D PS);
                    SR.Bounds(out double UR1, out double UR2, out double VR1, out double VR2);
                    SR.D0(UR1, VR1, out Point3D PR);
                    Point3D P0 = SPH.Location();
                    Vector3D VS = new Vector3D(P0, PS);
                    Vector3D VR = new Vector3D(P0, PR);
                    double angle = VS.Angle(VR);
                    Vector2D shift = new Vector2D(0, VS1 - VR1 + angle);
                    C.Translate(shift);
                }
            }
        }

        Tol = Tool.Tolerance(E);
        return true;
    }

    // OCCT dereferences the null swept surface: an access violation natively (decision 1024)
    private static NullReferenceException NullSweptSurface() =>
        new("ShapeCustom_SweptToElementary::NewCurve2d: the face's surface isn't swept (OCCT dereferences a null handle)");

    /// <summary>False: parameters are never modified.</summary>
    public override bool NewParameter(in Vertex V, in Edge E, ref double P, ref double Tol)
    {
        return false;
    }

    /// <summary>The continuity of the edge between the faces, unchanged.</summary>
    public override Continuity Continuity(in Edge E,
                                             in Face F1,
                                             in Face F2,
                                             in Edge NewE,
                                             in Face NewF1,
                                             in Face NewF2)
    {
        return Tool.Continuity(E, F1, F2);
    }
}
