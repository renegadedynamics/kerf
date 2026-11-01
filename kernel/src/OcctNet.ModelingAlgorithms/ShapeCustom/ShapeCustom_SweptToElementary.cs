// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_SweptToElementary.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_SweptToElementary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

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
    private static bool IsToConvert(Geom_Surface? S, out Geom_SweptSurface? SS)
    {
        SS = null;
        Geom_Surface? Stmp = null;

        if (S is Geom_SweptSurface aSwept)
        {
            SS = aSwept;
            return true;
        }
        if (S is Geom_RectangularTrimmedSurface RTS)
        {
            Stmp = RTS.BasisSurface();
        }
        else if (S is Geom_OffsetSurface OS)
        {
            Stmp = OS.BasisSurface();
        }
        if (Stmp is null)
        {
            return false;
        }
        // OCCT: tests S, which isn't swept (tested above), instead of Stmp: trimmed and offset swept surfaces are never
        // converted (occt-issues.md)
        if (S is Geom_SweptSurface)
        {
            SS = Stmp as Geom_SweptSurface;
            return true;
        }
        return false;
    }

    /// <summary>
    /// True if the face lies on a surface of revolution that is a cylinder, sphere, cone or torus, or on a surface of
    /// linear extrusion that is a cylinder: then S is that elementary surface.
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
        if (!IsToConvert(S, out Geom_SweptSurface? SS))
        {
            return false;
        }

        // case SurfaceOfRevolution
        if (SS is Geom_SurfaceOfRevolution SR)
        {
            Geom_Curve bc = SR.BasisCurve();
            gp_Ax1 ax1 = SR.Axis();
            GeomAdaptor_Curve HC = new GeomAdaptor_Curve();
            HC.Load(bc, bc.FirstParameter(), bc.LastParameter());
            GeomAdaptor_SurfaceOfRevolution AS = new GeomAdaptor_SurfaceOfRevolution(HC, ax1);
            switch (AS.GetType())
            {
                // skl 18.12.2003 - plane not used, problems in PRO14665.igs
                // case GeomAbs_Plane : {
                //  occ::handle<Geom_Plane> Pl = new Geom_Plane(AS.Plane());
                //  S = Pl;
                //} break;
                case GeomAbs_Cylinder:
                {
                    Geom_CylindricalSurface Cy = new Geom_CylindricalSurface(AS.Cylinder());
                    S = Cy;
                }
                    break;
                case GeomAbs_Sphere:
                {
                    Geom_SphericalSurface Sp = new Geom_SphericalSurface(AS.Sphere());
                    S = Sp;
                }
                    break;
                case GeomAbs_Cone:
                {
                    Geom_ConicalSurface Co = new Geom_ConicalSurface(AS.Cone());
                    S = Co;
                }
                    break;
                case GeomAbs_Torus:
                {
                    Geom_ToroidalSurface To = new Geom_ToroidalSurface(AS.Torus());
                    S = To;
                }
                    break;
                default:
                    return false;
            }
        }
        // case SurfaceOfLinearExtrusion
        else if (SS is Geom_SurfaceOfLinearExtrusion SLE)
        {
            Geom_Curve bc = SLE.BasisCurve();
            gp_Dir dir = SLE.Direction();
            GeomAdaptor_Curve HC = new GeomAdaptor_Curve();
            HC.Load(bc, bc.FirstParameter(), bc.LastParameter());
            GeomAdaptor_SurfaceOfLinearExtrusion AS = new GeomAdaptor_SurfaceOfLinearExtrusion(HC, dir);
            switch (AS.GetType())
            {
                // skl 18.12.2003 - plane not used, problems in ims013.igs
                // case GeomAbs_Plane : {
                //  occ::handle<Geom_Plane> Pl = new Geom_Plane(AS.Plane());
                //  S = Pl;
                //} break;
                case GeomAbs_Cylinder:
                {
                    Geom_CylindricalSurface Cy = new Geom_CylindricalSurface(AS.Cylinder());
                    S = Cy;
                }
                    break;
                default:
                    return false;
            }
        }

        SendMsg(F, new Message_Msg("SweptToElementary.NewSurface.MSG0"));

        Tol = BRep_Tool.Tolerance(F);
        RevWires = false;
        RevFace = false;
        return true;
    }

    /// <summary>
    /// True if the edge has a pcurve on a swept surface (so that the edge is copied): then C is a copy of its 3D curve.
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
    /// True if the face's surface is swept or the edge was copied: C is a copy of the pcurve, shifted to the new
    /// torus's or sphere's parametrisation.
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
        if (!IsToConvert(S, out Geom_SweptSurface? SS) && E.IsSame(NewE))
        {
            return false;
        }

        C = BRep_Tool.CurveOnSurface(E, F, out double f, out double l);
        if (C is not null)
        {
            C = (Geom2d_Curve)C.Copy();
            // OCCT: dereferences the null face the modifier passes when it asks whether to copy the vertices
            // (BRepTools_Modifier::CreateOtherVertices), an access violation natively (occt-issues.md); the port's
            // BRep_Tool throws a NullReferenceException there
            Geom_Surface? NS = BRep_Tool.Surface(NewF, out L);
            if (NS is not null && NS is Geom_ToroidalSurface)
            {
                // OCCT: SS is null when the face's surface isn't swept and only the edge was copied, an access
                // violation natively, a NullReferenceException in the port (occt-issues.md, decision 1024)
                if ((SS ?? throw NullSweptSurface()) is Geom_SurfaceOfRevolution SR)
                {
                    SR.Bounds(out double U1, out double U2, out double V1, out double V2);
                    SR.D0(U1, V1, out gp_Pnt P0);
                    ShapeAnalysis_Surface sas = new ShapeAnalysis_Surface(NS);
                    gp_Pnt2d p2d = sas.ValueOfUV(P0, Precision.Confusion());
                    gp_Vec2d shift = new gp_Vec2d(p2d.X() - U1, p2d.Y() - V1);
                    C.Translate(shift);
                }
            }
            if (NS is not null && NS is Geom_SphericalSurface)
            {
                // OCCT: as above
                if ((SS ?? throw NullSweptSurface()) is Geom_SurfaceOfRevolution SR)
                {
                    Geom_SphericalSurface SPH = (Geom_SphericalSurface)NS;
                    SPH.Bounds(out double US1, out double US2, out double VS1, out double VS2);
                    SPH.D0(US1, VS1, out gp_Pnt PS);
                    SR.Bounds(out double UR1, out double UR2, out double VR1, out double VR2);
                    SR.D0(UR1, VR1, out gp_Pnt PR);
                    gp_Pnt P0 = SPH.Location();
                    gp_Vec VS = new gp_Vec(P0, PS);
                    gp_Vec VR = new gp_Vec(P0, PR);
                    double angle = VS.Angle(VR);
                    gp_Vec2d shift = new gp_Vec2d(0, VS1 - VR1 + angle);
                    C.Translate(shift);
                }
            }
        }

        Tol = BRep_Tool.Tolerance(E);
        return true;
    }

    // OCCT dereferences the null swept surface: an access violation natively (decision 1024)
    private static NullReferenceException NullSweptSurface() =>
        new("ShapeCustom_SweptToElementary::NewCurve2d: the face's surface isn't swept (OCCT dereferences a null handle)");

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
