// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Surface.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Surface.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// A face's surface seen through a projector: its type (degree-1 Bezier surfaces count as planes), whether it is seen
/// edge-on (<see cref="IsSide"/>), and whether a projected edge lies above it (<see cref="IsAbove"/>).
/// </summary>
/// <remarks>
/// A working object, as OCCT's (decision 1092): HLRBRep_FaceData holds one by value. OCCT keeps a pointer to
/// HLRBRep_Data's projector; the port keeps the box that holds it (decision 1093). <see cref="Copy"/> is OCCT's
/// implicit copy. OCCT declares <c>UIntervalContinuity</c> and <c>VIntervalContinuity</c> without defining them; they
/// are left out.
/// </remarks>
public sealed class HLRBRep_Surface
{
    private readonly FaceSurfaceAdaptor mySurf;
    private SurfaceKind myType;
    private StrongBox<HLRAlgo_Projector>? myProj;

    /// <summary>Creates an undefined surface.</summary>
    public HLRBRep_Surface()
    {
        mySurf = new FaceSurfaceAdaptor();
        myType = OtherSurface;
        myProj = null;
    }

    private HLRBRep_Surface(HLRBRep_Surface theOther)
    {
        mySurf = (FaceSurfaceAdaptor)theOther.mySurf.ShallowCopy();
        myType = theOther.myType;
        myProj = theOther.myProj;
    }

    /// <summary>OCCT's implicit copy: the adaptor copied, the projector shared.</summary>
    public HLRBRep_Surface Copy() => new(this);

    /// <summary>Sets the projector (OCCT's pointer to the projector HLRBRep_Data holds).</summary>
    public void Projector(StrongBox<HLRAlgo_Projector> Proj) => myProj = Proj;

    private ref readonly HLRAlgo_Projector Proj() => ref myProj!.Value;

    /// <summary>Returns the 3D surface.</summary>
    public FaceSurfaceAdaptor Surface() => mySurf;

    /// <summary>Sets the 3D surface to be projected.</summary>
    public void Surface(in Face F)
    {
        mySurf.Initialize(F, true);
        SurfaceKind typ = HLRBRep_BSurfaceTool.GetType(mySurf);
        switch (typ)
        {

            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
                // unchanged type
                myType = typ;
                break;

            case SurfaceKind.BezierSurface:
                if (HLRBRep_BSurfaceTool.UDegree(mySurf) == 1 && HLRBRep_BSurfaceTool.VDegree(mySurf) == 1)
                {
                    myType = global::Kerf.Geometry.SurfaceKind.Plane;
                }
                else
                {
                    myType = typ;
                }
                break;

            default:
                myType = OtherSurface;
                break;
        }
    }

    /// <summary>Returns true if all the poles project on the same line (an iso row or column), or on a plane.</summary>
    private bool SideRowsOfPoles(double tol, int nbuPoles, int nbvPoles, NCollection_Array2<Point3D> Pnt)
    {
        int iu, iv;
        double x0, y0, x, y, z;
        bool result;
        double tole = (double)tol;
        Transform3D T = Proj().Transformation();

        for (iu = 1; iu <= nbuPoles; iu++)
        {
            for (iv = 1; iv <= nbvPoles; iv++)
            {
                Pnt[iu, iv].Transform(T);
            }
        }

        result = true;

        for (iu = 1; iu <= nbuPoles && result; iu++)
        { // Side iso u ?
            Pnt[iu, 1].Coord(out x0, out y0, out z);

            for (iv = 2; iv <= nbvPoles && result; iv++)
            {
                Pnt[iu, iv].Coord(out x, out y, out z);
                result = Abs(x - x0) < tole && Abs(y - y0) < tole;
            }
        }
        if (result)
        {
            return result;
        }
        result = true;

        for (iv = 1; iv <= nbvPoles && result; iv++)
        { // Side iso v ?
            Pnt[1, iv].Coord(out x0, out y0, out z);

            for (iu = 2; iu <= nbuPoles && result; iu++)
            {
                Pnt[iu, iv].Coord(out x, out y, out z);
                result = Abs(x - x0) < tole && Abs(y - y0) < tole;
            }
        }
        if (result)
        {
            return result;
        }

        // Are the Poles in a Side Plane ?
        NCollection_Array1<Point3D> p = new(1, nbuPoles * nbvPoles);
        int i = 0;

        for (iu = 1; iu <= nbuPoles; iu++)
        {
            for (iv = 1; iv <= nbvPoles; iv++)
            {
                i++;
                p[i] = Pnt[iu, iv];
            }
        }

        PEquation Pl = new(p, tol);
        if (Pl.IsPlanar)
        {
            result = Abs(Pl.Plane().Axis().Direction().Z()) < 0.0001;
        }

        return result;
    }

    /// <summary>Returns true if the surface is seen edge-on (its projection has no area).</summary>
    public bool IsSide(double tolF, double toler)
    {
        Point3D Pt;
        Vector3D D;
        double r;

        if (myType == global::Kerf.Geometry.SurfaceKind.Plane)
        {
            Plane3D Pl = Plane();
            Axis3D A = Pl.Axis();
            Pt = A.Location();
            D = new Vector3D(A.Direction());
            Pt.Transform(Proj().Transformation());
            D.Transform(Proj().Transformation());
            if (Proj().Perspective())
            {
                r = D.Z() * Proj().Focus() - (D.X() * Pt.X() + D.Y() * Pt.Y() + D.Z() * Pt.Z());
            }
            else
            {
                r = D.Z();
            }
            return Abs(r) < toler;
        }
        else if (myType == global::Kerf.Geometry.SurfaceKind.Cylinder)
        {
            if (Proj().Perspective())
            {
                return false;
            }
            Cylinder Cyl = HLRBRep_BSurfaceTool.Cylinder(mySurf);
            Axis3D A = Cyl.Axis();
            D = new Vector3D(A.Direction());
            D.Transform(Proj().Transformation());
            r = Sqrt(D.X() * D.X() + D.Y() * D.Y());
            return r < toler;
        }
        else if (myType == global::Kerf.Geometry.SurfaceKind.Cone)
        {
            if (!Proj().Perspective())
            {
                return false;
            }
            Cone Con = HLRBRep_BSurfaceTool.Cone(mySurf);
            Pt = Con.Apex();
            Pt.Transform(Proj().Transformation());
            double tol = 0.001;
            return Pt.IsEqual(new Point3D(0, 0, Proj().Focus()), tol);
        }
        else if (myType == SurfaceKind.BezierSurface)
        {
            if (Proj().Perspective())
            {
                return false;
            }
            int nu = HLRBRep_BSurfaceTool.NbUPoles(mySurf);
            int nv = HLRBRep_BSurfaceTool.NbVPoles(mySurf);
            NCollection_Array2<Point3D> Pnt = new(1, nu, 1, nv);
            NCollection_Array2<Point3D> aSrcPoles = HLRBRep_BSurfaceTool.Bezier(mySurf).InternalPoles();
            for (int iu = 1; iu <= nu; iu++)
            {
                for (int iv = 1; iv <= nv; iv++)
                {
                    Pnt[iu, iv] = aSrcPoles[iu, iv];
                }
            }
            return SideRowsOfPoles(tolF, nu, nv, Pnt);
        }
        else if (myType == SurfaceKind.BSplineSurface)
        {
            if (Proj().Perspective())
            {
                return false;
            }
            int nu = HLRBRep_BSurfaceTool.NbUPoles(mySurf);
            int nv = HLRBRep_BSurfaceTool.NbVPoles(mySurf);
            NCollection_Array2<Point3D> Pnt = new(1, nu, 1, nv);
            NCollection_Array2<Point3D> aSrcPoles = HLRBRep_BSurfaceTool.BSpline(mySurf).InternalPoles();
            for (int iu = 1; iu <= nu; iu++)
            {
                for (int iv = 1; iv <= nv; iv++)
                {
                    Pnt[iu, iv] = aSrcPoles[iu, iv];
                }
            }
            return SideRowsOfPoles(tolF, nu, nv, Pnt);
        }
        else
        {
            return false;
        }
    }

    /// <summary>
    /// Returns true if the curve A lies above the surface (only planes are tested: on the side of the normal, or of
    /// its opposite if <paramref name="back"/>), within tol.
    /// </summary>
    public bool IsAbove(bool back, HLRBRep_Curve A, double tol)
    {
        bool planar = (myType == global::Kerf.Geometry.SurfaceKind.Plane);
        if (planar)
        {
            Plane3D Pl = Plane();
            double a, b, c, d;
            Pl.Coefficients(out a, out b, out c, out d);
            double u, u1, u2, dd, x, y, z;
            Point3D P;
            u1 = A.Parameter3d(A.FirstParameter());
            u2 = A.Parameter3d(A.LastParameter());
            u = u1;
            A.D0(u, out P);
            P.Coord(out x, out y, out z);
            dd = a * x + b * y + c * z + d;
            if (back)
            {
                dd = -dd;
            }
            if (dd < -tol)
            {
                return false;
            }
            if (A.GetType() != Line)
            {
                int nbPnt = 30;
                double step = (u2 - u1) / (nbPnt + 1);
                for (int i = 1; i <= nbPnt; i++)
                {
                    u += step;
                    A.D0(u, out P);
                    P.Coord(out x, out y, out z);
                    dd = a * x + b * y + c * z + d;
                    if (back)
                    {
                        dd = -dd;
                    }
                    if (dd < -tol)
                    {
                        return false;
                    }
                }
            }
            u = u2;
            A.D0(u, out P);
            P.Coord(out x, out y, out z);
            dd = a * x + b * y + c * z + d;
            if (back)
            {
                dd = -dd;
            }
            if (dd < -tol)
            {
                return false;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    public double FirstUParameter() => HLRBRep_BSurfaceTool.FirstUParameter(mySurf);

    public double LastUParameter() => HLRBRep_BSurfaceTool.LastUParameter(mySurf);

    public double FirstVParameter() => HLRBRep_BSurfaceTool.FirstVParameter(mySurf);

    public double LastVParameter() => HLRBRep_BSurfaceTool.LastVParameter(mySurf);

    public Continuity UContinuity() => HLRBRep_BSurfaceTool.UContinuity(mySurf);

    public Continuity VContinuity() => HLRBRep_BSurfaceTool.VContinuity(mySurf);

    public int NbUIntervals(Continuity S) => HLRBRep_BSurfaceTool.NbUIntervals(mySurf, S);

    public int NbVIntervals(Continuity S) => HLRBRep_BSurfaceTool.NbVIntervals(mySurf, S);

    public bool IsUClosed() => HLRBRep_BSurfaceTool.IsUClosed(mySurf);

    public bool IsVClosed() => HLRBRep_BSurfaceTool.IsVClosed(mySurf);

    public bool IsUPeriodic() => HLRBRep_BSurfaceTool.IsUPeriodic(mySurf);

    public double UPeriod() => HLRBRep_BSurfaceTool.UPeriod(mySurf);

    public bool IsVPeriodic() => HLRBRep_BSurfaceTool.IsVPeriodic(mySurf);

    public double VPeriod() => HLRBRep_BSurfaceTool.VPeriod(mySurf);

    /// <summary>The point of parameters (U, V).</summary>
    public Point3D Value(double U, double V)
    {
        D0(U, V, out Point3D P);
        return P;
    }

    public void D0(double U, double V, out Point3D P)
    {
        HLRBRep_BSurfaceTool.D0(mySurf, U, V, out P);
    }

    public void D1(double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V)
    {
        HLRBRep_BSurfaceTool.D1(mySurf, U, V, out P, out D1U, out D1V);
    }

    public void D2(double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V, out Vector3D D2U, out Vector3D D2V,
                   out Vector3D D2UV)
    {
        HLRBRep_BSurfaceTool.D2(mySurf, U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV);
    }

    public void D3(double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V, out Vector3D D2U, out Vector3D D2V,
                   out Vector3D D2UV, out Vector3D D3U, out Vector3D D3V, out Vector3D D3UUV, out Vector3D D3UVV)
    {
        HLRBRep_BSurfaceTool.D3(mySurf, U, V, out P, out D1U, out D1V, out D2U, out D2V, out D2UV, out D3U, out D3V,
                                out D3UUV, out D3UVV);
    }

    public Vector3D DN(double U, double V, int Nu, int Nv) => HLRBRep_BSurfaceTool.DN(mySurf, U, V, Nu, Nv);

    /// <summary>The type of the projected surface.</summary>
    public new SurfaceKind GetType() => myType;

    /// <summary>The plane: the adaptor's, or for a Bezier surface of degree 1, the tangent plane at (0.5, 0.5).</summary>
    public Plane3D Plane()
    {
        SurfaceKind typ = HLRBRep_BSurfaceTool.GetType(mySurf);
        switch (typ)
        {
            case SurfaceKind.BezierSurface:
            {
                D1(0.5, 0.5, out Point3D P, out Vector3D D1U, out Vector3D D1V);
                return new Plane3D(P, new Direction3D(D1U.Crossed(D1V)));
            }

            default:
                return HLRBRep_BSurfaceTool.Plane(mySurf);
        }
    }

    public Cylinder Cylinder() => HLRBRep_BSurfaceTool.Cylinder(mySurf);

    public Cone Cone() => HLRBRep_BSurfaceTool.Cone(mySurf);

    public Sphere Sphere() => HLRBRep_BSurfaceTool.Sphere(mySurf);

    public Torus Torus() => HLRBRep_BSurfaceTool.Torus(mySurf);

    public int UDegree() => HLRBRep_BSurfaceTool.UDegree(mySurf);

    public int NbUPoles() => HLRBRep_BSurfaceTool.NbUPoles(mySurf);

    public int VDegree() => HLRBRep_BSurfaceTool.VDegree(mySurf);

    public int NbVPoles() => HLRBRep_BSurfaceTool.NbVPoles(mySurf);

    public int NbUKnots() => HLRBRep_BSurfaceTool.NbUKnots(mySurf);

    public int NbVKnots() => HLRBRep_BSurfaceTool.NbVKnots(mySurf);

    public Axis3D Axis() => HLRBRep_BSurfaceTool.AxeOfRevolution(mySurf);
}
