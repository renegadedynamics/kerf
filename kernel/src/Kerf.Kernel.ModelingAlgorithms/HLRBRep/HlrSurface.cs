// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Surface.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Surface.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.HiddenLines;

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
internal sealed class HlrSurface
{
    private readonly FaceSurfaceAdaptor _surf;
    private SurfaceKind _type;
    private StrongBox<AlgorithmProjector>? _proj;

    /// <summary>Creates an undefined surface.</summary>
    public HlrSurface()
    {
        _surf = new FaceSurfaceAdaptor();
        _type = OtherSurface;
        _proj = null;
    }

    private HlrSurface(HlrSurface other)
    {
        _surf = (FaceSurfaceAdaptor)other._surf.ShallowCopy();
        _type = other._type;
        _proj = other._proj;
    }

    /// <summary>OCCT's implicit copy: the adaptor copied, the projector shared.</summary>
    public HlrSurface Copy() => new(this);

    /// <summary>Sets the projector (OCCT's pointer to the projector HLRBRep_Data holds).</summary>
    public void Projector(StrongBox<AlgorithmProjector> proj) => _proj = proj;

    private ref readonly AlgorithmProjector Proj() => ref _proj!.Value;

    /// <summary>Returns the 3D surface.</summary>
    public FaceSurfaceAdaptor Surface() => _surf;

    /// <summary>Sets the 3D surface to be projected.</summary>
    public void Surface(in Face f)
    {
        _surf.Initialize(f, true);
        SurfaceKind typ = BSurfaceTool.GetType(_surf);
        switch (typ)
        {

            case global::Kerf.Geometry.SurfaceKind.Plane:
            case global::Kerf.Geometry.SurfaceKind.Cylinder:
            case global::Kerf.Geometry.SurfaceKind.Cone:
            case global::Kerf.Geometry.SurfaceKind.Sphere:
            case global::Kerf.Geometry.SurfaceKind.Torus:
                // unchanged type
                _type = typ;
                break;

            case SurfaceKind.BezierSurface:
                if (BSurfaceTool.UDegree(_surf) == 1 && BSurfaceTool.VDegree(_surf) == 1)
                {
                    _type = global::Kerf.Geometry.SurfaceKind.Plane;
                }
                else
                {
                    _type = typ;
                }
                break;

            default:
                _type = OtherSurface;
                break;
        }
    }

    /// <summary>Returns true if all the poles project on the same line (an iso row or column), or on a plane.</summary>
    private bool SideRowsOfPoles(double tol, int nbuPoles, int nbvPoles, NCollection_Array2<Point3D> pnt)
    {
        int iu, iv;
        double x0, y0, x, y, z;
        bool result;
        double tole = (double)tol;
        Transform3D t = Proj().Transformation();

        for (iu = 1; iu <= nbuPoles; iu++)
        {
            for (iv = 1; iv <= nbvPoles; iv++)
            {
                pnt[iu, iv] = pnt[iu, iv].Transformed(t);
            }
        }

        result = true;

        for (iu = 1; iu <= nbuPoles && result; iu++)
        { // Side iso u ?
            pnt[iu, 1].Coord(out x0, out y0, out z);

            for (iv = 2; iv <= nbvPoles && result; iv++)
            {
                pnt[iu, iv].Coord(out x, out y, out z);
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
            pnt[1, iv].Coord(out x0, out y0, out z);

            for (iu = 2; iu <= nbuPoles && result; iu++)
            {
                pnt[iu, iv].Coord(out x, out y, out z);
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
                p[i] = pnt[iu, iv];
            }
        }

        PEquation pl = new(p, tol);
        if (pl.IsPlanar)
        {
            result = Abs(pl.Plane().Axis.Direction.Z) < 0.0001;
        }

        return result;
    }

    /// <summary>Returns true if the surface is seen edge-on (its projection has no area).</summary>
    public bool IsSide(double tolF, double toler)
    {
        Point3D pt;
        Vector3D d;
        double r;

        if (_type == global::Kerf.Geometry.SurfaceKind.Plane)
        {
            Plane3D pl = Plane();
            Axis3D a = pl.Axis;
            pt = a.Location;
            d = new Vector3D(a.Direction);
            pt = pt.Transformed(Proj().Transformation());
            d = d.Transformed(Proj().Transformation());
            if (Proj().Perspective())
            {
                r = d.Z * Proj().Focus() - (d.X * pt.X + d.Y * pt.Y + d.Z * pt.Z);
            }
            else
            {
                r = d.Z;
            }
            return Abs(r) < toler;
        }
        else if (_type == global::Kerf.Geometry.SurfaceKind.Cylinder)
        {
            if (Proj().Perspective())
            {
                return false;
            }
            Cylinder3D cyl = BSurfaceTool.Cylinder(_surf);
            Axis3D a = cyl.Axis;
            d = new Vector3D(a.Direction);
            d = d.Transformed(Proj().Transformation());
            r = Sqrt(d.X * d.X + d.Y * d.Y);
            return r < toler;
        }
        else if (_type == global::Kerf.Geometry.SurfaceKind.Cone)
        {
            if (!Proj().Perspective())
            {
                return false;
            }
            Cone3D con = BSurfaceTool.Cone(_surf);
            pt = con.Apex;
            pt = pt.Transformed(Proj().Transformation());
            double tol = 0.001;
            return pt.IsEqual(new Point3D(0, 0, Proj().Focus()), tol);
        }
        else if (_type == SurfaceKind.BezierSurface)
        {
            if (Proj().Perspective())
            {
                return false;
            }
            int nu = BSurfaceTool.NbUPoles(_surf);
            int nv = BSurfaceTool.NbVPoles(_surf);
            NCollection_Array2<Point3D> pnt = new(1, nu, 1, nv);
            NCollection_Array2<Point3D> srcPoles = BSurfaceTool.Bezier(_surf).InternalPoles();
            for (int iu = 1; iu <= nu; iu++)
            {
                for (int iv = 1; iv <= nv; iv++)
                {
                    pnt[iu, iv] = srcPoles[iu, iv];
                }
            }
            return SideRowsOfPoles(tolF, nu, nv, pnt);
        }
        else if (_type == SurfaceKind.BSplineSurface)
        {
            if (Proj().Perspective())
            {
                return false;
            }
            int nu = BSurfaceTool.NbUPoles(_surf);
            int nv = BSurfaceTool.NbVPoles(_surf);
            NCollection_Array2<Point3D> pnt = new(1, nu, 1, nv);
            NCollection_Array2<Point3D> srcPoles = BSurfaceTool.BSpline(_surf).InternalPoles();
            for (int iu = 1; iu <= nu; iu++)
            {
                for (int iv = 1; iv <= nv; iv++)
                {
                    pnt[iu, iv] = srcPoles[iu, iv];
                }
            }
            return SideRowsOfPoles(tolF, nu, nv, pnt);
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
    public bool IsAbove(bool back, HiddenLineCurve A, double tol)
    {
        bool planar = (_type == global::Kerf.Geometry.SurfaceKind.Plane);
        if (planar)
        {
            Plane3D pl = Plane();
            double a, b, c, d;
            pl.Coefficients(out a, out b, out c, out d);
            double u, u1, u2, dd, x, y, z;
            Point3D p;
            u1 = A.Parameter3d(A.FirstParameter());
            u2 = A.Parameter3d(A.LastParameter());
            u = u1;
            A.D0(u, out p);
            p.Coord(out x, out y, out z);
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
                    A.D0(u, out p);
                    p.Coord(out x, out y, out z);
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
            A.D0(u, out p);
            p.Coord(out x, out y, out z);
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

    public double FirstUParameter() => BSurfaceTool.FirstUParameter(_surf);

    public double LastUParameter() => BSurfaceTool.LastUParameter(_surf);

    public double FirstVParameter() => BSurfaceTool.FirstVParameter(_surf);

    public double LastVParameter() => BSurfaceTool.LastVParameter(_surf);

    public Continuity UContinuity() => BSurfaceTool.UContinuity(_surf);

    public Continuity VContinuity() => BSurfaceTool.VContinuity(_surf);

    public int NbUIntervals(Continuity s) => BSurfaceTool.NbUIntervals(_surf, s);

    public int NbVIntervals(Continuity s) => BSurfaceTool.NbVIntervals(_surf, s);

    public bool IsUClosed => BSurfaceTool.IsUClosed(_surf);

    public bool IsVClosed => BSurfaceTool.IsVClosed(_surf);

    public bool IsUPeriodic => BSurfaceTool.IsUPeriodic(_surf);

    public double UPeriod() => BSurfaceTool.UPeriod(_surf);

    public bool IsVPeriodic => BSurfaceTool.IsVPeriodic(_surf);

    public double VPeriod() => BSurfaceTool.VPeriod(_surf);

    /// <summary>The point of parameters (U, V).</summary>
    public Point3D Value(double u, double v)
    {
        D0(u, v, out Point3D p);
        return p;
    }

    public void D0(double u, double v, out Point3D p)
    {
        BSurfaceTool.D0(_surf, u, v, out p);
    }

    public void D1(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V)
    {
        BSurfaceTool.D1(_surf, u, v, out p, out d1U, out d1V);
    }

    public void D2(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V, out Vector3D d2U, out Vector3D d2V,
                   out Vector3D d2UV)
    {
        BSurfaceTool.D2(_surf, u, v, out p, out d1U, out d1V, out d2U, out d2V, out d2UV);
    }

    public void D3(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V, out Vector3D d2U, out Vector3D d2V,
                   out Vector3D d2UV, out Vector3D d3U, out Vector3D d3V, out Vector3D d3UUV, out Vector3D d3UVV)
    {
        BSurfaceTool.D3(_surf, u, v, out p, out d1U, out d1V, out d2U, out d2V, out d2UV, out d3U, out d3V,
                                out d3UUV, out d3UVV);
    }

    public Vector3D DN(double u, double v, int nu, int nv) => BSurfaceTool.DN(_surf, u, v, nu, nv);

    /// <summary>The type of the projected surface.</summary>
    public new SurfaceKind GetType() => _type;

    /// <summary>The plane: the adaptor's, or for a Bezier surface of degree 1, the tangent plane at (0.5, 0.5).</summary>
    public Plane3D Plane()
    {
        SurfaceKind typ = BSurfaceTool.GetType(_surf);
        switch (typ)
        {
            case SurfaceKind.BezierSurface:
            {
                D1(0.5, 0.5, out Point3D p, out Vector3D d1U, out Vector3D d1V);
                return new Plane3D(p, new Direction3D(d1U.Crossed(d1V)));
            }

            default:
                return BSurfaceTool.Plane(_surf);
        }
    }

    public Cylinder3D Cylinder() => BSurfaceTool.Cylinder(_surf);

    public Cone3D Cone() => BSurfaceTool.Cone(_surf);

    public Sphere3D Sphere() => BSurfaceTool.Sphere(_surf);

    public Torus3D Torus() => BSurfaceTool.Torus(_surf);

    public int UDegree() => BSurfaceTool.UDegree(_surf);

    public int UPoleCount => BSurfaceTool.NbUPoles(_surf);

    public int VDegree() => BSurfaceTool.VDegree(_surf);

    public int VPoleCount => BSurfaceTool.NbVPoles(_surf);

    public int UKnotCount => BSurfaceTool.NbUKnots(_surf);

    public int VKnotCount => BSurfaceTool.NbVKnots(_surf);

    public Axis3D Axis() => BSurfaceTool.AxeOfRevolution(_surf);
}
