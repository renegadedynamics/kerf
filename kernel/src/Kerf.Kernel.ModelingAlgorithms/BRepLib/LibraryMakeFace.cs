// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeFace.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Construction.Library.LibraryFaceError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Makes a face: from a surface (gp or Geom), bounded by its natural bounds or by parameter values; from a planar
/// wire (the plane found by <see cref="FindSurface"/>); from a surface and a wire; or a face with wires added.
/// </summary>
/// <remarks>
/// <para>With parameter bounds the face gets a wire of iso-curve edges (degenerated where an iso-curve collapses, a
/// seam where the surface is closed); with infinite bounds, no wire on those sides. A face from a closed wire is
/// reversed if the wire would bound an infinite area (unless <c>Inside</c> is false).</para>
/// <para><see cref="Face"/> freezes the face (design 4.2); ported code reads <see cref="InternalFace"/>. Adding a
/// wire after a public read goes on with a new face holding the same wires. The face from a planar wire updates the
/// tolerances of the wire's edges and vertices and makes its edges same-parameter, as OCCT does, in a working overlay
/// (unit 10H3): the face holds the caller's own edges and vertices wherever they are unchanged, so faces built on
/// wires sharing edges still share them; <see cref="InternalHistory"/> records the ones replaced.</para>
/// </remarks>
internal class LibraryMakeFace : LibraryMakeShape
{
    private LibraryFaceError _error;
    // Kerf: the history of the wire's sub-shapes the face replaced (null if none).
    private History? _history;

    /// <summary>Not done.</summary>
    public LibraryMakeFace()
    {
        _error = LibraryFaceError.NoFace;
    }

    /// <summary>Loads the face (a copy holding the same wires), to add wires to it.</summary>
    public LibraryMakeFace(in Face f)
    {
        Init(f);
    }

    /// <summary>A face on the whole plane.</summary>
    public LibraryMakeFace(in Plane3D p)
    {
        var gp = new PlaneSurface(p);
        Init(gp, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole cylinder.</summary>
    public LibraryMakeFace(in Cylinder3D c)
    {
        var gc = new CylindricalSurface(c);
        Init(gc, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole cone.</summary>
    public LibraryMakeFace(in Cone3D c)
    {
        var gc = new ConicalSurface(c);
        Init(gc, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole sphere.</summary>
    public LibraryMakeFace(in Sphere3D s)
    {
        var gs = new SphericalSurface(s);
        Init(gs, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole torus.</summary>
    public LibraryMakeFace(in Torus3D t)
    {
        var gt = new ToroidalSurface(t);
        Init(gt, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole surface; <paramref name="tolDegen"/> decides degenerated boundaries.</summary>
    public LibraryMakeFace(Surface s, double tolDegen)
    {
        Init(s, true, tolDegen);
    }

    public LibraryMakeFace(in Plane3D p, double uMin, double uMax, double vMin, double vMax)
    {
        var gp = new PlaneSurface(p);
        Init(gp, uMin, uMax, vMin, vMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public LibraryMakeFace(in Cylinder3D c, double uMin, double uMax, double vMin, double vMax)
    {
        var gc = new CylindricalSurface(c);
        Init(gc, uMin, uMax, vMin, vMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public LibraryMakeFace(in Cone3D c, double uMin, double uMax, double vMin, double vMax)
    {
        var gc = new ConicalSurface(c);
        Init(gc, uMin, uMax, vMin, vMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public LibraryMakeFace(in Sphere3D s, double uMin, double uMax, double vMin, double vMax)
    {
        var gs = new SphericalSurface(s);
        Init(gs, uMin, uMax, vMin, vMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public LibraryMakeFace(in Torus3D t, double uMin, double uMax, double vMin, double vMax)
    {
        var gt = new ToroidalSurface(t);
        Init(gt, uMin, uMax, vMin, vMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public LibraryMakeFace(Surface s, double uMin, double uMax, double vMin, double vMax, double tolDegen)
    {
        Init(s, uMin, uMax, vMin, vMax, tolDegen);
    }

    /// <summary>
    /// A face on a surface through the wire (with <paramref name="onlyPlane"/>, a plane), found by
    /// <see cref="FindSurface"/>; BRepLib_NotPlanar if there is none.
    /// </summary>
    public LibraryMakeFace(in Wire w, bool onlyPlane = false)
    {
        // Find a surface through the wire
        var fs = new FindSurface(w, -1, onlyPlane, true);
        if (!fs.Found())
        {
            _error = LibraryFaceError.NotPlanar;
            return;
        }

        // build the face and add the wire
        var B = new BrepBuilder();
        _error = LibraryFaceError.FaceDone;

        double tol = Max(1.2 * fs.ToleranceReached(), fs.Tolerance());

        Face face = new();
        B.MakeFace(ref face, fs.Surface(), fs.Location(), tol);
        _shape = face;

        Wire aW = new();
        if (onlyPlane)
        {
            // get rid of degenerative edges in the input wire
            var b = new BrepBuilder();
            b.MakeWire(ref aW);

            Wire wForw = w;
            bool hasDegenerated = false;
            wForw.Orient(Forward);
            var iter = new Iterator(wForw);
            for (; iter.More(); iter.Next())
            {
                Edge e = ShapeCasts.Edge(iter.Value());

                if (Tool.Degenerated(e))
                {
                    hasDegenerated = true;
                }
                else
                {
                    b.Add(aW, e);
                }
            }

            if (hasDegenerated)
            {
                aW.Orient(w.Orientation); // return to original orient
                aW.Closed(w.ClosedFlag);
            }
            else
            {
                aW = w;
            }
        }
        else
        {
            aW = w;
        }

        // Kerf: UpdateTolerances and SameParameter below write into the wire's edges and vertices: in a working overlay
        // (unit 10H3), so the face holds the caller's own edges and vertices wherever they are unchanged, and faces
        // built on wires sharing edges share them. UpdateTolerances raises every vertex by its rounding guard
        // (2 * Epsilon(tol)); a raise of no more than that is ignored, else no frozen vertex could ever be kept.
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(aW);
        Add(aW);
        //
        BrepLibrary.UpdateTolerances(_shape);
        //
        BrepLibrary.SameParameter(_shape, tol, true);
        //
        if (overlay is not null)
        {
            _shape = overlay.Finish(_shape, null, true);
            _history = overlay.History();
            aW = ShapeCasts.Wire(overlay.Final(aW));
        }
        //
        if (Tool.IsClosed(aW))
        {
            CheckInside();
        }
    }

    /// <summary>A face on the plane bounded by the wire; reversed if the wire would bound an infinite area.</summary>
    public LibraryMakeFace(in Plane3D p, in Wire w, bool inside = true)
    {
        var pl = new PlaneSurface(p);
        Init(pl, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(w);
        if (inside && Tool.IsClosed(w))
        {
            CheckInside();
        }
    }

    public LibraryMakeFace(in Cylinder3D c, in Wire w, bool inside = true)
    {
        var gc = new CylindricalSurface(c);
        Init(gc, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(w);
        if (inside && Tool.IsClosed(w))
        {
            CheckInside();
        }
    }

    public LibraryMakeFace(in Cone3D c, in Wire w, bool inside = true)
    {
        var gc = new ConicalSurface(c);
        Init(gc, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(w);
        if (inside && Tool.IsClosed(w))
        {
            CheckInside();
        }
    }

    public LibraryMakeFace(in Sphere3D s, in Wire w, bool inside = true)
    {
        var gs = new SphericalSurface(s);
        Init(gs, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(w);
        if (inside && Tool.IsClosed(w))
        {
            CheckInside();
        }
    }

    public LibraryMakeFace(in Torus3D t, in Wire w, bool inside = true)
    {
        var gt = new ToroidalSurface(t);
        Init(gt, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(w);
        if (inside && Tool.IsClosed(w))
        {
            CheckInside();
        }
    }

    public LibraryMakeFace(Surface s, in Wire w, bool inside = true)
    {
        Init(s, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(w);
        if (inside && Tool.IsClosed(w))
        {
            CheckInside();
        }
    }

    /// <summary>A copy of the face with the wire added.</summary>
    public LibraryMakeFace(in Face f, in Wire w)
    {
        Init(f);
        Add(w);
    }

    /// <summary>Loads the face: a new face on its surface holding the same wires.</summary>
    public void Init(in Face f)
    {
        // copy the face
        _shape = f.EmptyCopied();
        _error = LibraryFaceError.FaceDone;

        var b = new BrepBuilder();
        var it = new Iterator(f);
        while (it.More())
        {
            b.Add(_shape, it.Value());
            it.Next();
        }
    }

    /// <summary>A face on the surface; with <paramref name="bound"/>, bounded by its natural bounds.</summary>
    public void Init(Surface s, bool bound, double tolDegen)
    {
        _error = LibraryFaceError.FaceDone;
        if (bound)
        {
            s.ParameterBounds(out double uMin, out double uMax, out double vMin, out double vMax);
            Init(s, uMin, uMax, vMin, vMax, tolDegen);
        }
        else
        {
            var b0 = new BrepBuilder();
            Face face = new();
            b0.MakeFace(ref face, s, global::Kerf.Engine.Foundation.Precision.Confusion());
            _shape = face;
        }
        var b = new BrepBuilder();
        b.NaturalRestriction(ShapeCasts.Face(_shape), true);
    }

    /// <summary>
    /// True if the curve is degenerated within <paramref name="maxTol"/> (a circle of smaller radius, a B-spline
    /// or Bezier curve whose poles are that close to the first); <paramref name="actTol"/> is the tolerance that
    /// decided it. For BRepLib_MakeFace and BRepLib_MakeShell.
    /// </summary>
    public static bool IsDegenerated(Curve3D curve, double maxTol, out double actTol)
    {
        var ac = new CurveAdaptor(curve);
        double confusion = global::Kerf.Engine.Foundation.Precision.Confusion();
        actTol = confusion;
        CurveKind type = ac.GetType();

        if (type == Circle)
        {
            Circle3D circ = ac.Circle();
            if (circ.Radius > maxTol)
            {
                return false;
            }
            actTol = Max(circ.Radius, confusion);
            return true;
        }
        else if (type == BSplineCurve)
        {
            BSplineCurve3D bs = ac.BSpline();
            int nbPoles = bs.PoleCount;
            double maxPoleDist2 = 0.0, maxTol2 = maxTol * maxTol;
            Point3D p1, p2;
            p1 = bs.Pole(1);
            for (int i = 2; i <= nbPoles; i++)
            {
                p2 = bs.Pole(i);
                double poleDist2 = p1.SquareDistanceTo(p2);
                if (poleDist2 > maxTol2)
                {
                    return false;
                }
                if (poleDist2 > maxPoleDist2)
                {
                    maxPoleDist2 = poleDist2;
                }
            }
            actTol = Max(1.000001 * Math.Sqrt(maxPoleDist2), confusion);
            return true;
        }
        else if (type == BezierCurve)
        {
            BezierCurve3D bz = ac.Bezier();
            int nbPoles = bz.PoleCount;
            double maxPoleDist2 = 0.0, maxTol2 = maxTol * maxTol;
            Point3D p1, p2;
            p1 = bz.Pole(1);
            for (int i = 2; i <= nbPoles; i++)
            {
                p2 = bz.Pole(i);
                double poleDist2 = p1.SquareDistanceTo(p2);
                if (poleDist2 > maxTol2)
                {
                    return false;
                }
                if (poleDist2 > maxPoleDist2)
                {
                    maxPoleDist2 = poleDist2;
                }
            }
            actTol = Max(1.000001 * Math.Sqrt(maxPoleDist2), confusion);
            return true;
        }

        return false;
    }

    /// <summary>A face on the surface between the parameter values (adjusted into the periods of a periodic surface).</summary>
    public void Init(Surface ss, double Um, double um, double Vm, double vm, double tolDegen)
    {
        _error = LibraryFaceError.FaceDone;

        double uMin = Um;
        double uMax = um;
        double vMin = Vm;
        double vMax = vm;

        double t;

        Surface s = ss, bs = ss;
        if (s is TrimmedSurface rs)
        {
            bs = rs.BasisSurface;
        }

        bool offsetSurface = bs.GetType() == typeof(OffsetSurface);

        // adjust periodical surface or reordonate
        // check if the values are in the natural range
        double epsilon = global::Kerf.Engine.Foundation.Precision.PConfusion();

        bs.ParameterBounds(out double umin, out double umax, out double vmin, out double vmax);

        if (offsetSurface)
        {
            var os = (OffsetSurface)bs;
            Surface Base = os.BasisSurface;

            if (Base.GetType() == typeof(ExtrusionSurface))
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(umin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(umax))
                {
                    s = new TrimmedSurface(os, uMin, uMax, vMin, vMax);
                }
                else
                {
                    s = new TrimmedSurface(os, vMin, vMax, false);
                }
            }
            else if (Base.GetType() == typeof(RevolutionSurface))
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(vmin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vmax))
                {
                    s = new TrimmedSurface(os, vMin, vMax, false);
                }
            }
        }

        // OCCT checks the natural range only when the parameters come reversed (occt-issues.md): ordered parameters
        // outside it are accepted.
        if (s.IsUPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(umin, umax, epsilon, ref uMin, ref uMax);
        }
        else if (uMin > uMax)
        {
            t = uMin;
            uMin = uMax;
            uMax = t;
            if ((umin - uMin > epsilon) || (uMax - umax > epsilon))
            {
                _error = LibraryFaceError.ParametersOutOfRange;
                return;
            }
        }

        if (s.IsVPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(vmin, vmax, epsilon, ref vMin, ref vMax);
        }
        else if (vMin > vMax)
        {
            t = vMin;
            vMin = vMax;
            vMax = t;
            if ((vmin - vMin > epsilon) || (vMax - vmax > epsilon))
            {
                _error = LibraryFaceError.ParametersOutOfRange;
                return;
            }
        }

        // compute infinite flags
        bool umininf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uMin);
        bool umaxinf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(uMax);
        bool vmininf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin);
        bool vmaxinf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax);

        // closed flag
        bool uclosed = s.IsUClosed && Math.Abs(uMin - umin) < epsilon && Math.Abs(uMax - umax) < epsilon;

        bool vclosed = s.IsVClosed && Math.Abs(vMin - vmin) < epsilon && Math.Abs(vMax - vmax) < epsilon;

        // compute 3d curves and degenerate flag
        double maxTol = tolDegen;
        Curve3D? cumin = null, cumax = null, cvmin = null, cvmax = null;
        bool dumin, dumax, dvmin, dvmax;
        dumin = dumax = dvmin = dvmax = false;
        double uminTol = global::Kerf.Engine.Foundation.Precision.Confusion(), umaxTol = global::Kerf.Engine.Foundation.Precision.Confusion(),
               vminTol = global::Kerf.Engine.Foundation.Precision.Confusion(), vmaxTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        if (!umininf)
        {
            cumin = s.UIso(uMin);
            dumin = IsDegenerated(cumin, maxTol, out uminTol);
        }
        if (!umaxinf)
        {
            cumax = s.UIso(uMax);
            dumax = IsDegenerated(cumax, maxTol, out umaxTol);
        }
        if (!vmininf)
        {
            cvmin = s.VIso(vMin);
            dvmin = IsDegenerated(cvmin, maxTol, out vminTol);
        }
        if (!vmaxinf)
        {
            cvmax = s.VIso(vMax);
            dvmax = IsDegenerated(cvmax, maxTol, out vmaxTol);
        }

        // compute vertices
        var b = new BrepBuilder();

        Vertex v00 = new(), v10 = new(), v11 = new(), v01 = new();

        if (!umininf)
        {
            if (!vmininf)
            {
                b.MakeVertex(ref v00, s.Value(uMin, vMin), Max(uminTol, vminTol));
            }
            if (!vmaxinf)
            {
                b.MakeVertex(ref v01, s.Value(uMin, vMax), Max(uminTol, vmaxTol));
            }
        }
        if (!umaxinf)
        {
            if (!vmininf)
            {
                b.MakeVertex(ref v10, s.Value(uMax, vMin), Max(umaxTol, vminTol));
            }
            if (!vmaxinf)
            {
                b.MakeVertex(ref v11, s.Value(uMax, vMax), Max(umaxTol, vmaxTol));
            }
        }

        if (uclosed)
        {
            v10 = v00;
            v11 = v01;
        }

        if (vclosed)
        {
            v01 = v00;
            v11 = v10;
        }

        if (dumin)
        {
            v00 = v01;
        }
        if (dumax)
        {
            v10 = v11;
        }
        if (dvmin)
        {
            v00 = v10;
        }
        if (dvmax)
        {
            v01 = v11;
        }

        // make the lines
        LineCurve2D? lumin = null, lumax = null, lvmin = null, lvmax = null;
        if (!umininf)
        {
            lumin = new LineCurve2D(new Point2D(uMin, 0), new Direction2D(Direction2D.D.Y));
        }
        if (!umaxinf)
        {
            lumax = new LineCurve2D(new Point2D(uMax, 0), new Direction2D(Direction2D.D.Y));
        }
        if (!vmininf)
        {
            lvmin = new LineCurve2D(new Point2D(0, vMin), new Direction2D(Direction2D.D.X));
        }
        if (!vmaxinf)
        {
            lvmax = new LineCurve2D(new Point2D(0, vMax), new Direction2D(Direction2D.D.X));
        }

        // make the face
        // Kerf: OCCT remakes the face in myShape; the port makes it in a new variable (as BRepLib_MakeEdge).
        Face f = new();
        b.MakeFace(ref f, s, global::Kerf.Engine.Foundation.Precision.Confusion());
        _shape = f;

        // make the edges
        Edge eumin = new(), eumax = new(), evmin = new(), evmax = new();

        if (!umininf)
        {
            if (!dumin)
            {
                b.MakeEdge(ref eumin, cumin, uminTol);
            }
            else
            {
                b.MakeEdge(ref eumin);
            }
            if (uclosed)
            {
                b.UpdateEdge(eumin, lumax, lumin, f, Max(uminTol, umaxTol));
            }
            else
            {
                b.UpdateEdge(eumin, lumin, f, uminTol);
            }
            b.Degenerated(eumin, dumin);
            if (!vmininf)
            {
                v00.Orient(Forward);
                b.Add(eumin, v00);
            }
            if (!vmaxinf)
            {
                v01.Orient(Reversed);
                b.Add(eumin, v01);
            }
            b.Range(eumin, vMin, vMax);
        }

        if (!umaxinf)
        {
            if (uclosed)
            {
                eumax = eumin;
            }
            else
            {
                if (!dumax)
                {
                    b.MakeEdge(ref eumax, cumax, umaxTol);
                }
                else
                {
                    b.MakeEdge(ref eumax);
                }
                b.UpdateEdge(eumax, lumax, f, umaxTol);
                b.Degenerated(eumax, dumax);
                if (!vmininf)
                {
                    v10.Orient(Forward);
                    b.Add(eumax, v10);
                }
                if (!vmaxinf)
                {
                    v11.Orient(Reversed);
                    b.Add(eumax, v11);
                }
                b.Range(eumax, vMin, vMax);
            }
        }

        if (!vmininf)
        {
            if (!dvmin)
            {
                b.MakeEdge(ref evmin, cvmin, vminTol);
            }
            else
            {
                b.MakeEdge(ref evmin);
            }
            if (vclosed)
            {
                b.UpdateEdge(evmin, lvmin, lvmax, f, Max(vminTol, vmaxTol));
            }
            else
            {
                b.UpdateEdge(evmin, lvmin, f, vminTol);
            }
            b.Degenerated(evmin, dvmin);
            if (!umininf)
            {
                v00.Orient(Forward);
                b.Add(evmin, v00);
            }
            if (!umaxinf)
            {
                v10.Orient(Reversed);
                b.Add(evmin, v10);
            }
            b.Range(evmin, uMin, uMax);
        }

        if (!vmaxinf)
        {
            if (vclosed)
            {
                evmax = evmin;
            }
            else
            {
                if (!dvmax)
                {
                    b.MakeEdge(ref evmax, cvmax, vmaxTol);
                }
                else
                {
                    b.MakeEdge(ref evmax);
                }
                b.UpdateEdge(evmax, lvmax, f, vmaxTol);
                b.Degenerated(evmax, dvmax);
                if (!umininf)
                {
                    v01.Orient(Forward);
                    b.Add(evmax, v01);
                }
                if (!umaxinf)
                {
                    v11.Orient(Reversed);
                    b.Add(evmax, v11);
                }
                b.Range(evmax, uMin, uMax);
            }
        }

        // make the wires and add them to the face
        eumin.Orient(Reversed);
        evmax.Orient(Reversed);

        Wire w = new();

        if (!umininf && !umaxinf && vmininf && vmaxinf)
        {
            // two wires in u
            b.MakeWire(ref w);
            b.Add(w, eumin);
            b.Add(f, w);
            b.MakeWire(ref w);
            b.Add(w, eumax);
            b.Add(f, w);
            f.Closed(uclosed);
        }

        else if (umininf && umaxinf && !vmininf && !vmaxinf)
        {
            // two wires in v
            b.MakeWire(ref w);
            b.Add(w, evmin);
            b.Add(f, w);
            b.MakeWire(ref w);
            b.Add(w, evmax);
            b.Add(f, w);
            f.Closed(vclosed);
        }

        else if (!umininf || !umaxinf || !vmininf || !vmaxinf)
        {
            // one wire
            b.MakeWire(ref w);
            if (!umininf)
            {
                b.Add(w, eumin);
            }
            if (!vmininf)
            {
                b.Add(w, evmin);
            }
            if (!umaxinf)
            {
                b.Add(w, eumax);
            }
            if (!vmaxinf)
            {
                b.Add(w, evmax);
            }
            b.Add(f, w);
            w.Closed(!umininf && !umaxinf && !vmininf && !vmaxinf);
            f.Closed(uclosed && vclosed);
        }

        if (offsetSurface)
        {
            // Les Isos sont Approximees a Precision::Approximation()
            // et on code Precision::Confusion() dans l'arete.
            // ==> Un petit passage dans SamePrameter pour regler les tolerances.
            BrepLibrary.SameParameter(f, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        }

        Done();
    }

    /// <summary>Adds the wire to the face (a hole, or another boundary); the face is no longer a natural restriction.</summary>
    public void Add(in Wire w)
    {
        WorkingOverlay? overlay = BeginResultChange(); // Kerf: a published face changes into a new one
        var b = new BrepBuilder();
        b.Add(_shape, w);
        b.NaturalRestriction(ShapeCasts.Face(_shape), false);
        EndResultChange(overlay);
        Done();
    }

    /// <summary>The face (frozen). As in OCCT, it isn't checked that the face is done.</summary>
    public Face Face()
    {
        Face face = ShapeCasts.Face(_shape);
        face.TShape()?.FreezeGraph();
        return face;
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal Face InternalFace()
    {
        return ShapeCasts.Face(_shape);
    }

    /// <summary>The face (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Face(LibraryMakeFace maker) => maker.Face();

    /// <summary>Returns the construction status.</summary>
    public LibraryFaceError Error()
    {
        return _error;
    }

    /// <summary>
    /// Kerf: the history of the copy for write of the wire that the face from a wire was built on, when the
    /// wire held frozen sub-shapes: each of its edges and vertices modified into its copy. Null if no copy was made.
    /// </summary>
    internal History? InternalHistory() => _history;

    // Reverses the current face if not a bounded area
    private void CheckInside()
    {
        // compute the area and return the face if the area is negative
        Face f = ShapeCasts.Face(_shape);
        var fClass = new FClass2d(f, 0.0);
        if (fClass.PerformInfinitePoint() == In)
        {
            var b = new BrepBuilder();
            Shape s = _shape.EmptyCopied();
            var it = new Iterator(_shape);
            while (it.More())
            {
                b.Add(s, it.Value().Reversed());
                it.Next();
            }
            _shape = s;
        }
    }
}
