// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeFace.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepLib_FaceError;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Makes a face: from a surface (gp or Geom), bounded by its natural bounds or by parameter values; from a planar
/// wire (the plane found by <see cref="BRepLib_FindSurface"/>); from a surface and a wire; or a face with wires added.
/// </summary>
/// <remarks>
/// <para>With parameter bounds the face gets a wire of iso-curve edges (degenerated where an iso-curve collapses, a
/// seam where the surface is closed); with infinite bounds, no wire on those sides. A face from a closed wire is
/// reversed if the wire would bound an infinite area (unless <c>Inside</c> is false).</para>
/// <para><see cref="Face"/> freezes the face (design 4.2); ported code reads <see cref="InternalFace"/>. Adding a
/// wire after a public read goes on with a new face holding the same wires. The face from a planar wire updates the
/// tolerances of the wire's edges and vertices and makes its edges same-parameter, as OCCT does; when the wire holds
/// frozen sub-shapes, it does so on a copy of the wire for write (design 4.6) and then takes back the caller's own
/// edges and vertices wherever the copy is unchanged (<c>BRepTools.KeepUnchanged</c>), so faces built on wires
/// sharing edges still share them; <see cref="InternalHistory"/> records the ones replaced.</para>
/// </remarks>
public class BRepLib_MakeFace : BRepLib_MakeShape
{
    private BRepLib_FaceError myError;
    // OCCT.NET: the history of the copy for write of the wire (null if none).
    private History? myHistory;

    /// <summary>Not done.</summary>
    public BRepLib_MakeFace()
    {
        myError = BRepLib_NoFace;
    }

    /// <summary>Loads the face (a copy holding the same wires), to add wires to it.</summary>
    public BRepLib_MakeFace(in Face F)
    {
        Init(F);
    }

    /// <summary>A face on the whole plane.</summary>
    public BRepLib_MakeFace(in Plane3D P)
    {
        var GP = new PlaneSurface(P);
        Init(GP, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole cylinder.</summary>
    public BRepLib_MakeFace(in Cylinder C)
    {
        var GC = new CylindricalSurface(C);
        Init(GC, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole cone.</summary>
    public BRepLib_MakeFace(in Cone C)
    {
        var GC = new ConicalSurface(C);
        Init(GC, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole sphere.</summary>
    public BRepLib_MakeFace(in Sphere S)
    {
        var GS = new SphericalSurface(S);
        Init(GS, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole torus.</summary>
    public BRepLib_MakeFace(in Torus T)
    {
        var GT = new ToroidalSurface(T);
        Init(GT, true, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>A face on the whole surface; <paramref name="TolDegen"/> decides degenerated boundaries.</summary>
    public BRepLib_MakeFace(Surface S, double TolDegen)
    {
        Init(S, true, TolDegen);
    }

    public BRepLib_MakeFace(in Plane3D P, double UMin, double UMax, double VMin, double VMax)
    {
        var GP = new PlaneSurface(P);
        Init(GP, UMin, UMax, VMin, VMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public BRepLib_MakeFace(in Cylinder C, double UMin, double UMax, double VMin, double VMax)
    {
        var GC = new CylindricalSurface(C);
        Init(GC, UMin, UMax, VMin, VMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public BRepLib_MakeFace(in Cone C, double UMin, double UMax, double VMin, double VMax)
    {
        var GC = new ConicalSurface(C);
        Init(GC, UMin, UMax, VMin, VMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public BRepLib_MakeFace(in Sphere S, double UMin, double UMax, double VMin, double VMax)
    {
        var GS = new SphericalSurface(S);
        Init(GS, UMin, UMax, VMin, VMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public BRepLib_MakeFace(in Torus T, double UMin, double UMax, double VMin, double VMax)
    {
        var GT = new ToroidalSurface(T);
        Init(GT, UMin, UMax, VMin, VMax, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    public BRepLib_MakeFace(Surface S, double UMin, double UMax, double VMin, double VMax, double TolDegen)
    {
        Init(S, UMin, UMax, VMin, VMax, TolDegen);
    }

    /// <summary>
    /// A face on a surface through the wire (with <paramref name="OnlyPlane"/>, a plane), found by
    /// <see cref="BRepLib_FindSurface"/>; BRepLib_NotPlanar if there is none.
    /// </summary>
    public BRepLib_MakeFace(in Wire W, bool OnlyPlane = false)
    {
        // Find a surface through the wire
        var FS = new BRepLib_FindSurface(W, -1, OnlyPlane, true);
        if (!FS.Found())
        {
            myError = BRepLib_NotPlanar;
            return;
        }

        // build the face and add the wire
        var B = new BrepBuilder();
        myError = BRepLib_FaceDone;

        double tol = Max(1.2 * FS.ToleranceReached(), FS.Tolerance());

        Face aFace = new();
        B.MakeFace(ref aFace, FS.Surface(), FS.Location(), tol);
        myShape = aFace;

        Wire aW = new();
        if (OnlyPlane)
        {
            // get rid of degenerative edges in the input wire
            var aB = new BrepBuilder();
            aB.MakeWire(ref aW);

            Wire aWForw = W;
            bool hasDegenerated = false;
            aWForw.Orientation(Forward);
            var anIter = new Iterator(aWForw);
            for (; anIter.More(); anIter.Next())
            {
                Edge aE = ShapeCasts.Edge(anIter.Value());

                if (Tool.Degenerated(aE))
                {
                    hasDegenerated = true;
                }
                else
                {
                    aB.Add(aW, aE);
                }
            }

            if (hasDegenerated)
            {
                aW.Orientation(W.Orientation()); // return to original orient
                aW.Closed(W.Closed());
            }
            else
            {
                aW = W;
            }
        }
        else
        {
            aW = W;
        }

        // OCCT.NET: UpdateTolerances and SameParameter below write into the wire's edges and vertices, so a wire
        // holding frozen sub-shapes is copied for write first (design 4.6), and the edges and vertices they leave
        // unchanged are the caller's own again afterwards, so that faces built on wires sharing edges share them.
        // UpdateTolerances raises every vertex by its rounding guard (2 * Epsilon(tol)); a raise of no more than that
        // is ignored, else no frozen vertex could ever be kept.
        Wire aWOriginal = new();
        if (hasFrozenSubShape(aW))
        {
            aWOriginal = aW;
            aW = ShapeCasts.Wire(BrepTools.ExactCopyForWrite(aW, out _));
        }

        Add(aW);
        //
        BRepLib.UpdateTolerances(myShape);
        //
        BRepLib.SameParameter(myShape, tol, true);
        //
        if (!aWOriginal.IsNull)
        {
            aW = ShapeCasts.Wire(BrepTools.KeepUnchanged(aWOriginal, aW, out History aHistory, true));
            myHistory = aHistory;
            if (!ReferenceEquals(aW.TShape(), myShape.TShape()!._shapes.First().TShape()))
            {
                // the face holds the copy: it gets the wire with the kept edges in its place
                myShape.TShape()!._shapes.Clear();
                Add(aW);
            }
        }
        //
        if (Tool.IsClosed(aW))
        {
            CheckInside();
        }
    }

    // True if the shape or any of its sub-shapes is frozen.
    private static bool hasFrozenSubShape(in Shape theShape)
    {
        if (theShape.TShape()!.IsFrozen)
        {
            return true;
        }
        for (var anExp = new Explorer(theShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (anExp.Current().TShape()!.IsFrozen)
            {
                return true;
            }
        }
        for (var anExp = new Explorer(theShape, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            if (anExp.Current().TShape()!.IsFrozen)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>A face on the plane bounded by the wire; reversed if the wire would bound an infinite area.</summary>
    public BRepLib_MakeFace(in Plane3D P, in Wire W, bool Inside = true)
    {
        var Pl = new PlaneSurface(P);
        Init(Pl, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(W);
        if (Inside && Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(in Cylinder C, in Wire W, bool Inside = true)
    {
        var GC = new CylindricalSurface(C);
        Init(GC, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(W);
        if (Inside && Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(in Cone C, in Wire W, bool Inside = true)
    {
        var GC = new ConicalSurface(C);
        Init(GC, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(W);
        if (Inside && Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(in Sphere S, in Wire W, bool Inside = true)
    {
        var GS = new SphericalSurface(S);
        Init(GS, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(W);
        if (Inside && Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(in Torus T, in Wire W, bool Inside = true)
    {
        var GT = new ToroidalSurface(T);
        Init(GT, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(W);
        if (Inside && Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(Surface S, in Wire W, bool Inside = true)
    {
        Init(S, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(W);
        if (Inside && Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    /// <summary>A copy of the face with the wire added.</summary>
    public BRepLib_MakeFace(in Face F, in Wire W)
    {
        Init(F);
        Add(W);
    }

    /// <summary>Loads the face: a new face on its surface holding the same wires.</summary>
    public void Init(in Face F)
    {
        // copy the face
        myShape = F.EmptyCopied();
        myError = BRepLib_FaceDone;

        var B = new BrepBuilder();
        var It = new Iterator(F);
        while (It.More())
        {
            B.Add(myShape, It.Value());
            It.Next();
        }
    }

    /// <summary>A face on the surface; with <paramref name="Bound"/>, bounded by its natural bounds.</summary>
    public void Init(Surface S, bool Bound, double TolDegen)
    {
        myError = BRepLib_FaceDone;
        if (Bound)
        {
            S.Bounds(out double UMin, out double UMax, out double VMin, out double VMax);
            Init(S, UMin, UMax, VMin, VMax, TolDegen);
        }
        else
        {
            var B0 = new BrepBuilder();
            Face aFace = new();
            B0.MakeFace(ref aFace, S, global::Kerf.Engine.Foundation.Precision.Confusion());
            myShape = aFace;
        }
        var B = new BrepBuilder();
        B.NaturalRestriction(ShapeCasts.Face(myShape), true);
    }

    /// <summary>
    /// True if the curve is degenerated within <paramref name="theMaxTol"/> (a circle of smaller radius, a B-spline
    /// or Bezier curve whose poles are that close to the first); <paramref name="theActTol"/> is the tolerance that
    /// decided it. For BRepLib_MakeFace and BRepLib_MakeShell.
    /// </summary>
    public static bool IsDegenerated(Curve3D theCurve, double theMaxTol, out double theActTol)
    {
        var AC = new CurveAdaptor(theCurve);
        double aConfusion = global::Kerf.Engine.Foundation.Precision.Confusion();
        theActTol = aConfusion;
        CurveKind Type = AC.GetType();

        if (Type == Circle)
        {
            Circle3D Circ = AC.Circle();
            if (Circ.Radius() > theMaxTol)
            {
                return false;
            }
            theActTol = Max(Circ.Radius(), aConfusion);
            return true;
        }
        else if (Type == BSplineCurve)
        {
            BSplineCurve3D BS = AC.BSpline();
            int NbPoles = BS.PoleCount;
            double aMaxPoleDist2 = 0.0, aMaxTol2 = theMaxTol * theMaxTol;
            Point3D P1, P2;
            P1 = BS.Pole(1);
            for (int i = 2; i <= NbPoles; i++)
            {
                P2 = BS.Pole(i);
                double aPoleDist2 = P1.SquareDistance(P2);
                if (aPoleDist2 > aMaxTol2)
                {
                    return false;
                }
                if (aPoleDist2 > aMaxPoleDist2)
                {
                    aMaxPoleDist2 = aPoleDist2;
                }
            }
            theActTol = Max(1.000001 * Math.Sqrt(aMaxPoleDist2), aConfusion);
            return true;
        }
        else if (Type == BezierCurve)
        {
            BezierCurve3D BZ = AC.Bezier();
            int NbPoles = BZ.PoleCount;
            double aMaxPoleDist2 = 0.0, aMaxTol2 = theMaxTol * theMaxTol;
            Point3D P1, P2;
            P1 = BZ.Pole(1);
            for (int i = 2; i <= NbPoles; i++)
            {
                P2 = BZ.Pole(i);
                double aPoleDist2 = P1.SquareDistance(P2);
                if (aPoleDist2 > aMaxTol2)
                {
                    return false;
                }
                if (aPoleDist2 > aMaxPoleDist2)
                {
                    aMaxPoleDist2 = aPoleDist2;
                }
            }
            theActTol = Max(1.000001 * Math.Sqrt(aMaxPoleDist2), aConfusion);
            return true;
        }

        return false;
    }

    /// <summary>A face on the surface between the parameter values (adjusted into the periods of a periodic surface).</summary>
    public void Init(Surface SS, double Um, double UM, double Vm, double VM, double TolDegen)
    {
        myError = BRepLib_FaceDone;

        double UMin = Um;
        double UMax = UM;
        double VMin = Vm;
        double VMax = VM;

        double T;

        Surface S = SS, BS = SS;
        if (S is TrimmedSurface RS)
        {
            BS = RS.BasisSurface();
        }

        bool OffsetSurface = BS.GetType() == typeof(OffsetSurface);

        // adjust periodical surface or reordonate
        // check if the values are in the natural range
        double epsilon = global::Kerf.Engine.Foundation.Precision.PConfusion();

        BS.Bounds(out double umin, out double umax, out double vmin, out double vmax);

        if (OffsetSurface)
        {
            var OS = (OffsetSurface)BS;
            Surface Base = OS.BasisSurface();

            if (Base.GetType() == typeof(ExtrusionSurface))
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(umin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(umax))
                {
                    S = new TrimmedSurface(OS, UMin, UMax, VMin, VMax);
                }
                else
                {
                    S = new TrimmedSurface(OS, VMin, VMax, false);
                }
            }
            else if (Base.GetType() == typeof(RevolutionSurface))
            {
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(vmin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vmax))
                {
                    S = new TrimmedSurface(OS, VMin, VMax, false);
                }
            }
        }

        // OCCT checks the natural range only when the parameters come reversed (occt-issues.md): ordered parameters
        // outside it are accepted.
        if (S.IsUPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(umin, umax, epsilon, ref UMin, ref UMax);
        }
        else if (UMin > UMax)
        {
            T = UMin;
            UMin = UMax;
            UMax = T;
            if ((umin - UMin > epsilon) || (UMax - umax > epsilon))
            {
                myError = BRepLib_ParametersOutOfRange;
                return;
            }
        }

        if (S.IsVPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(vmin, vmax, epsilon, ref VMin, ref VMax);
        }
        else if (VMin > VMax)
        {
            T = VMin;
            VMin = VMax;
            VMax = T;
            if ((vmin - VMin > epsilon) || (VMax - vmax > epsilon))
            {
                myError = BRepLib_ParametersOutOfRange;
                return;
            }
        }

        // compute infinite flags
        bool umininf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(UMin);
        bool umaxinf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(UMax);
        bool vmininf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin);
        bool vmaxinf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax);

        // closed flag
        bool uclosed = S.IsUClosed && Math.Abs(UMin - umin) < epsilon && Math.Abs(UMax - umax) < epsilon;

        bool vclosed = S.IsVClosed && Math.Abs(VMin - vmin) < epsilon && Math.Abs(VMax - vmax) < epsilon;

        // compute 3d curves and degenerate flag
        double maxTol = TolDegen;
        Curve3D? Cumin = null, Cumax = null, Cvmin = null, Cvmax = null;
        bool Dumin, Dumax, Dvmin, Dvmax;
        Dumin = Dumax = Dvmin = Dvmax = false;
        double uminTol = global::Kerf.Engine.Foundation.Precision.Confusion(), umaxTol = global::Kerf.Engine.Foundation.Precision.Confusion(),
               vminTol = global::Kerf.Engine.Foundation.Precision.Confusion(), vmaxTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        if (!umininf)
        {
            Cumin = S.UIso(UMin);
            Dumin = IsDegenerated(Cumin, maxTol, out uminTol);
        }
        if (!umaxinf)
        {
            Cumax = S.UIso(UMax);
            Dumax = IsDegenerated(Cumax, maxTol, out umaxTol);
        }
        if (!vmininf)
        {
            Cvmin = S.VIso(VMin);
            Dvmin = IsDegenerated(Cvmin, maxTol, out vminTol);
        }
        if (!vmaxinf)
        {
            Cvmax = S.VIso(VMax);
            Dvmax = IsDegenerated(Cvmax, maxTol, out vmaxTol);
        }

        // compute vertices
        var B = new BrepBuilder();

        Vertex V00 = new(), V10 = new(), V11 = new(), V01 = new();

        if (!umininf)
        {
            if (!vmininf)
            {
                B.MakeVertex(ref V00, S.Value(UMin, VMin), Max(uminTol, vminTol));
            }
            if (!vmaxinf)
            {
                B.MakeVertex(ref V01, S.Value(UMin, VMax), Max(uminTol, vmaxTol));
            }
        }
        if (!umaxinf)
        {
            if (!vmininf)
            {
                B.MakeVertex(ref V10, S.Value(UMax, VMin), Max(umaxTol, vminTol));
            }
            if (!vmaxinf)
            {
                B.MakeVertex(ref V11, S.Value(UMax, VMax), Max(umaxTol, vmaxTol));
            }
        }

        if (uclosed)
        {
            V10 = V00;
            V11 = V01;
        }

        if (vclosed)
        {
            V01 = V00;
            V11 = V10;
        }

        if (Dumin)
        {
            V00 = V01;
        }
        if (Dumax)
        {
            V10 = V11;
        }
        if (Dvmin)
        {
            V00 = V10;
        }
        if (Dvmax)
        {
            V01 = V11;
        }

        // make the lines
        LineCurve2D? Lumin = null, Lumax = null, Lvmin = null, Lvmax = null;
        if (!umininf)
        {
            Lumin = new LineCurve2D(new Point2D(UMin, 0), new Direction2D(Direction2D.D.Y));
        }
        if (!umaxinf)
        {
            Lumax = new LineCurve2D(new Point2D(UMax, 0), new Direction2D(Direction2D.D.Y));
        }
        if (!vmininf)
        {
            Lvmin = new LineCurve2D(new Point2D(0, VMin), new Direction2D(Direction2D.D.X));
        }
        if (!vmaxinf)
        {
            Lvmax = new LineCurve2D(new Point2D(0, VMax), new Direction2D(Direction2D.D.X));
        }

        // make the face
        // OCCT.NET: OCCT remakes the face in myShape; the port makes it in a new variable (as BRepLib_MakeEdge).
        Face F = new();
        B.MakeFace(ref F, S, global::Kerf.Engine.Foundation.Precision.Confusion());
        myShape = F;

        // make the edges
        Edge eumin = new(), eumax = new(), evmin = new(), evmax = new();

        if (!umininf)
        {
            if (!Dumin)
            {
                B.MakeEdge(ref eumin, Cumin, uminTol);
            }
            else
            {
                B.MakeEdge(ref eumin);
            }
            if (uclosed)
            {
                B.UpdateEdge(eumin, Lumax, Lumin, F, Max(uminTol, umaxTol));
            }
            else
            {
                B.UpdateEdge(eumin, Lumin, F, uminTol);
            }
            B.Degenerated(eumin, Dumin);
            if (!vmininf)
            {
                V00.Orientation(Forward);
                B.Add(eumin, V00);
            }
            if (!vmaxinf)
            {
                V01.Orientation(Reversed);
                B.Add(eumin, V01);
            }
            B.Range(eumin, VMin, VMax);
        }

        if (!umaxinf)
        {
            if (uclosed)
            {
                eumax = eumin;
            }
            else
            {
                if (!Dumax)
                {
                    B.MakeEdge(ref eumax, Cumax, umaxTol);
                }
                else
                {
                    B.MakeEdge(ref eumax);
                }
                B.UpdateEdge(eumax, Lumax, F, umaxTol);
                B.Degenerated(eumax, Dumax);
                if (!vmininf)
                {
                    V10.Orientation(Forward);
                    B.Add(eumax, V10);
                }
                if (!vmaxinf)
                {
                    V11.Orientation(Reversed);
                    B.Add(eumax, V11);
                }
                B.Range(eumax, VMin, VMax);
            }
        }

        if (!vmininf)
        {
            if (!Dvmin)
            {
                B.MakeEdge(ref evmin, Cvmin, vminTol);
            }
            else
            {
                B.MakeEdge(ref evmin);
            }
            if (vclosed)
            {
                B.UpdateEdge(evmin, Lvmin, Lvmax, F, Max(vminTol, vmaxTol));
            }
            else
            {
                B.UpdateEdge(evmin, Lvmin, F, vminTol);
            }
            B.Degenerated(evmin, Dvmin);
            if (!umininf)
            {
                V00.Orientation(Forward);
                B.Add(evmin, V00);
            }
            if (!umaxinf)
            {
                V10.Orientation(Reversed);
                B.Add(evmin, V10);
            }
            B.Range(evmin, UMin, UMax);
        }

        if (!vmaxinf)
        {
            if (vclosed)
            {
                evmax = evmin;
            }
            else
            {
                if (!Dvmax)
                {
                    B.MakeEdge(ref evmax, Cvmax, vmaxTol);
                }
                else
                {
                    B.MakeEdge(ref evmax);
                }
                B.UpdateEdge(evmax, Lvmax, F, vmaxTol);
                B.Degenerated(evmax, Dvmax);
                if (!umininf)
                {
                    V01.Orientation(Forward);
                    B.Add(evmax, V01);
                }
                if (!umaxinf)
                {
                    V11.Orientation(Reversed);
                    B.Add(evmax, V11);
                }
                B.Range(evmax, UMin, UMax);
            }
        }

        // make the wires and add them to the face
        eumin.Orientation(Reversed);
        evmax.Orientation(Reversed);

        Wire W = new();

        if (!umininf && !umaxinf && vmininf && vmaxinf)
        {
            // two wires in u
            B.MakeWire(ref W);
            B.Add(W, eumin);
            B.Add(F, W);
            B.MakeWire(ref W);
            B.Add(W, eumax);
            B.Add(F, W);
            F.Closed(uclosed);
        }

        else if (umininf && umaxinf && !vmininf && !vmaxinf)
        {
            // two wires in v
            B.MakeWire(ref W);
            B.Add(W, evmin);
            B.Add(F, W);
            B.MakeWire(ref W);
            B.Add(W, evmax);
            B.Add(F, W);
            F.Closed(vclosed);
        }

        else if (!umininf || !umaxinf || !vmininf || !vmaxinf)
        {
            // one wire
            B.MakeWire(ref W);
            if (!umininf)
            {
                B.Add(W, eumin);
            }
            if (!vmininf)
            {
                B.Add(W, evmin);
            }
            if (!umaxinf)
            {
                B.Add(W, eumax);
            }
            if (!vmaxinf)
            {
                B.Add(W, evmax);
            }
            B.Add(F, W);
            W.Closed(!umininf && !umaxinf && !vmininf && !vmaxinf);
            F.Closed(uclosed && vclosed);
        }

        if (OffsetSurface)
        {
            // Les Isos sont Approximees a Precision::Approximation()
            // et on code Precision::Confusion() dans l'arete.
            // ==> Un petit passage dans SamePrameter pour regler les tolerances.
            BRepLib.SameParameter(F, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        }

        Done();
    }

    /// <summary>Adds the wire to the face (a hole, or another boundary); the face is no longer a natural restriction.</summary>
    public void Add(in Wire W)
    {
        UnfreezeResult(); // OCCT.NET: frozen
        var B = new BrepBuilder();
        B.Add(myShape, W);
        B.NaturalRestriction(ShapeCasts.Face(myShape), false);
        Done();
    }

    /// <summary>The face (frozen). As in OCCT, it isn't checked that the face is done.</summary>
    public Face Face()
    {
        Face aFace = ShapeCasts.Face(myShape);
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal Face InternalFace()
    {
        return ShapeCasts.Face(myShape);
    }

    /// <summary>The face (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Face(BRepLib_MakeFace theMaker) => theMaker.Face();

    /// <summary>Returns the construction status.</summary>
    public BRepLib_FaceError Error()
    {
        return myError;
    }

    /// <summary>
    /// OCCT.NET: the history of the copy for write of the wire that the face from a wire was built on, when the
    /// wire held frozen sub-shapes: each of its edges and vertices modified into its copy. Null if no copy was made.
    /// </summary>
    internal History? InternalHistory() => myHistory;

    // Reverses the current face if not a bounded area
    private void CheckInside()
    {
        // compute the area and return the face if the area is negative
        Face F = ShapeCasts.Face(myShape);
        var FClass = new BRepTopAdaptor_FClass2d(F, 0.0);
        if (FClass.PerformInfinitePoint() == In)
        {
            var B = new BrepBuilder();
            Shape S = myShape.EmptyCopied();
            var it = new Iterator(myShape);
            while (it.More())
            {
                B.Add(S, it.Value().Reversed());
                it.Next();
            }
            myShape = S;
        }
    }
}
