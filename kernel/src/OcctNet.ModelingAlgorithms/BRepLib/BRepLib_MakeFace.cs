// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeFace.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepLib_FaceError;
using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

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
    private BRepTools_History? myHistory;

    /// <summary>Not done.</summary>
    public BRepLib_MakeFace()
    {
        myError = BRepLib_NoFace;
    }

    /// <summary>Loads the face (a copy holding the same wires), to add wires to it.</summary>
    public BRepLib_MakeFace(in TopoDS_Face F)
    {
        Init(F);
    }

    /// <summary>A face on the whole plane.</summary>
    public BRepLib_MakeFace(in gp_Pln P)
    {
        var GP = new Geom_Plane(P);
        Init(GP, true, Precision.Confusion());
    }

    /// <summary>A face on the whole cylinder.</summary>
    public BRepLib_MakeFace(in gp_Cylinder C)
    {
        var GC = new Geom_CylindricalSurface(C);
        Init(GC, true, Precision.Confusion());
    }

    /// <summary>A face on the whole cone.</summary>
    public BRepLib_MakeFace(in gp_Cone C)
    {
        var GC = new Geom_ConicalSurface(C);
        Init(GC, true, Precision.Confusion());
    }

    /// <summary>A face on the whole sphere.</summary>
    public BRepLib_MakeFace(in gp_Sphere S)
    {
        var GS = new Geom_SphericalSurface(S);
        Init(GS, true, Precision.Confusion());
    }

    /// <summary>A face on the whole torus.</summary>
    public BRepLib_MakeFace(in gp_Torus T)
    {
        var GT = new Geom_ToroidalSurface(T);
        Init(GT, true, Precision.Confusion());
    }

    /// <summary>A face on the whole surface; <paramref name="TolDegen"/> decides degenerated boundaries.</summary>
    public BRepLib_MakeFace(Geom_Surface S, double TolDegen)
    {
        Init(S, true, TolDegen);
    }

    public BRepLib_MakeFace(in gp_Pln P, double UMin, double UMax, double VMin, double VMax)
    {
        var GP = new Geom_Plane(P);
        Init(GP, UMin, UMax, VMin, VMax, Precision.Confusion());
    }

    public BRepLib_MakeFace(in gp_Cylinder C, double UMin, double UMax, double VMin, double VMax)
    {
        var GC = new Geom_CylindricalSurface(C);
        Init(GC, UMin, UMax, VMin, VMax, Precision.Confusion());
    }

    public BRepLib_MakeFace(in gp_Cone C, double UMin, double UMax, double VMin, double VMax)
    {
        var GC = new Geom_ConicalSurface(C);
        Init(GC, UMin, UMax, VMin, VMax, Precision.Confusion());
    }

    public BRepLib_MakeFace(in gp_Sphere S, double UMin, double UMax, double VMin, double VMax)
    {
        var GS = new Geom_SphericalSurface(S);
        Init(GS, UMin, UMax, VMin, VMax, Precision.Confusion());
    }

    public BRepLib_MakeFace(in gp_Torus T, double UMin, double UMax, double VMin, double VMax)
    {
        var GT = new Geom_ToroidalSurface(T);
        Init(GT, UMin, UMax, VMin, VMax, Precision.Confusion());
    }

    public BRepLib_MakeFace(Geom_Surface S, double UMin, double UMax, double VMin, double VMax, double TolDegen)
    {
        Init(S, UMin, UMax, VMin, VMax, TolDegen);
    }

    /// <summary>
    /// A face on a surface through the wire (with <paramref name="OnlyPlane"/>, a plane), found by
    /// <see cref="BRepLib_FindSurface"/>; BRepLib_NotPlanar if there is none.
    /// </summary>
    public BRepLib_MakeFace(in TopoDS_Wire W, bool OnlyPlane = false)
    {
        // Find a surface through the wire
        var FS = new BRepLib_FindSurface(W, -1, OnlyPlane, true);
        if (!FS.Found())
        {
            myError = BRepLib_NotPlanar;
            return;
        }

        // build the face and add the wire
        var B = new BRep_Builder();
        myError = BRepLib_FaceDone;

        double tol = Max(1.2 * FS.ToleranceReached(), FS.Tolerance());

        TopoDS_Face aFace = new();
        B.MakeFace(ref aFace, FS.Surface(), FS.Location(), tol);
        myShape = aFace;

        TopoDS_Wire aW = new();
        if (OnlyPlane)
        {
            // get rid of degenerative edges in the input wire
            var aB = new BRep_Builder();
            aB.MakeWire(ref aW);

            TopoDS_Wire aWForw = W;
            bool hasDegenerated = false;
            aWForw.Orientation(TopAbs_FORWARD);
            var anIter = new TopoDS_Iterator(aWForw);
            for (; anIter.More(); anIter.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(anIter.Value());

                if (BRep_Tool.Degenerated(aE))
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
        TopoDS_Wire aWOriginal = new();
        if (hasFrozenSubShape(aW))
        {
            aWOriginal = aW;
            aW = TopoDS.Wire(BRepTools.CopyForWrite(aW, out _));
        }

        Add(aW);
        //
        BRepLib.UpdateTolerances(myShape);
        //
        BRepLib.SameParameter(myShape, tol, true);
        //
        if (!aWOriginal.IsNull())
        {
            aW = TopoDS.Wire(BRepTools.KeepUnchanged(aWOriginal, aW, out BRepTools_History aHistory, true));
            myHistory = aHistory;
            if (!ReferenceEquals(aW.TShape(), myShape.TShape()!.myShapes.First().TShape()))
            {
                // the face holds the copy: it gets the wire with the kept edges in its place
                myShape.TShape()!.myShapes.Clear();
                Add(aW);
            }
        }
        //
        if (BRep_Tool.IsClosed(aW))
        {
            CheckInside();
        }
    }

    // True if the shape or any of its sub-shapes is frozen.
    private static bool hasFrozenSubShape(in TopoDS_Shape theShape)
    {
        if (theShape.TShape()!.IsFrozen())
        {
            return true;
        }
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (anExp.Current().TShape()!.IsFrozen())
            {
                return true;
            }
        }
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            if (anExp.Current().TShape()!.IsFrozen())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>A face on the plane bounded by the wire; reversed if the wire would bound an infinite area.</summary>
    public BRepLib_MakeFace(in gp_Pln P, in TopoDS_Wire W, bool Inside = true)
    {
        var Pl = new Geom_Plane(P);
        Init(Pl, false, Precision.Confusion());
        Add(W);
        if (Inside && BRep_Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(in gp_Cylinder C, in TopoDS_Wire W, bool Inside = true)
    {
        var GC = new Geom_CylindricalSurface(C);
        Init(GC, false, Precision.Confusion());
        Add(W);
        if (Inside && BRep_Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(in gp_Cone C, in TopoDS_Wire W, bool Inside = true)
    {
        var GC = new Geom_ConicalSurface(C);
        Init(GC, false, Precision.Confusion());
        Add(W);
        if (Inside && BRep_Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(in gp_Sphere S, in TopoDS_Wire W, bool Inside = true)
    {
        var GS = new Geom_SphericalSurface(S);
        Init(GS, false, Precision.Confusion());
        Add(W);
        if (Inside && BRep_Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(in gp_Torus T, in TopoDS_Wire W, bool Inside = true)
    {
        var GT = new Geom_ToroidalSurface(T);
        Init(GT, false, Precision.Confusion());
        Add(W);
        if (Inside && BRep_Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    public BRepLib_MakeFace(Geom_Surface S, in TopoDS_Wire W, bool Inside = true)
    {
        Init(S, false, Precision.Confusion());
        Add(W);
        if (Inside && BRep_Tool.IsClosed(W))
        {
            CheckInside();
        }
    }

    /// <summary>A copy of the face with the wire added.</summary>
    public BRepLib_MakeFace(in TopoDS_Face F, in TopoDS_Wire W)
    {
        Init(F);
        Add(W);
    }

    /// <summary>Loads the face: a new face on its surface holding the same wires.</summary>
    public void Init(in TopoDS_Face F)
    {
        // copy the face
        myShape = F.EmptyCopied();
        myError = BRepLib_FaceDone;

        var B = new BRep_Builder();
        var It = new TopoDS_Iterator(F);
        while (It.More())
        {
            B.Add(myShape, It.Value());
            It.Next();
        }
    }

    /// <summary>A face on the surface; with <paramref name="Bound"/>, bounded by its natural bounds.</summary>
    public void Init(Geom_Surface S, bool Bound, double TolDegen)
    {
        myError = BRepLib_FaceDone;
        if (Bound)
        {
            S.Bounds(out double UMin, out double UMax, out double VMin, out double VMax);
            Init(S, UMin, UMax, VMin, VMax, TolDegen);
        }
        else
        {
            var B0 = new BRep_Builder();
            TopoDS_Face aFace = new();
            B0.MakeFace(ref aFace, S, Precision.Confusion());
            myShape = aFace;
        }
        var B = new BRep_Builder();
        B.NaturalRestriction(TopoDS.Face(myShape), true);
    }

    /// <summary>
    /// True if the curve is degenerated within <paramref name="theMaxTol"/> (a circle of smaller radius, a B-spline
    /// or Bezier curve whose poles are that close to the first); <paramref name="theActTol"/> is the tolerance that
    /// decided it. For BRepLib_MakeFace and BRepLib_MakeShell.
    /// </summary>
    public static bool IsDegenerated(Geom_Curve theCurve, double theMaxTol, out double theActTol)
    {
        var AC = new GeomAdaptor_Curve(theCurve);
        double aConfusion = Precision.Confusion();
        theActTol = aConfusion;
        GeomAbs_CurveType Type = AC.GetType();

        if (Type == GeomAbs_Circle)
        {
            gp_Circ Circ = AC.Circle();
            if (Circ.Radius() > theMaxTol)
            {
                return false;
            }
            theActTol = Max(Circ.Radius(), aConfusion);
            return true;
        }
        else if (Type == GeomAbs_BSplineCurve)
        {
            Geom_BSplineCurve BS = AC.BSpline();
            int NbPoles = BS.NbPoles();
            double aMaxPoleDist2 = 0.0, aMaxTol2 = theMaxTol * theMaxTol;
            gp_Pnt P1, P2;
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
        else if (Type == GeomAbs_BezierCurve)
        {
            Geom_BezierCurve BZ = AC.Bezier();
            int NbPoles = BZ.NbPoles();
            double aMaxPoleDist2 = 0.0, aMaxTol2 = theMaxTol * theMaxTol;
            gp_Pnt P1, P2;
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
    public void Init(Geom_Surface SS, double Um, double UM, double Vm, double VM, double TolDegen)
    {
        myError = BRepLib_FaceDone;

        double UMin = Um;
        double UMax = UM;
        double VMin = Vm;
        double VMax = VM;

        double T;

        Geom_Surface S = SS, BS = SS;
        if (S is Geom_RectangularTrimmedSurface RS)
        {
            BS = RS.BasisSurface();
        }

        bool OffsetSurface = BS.GetType() == typeof(Geom_OffsetSurface);

        // adjust periodical surface or reordonate
        // check if the values are in the natural range
        double epsilon = Precision.PConfusion();

        BS.Bounds(out double umin, out double umax, out double vmin, out double vmax);

        if (OffsetSurface)
        {
            var OS = (Geom_OffsetSurface)BS;
            Geom_Surface Base = OS.BasisSurface();

            if (Base.GetType() == typeof(Geom_SurfaceOfLinearExtrusion))
            {
                if (Precision.IsInfinite(umin) || Precision.IsInfinite(umax))
                {
                    S = new Geom_RectangularTrimmedSurface(OS, UMin, UMax, VMin, VMax);
                }
                else
                {
                    S = new Geom_RectangularTrimmedSurface(OS, VMin, VMax, false);
                }
            }
            else if (Base.GetType() == typeof(Geom_SurfaceOfRevolution))
            {
                if (Precision.IsInfinite(vmin) || Precision.IsInfinite(vmax))
                {
                    S = new Geom_RectangularTrimmedSurface(OS, VMin, VMax, false);
                }
            }
        }

        // OCCT checks the natural range only when the parameters come reversed (occt-issues.md): ordered parameters
        // outside it are accepted.
        if (S.IsUPeriodic())
        {
            ElCLib.AdjustPeriodic(umin, umax, epsilon, ref UMin, ref UMax);
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

        if (S.IsVPeriodic())
        {
            ElCLib.AdjustPeriodic(vmin, vmax, epsilon, ref VMin, ref VMax);
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
        bool umininf = Precision.IsNegativeInfinite(UMin);
        bool umaxinf = Precision.IsPositiveInfinite(UMax);
        bool vmininf = Precision.IsNegativeInfinite(VMin);
        bool vmaxinf = Precision.IsPositiveInfinite(VMax);

        // closed flag
        bool uclosed = S.IsUClosed() && Math.Abs(UMin - umin) < epsilon && Math.Abs(UMax - umax) < epsilon;

        bool vclosed = S.IsVClosed() && Math.Abs(VMin - vmin) < epsilon && Math.Abs(VMax - vmax) < epsilon;

        // compute 3d curves and degenerate flag
        double maxTol = TolDegen;
        Geom_Curve? Cumin = null, Cumax = null, Cvmin = null, Cvmax = null;
        bool Dumin, Dumax, Dvmin, Dvmax;
        Dumin = Dumax = Dvmin = Dvmax = false;
        double uminTol = Precision.Confusion(), umaxTol = Precision.Confusion(),
               vminTol = Precision.Confusion(), vmaxTol = Precision.Confusion();

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
        var B = new BRep_Builder();

        TopoDS_Vertex V00 = new(), V10 = new(), V11 = new(), V01 = new();

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
        Geom2d_Line? Lumin = null, Lumax = null, Lvmin = null, Lvmax = null;
        if (!umininf)
        {
            Lumin = new Geom2d_Line(new gp_Pnt2d(UMin, 0), new gp_Dir2d(gp_Dir2d.D.Y));
        }
        if (!umaxinf)
        {
            Lumax = new Geom2d_Line(new gp_Pnt2d(UMax, 0), new gp_Dir2d(gp_Dir2d.D.Y));
        }
        if (!vmininf)
        {
            Lvmin = new Geom2d_Line(new gp_Pnt2d(0, VMin), new gp_Dir2d(gp_Dir2d.D.X));
        }
        if (!vmaxinf)
        {
            Lvmax = new Geom2d_Line(new gp_Pnt2d(0, VMax), new gp_Dir2d(gp_Dir2d.D.X));
        }

        // make the face
        // OCCT.NET: OCCT remakes the face in myShape; the port makes it in a new variable (as BRepLib_MakeEdge).
        TopoDS_Face F = new();
        B.MakeFace(ref F, S, Precision.Confusion());
        myShape = F;

        // make the edges
        TopoDS_Edge eumin = new(), eumax = new(), evmin = new(), evmax = new();

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
                V00.Orientation(TopAbs_FORWARD);
                B.Add(eumin, V00);
            }
            if (!vmaxinf)
            {
                V01.Orientation(TopAbs_REVERSED);
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
                    V10.Orientation(TopAbs_FORWARD);
                    B.Add(eumax, V10);
                }
                if (!vmaxinf)
                {
                    V11.Orientation(TopAbs_REVERSED);
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
                V00.Orientation(TopAbs_FORWARD);
                B.Add(evmin, V00);
            }
            if (!umaxinf)
            {
                V10.Orientation(TopAbs_REVERSED);
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
                    V01.Orientation(TopAbs_FORWARD);
                    B.Add(evmax, V01);
                }
                if (!umaxinf)
                {
                    V11.Orientation(TopAbs_REVERSED);
                    B.Add(evmax, V11);
                }
                B.Range(evmax, UMin, UMax);
            }
        }

        // make the wires and add them to the face
        eumin.Orientation(TopAbs_REVERSED);
        evmax.Orientation(TopAbs_REVERSED);

        TopoDS_Wire W = new();

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
            BRepLib.SameParameter(F, Precision.Confusion(), true);
        }

        Done();
    }

    /// <summary>Adds the wire to the face (a hole, or another boundary); the face is no longer a natural restriction.</summary>
    public void Add(in TopoDS_Wire W)
    {
        UnfreezeResult(); // OCCT.NET: frozen
        var B = new BRep_Builder();
        B.Add(myShape, W);
        B.NaturalRestriction(TopoDS.Face(myShape), false);
        Done();
    }

    /// <summary>The face (frozen). As in OCCT, it isn't checked that the face is done.</summary>
    public TopoDS_Face Face()
    {
        TopoDS_Face aFace = TopoDS.Face(myShape);
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal TopoDS_Face InternalFace()
    {
        return TopoDS.Face(myShape);
    }

    /// <summary>The face (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Face(BRepLib_MakeFace theMaker) => theMaker.Face();

    /// <summary>Returns the construction status.</summary>
    public BRepLib_FaceError Error()
    {
        return myError;
    }

    /// <summary>
    /// OCCT.NET: the history of the copy for write of the wire that the face from a wire was built on, when the
    /// wire held frozen sub-shapes: each of its edges and vertices modified into its copy. Null if no copy was made.
    /// </summary>
    internal BRepTools_History? InternalHistory() => myHistory;

    // Reverses the current face if not a bounded area
    private void CheckInside()
    {
        // compute the area and return the face if the area is negative
        TopoDS_Face F = TopoDS.Face(myShape);
        var FClass = new BRepTopAdaptor_FClass2d(F, 0.0);
        if (FClass.PerformInfinitePoint() == TopAbs_IN)
        {
            var B = new BRep_Builder();
            TopoDS_Shape S = myShape.EmptyCopied();
            var it = new TopoDS_Iterator(myShape);
            while (it.More())
            {
                B.Add(S, it.Value().Reversed());
                it.Next();
            }
            myShape = S;
        }
    }
}
