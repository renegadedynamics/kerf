// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeShell.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Library.LibraryShellError;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Makes a shell from a surface: one face per patch between its C2 discontinuities (each face on a copy of the
/// surface, segmented with <c>Segment</c> for a B-spline surface), sewn by shared edges, with 3D curves built and
/// regularities encoded.
/// </summary>
/// <remarks><see cref="Shell"/> freezes the shell (design 4.2); ported code reads <see cref="InternalShell"/>.</remarks>
public class LibraryMakeShell : LibraryMakeShape
{
    private LibraryShellError _error;

    /// <summary>Not done.</summary>
    public LibraryMakeShell()
    {
        _error = LibraryShellError.EmptyShell;
    }

    /// <summary>A shell on the whole surface.</summary>
    public LibraryMakeShell(Surface s, bool segment = false)
    {
        s.ParameterBounds(out double uMin, out double uMax, out double vMin, out double vMax);
        Init(s, uMin, uMax, vMin, vMax, segment);
    }

    /// <summary>A shell on the surface between the parameter values.</summary>
    public LibraryMakeShell(Surface s, double uMin, double uMax, double vMin, double vMax, bool segment = false)
    {
        Init(s, uMin, uMax, vMin, vMax, segment);
    }

    /// <summary>Makes the shell on the surface between the parameter values.</summary>
    public void Init(Surface s, double uMin, double uMax, double vMin, double vMax, bool segment = false)
    {
        Surface bs = s;
        if (s.GetType() == typeof(TrimmedSurface))
        {
            var rts = (TrimmedSurface)s;
            bs = rts.BasisSurface;
        }
        _error = LibraryShellError.EmptyShell;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        // Make a shell from a surface
        var gs = new SurfaceAdaptor(bs, uMin, uMax, vMin, vMax);

        int nu = gs.NbUIntervals(C2);
        int nv = gs.NbVIntervals(C2);

        bool uperiodic = gs.IsUPeriodic;
        bool vperiodic = gs.IsVPeriodic;

        if (nu == 0 || nv == 0)
        {
            return;
        }

        // arrays of parameters and pcurves
        var upars = new NCollection_Array1<double>(1, nu + 1);
        var vpars = new NCollection_Array1<double>(1, nv + 1);
        var uisos = new NCollection_Array1<Curve2D?>(1, nu + 1);
        var visos = new NCollection_Array1<Curve2D?>(1, nv + 1);

        int iu, iv;

        gs.UIntervals(upars, C2);
        var dv = new Direction2D(Direction2D.D.Y);
        for (iu = 1; iu <= nu + 1; iu++)
        {
            double u = upars[iu];
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u))
            {
                uisos[iu] = new LineCurve2D(new Point2D(u, 0.0), dv);
            }
        }

        gs.VIntervals(vpars, C2);
        var du = new Direction2D(Direction2D.D.X);
        for (iv = 1; iv <= nv + 1; iv++)
        {
            double v = vpars[iv];
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v))
            {
                visos[iv] = new LineCurve2D(new Point2D(0.0, v), du);
            }
        }

        // create row by row

        // create the shell
        var b = new BrepBuilder();
        // OCCT.NET: a new shell in a new variable (OCCT remakes the one in myShape).
        Shell shell = new();
        b.MakeShell(ref shell);
        _shape = shell;

        // arrays of edges and vertices for each row
        var botedges = new NCollection_Array1<Shape>(1, nu);
        var botvertices = new NCollection_Array1<Shape>(1, nu + 1);

        // copies of the first ones for periodic case
        var fbotedges = new NCollection_Array1<Shape>(1, nu);
        var fbotvertices = new NCollection_Array1<Shape>(1, nu + 1);

        Face f = new();
        Wire w = new();
        Edge eleft = new(), eright = new(), etop = new(), ebot = new(), feleft = new();
        Vertex vlb = new(), vlt = new(), vrb = new(), vrt = new(), fvlt = new();

        // init the botedges and botvertices
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vpars[1]))
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[1]))
            {
                b.MakeVertex(ref vrt, s.Value(upars[1], vpars[1]), tol);
            }
            fbotvertices[1] = botvertices[1] = vrt;

            for (iu = 1; iu <= nu; iu++)
            {
                vlt = vrt;

                if (uperiodic && iu == nu)
                {
                    vrt = ShapeCasts.Vertex(botvertices[1]);
                }
                else if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[iu + 1]))
                {
                    b.MakeVertex(ref vrt, s.Value(upars[iu + 1], vpars[1]), tol);
                }

                fbotvertices[iu + 1] = botvertices[iu + 1] = vrt;
                b.MakeEdge(ref etop);
                if (!vlt.IsNull)
                {
                    vlt.Orient(Forward);
                    b.Add(etop, vlt);
                }
                if (!vrt.IsNull)
                {
                    vrt.Orient(Reversed);
                    b.Add(etop, vrt);
                }
                fbotedges[iu] = botedges[iu] = etop;
            }
        }

        for (iv = 1; iv <= nv; iv++)
        {
            // compute the first edge and vertices of the line
            vrb = ShapeCasts.Vertex(botvertices[1]);

            if (vperiodic && iv == nv)
            {
                vrt = ShapeCasts.Vertex(fbotvertices[1]);
            }
            else
            {
                vrt.Nullify();
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vpars[iv + 1]))
                {
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[1]))
                    {
                        b.MakeVertex(ref vrt, s.Value(upars[1], vpars[iv + 1]), tol);
                    }
                }
            }

            eright.Nullify();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[1]))
            {
                b.MakeEdge(ref eright);
                if (!vrb.IsNull)
                {
                    vrb.Orient(Forward);
                    b.Add(eright, vrb);
                }
                if (!vrt.IsNull)
                {
                    vrt.Orient(Reversed);
                    b.Add(eright, vrt);
                }
            }

            fvlt = vrt;
            feleft = eright;

            // make the row of faces

            for (iu = 1; iu <= nu; iu++)
            {
                // create the face at iu, iv

                // the surface
                var ss = (Surface)bs.Copy();
                if (gs.GetType() == SurfaceKind.BSplineSurface && segment)
                {
                    ((global::Kerf.Geometry.BSplineSurface)ss).Segment(upars[iu], upars[iu + 1], vpars[iv], vpars[iv + 1]);
                }
                b.MakeFace(ref f, ss, tol);

                // the wire

                b.MakeWire(ref w);

                // the vertices

                vlb = vrb;
                vrb = ShapeCasts.Vertex(botvertices[iu + 1]);
                vlt = vrt;

                if (uperiodic && iu == nu)
                {
                    vrt = fvlt;
                }
                else
                {
                    vrt.Nullify();
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vpars[iv + 1]))
                    {
                        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[iu + 1]))
                        {
                            b.MakeVertex(ref vrt, s.Value(upars[iu + 1], vpars[iv + 1]), tol);
                        }
                    }
                }

                botvertices[iu] = vlt;
                botvertices[iu + 1] = vrt;

                // the edges

                eleft = eright;

                if (uperiodic && iu == nu)
                {
                    eright = feleft;
                }
                else
                {
                    eright.Nullify();
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[iu + 1]))
                    {
                        b.MakeEdge(ref eright);
                        if (!vrb.IsNull)
                        {
                            vrb.Orient(Forward);
                            b.Add(eright, vrb);
                        }
                        if (!vrt.IsNull)
                        {
                            vrt.Orient(Reversed);
                            b.Add(eright, vrt);
                        }
                    }
                }

                if (uperiodic && nu == 1)
                {
                    if (!eleft.IsNull && !eright.IsNull)
                    {
                        b.UpdateEdge(eleft, uisos[2], uisos[1], f, tol);
                        b.Range(eleft, f, vpars[iv], vpars[iv + 1]);
                    }
                }
                else
                {
                    if (!eleft.IsNull)
                    {
                        b.UpdateEdge(eleft, uisos[iu], f, tol);
                        b.Range(eleft, f, vpars[iv], vpars[iv + 1]);
                    }
                    if (!eright.IsNull)
                    {
                        b.UpdateEdge(eright, uisos[iu + 1], f, tol);
                        b.Range(eright, f, vpars[iv], vpars[iv + 1]);
                    }
                }

                ebot = ShapeCasts.Edge(botedges[iu]);

                if (vperiodic && iv == nv)
                {
                    etop = ShapeCasts.Edge(fbotedges[iu]);
                }
                else
                {
                    etop.Nullify();
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vpars[iv + 1]))
                    {
                        b.MakeEdge(ref etop);
                        if (!vlt.IsNull)
                        {
                            vlt.Orient(Forward);
                            b.Add(etop, vlt);
                        }
                        if (!vrt.IsNull)
                        {
                            vrt.Orient(Reversed);
                            b.Add(etop, vrt);
                        }
                    }
                }

                if (vperiodic && nv == 1)
                {
                    if (!ebot.IsNull && !etop.IsNull)
                    {
                        b.UpdateEdge(ebot, visos[1], visos[2], f, tol);
                        b.Range(ebot, f, vpars[iv], vpars[iv + 1]);
                    }
                }
                else
                {
                    if (!ebot.IsNull)
                    {
                        b.UpdateEdge(ebot, visos[iv], f, tol);
                        b.Range(ebot, f, upars[iu], upars[iu + 1]);
                    }
                    if (!etop.IsNull)
                    {
                        b.UpdateEdge(etop, visos[iv + 1], f, tol);
                        b.Range(etop, f, upars[iu], upars[iu + 1]);
                    }
                }

                botedges[iu] = etop;

                if (!eleft.IsNull)
                {
                    eleft.Orient(Reversed);
                    b.Add(w, eleft);
                }
                if (!ebot.IsNull)
                {
                    ebot.Orient(Forward);
                    b.Add(w, ebot);
                }
                if (!eright.IsNull)
                {
                    eright.Orient(Forward);
                    b.Add(w, eright);
                }
                if (!etop.IsNull)
                {
                    etop.Orient(Reversed);
                    b.Add(w, etop);
                }

                b.Add(f, w);
                b.Add(_shape, f);
            }
        }

        // codage des courbes 3d et regularites.
        BrepLibrary.BuildCurves3d(_shape, tol);
        BrepLibrary.EncodeRegularity(_shape);
        _shape.Closed(Tool.IsClosed(_shape));

        // Additional checking for degenerated edges
        bool isDegenerated;
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        var exp = new Explorer(_shape, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            // OCCT adapts a null curve when BuildCurves3d failed on the edge (and throws); so does the port.
            Curve3D? curve = Tool.Curve(edge, out _, out _);
            isDegenerated = LibraryMakeFace.IsDegenerated(curve!, aTol, out _);
            b.Degenerated(edge, isDegenerated);
        }

        _error = LibraryShellError.ShellDone;
        Done();
    }

    /// <summary>Returns the construction status.</summary>
    public LibraryShellError Error()
    {
        return _error;
    }

    /// <summary>The shell (frozen). As in OCCT, it isn't checked that the shell is done.</summary>
    public Shell Shell()
    {
        Shell shell = ShapeCasts.Shell(_shape);
        shell.TShape()?.FreezeGraph();
        return shell;
    }

    /// <summary><see cref="Shell"/> without freezing, for ported code.</summary>
    internal Shell InternalShell()
    {
        return ShapeCasts.Shell(_shape);
    }

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shell(LibraryMakeShell maker) => maker.Shell();
}
