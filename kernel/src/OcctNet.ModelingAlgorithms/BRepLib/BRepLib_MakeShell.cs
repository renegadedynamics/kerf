// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeShell.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.BRepLib_ShellError;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Makes a shell from a surface: one face per patch between its C2 discontinuities (each face on a copy of the
/// surface, segmented with <c>Segment</c> for a B-spline surface), sewn by shared edges, with 3D curves built and
/// regularities encoded.
/// </summary>
/// <remarks><see cref="Shell"/> freezes the shell (design 4.2); ported code reads <see cref="InternalShell"/>.</remarks>
public class BRepLib_MakeShell : BRepLib_MakeShape
{
    private BRepLib_ShellError myError;

    /// <summary>Not done.</summary>
    public BRepLib_MakeShell()
    {
        myError = BRepLib_EmptyShell;
    }

    /// <summary>A shell on the whole surface.</summary>
    public BRepLib_MakeShell(Geom_Surface S, bool Segment = false)
    {
        S.Bounds(out double UMin, out double UMax, out double VMin, out double VMax);
        Init(S, UMin, UMax, VMin, VMax, Segment);
    }

    /// <summary>A shell on the surface between the parameter values.</summary>
    public BRepLib_MakeShell(Geom_Surface S, double UMin, double UMax, double VMin, double VMax, bool Segment = false)
    {
        Init(S, UMin, UMax, VMin, VMax, Segment);
    }

    /// <summary>Makes the shell on the surface between the parameter values.</summary>
    public void Init(Geom_Surface S, double UMin, double UMax, double VMin, double VMax, bool Segment = false)
    {
        Geom_Surface BS = S;
        if (S.GetType() == typeof(Geom_RectangularTrimmedSurface))
        {
            var RTS = (Geom_RectangularTrimmedSurface)S;
            BS = RTS.BasisSurface();
        }
        myError = BRepLib_EmptyShell;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        // Make a shell from a surface
        var GS = new GeomAdaptor_Surface(BS, UMin, UMax, VMin, VMax);

        int nu = GS.NbUIntervals(C2);
        int nv = GS.NbVIntervals(C2);

        bool uperiodic = GS.IsUPeriodic();
        bool vperiodic = GS.IsVPeriodic();

        if (nu == 0 || nv == 0)
        {
            return;
        }

        // arrays of parameters and pcurves
        var upars = new NCollection_Array1<double>(1, nu + 1);
        var vpars = new NCollection_Array1<double>(1, nv + 1);
        var uisos = new NCollection_Array1<Geom2d_Curve?>(1, nu + 1);
        var visos = new NCollection_Array1<Geom2d_Curve?>(1, nv + 1);

        int iu, iv;

        GS.UIntervals(upars, C2);
        var dv = new Direction2D(Direction2D.D.Y);
        for (iu = 1; iu <= nu + 1; iu++)
        {
            double u = upars[iu];
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u))
            {
                uisos[iu] = new Geom2d_Line(new Point2D(u, 0.0), dv);
            }
        }

        GS.VIntervals(vpars, C2);
        var du = new Direction2D(Direction2D.D.X);
        for (iv = 1; iv <= nv + 1; iv++)
        {
            double v = vpars[iv];
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v))
            {
                visos[iv] = new Geom2d_Line(new Point2D(0.0, v), du);
            }
        }

        // create row by row

        // create the shell
        var B = new BRep_Builder();
        // OCCT.NET: a new shell in a new variable (OCCT remakes the one in myShape).
        TopoDS_Shell aShell = new();
        B.MakeShell(ref aShell);
        myShape = aShell;

        // arrays of edges and vertices for each row
        var botedges = new NCollection_Array1<TopoDS_Shape>(1, nu);
        var botvertices = new NCollection_Array1<TopoDS_Shape>(1, nu + 1);

        // copies of the first ones for periodic case
        var fbotedges = new NCollection_Array1<TopoDS_Shape>(1, nu);
        var fbotvertices = new NCollection_Array1<TopoDS_Shape>(1, nu + 1);

        TopoDS_Face F = new();
        TopoDS_Wire W = new();
        TopoDS_Edge eleft = new(), eright = new(), etop = new(), ebot = new(), feleft = new();
        TopoDS_Vertex vlb = new(), vlt = new(), vrb = new(), vrt = new(), fvlt = new();

        // init the botedges and botvertices
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vpars[1]))
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[1]))
            {
                B.MakeVertex(ref vrt, S.Value(upars[1], vpars[1]), tol);
            }
            fbotvertices[1] = botvertices[1] = vrt;

            for (iu = 1; iu <= nu; iu++)
            {
                vlt = vrt;

                if (uperiodic && iu == nu)
                {
                    vrt = TopoDS.Vertex(botvertices[1]);
                }
                else if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[iu + 1]))
                {
                    B.MakeVertex(ref vrt, S.Value(upars[iu + 1], vpars[1]), tol);
                }

                fbotvertices[iu + 1] = botvertices[iu + 1] = vrt;
                B.MakeEdge(ref etop);
                if (!vlt.IsNull())
                {
                    vlt.Orientation(TopAbs_FORWARD);
                    B.Add(etop, vlt);
                }
                if (!vrt.IsNull())
                {
                    vrt.Orientation(TopAbs_REVERSED);
                    B.Add(etop, vrt);
                }
                fbotedges[iu] = botedges[iu] = etop;
            }
        }

        for (iv = 1; iv <= nv; iv++)
        {
            // compute the first edge and vertices of the line
            vrb = TopoDS.Vertex(botvertices[1]);

            if (vperiodic && iv == nv)
            {
                vrt = TopoDS.Vertex(fbotvertices[1]);
            }
            else
            {
                vrt.Nullify();
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vpars[iv + 1]))
                {
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[1]))
                    {
                        B.MakeVertex(ref vrt, S.Value(upars[1], vpars[iv + 1]), tol);
                    }
                }
            }

            eright.Nullify();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(upars[1]))
            {
                B.MakeEdge(ref eright);
                if (!vrb.IsNull())
                {
                    vrb.Orientation(TopAbs_FORWARD);
                    B.Add(eright, vrb);
                }
                if (!vrt.IsNull())
                {
                    vrt.Orientation(TopAbs_REVERSED);
                    B.Add(eright, vrt);
                }
            }

            fvlt = vrt;
            feleft = eright;

            // make the row of faces

            for (iu = 1; iu <= nu; iu++)
            {
                // create the face at iu, iv

                // the surface
                var SS = (Geom_Surface)BS.Copy();
                if (GS.GetType() == BSplineSurface && Segment)
                {
                    ((Geom_BSplineSurface)SS).Segment(upars[iu], upars[iu + 1], vpars[iv], vpars[iv + 1]);
                }
                B.MakeFace(ref F, SS, tol);

                // the wire

                B.MakeWire(ref W);

                // the vertices

                vlb = vrb;
                vrb = TopoDS.Vertex(botvertices[iu + 1]);
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
                            B.MakeVertex(ref vrt, S.Value(upars[iu + 1], vpars[iv + 1]), tol);
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
                        B.MakeEdge(ref eright);
                        if (!vrb.IsNull())
                        {
                            vrb.Orientation(TopAbs_FORWARD);
                            B.Add(eright, vrb);
                        }
                        if (!vrt.IsNull())
                        {
                            vrt.Orientation(TopAbs_REVERSED);
                            B.Add(eright, vrt);
                        }
                    }
                }

                if (uperiodic && nu == 1)
                {
                    if (!eleft.IsNull() && !eright.IsNull())
                    {
                        B.UpdateEdge(eleft, uisos[2], uisos[1], F, tol);
                        B.Range(eleft, F, vpars[iv], vpars[iv + 1]);
                    }
                }
                else
                {
                    if (!eleft.IsNull())
                    {
                        B.UpdateEdge(eleft, uisos[iu], F, tol);
                        B.Range(eleft, F, vpars[iv], vpars[iv + 1]);
                    }
                    if (!eright.IsNull())
                    {
                        B.UpdateEdge(eright, uisos[iu + 1], F, tol);
                        B.Range(eright, F, vpars[iv], vpars[iv + 1]);
                    }
                }

                ebot = TopoDS.Edge(botedges[iu]);

                if (vperiodic && iv == nv)
                {
                    etop = TopoDS.Edge(fbotedges[iu]);
                }
                else
                {
                    etop.Nullify();
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vpars[iv + 1]))
                    {
                        B.MakeEdge(ref etop);
                        if (!vlt.IsNull())
                        {
                            vlt.Orientation(TopAbs_FORWARD);
                            B.Add(etop, vlt);
                        }
                        if (!vrt.IsNull())
                        {
                            vrt.Orientation(TopAbs_REVERSED);
                            B.Add(etop, vrt);
                        }
                    }
                }

                if (vperiodic && nv == 1)
                {
                    if (!ebot.IsNull() && !etop.IsNull())
                    {
                        B.UpdateEdge(ebot, visos[1], visos[2], F, tol);
                        B.Range(ebot, F, vpars[iv], vpars[iv + 1]);
                    }
                }
                else
                {
                    if (!ebot.IsNull())
                    {
                        B.UpdateEdge(ebot, visos[iv], F, tol);
                        B.Range(ebot, F, upars[iu], upars[iu + 1]);
                    }
                    if (!etop.IsNull())
                    {
                        B.UpdateEdge(etop, visos[iv + 1], F, tol);
                        B.Range(etop, F, upars[iu], upars[iu + 1]);
                    }
                }

                botedges[iu] = etop;

                if (!eleft.IsNull())
                {
                    eleft.Orientation(TopAbs_REVERSED);
                    B.Add(W, eleft);
                }
                if (!ebot.IsNull())
                {
                    ebot.Orientation(TopAbs_FORWARD);
                    B.Add(W, ebot);
                }
                if (!eright.IsNull())
                {
                    eright.Orientation(TopAbs_FORWARD);
                    B.Add(W, eright);
                }
                if (!etop.IsNull())
                {
                    etop.Orientation(TopAbs_REVERSED);
                    B.Add(W, etop);
                }

                B.Add(F, W);
                B.Add(myShape, F);
            }
        }

        // codage des courbes 3d et regularites.
        BRepLib.BuildCurves3d(myShape, tol);
        BRepLib.EncodeRegularity(myShape);
        myShape.Closed(BRep_Tool.IsClosed(myShape));

        // Additional checking for degenerated edges
        bool isDegenerated;
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        var anExp = new TopExp_Explorer(myShape, TopAbs_EDGE);
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
            // OCCT adapts a null curve when BuildCurves3d failed on the edge (and throws); so does the port.
            Geom_Curve? aCurve = BRep_Tool.Curve(anEdge, out _, out _);
            isDegenerated = BRepLib_MakeFace.IsDegenerated(aCurve!, aTol, out _);
            B.Degenerated(anEdge, isDegenerated);
        }

        myError = BRepLib_ShellDone;
        Done();
    }

    /// <summary>Returns the construction status.</summary>
    public BRepLib_ShellError Error()
    {
        return myError;
    }

    /// <summary>The shell (frozen). As in OCCT, it isn't checked that the shell is done.</summary>
    public TopoDS_Shell Shell()
    {
        TopoDS_Shell aShell = TopoDS.Shell(myShape);
        aShell.TShape()?.FreezeGraph();
        return aShell;
    }

    /// <summary><see cref="Shell"/> without freezing, for ported code.</summary>
    internal TopoDS_Shell InternalShell()
    {
        return TopoDS.Shell(myShape);
    }

    /// <summary>The shell (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Shell(BRepLib_MakeShell theMaker) => theMaker.Shell();
}
