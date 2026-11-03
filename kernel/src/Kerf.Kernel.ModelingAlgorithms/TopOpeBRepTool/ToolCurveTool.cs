// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_CurveTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_CurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>
/// The curve tool of the legacy Booleans: computes the 3D curve and pcurves of an intersection line with the settings
/// of its <see cref="ToolGeomTool"/>, and projects curves on faces.
/// </summary>
/// <remarks>
/// <para>
/// Only what the face maker of the 2D offset reaches is ported (plan 2.11, decision 585): the constructors, the geomtool
/// accessors, <see cref="MakePCurveOnFace"/> and the file's <c>MakePCurve</c>. Left out: <c>MakeCurves</c>,
/// <c>MakeBSpline1fromPnt2d</c>, <c>MakeBSpline1fromWALKING3d</c>, <c>MakeBSpline1fromWALKING2d</c>, <c>IsProjectable</c>,
/// <c>ConvertProjector</c> (the Booleans' intersection lines).
/// </para>
/// <para>A working object (it holds its geomtool by value, as in OCCT).</para>
/// </remarks>
public sealed class ToolCurveTool
{
    private ToolGeomTool _geomTool = new();

    /// <summary>A curve tool with OCCT's default geomtool.</summary>
    public ToolCurveTool()
    {
    }

    /// <summary>A curve tool computing 3D curves of kind <paramref name="o"/>.</summary>
    public ToolCurveTool(OutCurveType o)
    {
        ToolGeomTool gt = new(o);
        SetGeomTool(gt);
    }

    /// <summary>A curve tool with the settings <paramref name="gt"/>.</summary>
    public ToolCurveTool(in ToolGeomTool gt)
    {
        SetGeomTool(gt);
    }

    /// <summary>OCCT.NET: a copy of <paramref name="other"/> (OCCT copies the class by value).</summary>
    internal ToolCurveTool(ToolCurveTool other)
    {
        _geomTool = other._geomTool;
    }

    internal ref ToolGeomTool ChangeGeomTool() => ref _geomTool;

    /// <summary>The settings.</summary>
    public ToolGeomTool GetGeomTool() => _geomTool;

    internal void SetGeomTool(in ToolGeomTool gt)
    {
        _geomTool.Define(gt);
    }

    // MakeCurves, MakeBSpline1fromPnt2d, MakeBSpline1fromWALKING3d, MakeBSpline1fromWALKING2d, IsProjectable and
    // ConvertProjector: not ported (the Booleans' intersection lines; plan 2.11).

    /// <summary>The pcurve of a projected curve, as a Geom2d curve of its type (OCCT's file-level <c>MakePCurve</c>).</summary>
    /// <exception cref="MethodNotImplementedException">The projection is of another type.</exception>
    internal static Curve2D MakePCurve(ProjectedCurve pc)
    {
        Curve2D c2D;
        switch (pc.GetType())
        {
            case Line:
                c2D = new LineCurve2D(pc.Line());
                break;
            case Circle:
                c2D = new CircleCurve2D(pc.Circle());
                break;
            case Ellipse:
                c2D = new EllipseCurve2D(pc.Ellipse());
                break;
            case Parabola:
                c2D = new ParabolaCurve2D(pc.Parabola());
                break;
            case Hyperbola:
                c2D = new HyperbolaCurve2D(pc.Hyperbola());
                break;
            case BSplineCurve:
                c2D = pc.BSpline();
                break;
            default:
                throw new MethodNotImplementedException("CurveTool::MakePCurve");
        }
        return c2D;
    }

    /// <summary>
    /// The projection of <paramref name="c3D"/> (trimmed to [<paramref name="first"/>, <paramref name="last"/>] when
    /// first &lt; last) on the face <paramref name="s"/>, shifted into the face's bounds on a periodic surface, and the
    /// tolerance reached.
    /// </summary>
    public static Curve2D MakePCurveOnFace(in Shape s,
                                                Curve3D c3D,
                                                out double tolReached2d,
                                                double first = 0.0,
                                                double last = 0.0)
    {
        bool trim = false;
        if (first < last)
        {
            trim = true;
        }

        Face F = ShapeCasts.Face(s);
        bool compminmaxUV = false;
        FaceSurfaceAdaptor bas = new(F, compminmaxUV);
        CurveAdaptor gac = new();
        if (trim)
        {
            gac.Load(c3D, first, last);
        }
        else
        {
            gac.Load(c3D);
        }

        // OCCT copies the two adaptors into handles; neither is changed afterwards
        FaceSurfaceAdaptor bahs = bas;
        CurveAdaptor bahc = gac;
        ProjectedCurve projcurv = new(bahs, bahc);
        Curve2D c2D = MakePCurve(projcurv);
        tolReached2d = projcurv.GetTolerance();

        BrepTools.UVBounds(F, out double uMin, out double uMax, out double vMin, out double vMax);

        double f = gac.FirstParameter();
        double l = gac.LastParameter();
        double t = (f + l) * .5;
        Point2D pC2D = c2D.Value(t);
        double u2 = pC2D.X;
        double v2 = pC2D.Y;

        if (bas.GetType() == SurfaceKind.Sphere)
        {
            // MSV: consider quasiperiodic shift of pcurve
            double vFirst = bas.FirstVParameter();
            double vLast = bas.LastVParameter();
            bool mincond = v2 < vFirst;
            bool maxcond = v2 > vLast;
            if (mincond || maxcond)
            {
                Curve2D pct = (Curve2D)c2D.Copy();
                // make mirror relative to the isoline of apex -PI/2 or PI/2
                Transform2D trsf = new();
                Point2D po = new(0, -M_PI / 2);
                if (maxcond)
                {
                    po = po.WithY(M_PI / 2);
                }
                trsf = trsf.WithMirror(new Axis2D(po, new Direction2D(Direction2D.D.X)));
                pct.Transform(trsf);
                // add translation along U direction on PI
                Vector2D vec = new(M_PI, 0);
                double uFirst = bas.FirstUParameter();
                if (u2 - uFirst - M_PI > -1e-7)
                {
                    vec = vec.Reversed();
                }
                pct.Translate(vec);
                c2D = pct;
                // recompute the test point
                pC2D = c2D.Value(t);
                u2 = pC2D.X;
                v2 = pC2D.Y;
            }
        }

        double du = 0.0;
        if (bahs.IsUPeriodic)
        {
            // modified by NIZHNY-MZV  Thu Mar 30 10:03:15 2000
            bool mincond = uMin - u2 > 1e-7;
            bool maxcond = u2 - uMax > 1e-7;
            bool decalu = mincond || maxcond;
            if (decalu)
            {
                du = mincond ? bahs.UPeriod() : -bahs.UPeriod();
            }
        }
        double dv = 0.0;
        if (bahs.IsVPeriodic)
        {
            // modified by NIZHNY-MZV  Thu Mar 30 10:06:24 2000
            bool mincond = vMin - v2 > 1e-7;
            bool maxcond = v2 - vMax > 1e-7;
            bool decalv = mincond || maxcond;
            if (decalv)
            {
                dv = mincond ? bahs.VPeriod() : -bahs.VPeriod();
            }
        }

        if (du != 0.0 || dv != 0.0)
        {
            Curve2D pct = (Curve2D)c2D.Copy();
            pct.Translate(new Vector2D(du, dv));
            c2D = pct;
        }

        return c2D;
    }
}
