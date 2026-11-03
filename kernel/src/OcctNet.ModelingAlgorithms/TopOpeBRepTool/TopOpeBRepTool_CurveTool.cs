// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_CurveTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_CurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The curve tool of the legacy Booleans: computes the 3D curve and pcurves of an intersection line with the settings
/// of its <see cref="TopOpeBRepTool_GeomTool"/>, and projects curves on faces.
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
public sealed class TopOpeBRepTool_CurveTool
{
    private TopOpeBRepTool_GeomTool myGeomTool = new();

    /// <summary>A curve tool with OCCT's default geomtool.</summary>
    public TopOpeBRepTool_CurveTool()
    {
    }

    /// <summary>A curve tool computing 3D curves of kind <paramref name="O"/>.</summary>
    public TopOpeBRepTool_CurveTool(TopOpeBRepTool_OutCurveType O)
    {
        TopOpeBRepTool_GeomTool GT = new(O);
        SetGeomTool(GT);
    }

    /// <summary>A curve tool with the settings <paramref name="GT"/>.</summary>
    public TopOpeBRepTool_CurveTool(in TopOpeBRepTool_GeomTool GT)
    {
        SetGeomTool(GT);
    }

    /// <summary>OCCT.NET: a copy of <paramref name="theOther"/> (OCCT copies the class by value).</summary>
    internal TopOpeBRepTool_CurveTool(TopOpeBRepTool_CurveTool theOther)
    {
        myGeomTool = theOther.myGeomTool;
    }

    internal ref TopOpeBRepTool_GeomTool ChangeGeomTool() => ref myGeomTool;

    /// <summary>The settings.</summary>
    public TopOpeBRepTool_GeomTool GetGeomTool() => myGeomTool;

    internal void SetGeomTool(in TopOpeBRepTool_GeomTool GT)
    {
        myGeomTool.Define(GT);
    }

    // MakeCurves, MakeBSpline1fromPnt2d, MakeBSpline1fromWALKING3d, MakeBSpline1fromWALKING2d, IsProjectable and
    // ConvertProjector: not ported (the Booleans' intersection lines; plan 2.11).

    /// <summary>The pcurve of a projected curve, as a Geom2d curve of its type (OCCT's file-level <c>MakePCurve</c>).</summary>
    /// <exception cref="MethodNotImplementedException">The projection is of another type.</exception>
    internal static Geom2d_Curve MakePCurve(ProjLib_ProjectedCurve PC)
    {
        Geom2d_Curve C2D;
        switch (PC.GetType())
        {
            case Line:
                C2D = new Geom2d_Line(PC.Line());
                break;
            case Circle:
                C2D = new Geom2d_Circle(PC.Circle());
                break;
            case Ellipse:
                C2D = new Geom2d_Ellipse(PC.Ellipse());
                break;
            case Parabola:
                C2D = new Geom2d_Parabola(PC.Parabola());
                break;
            case Hyperbola:
                C2D = new Geom2d_Hyperbola(PC.Hyperbola());
                break;
            case BSplineCurve:
                C2D = PC.BSpline();
                break;
            default:
                throw new MethodNotImplementedException("CurveTool::MakePCurve");
        }
        return C2D;
    }

    /// <summary>
    /// The projection of <paramref name="C3D"/> (trimmed to [<paramref name="first"/>, <paramref name="last"/>] when
    /// first &lt; last) on the face <paramref name="S"/>, shifted into the face's bounds on a periodic surface, and the
    /// tolerance reached.
    /// </summary>
    public static Geom2d_Curve MakePCurveOnFace(in TopoDS_Shape S,
                                                Geom_Curve C3D,
                                                out double TolReached2d,
                                                double first = 0.0,
                                                double last = 0.0)
    {
        bool trim = false;
        if (first < last)
        {
            trim = true;
        }

        TopoDS_Face F = TopoDS.Face(S);
        bool compminmaxUV = false;
        BRepAdaptor_Surface BAS = new(F, compminmaxUV);
        GeomAdaptor_Curve GAC = new();
        if (trim)
        {
            GAC.Load(C3D, first, last);
        }
        else
        {
            GAC.Load(C3D);
        }

        // OCCT copies the two adaptors into handles; neither is changed afterwards
        BRepAdaptor_Surface BAHS = BAS;
        GeomAdaptor_Curve BAHC = GAC;
        ProjLib_ProjectedCurve projcurv = new(BAHS, BAHC);
        Geom2d_Curve C2D = MakePCurve(projcurv);
        TolReached2d = projcurv.GetTolerance();

        BRepTools.UVBounds(F, out double UMin, out double UMax, out double VMin, out double VMax);

        double f = GAC.FirstParameter();
        double l = GAC.LastParameter();
        double t = (f + l) * .5;
        Point2D pC2D = C2D.Value(t);
        double u2 = pC2D.X();
        double v2 = pC2D.Y();

        if (BAS.GetType() == SurfaceKind.Sphere)
        {
            // MSV: consider quasiperiodic shift of pcurve
            double VFirst = BAS.FirstVParameter();
            double VLast = BAS.LastVParameter();
            bool mincond = v2 < VFirst;
            bool maxcond = v2 > VLast;
            if (mincond || maxcond)
            {
                Geom2d_Curve PCT = (Geom2d_Curve)C2D.Copy();
                // make mirror relative to the isoline of apex -PI/2 or PI/2
                Transform2D aTrsf = new();
                Point2D po = new(0, -M_PI / 2);
                if (maxcond)
                {
                    po.SetY(M_PI / 2);
                }
                aTrsf.SetMirror(new Axis2D(po, new Direction2D(Direction2D.D.X)));
                PCT.Transform(aTrsf);
                // add translation along U direction on PI
                Vector2D vec = new(M_PI, 0);
                double UFirst = BAS.FirstUParameter();
                if (u2 - UFirst - M_PI > -1e-7)
                {
                    vec.Reverse();
                }
                PCT.Translate(vec);
                C2D = PCT;
                // recompute the test point
                pC2D = C2D.Value(t);
                u2 = pC2D.X();
                v2 = pC2D.Y();
            }
        }

        double du = 0.0;
        if (BAHS.IsUPeriodic())
        {
            // modified by NIZHNY-MZV  Thu Mar 30 10:03:15 2000
            bool mincond = UMin - u2 > 1e-7;
            bool maxcond = u2 - UMax > 1e-7;
            bool decalu = mincond || maxcond;
            if (decalu)
            {
                du = mincond ? BAHS.UPeriod() : -BAHS.UPeriod();
            }
        }
        double dv = 0.0;
        if (BAHS.IsVPeriodic())
        {
            // modified by NIZHNY-MZV  Thu Mar 30 10:06:24 2000
            bool mincond = VMin - v2 > 1e-7;
            bool maxcond = v2 - VMax > 1e-7;
            bool decalv = mincond || maxcond;
            if (decalv)
            {
                dv = mincond ? BAHS.VPeriod() : -BAHS.VPeriod();
            }
        }

        if (du != 0.0 || dv != 0.0)
        {
            Geom2d_Curve PCT = (Geom2d_Curve)C2D.Copy();
            PCT.Translate(new Vector2D(du, dv));
            C2D = PCT;
        }

        return C2D;
    }
}
