// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using OcctNet.DataExchange.Tests.StepGen;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.DataExchange.Tests.GeomToStep;

/// <summary>
/// The port's twin of tools/geomsteprec (unit 8D): the same lines, for StepToGeom on a model's geometric entities
/// ("s2g") and for GeomToStep on the constructed cases and on a shape's geometry ("g2s"), with the default factors
/// and with inches and degrees. Results written as STEP entities go through StepAP214_Protocol, as native's.
/// </summary>
internal static class GeomStepRecorder
{
    public static readonly StepData_Factors Millimetres = new();
    public static readonly StepData_Factors Inches = new StepData_Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    /// <summary>StepAP214's protocol (unit 8C2), as native's recorder reads and writes with.</summary>
    internal static readonly Lazy<StepData_Protocol?> TheProtocol = new(() => StepRoundTripTests.Protocol());

    // ---------------------------------------------------------------------------------------------------------
    // Text (as the recorder's ostream at precision 17)

    private static string R(double theValue) => Standard_Stream.FormatReal(theValue, 17);

    private static string OneLine(string theText) => theText.Replace('\n', ' ').Replace('\r', ' ').TrimEnd(' ');

    private static string Pnt(in gp_Pnt P) => R(P.X()) + " " + R(P.Y()) + " " + R(P.Z());
    private static string Pnt(in gp_Pnt2d P) => R(P.X()) + " " + R(P.Y());
    private static string Dir(in gp_Dir D) => R(D.X()) + " " + R(D.Y()) + " " + R(D.Z());
    private static string Dir(in gp_Dir2d D) => R(D.X()) + " " + R(D.Y());

    private static StringWriter Writer()
    {
        var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Standard_Stream.Precision(aWriter, 17);
        return aWriter;
    }

    public static string Curve(Geom_Curve C)
    {
        var aS = Writer();
        GeomTools_CurveSet.PrintCurve(C, aS, true);
        return OneLine(aS.ToString());
    }

    public static string Curve2d(Geom2d_Curve C)
    {
        var aS = Writer();
        GeomTools_Curve2dSet.PrintCurve2d(C, aS, true);
        return OneLine(aS.ToString());
    }

    public static string Surface(Geom_Surface S)
    {
        var aS = Writer();
        GeomTools_SurfaceSet.PrintSurface(S, aS, true);
        return OneLine(aS.ToString());
    }

    private static string Trsf(in gp_Trsf T)
    {
        var aRes = new StringBuilder("trsf");
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 4; c++)
            {
                aRes.Append(' ').Append(R(T.Value(r, c)));
            }
        }

        return aRes.Append(' ').Append(R(T.ScaleFactor())).Append(' ').Append((int)T.Form()).ToString();
    }

    private static string Trsf2d(in gp_Trsf2d T)
    {
        var aRes = new StringBuilder("trsf2d");
        for (int r = 1; r <= 2; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                aRes.Append(' ').Append(R(T.Value(r, c)));
            }
        }

        return aRes.Append(' ').Append(R(T.ScaleFactor())).Append(' ').Append((int)T.Form()).ToString();
    }

    // ---------------------------------------------------------------------------------------------------------
    // StepToGeom on one entity

    private static void Try(List<string> theLines, string thePrefix, string theName, Func<string> theExpr)
    {
        string aRes;
        try
        {
            aRes = theExpr();
        }
        catch (Exception e) when (e is not OutOfMemoryException)
        {
            aRes = "EXC";
        }

        theLines.Add(thePrefix + theName + " " + aRes);
    }

    private static string Null<T>(T? theValue, Func<T, string> theText) where T : class => theValue is null ? "null" : theText(theValue);

    public static void StepToGeomLines(Standard_Transient? theEnt, StepData_Factors F, string thePrefix, List<string> theLines)
    {
        if (theEnt is StepGeom_CartesianPoint P)
        {
            Try(theLines, thePrefix, "pnt", () => Null(StepToGeom.MakeCartesianPoint(P, F), r => Pnt(r.Pnt())));
            Try(theLines, thePrefix, "pnt2d", () => Null(StepToGeom.MakeCartesianPoint2d(P, F), r => Pnt(r.Pnt2d())));
        }
        else if (theEnt is StepGeom_Direction D)
        {
            Try(theLines, thePrefix, "dir", () => Null(StepToGeom.MakeDirection(D), r => Dir(r.Dir())));
            Try(theLines, thePrefix, "dir2d", () => Null(StepToGeom.MakeDirection2d(D), r => Dir(r.Dir2d())));
        }
        else if (theEnt is StepGeom_Vector V)
        {
            Try(theLines, thePrefix, "vec", () => Null(StepToGeom.MakeVectorWithMagnitude(V, F), r => Pnt(new gp_Pnt(r.Vec().XYZ()))));
            Try(theLines, thePrefix, "vec2d", () => Null(StepToGeom.MakeVectorWithMagnitude2d(V), r => Pnt(new gp_Pnt2d(r.Vec2d().XY()))));
        }
        else if (theEnt is StepGeom_Axis1Placement A1)
        {
            Try(theLines, thePrefix, "ax1", () => Null(StepToGeom.MakeAxis1Placement(A1, F), r => Pnt(r.Ax1().Location()) + " " + Dir(r.Ax1().Direction())));
        }
        else if (theEnt is StepGeom_Axis2Placement3d A2)
        {
            Try(theLines, thePrefix, "ax2", () => Null(StepToGeom.MakeAxis2Placement(A2, F),
                                                       r => Pnt(r.Ax2().Location()) + " " + Dir(r.Ax2().Direction()) + " " + Dir(r.Ax2().XDirection())));
        }
        else if (theEnt is StepGeom_Axis2Placement2d A22)
        {
            Try(theLines, thePrefix, "ax2d", () => Null(StepToGeom.MakeAxisPlacement(A22, F), r => Pnt(r.Ax2d().Location()) + " " + Dir(r.Ax2d().Direction())));
        }
        else if (theEnt is StepGeom_CartesianTransformationOperator3d T3)
        {
            Try(theLines, thePrefix, "trsf", () =>
            {
                gp_Trsf T = new gp_Trsf();
                return StepToGeom.MakeTransformation3d(T3, ref T, F) ? Trsf(T) : "false";
            });
        }
        else if (theEnt is StepGeom_CartesianTransformationOperator2d T2)
        {
            Try(theLines, thePrefix, "trsf2d", () =>
            {
                gp_Trsf2d T = new gp_Trsf2d();
                return StepToGeom.MakeTransformation2d(T2, ref T, F) ? Trsf2d(T) : "false";
            });
        }
        else if (theEnt is StepGeom_SuParameters SU)
        {
            Try(theLines, thePrefix, "su", () => Null(StepToGeom.MakeAxis2Placement(SU),
                                                      r => Pnt(r.Ax2().Location()) + " " + Dir(r.Ax2().Direction()) + " " + Dir(r.Ax2().XDirection())));
        }

        if (theEnt is StepGeom_Curve C)
        {
            Try(theLines, thePrefix, "curve", () => Null(StepToGeom.MakeCurve(C, F), Curve));
            Try(theLines, thePrefix, "curve2d", () => Null(StepToGeom.MakeCurve2d(C, F), Curve2d));
        }

        if (theEnt is StepGeom_Surface S)
        {
            Try(theLines, thePrefix, "surface", () => Null(StepToGeom.MakeSurface(S, F), Surface));
        }
    }

    /// <summary>The s2g lines of a model, with the given factors.</summary>
    public static List<string> ModelLines(StepData_StepModel theModel, StepData_Factors F)
    {
        var aLines = new List<string>();
        for (int i = 1; i <= theModel.NbEntities(); i++)
        {
            StepToGeomLines(theModel.Value(i), F, i.ToString(CultureInfo.InvariantCulture) + " ", aLines);
        }

        return aLines;
    }

    // ---------------------------------------------------------------------------------------------------------
    // GeomToStep: a result written as STEP entities, then converted back

    public static string Written(Standard_Transient theEnt, StepData_Protocol theProtocol)
    {
        var aModel = new StepData_StepModel();
        aModel.SetProtocol(theProtocol);
        aModel.AddWithRefs(theEnt, theProtocol);
        var aWriter = new StepData_StepWriter(aModel);
        var aLib = new StepData_WriterLib(theProtocol);
        for (int i = 1; i <= aModel.NbEntities(); i++)
        {
            aWriter.SendEntity(i, aLib);
        }

        var aRes = new StringBuilder();
        for (int i = 1; i <= aWriter.NbLines(); i++)
        {
            aRes.Append(i > 1 ? " | " : "").Append(aWriter.Line(i).ToCString());
        }

        return aRes.ToString();
    }

    private sealed class Context(StepData_Protocol theProtocol, StepData_Factors theFactors, List<string> theLines)
    {
        public readonly StepData_Protocol Protocol = theProtocol;
        public readonly StepData_Factors F = theFactors;
        public readonly List<string> Lines = theLines;

        // One maker: "<id> <maker> <entities>" or NOTDONE or EXC, then the entity converted back
        public void Make(string theId, string theName, Func<(bool Done, Func<Standard_Transient?> Value)> theMaker)
        {
            Standard_Transient? anEnt = null;
            string aRes;
            try
            {
                var (isDone, aValue) = theMaker();
                if (!isDone)
                {
                    aRes = "NOTDONE";
                }
                else
                {
                    anEnt = aValue();
                    aRes = anEnt is null ? "null" : Written(anEnt, Protocol);
                }
            }
            catch (Exception e) when (e is not OutOfMemoryException)
            {
                // (an entity made but not written is still converted back, as the recorder does)
                aRes = "EXC";
            }

            Lines.Add(theId + " " + theName + " " + aRes);
            if (anEnt is not null)
            {
                StepToGeomLines(anEnt, F, theId + " " + theName + " back ", Lines);
            }
        }
    }

    private static (bool, Func<Standard_Transient?>) M(GeomToStep_Root theMaker, Func<Standard_Transient?> theValue) => (theMaker.IsDone(), theValue);

    private static void CurveMakers(string theId, Geom_Curve C, Context X)
    {
        X.Lines.Add(theId + " " + C.GetType().Name);
        var F = X.F;
        X.Make(theId, "curve", () => { var m = new GeomToStep_MakeCurve(C, F); return M(m, m.Value); });
        if (C is Geom_Line L)
        {
            X.Make(theId, "line", () => { var m = new GeomToStep_MakeLine(L, F); return M(m, m.Value); });
            X.Make(theId, "lin", () => { var m = new GeomToStep_MakeLine(L.Lin(), F); return M(m, m.Value); });
        }

        if (C is Geom_Conic Co)
        {
            X.Make(theId, "conic", () => { var m = new GeomToStep_MakeConic(Co, F); return M(m, m.Value); });
        }

        if (C is Geom_Circle Ci)
        {
            X.Make(theId, "circle", () => { var m = new GeomToStep_MakeCircle(Ci, F); return M(m, m.Value); });
            X.Make(theId, "circ", () => { var m = new GeomToStep_MakeCircle(Ci.Circ(), F); return M(m, m.Value); });
        }

        if (C is Geom_Ellipse E)
        {
            X.Make(theId, "ellipse", () => { var m = new GeomToStep_MakeEllipse(E, F); return M(m, m.Value); });
            X.Make(theId, "elips", () => { var m = new GeomToStep_MakeEllipse(E.Elips(), F); return M(m, m.Value); });
        }

        if (C is Geom_Hyperbola H)
        {
            X.Make(theId, "hyperbola", () => { var m = new GeomToStep_MakeHyperbola(H, F); return M(m, m.Value); });
        }

        if (C is Geom_Parabola P)
        {
            X.Make(theId, "parabola", () => { var m = new GeomToStep_MakeParabola(P, F); return M(m, m.Value); });
        }

        if (C is Geom_BoundedCurve B)
        {
            X.Make(theId, "bounded", () => { var m = new GeomToStep_MakeBoundedCurve(B, F); return M(m, m.Value); });
        }

        if (C is Geom_BSplineCurve BS)
        {
            X.Make(theId, "bspline", () => { var m = new GeomToStep_MakeBSplineCurveWithKnots(BS, F); return M(m, m.Value); });
            X.Make(theId, "rbspline", () => { var m = new GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BS, F); return M(m, m.Value); });
        }
    }

    private static void Curve2dMakers(string theId, Geom2d_Curve C, Context X)
    {
        X.Lines.Add(theId + " " + C.GetType().Name);
        var F = X.F;
        X.Make(theId, "curve", () => { var m = new GeomToStep_MakeCurve(C, F); return M(m, m.Value); });
        if (C is Geom2d_Line L)
        {
            X.Make(theId, "line", () => { var m = new GeomToStep_MakeLine(L, F); return M(m, m.Value); });
            X.Make(theId, "lin", () => { var m = new GeomToStep_MakeLine(L.Lin2d(), F); return M(m, m.Value); });
        }

        if (C is Geom2d_Conic Co)
        {
            X.Make(theId, "conic", () => { var m = new GeomToStep_MakeConic(Co, F); return M(m, m.Value); });
        }

        if (C is Geom2d_Circle Ci)
        {
            X.Make(theId, "circle", () => { var m = new GeomToStep_MakeCircle(Ci, F); return M(m, m.Value); });
        }

        if (C is Geom2d_Ellipse E)
        {
            X.Make(theId, "ellipse", () => { var m = new GeomToStep_MakeEllipse(E, F); return M(m, m.Value); });
        }

        if (C is Geom2d_Hyperbola H)
        {
            X.Make(theId, "hyperbola", () => { var m = new GeomToStep_MakeHyperbola(H, F); return M(m, m.Value); });
        }

        if (C is Geom2d_Parabola P)
        {
            X.Make(theId, "parabola", () => { var m = new GeomToStep_MakeParabola(P, F); return M(m, m.Value); });
        }

        if (C is Geom2d_BoundedCurve B)
        {
            X.Make(theId, "bounded", () => { var m = new GeomToStep_MakeBoundedCurve(B, F); return M(m, m.Value); });
        }

        if (C is Geom2d_BSplineCurve BS)
        {
            X.Make(theId, "bspline", () => { var m = new GeomToStep_MakeBSplineCurveWithKnots(BS, F); return M(m, m.Value); });
            X.Make(theId, "rbspline", () => { var m = new GeomToStep_MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BS, F); return M(m, m.Value); });
        }
    }

    private static void SurfaceMakers(string theId, Geom_Surface S, Context X)
    {
        X.Lines.Add(theId + " " + S.GetType().Name);
        var F = X.F;
        X.Make(theId, "surface", () => { var m = new GeomToStep_MakeSurface(S, F); return M(m, m.Value); });
        if (S is Geom_BoundedSurface B)
        {
            X.Make(theId, "bounded", () => { var m = new GeomToStep_MakeBoundedSurface(B, F); return M(m, m.Value); });
        }

        if (S is Geom_ElementarySurface E)
        {
            X.Make(theId, "elementary", () => { var m = new GeomToStep_MakeElementarySurface(E, F); return M(m, m.Value); });
        }

        if (S is Geom_SweptSurface W)
        {
            X.Make(theId, "swept", () => { var m = new GeomToStep_MakeSweptSurface(W, F); return M(m, m.Value); });
        }

        if (S is Geom_Plane P)
        {
            X.Make(theId, "plane", () => { var m = new GeomToStep_MakePlane(P, F); return M(m, m.Value); });
            X.Make(theId, "pln", () => { var m = new GeomToStep_MakePlane(P.Pln(), F); return M(m, m.Value); });
        }

        if (S is Geom_CylindricalSurface Cy)
        {
            X.Make(theId, "cylinder", () => { var m = new GeomToStep_MakeCylindricalSurface(Cy, F); return M(m, m.Value); });
        }

        if (S is Geom_ConicalSurface Co)
        {
            X.Make(theId, "cone", () => { var m = new GeomToStep_MakeConicalSurface(Co, F); return M(m, m.Value); });
        }

        if (S is Geom_SphericalSurface Sp)
        {
            X.Make(theId, "sphere", () => { var m = new GeomToStep_MakeSphericalSurface(Sp, F); return M(m, m.Value); });
        }

        if (S is Geom_ToroidalSurface To)
        {
            X.Make(theId, "torus", () => { var m = new GeomToStep_MakeToroidalSurface(To, F); return M(m, m.Value); });
        }

        if (S is Geom_SurfaceOfLinearExtrusion Ex)
        {
            X.Make(theId, "extrusion", () => { var m = new GeomToStep_MakeSurfaceOfLinearExtrusion(Ex, F); return M(m, m.Value); });
        }

        if (S is Geom_SurfaceOfRevolution Rv)
        {
            X.Make(theId, "revolution", () => { var m = new GeomToStep_MakeSurfaceOfRevolution(Rv, F); return M(m, m.Value); });
        }

        if (S is Geom_RectangularTrimmedSurface RT)
        {
            X.Make(theId, "rtrimmed", () => { var m = new GeomToStep_MakeRectangularTrimmedSurface(RT, F); return M(m, m.Value); });
        }

        if (S is Geom_BSplineSurface BS)
        {
            X.Make(theId, "bspline", () => { var m = new GeomToStep_MakeBSplineSurfaceWithKnots(BS, F); return M(m, m.Value); });
            X.Make(theId, "rbspline", () => { var m = new GeomToStep_MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface(BS, F); return M(m, m.Value); });
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // The cases

    private static gp_Trsf MakeTrsf(List<double> v)
    {
        gp_Trsf T = new gp_Trsf();
        for (int i = 0; i + 7 < v.Count; i += 8)
        {
            gp_Trsf t = new gp_Trsf();
            gp_Pnt p = new gp_Pnt(v[i + 1], v[i + 2], v[i + 3]);
            switch ((int)v[i])
            {
                case 1: t.SetRotation(new gp_Ax1(p, new gp_Dir(v[i + 4], v[i + 5], v[i + 6])), v[i + 7]); break;
                case 2: t.SetTranslation(new gp_Vec(v[i + 1], v[i + 2], v[i + 3])); break;
                case 3: t.SetScale(p, v[i + 7]); break;
                default: t.SetMirror(p); break;
            }

            T.PreMultiply(t);
        }

        return T;
    }

    private static void Add(Context X, string theId, string theName, Func<Standard_Transient> theValue)
    {
        try
        {
            X.Lines.Add(theId + " " + theName + " " + Written(theValue(), X.Protocol));
        }
        catch (Exception e) when (e is not OutOfMemoryException)
        {
            X.Lines.Add(theId + " " + theName + " EXC");
        }
    }

    private static void CaseLines(string theId, string theLine, Context X)
    {
        int aSpace = theLine.IndexOf(' ');
        string aKind = aSpace < 0 ? theLine : theLine[..aSpace];
        string aRest = aSpace < 0 ? "" : theLine[(aSpace + 1)..];
        var F = X.F;
        switch (aKind)
        {
            case "C3":
                CurveMakers(theId, GeomTools_CurveSet.ReadCurve(new StringReader(aRest))!, X);
                return;
            case "C2":
                Curve2dMakers(theId, GeomTools_Curve2dSet.ReadCurve2d(new StringReader(aRest))!, X);
                return;
            case "S":
                SurfaceMakers(theId, GeomTools_SurfaceSet.ReadSurface(new StringReader(aRest))!, X);
                return;
        }

        var v = aRest.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(t => double.Parse(t, CultureInfo.InvariantCulture)).ToList();
        X.Lines.Add(theId + " " + aKind);
        switch (aKind)
        {
            case "PNT":
            {
                gp_Pnt P = new gp_Pnt(v[0], v[1], v[2]);
                Add(X, theId, "pnt", () => new GeomToStep_MakeCartesianPoint(P, F.LengthFactor()).Value());
                Add(X, theId, "gpnt", () => new GeomToStep_MakeCartesianPoint(new Geom_CartesianPoint(P), F.LengthFactor()).Value());
                break;
            }
            case "PNT2":
            {
                gp_Pnt2d P = new gp_Pnt2d(v[0], v[1]);
                Add(X, theId, "pnt", () => new GeomToStep_MakeCartesianPoint(P, F.LengthFactor()).Value());
                Add(X, theId, "gpnt", () => new GeomToStep_MakeCartesianPoint(new Geom2d_CartesianPoint(P)).Value());
                break;
            }
            case "DIR":
            {
                gp_Dir D = new gp_Dir(v[0], v[1], v[2]);
                X.Make(theId, "dir", () => { var m = new GeomToStep_MakeDirection(D); return M(m, m.Value); });
                X.Make(theId, "gdir", () => { var m = new GeomToStep_MakeDirection(new Geom_Direction(D)); return M(m, m.Value); });
                break;
            }
            case "DIR2":
            {
                gp_Dir2d D = new gp_Dir2d(v[0], v[1]);
                X.Make(theId, "dir", () => { var m = new GeomToStep_MakeDirection(D); return M(m, m.Value); });
                X.Make(theId, "gdir", () => { var m = new GeomToStep_MakeDirection(new Geom2d_Direction(D)); return M(m, m.Value); });
                break;
            }
            case "VEC":
            {
                gp_Vec V = new gp_Vec(v[0], v[1], v[2]);
                X.Make(theId, "vec", () => { var m = new GeomToStep_MakeVector(V, F); return M(m, m.Value); });
                X.Make(theId, "gvec", () => { var m = new GeomToStep_MakeVector(new Geom_VectorWithMagnitude(V), F); return M(m, m.Value); });
                break;
            }
            case "VEC2":
            {
                gp_Vec2d V = new gp_Vec2d(v[0], v[1]);
                X.Make(theId, "vec", () => { var m = new GeomToStep_MakeVector(V, F); return M(m, m.Value); });
                X.Make(theId, "gvec", () => { var m = new GeomToStep_MakeVector(new Geom2d_VectorWithMagnitude(V), F); return M(m, m.Value); });
                break;
            }
            case "AX1":
            {
                gp_Ax1 A = new gp_Ax1(new gp_Pnt(v[0], v[1], v[2]), new gp_Dir(v[3], v[4], v[5]));
                X.Make(theId, "ax1", () => { var m = new GeomToStep_MakeAxis1Placement(A, F); return M(m, m.Value); });
                X.Make(theId, "gax1", () => { var m = new GeomToStep_MakeAxis1Placement(new Geom_Axis1Placement(A), F); return M(m, m.Value); });
                break;
            }
            case "AX2D":
            {
                gp_Ax2d A = new gp_Ax2d(new gp_Pnt2d(v[0], v[1]), new gp_Dir2d(v[2], v[3]));
                X.Make(theId, "ax2d", () => { var m = new GeomToStep_MakeAxis1Placement(A, F); return M(m, m.Value); });
                X.Make(theId, "gax2d", () => { var m = new GeomToStep_MakeAxis1Placement(new Geom2d_AxisPlacement(A), F); return M(m, m.Value); });
                break;
            }
            case "AX2":
            {
                gp_Ax2 A = new gp_Ax2(new gp_Pnt(v[0], v[1], v[2]), new gp_Dir(v[3], v[4], v[5]), new gp_Dir(v[6], v[7], v[8]));
                X.Make(theId, "ax2", () => { var m = new GeomToStep_MakeAxis2Placement3d(A, F); return M(m, m.Value); });
                X.Make(theId, "gax2", () => { var m = new GeomToStep_MakeAxis2Placement3d(new Geom_Axis2Placement(A), F); return M(m, m.Value); });
                X.Make(theId, "ax2to2d", () => { var m = new GeomToStep_MakeAxis2Placement2d(A, F); return M(m, m.Value); });
                gp_Ax3 A3 = new gp_Ax3(A);
                if (v.Count > 9 && v[9] == 0.0)
                {
                    A3.YReverse();
                }

                X.Make(theId, "ax3", () => { var m = new GeomToStep_MakeAxis2Placement3d(A3, F); return M(m, m.Value); });
                break;
            }
            case "AX22D":
            {
                gp_Ax22d A = new gp_Ax22d(new gp_Pnt2d(v[0], v[1]), new gp_Dir2d(v[2], v[3]), v[4] != 0.0);
                X.Make(theId, "ax22d", () => { var m = new GeomToStep_MakeAxis2Placement2d(A, F); return M(m, m.Value); });
                break;
            }
            case "TRSF":
            {
                gp_Trsf T = MakeTrsf(v);
                X.Lines.Add(theId + " " + Trsf(T));
                X.Make(theId, "cto", () => { var m = new GeomToStep_MakeCartesianTransformationOperator(T, F); return M(m, m.Value); });
                X.Make(theId, "ax2trsf", () => { var m = new GeomToStep_MakeAxis2Placement3d(T, F); return M(m, m.Value); });
                break;
            }
            case "ORIGIN":
                Add(X, theId, "origin", () => new GeomToStep_MakeAxis2Placement3d(F).Value());
                break;
            case "PL3":
            {
                int n = (int)v[0];
                var P = new NCollection_Array1<gp_Pnt>(1, n);
                for (int i = 1; i <= n; i++)
                {
                    P.SetValue(i, new gp_Pnt(v[3 * i - 2], v[3 * i - 1], v[3 * i]));
                }

                X.Make(theId, "polyline", () => { var m = new GeomToStep_MakePolyline(P, F); return M(m, m.Value); });
                break;
            }
            case "PL2":
            {
                int n = (int)v[0];
                var P = new NCollection_Array1<gp_Pnt2d>(1, n);
                for (int i = 1; i <= n; i++)
                {
                    P.SetValue(i, new gp_Pnt2d(v[2 * i - 1], v[2 * i]));
                }

                X.Make(theId, "polyline", () => { var m = new GeomToStep_MakePolyline(P, F); return M(m, m.Value); });
                break;
            }
        }
    }

    /// <summary>The g2s lines of the cases, with the given factors.</summary>
    public static List<string> CasesLines(IReadOnlyList<string> theCases, StepData_Protocol theProtocol, StepData_Factors F)
    {
        var aLines = new List<string>();
        var X = new Context(theProtocol, F, aLines);
        for (int i = 0; i < theCases.Count; i++)
        {
            string anId = "k" + (i + 1).ToString(CultureInfo.InvariantCulture);
            try
            {
                CaseLines(anId, theCases[i], X);
            }
            catch (Exception e) when (e is not OutOfMemoryException)
            {
                aLines.Add(anId + " EXC");
            }
        }

        return aLines;
    }

    /// <summary>The g2s lines of a shape's geometry, with the given factors.</summary>
    public static List<string> ShapeLines(in TopoDS_Shape theShape, StepData_Protocol theProtocol, StepData_Factors F)
    {
        var aLines = new List<string>();
        var X = new Context(theProtocol, F, aLines);
        var aSeen = new HashSet<object>(ReferenceEqualityComparer.Instance);
        int aNb = 0;
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            Geom_Surface? S = BRep_Tool.Surface(TopoDS.Face(anExp.Current()), out _);
            if (S is null || !aSeen.Add(S))
            {
                continue;
            }

            SurfaceMakers("s" + (++aNb).ToString(CultureInfo.InvariantCulture), S, X);
        }

        for (var anExp = new TopExp_Explorer(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            var TE = (BRep_TEdge)anExp.Current().TShape()!;
            foreach (BRep_CurveRepresentation aRep in TE.Curves())
            {
                if (aRep.IsCurve3D())
                {
                    Geom_Curve? C = aRep.Curve3D();
                    if (C is null || !aSeen.Add(C))
                    {
                        continue;
                    }

                    string anId = "c" + (++aNb).ToString(CultureInfo.InvariantCulture);
                    CurveMakers(anId, C, X);
                    var GC = (BRep_GCurve)aRep;
                    Geom_Curve? T = null;
                    try
                    {
                        T = new Geom_TrimmedCurve(C, GC.First(), GC.Last());
                    }
                    catch (Exception e) when (e is not OutOfMemoryException)
                    {
                        aLines.Add(anId + "t EXC");
                    }

                    if (T is not null)
                    {
                        CurveMakers(anId + "t", T, X);
                    }
                }
                else if (aRep.IsCurveOnSurface())
                {
                    Geom2d_Curve?[] P = [aRep.PCurve(), aRep.IsCurveOnClosedSurface() ? aRep.PCurve2() : null];
                    foreach (Geom2d_Curve? C in P)
                    {
                        if (C is null || !aSeen.Add(C))
                        {
                            continue;
                        }

                        Curve2dMakers("p" + (++aNb).ToString(CultureInfo.InvariantCulture), C, X);
                    }
                }
            }
        }

        return aLines;
    }
}
