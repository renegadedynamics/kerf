// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export.Geometry;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Text;
using Kerf.Kernel.DataExchange.Tests.StepGen;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DataExchange.Tests.GeomToStep;

/// <summary>
/// The port's twin of tools/geomsteprec (unit 8D): the same lines, for StepToGeom on a model's geometric entities
/// ("s2g") and for GeomToStep on the constructed cases and on a shape's geometry ("g2s"), with the default factors
/// and with inches and degrees. Results written as STEP entities go through StepAP214_Protocol, as native's.
/// </summary>
internal static class GeomStepRecorder
{
    public static readonly Factors Millimetres = new();
    public static readonly Factors Inches = new Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    /// <summary>StepAP214's protocol (unit 8C2), as native's recorder reads and writes with.</summary>
    internal static readonly Lazy<StepProtocol?> TheProtocol = new(() => StepRoundTripTests.Protocol());

    // ---------------------------------------------------------------------------------------------------------
    // Text (as the recorder's ostream at precision 17)

    private static string R(double theValue) => StreamUtilities.FormatReal(theValue, 17);

    private static string OneLine(string theText) => theText.Replace('\n', ' ').Replace('\r', ' ').TrimEnd(' ');

    private static string Pnt(in Point3D P) => R(P.X) + " " + R(P.Y) + " " + R(P.Z);
    private static string Pnt(in Point2D P) => R(P.X) + " " + R(P.Y);
    private static string Dir(in Direction3D D) => R(D.X) + " " + R(D.Y) + " " + R(D.Z);
    private static string Dir(in Direction2D D) => R(D.X) + " " + R(D.Y);

    private static StringWriter Writer()
    {
        var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        StreamUtilities.Precision(aWriter, 17);
        return aWriter;
    }

    public static string Curve(Curve3D C)
    {
        var aS = Writer();
        CurveSet.PrintCurve(C, aS, true);
        return OneLine(aS.ToString());
    }

    public static string Curve2d(Curve2D C)
    {
        var aS = Writer();
        Curve2dSet.PrintCurve2d(C, aS, true);
        return OneLine(aS.ToString());
    }

    public static string Surface(Surface S)
    {
        var aS = Writer();
        SurfaceSet.PrintSurface(S, aS, true);
        return OneLine(aS.ToString());
    }

    private static string Trsf(in Transform3D T)
    {
        var aRes = new StringBuilder("trsf");
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 4; c++)
            {
                aRes.Append(' ').Append(R(T.Value(r, c)));
            }
        }

        return aRes.Append(' ').Append(R(T.ScaleFactor)).Append(' ').Append((int)T.Form).ToString();
    }

    private static string Trsf2d(in Transform2D T)
    {
        var aRes = new StringBuilder("trsf2d");
        for (int r = 1; r <= 2; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                aRes.Append(' ').Append(R(T.Value(r, c)));
            }
        }

        return aRes.Append(' ').Append(R(T.ScaleFactor)).Append(' ').Append((int)T.Form).ToString();
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

    public static void StepToGeomLines(Transient? theEnt, Factors F, string thePrefix, List<string> theLines)
    {
        if (theEnt is StepCartesianPoint P)
        {
            Try(theLines, thePrefix, "pnt", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(P, F), r => Pnt(r.Pnt())));
            Try(theLines, thePrefix, "pnt2d", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint2d(P, F), r => Pnt(r.Pnt2d())));
        }
        else if (theEnt is StepDirection D)
        {
            Try(theLines, thePrefix, "dir", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeDirection(D), r => Dir(r.Dir())));
            Try(theLines, thePrefix, "dir2d", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeDirection2d(D), r => Dir(r.Dir2d())));
        }
        else if (theEnt is StepVector V)
        {
            Try(theLines, thePrefix, "vec", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeVectorWithMagnitude(V, F), r => Pnt(new Point3D(r.Vec().XYZ()))));
            Try(theLines, thePrefix, "vec2d", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeVectorWithMagnitude2d(V), r => Pnt(new Point2D(r.Vec2d().XY()))));
        }
        else if (theEnt is StepAxis1Placement A1)
        {
            Try(theLines, thePrefix, "ax1", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis1Placement(A1, F), r => Pnt(r.Ax1().Location) + " " + Dir(r.Ax1().Direction)));
        }
        else if (theEnt is StepAxis2Placement3d A2)
        {
            Try(theLines, thePrefix, "ax2", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(A2, F),
                                                       r => Pnt(r.Ax2().Location) + " " + Dir(r.Ax2().Direction) + " " + Dir(r.Ax2().XDirection)));
        }
        else if (theEnt is StepAxis2Placement2d A22)
        {
            Try(theLines, thePrefix, "ax2d", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxisPlacement(A22, F), r => Pnt(r.Ax2d().Location) + " " + Dir(r.Ax2d().Direction)));
        }
        else if (theEnt is StepCartesianTransformationOperator3d T3)
        {
            Try(theLines, thePrefix, "trsf", () =>
            {
                Transform3D T = new Transform3D();
                return global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation3d(T3, ref T, F) ? Trsf(T) : "false";
            });
        }
        else if (theEnt is StepCartesianTransformationOperator2d T2)
        {
            Try(theLines, thePrefix, "trsf2d", () =>
            {
                Transform2D T = new Transform2D();
                return global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeTransformation2d(T2, ref T, F) ? Trsf2d(T) : "false";
            });
        }
        else if (theEnt is StepSuParameters SU)
        {
            Try(theLines, thePrefix, "su", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(SU),
                                                      r => Pnt(r.Ax2().Location) + " " + Dir(r.Ax2().Direction) + " " + Dir(r.Ax2().XDirection)));
        }

        if (theEnt is StepCurve C)
        {
            Try(theLines, thePrefix, "curve", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve(C, F), Curve));
            Try(theLines, thePrefix, "curve2d", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve2d(C, F), Curve2d));
        }

        if (theEnt is StepSurface S)
        {
            Try(theLines, thePrefix, "surface", () => Null(global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeSurface(S, F), Surface));
        }
    }

    /// <summary>The s2g lines of a model, with the given factors.</summary>
    public static List<string> ModelLines(StepModel theModel, Factors F)
    {
        var aLines = new List<string>();
        for (int i = 1; i <= theModel.EntityCount; i++)
        {
            StepToGeomLines(theModel.Value(i), F, i.ToString(CultureInfo.InvariantCulture) + " ", aLines);
        }

        return aLines;
    }

    // ---------------------------------------------------------------------------------------------------------
    // GeomToStep: a result written as STEP entities, then converted back

    public static string Written(Transient theEnt, StepProtocol theProtocol)
    {
        var aModel = new StepModel();
        aModel.SetProtocol(theProtocol);
        aModel.AddWithRefs(theEnt, theProtocol);
        var aWriter = new StepWriter(aModel);
        var aLib = new WriterLib(theProtocol);
        for (int i = 1; i <= aModel.EntityCount; i++)
        {
            aWriter.SendEntity(i, aLib);
        }

        var aRes = new StringBuilder();
        for (int i = 1; i <= aWriter.LineCount; i++)
        {
            aRes.Append(i > 1 ? " | " : "").Append(aWriter.Line(i).ToCString());
        }

        return aRes.ToString();
    }

    private sealed class Context(StepProtocol theProtocol, Factors theFactors, List<string> theLines)
    {
        public readonly StepProtocol Protocol = theProtocol;
        public readonly Factors F = theFactors;
        public readonly List<string> Lines = theLines;

        // One maker: "<id> <maker> <entities>" or NOTDONE or EXC, then the entity converted back
        public void Make(string theId, string theName, Func<(bool Done, Func<Transient?> Value)> theMaker)
        {
            Transient? anEnt = null;
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

    private static (bool, Func<Transient?>) M(GeometryRoot theMaker, Func<Transient?> theValue) => (theMaker.IsDone, theValue);

    private static void CurveMakers(string theId, Curve3D C, Context X)
    {
        X.Lines.Add(theId + " " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(C.GetType()));
        var F = X.F;
        X.Make(theId, "curve", () => { var m = new MakeCurve(C, F); return M(m, m.Value); });
        if (C is LineCurve3D L)
        {
            X.Make(theId, "line", () => { var m = new GeometryMakeLine(L, F); return M(m, m.Value); });
            X.Make(theId, "lin", () => { var m = new GeometryMakeLine(L.Line, F); return M(m, m.Value); });
        }

        if (C is ConicCurve3D Co)
        {
            X.Make(theId, "conic", () => { var m = new MakeConic(Co, F); return M(m, m.Value); });
        }

        if (C is CircleCurve3D Ci)
        {
            X.Make(theId, "circle", () => { var m = new GeometryMakeCircle(Ci, F); return M(m, m.Value); });
            X.Make(theId, "circ", () => { var m = new GeometryMakeCircle(Ci.Circle, F); return M(m, m.Value); });
        }

        if (C is EllipseCurve3D E)
        {
            X.Make(theId, "ellipse", () => { var m = new GeometryMakeEllipse(E, F); return M(m, m.Value); });
            X.Make(theId, "elips", () => { var m = new GeometryMakeEllipse(E.Ellipse, F); return M(m, m.Value); });
        }

        if (C is HyperbolaCurve3D H)
        {
            X.Make(theId, "hyperbola", () => { var m = new GeometryMakeHyperbola(H, F); return M(m, m.Value); });
        }

        if (C is ParabolaCurve3D P)
        {
            X.Make(theId, "parabola", () => { var m = new MakeParabola(P, F); return M(m, m.Value); });
        }

        if (C is BoundedCurve3D B)
        {
            X.Make(theId, "bounded", () => { var m = new MakeBoundedCurve(B, F); return M(m, m.Value); });
        }

        if (C is BSplineCurve3D BS)
        {
            X.Make(theId, "bspline", () => { var m = new MakeBSplineCurveWithKnots(BS, F); return M(m, m.Value); });
            X.Make(theId, "rbspline", () => { var m = new MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BS, F); return M(m, m.Value); });
        }
    }

    private static void Curve2dMakers(string theId, Curve2D C, Context X)
    {
        X.Lines.Add(theId + " " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(C.GetType()));
        var F = X.F;
        X.Make(theId, "curve", () => { var m = new MakeCurve(C, F); return M(m, m.Value); });
        if (C is LineCurve2D L)
        {
            X.Make(theId, "line", () => { var m = new GeometryMakeLine(L, F); return M(m, m.Value); });
            X.Make(theId, "lin", () => { var m = new GeometryMakeLine(L.Line, F); return M(m, m.Value); });
        }

        if (C is ConicCurve2D Co)
        {
            X.Make(theId, "conic", () => { var m = new MakeConic(Co, F); return M(m, m.Value); });
        }

        if (C is CircleCurve2D Ci)
        {
            X.Make(theId, "circle", () => { var m = new GeometryMakeCircle(Ci, F); return M(m, m.Value); });
        }

        if (C is EllipseCurve2D E)
        {
            X.Make(theId, "ellipse", () => { var m = new GeometryMakeEllipse(E, F); return M(m, m.Value); });
        }

        if (C is HyperbolaCurve2D H)
        {
            X.Make(theId, "hyperbola", () => { var m = new GeometryMakeHyperbola(H, F); return M(m, m.Value); });
        }

        if (C is ParabolaCurve2D P)
        {
            X.Make(theId, "parabola", () => { var m = new MakeParabola(P, F); return M(m, m.Value); });
        }

        if (C is BoundedCurve2D B)
        {
            X.Make(theId, "bounded", () => { var m = new MakeBoundedCurve(B, F); return M(m, m.Value); });
        }

        if (C is BSplineCurve2D BS)
        {
            X.Make(theId, "bspline", () => { var m = new MakeBSplineCurveWithKnots(BS, F); return M(m, m.Value); });
            X.Make(theId, "rbspline", () => { var m = new MakeBSplineCurveWithKnotsAndRationalBSplineCurve(BS, F); return M(m, m.Value); });
        }
    }

    private static void SurfaceMakers(string theId, Surface S, Context X)
    {
        X.Lines.Add(theId + " " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(S.GetType()));
        var F = X.F;
        X.Make(theId, "surface", () => { var m = new MakeSurface(S, F); return M(m, m.Value); });
        if (S is BoundedSurface B)
        {
            X.Make(theId, "bounded", () => { var m = new MakeBoundedSurface(B, F); return M(m, m.Value); });
        }

        if (S is ElementarySurface E)
        {
            X.Make(theId, "elementary", () => { var m = new MakeElementarySurface(E, F); return M(m, m.Value); });
        }

        if (S is SweptSurface W)
        {
            X.Make(theId, "swept", () => { var m = new MakeSweptSurface(W, F); return M(m, m.Value); });
        }

        if (S is PlaneSurface P)
        {
            X.Make(theId, "plane", () => { var m = new GeometryMakePlane(P, F); return M(m, m.Value); });
            X.Make(theId, "pln", () => { var m = new GeometryMakePlane(P.Plane, F); return M(m, m.Value); });
        }

        if (S is CylindricalSurface Cy)
        {
            X.Make(theId, "cylinder", () => { var m = new GeometryMakeCylindricalSurface(Cy, F); return M(m, m.Value); });
        }

        if (S is ConicalSurface Co)
        {
            X.Make(theId, "cone", () => { var m = new GeometryMakeConicalSurface(Co, F); return M(m, m.Value); });
        }

        if (S is SphericalSurface Sp)
        {
            X.Make(theId, "sphere", () => { var m = new MakeSphericalSurface(Sp, F); return M(m, m.Value); });
        }

        if (S is ToroidalSurface To)
        {
            X.Make(theId, "torus", () => { var m = new MakeToroidalSurface(To, F); return M(m, m.Value); });
        }

        if (S is ExtrusionSurface Ex)
        {
            X.Make(theId, "extrusion", () => { var m = new MakeSurfaceOfLinearExtrusion(Ex, F); return M(m, m.Value); });
        }

        if (S is RevolutionSurface Rv)
        {
            X.Make(theId, "revolution", () => { var m = new MakeSurfaceOfRevolution(Rv, F); return M(m, m.Value); });
        }

        if (S is TrimmedSurface RT)
        {
            X.Make(theId, "rtrimmed", () => { var m = new MakeRectangularTrimmedSurface(RT, F); return M(m, m.Value); });
        }

        if (S is BSplineSurface BS)
        {
            X.Make(theId, "bspline", () => { var m = new MakeBSplineSurfaceWithKnots(BS, F); return M(m, m.Value); });
            X.Make(theId, "rbspline", () => { var m = new MakeBSplineSurfaceWithKnotsAndRationalBSplineSurface(BS, F); return M(m, m.Value); });
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // The cases

    private static Transform3D MakeTrsf(List<double> v)
    {
        Transform3D T = new Transform3D();
        for (int i = 0; i + 7 < v.Count; i += 8)
        {
            Transform3D t = new Transform3D();
            Point3D p = new Point3D(v[i + 1], v[i + 2], v[i + 3]);
            switch ((int)v[i])
            {
                case 1: t = t.WithRotation(new Axis3D(p, new Direction3D(v[i + 4], v[i + 5], v[i + 6])), v[i + 7]); break;
                case 2: t = t.WithTranslation(new Vector3D(v[i + 1], v[i + 2], v[i + 3])); break;
                case 3: t = t.WithScale(p, v[i + 7]); break;
                default: t = t.WithMirror(p); break;
            }

            T = T.PreMultiplied(t);
        }

        return T;
    }

    private static void Add(Context X, string theId, string theName, Func<Transient> theValue)
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
                CurveMakers(theId, CurveSet.ReadCurve(new StringReader(aRest))!, X);
                return;
            case "C2":
                Curve2dMakers(theId, Curve2dSet.ReadCurve2d(new StringReader(aRest))!, X);
                return;
            case "S":
                SurfaceMakers(theId, SurfaceSet.ReadSurface(new StringReader(aRest))!, X);
                return;
        }

        var v = aRest.Split(' ', StringSplitOptions.RemoveEmptyEntries).Select(t => double.Parse(t, CultureInfo.InvariantCulture)).ToList();
        X.Lines.Add(theId + " " + aKind);
        switch (aKind)
        {
            case "PNT":
            {
                Point3D P = new Point3D(v[0], v[1], v[2]);
                Add(X, theId, "pnt", () => new MakeCartesianPoint(P, F.LengthFactor()).Value());
                Add(X, theId, "gpnt", () => new MakeCartesianPoint(new CartesianPoint(P), F.LengthFactor()).Value());
                break;
            }
            case "PNT2":
            {
                Point2D P = new Point2D(v[0], v[1]);
                Add(X, theId, "pnt", () => new MakeCartesianPoint(P, F.LengthFactor()).Value());
                Add(X, theId, "gpnt", () => new MakeCartesianPoint(new CartesianPoint2(P)).Value());
                break;
            }
            case "DIR":
            {
                Direction3D D = new Direction3D(v[0], v[1], v[2]);
                X.Make(theId, "dir", () => { var m = new MakeDirection(D); return M(m, m.Value); });
                X.Make(theId, "gdir", () => { var m = new MakeDirection(new Direction(D)); return M(m, m.Value); });
                break;
            }
            case "DIR2":
            {
                Direction2D D = new Direction2D(v[0], v[1]);
                X.Make(theId, "dir", () => { var m = new MakeDirection(D); return M(m, m.Value); });
                X.Make(theId, "gdir", () => { var m = new MakeDirection(new Direction2(D)); return M(m, m.Value); });
                break;
            }
            case "VEC":
            {
                Vector3D V = new Vector3D(v[0], v[1], v[2]);
                X.Make(theId, "vec", () => { var m = new MakeVector(V, F); return M(m, m.Value); });
                X.Make(theId, "gvec", () => { var m = new MakeVector(new VectorWithMagnitude(V), F); return M(m, m.Value); });
                break;
            }
            case "VEC2":
            {
                Vector2D V = new Vector2D(v[0], v[1]);
                X.Make(theId, "vec", () => { var m = new MakeVector(V, F); return M(m, m.Value); });
                X.Make(theId, "gvec", () => { var m = new MakeVector(new VectorWithMagnitude2(V), F); return M(m, m.Value); });
                break;
            }
            case "AX1":
            {
                Axis3D A = new Axis3D(new Point3D(v[0], v[1], v[2]), new Direction3D(v[3], v[4], v[5]));
                X.Make(theId, "ax1", () => { var m = new MakeAxis1Placement(A, F); return M(m, m.Value); });
                X.Make(theId, "gax1", () => { var m = new MakeAxis1Placement(new Axis1Placement(A), F); return M(m, m.Value); });
                break;
            }
            case "AX2D":
            {
                Axis2D A = new Axis2D(new Point2D(v[0], v[1]), new Direction2D(v[2], v[3]));
                X.Make(theId, "ax2d", () => { var m = new MakeAxis1Placement(A, F); return M(m, m.Value); });
                X.Make(theId, "gax2d", () => { var m = new MakeAxis1Placement(new AxisPlacement2(A), F); return M(m, m.Value); });
                break;
            }
            case "AX2":
            {
                Frame3D A = new Frame3D(new Point3D(v[0], v[1], v[2]), new Direction3D(v[3], v[4], v[5]), new Direction3D(v[6], v[7], v[8]));
                X.Make(theId, "ax2", () => { var m = new Axis2Placement3dMaker(A, F); return M(m, m.Value); });
                X.Make(theId, "gax2", () => { var m = new Axis2Placement3dMaker(new Axis2Placement(A), F); return M(m, m.Value); });
                X.Make(theId, "ax2to2d", () => { var m = new MakeAxis2Placement2d(A, F); return M(m, m.Value); });
                CoordinateSystem A3 = new CoordinateSystem(A);
                if (v.Count > 9 && v[9] == 0.0)
                {
                    A3 = A3.YReversed();
                }

                X.Make(theId, "ax3", () => { var m = new Axis2Placement3dMaker(A3, F); return M(m, m.Value); });
                break;
            }
            case "AX22D":
            {
                Frame2D A = new Frame2D(new Point2D(v[0], v[1]), new Direction2D(v[2], v[3]), v[4] != 0.0);
                X.Make(theId, "ax22d", () => { var m = new MakeAxis2Placement2d(A, F); return M(m, m.Value); });
                break;
            }
            case "TRSF":
            {
                Transform3D T = MakeTrsf(v);
                X.Lines.Add(theId + " " + Trsf(T));
                X.Make(theId, "cto", () => { var m = new MakeCartesianTransformationOperator(T, F); return M(m, m.Value); });
                X.Make(theId, "ax2trsf", () => { var m = new Axis2Placement3dMaker(T, F); return M(m, m.Value); });
                break;
            }
            case "ORIGIN":
                Add(X, theId, "origin", () => new Axis2Placement3dMaker(F).Value());
                break;
            case "PL3":
            {
                int n = (int)v[0];
                var P = new NCollection_Array1<Point3D>(1, n);
                for (int i = 1; i <= n; i++)
                {
                    P.SetValue(i, new Point3D(v[3 * i - 2], v[3 * i - 1], v[3 * i]));
                }

                X.Make(theId, "polyline", () => { var m = new MakePolyline(P, F); return M(m, m.Value); });
                break;
            }
            case "PL2":
            {
                int n = (int)v[0];
                var P = new NCollection_Array1<Point2D>(1, n);
                for (int i = 1; i <= n; i++)
                {
                    P.SetValue(i, new Point2D(v[2 * i - 1], v[2 * i]));
                }

                X.Make(theId, "polyline", () => { var m = new MakePolyline(P, F); return M(m, m.Value); });
                break;
            }
        }
    }

    /// <summary>The g2s lines of the cases, with the given factors.</summary>
    public static List<string> CasesLines(IReadOnlyList<string> theCases, StepProtocol theProtocol, Factors F)
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
    public static List<string> ShapeLines(in Shape theShape, StepProtocol theProtocol, Factors F)
    {
        var aLines = new List<string>();
        var X = new Context(theProtocol, F, aLines);
        var aSeen = new HashSet<object>(ReferenceEqualityComparer.Instance);
        int aNb = 0;
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Surface? S = Tool.Surface(ShapeCasts.Face(anExp.Current()), out _);
            if (S is null || !aSeen.Add(S))
            {
                continue;
            }

            SurfaceMakers("s" + (++aNb).ToString(CultureInfo.InvariantCulture), S, X);
        }

        for (var anExp = new Explorer(theShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            var TE = (BrepTEdge)anExp.Current().TShape()!;
            foreach (CurveRepresentation aRep in TE.Curves())
            {
                if (aRep.IsCurve3D)
                {
                    Curve3D? C = aRep.Curve3D();
                    if (C is null || !aSeen.Add(C))
                    {
                        continue;
                    }

                    string anId = "c" + (++aNb).ToString(CultureInfo.InvariantCulture);
                    CurveMakers(anId, C, X);
                    var GC = (GCurve)aRep;
                    Curve3D? T = null;
                    try
                    {
                        T = new TrimmedCurve3D(C, GC.First(), GC.Last());
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
                    Curve2D?[] P = [aRep.PCurve(), aRep.IsCurveOnClosedSurface ? aRep.PCurve2() : null];
                    foreach (Curve2D? C in P)
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
