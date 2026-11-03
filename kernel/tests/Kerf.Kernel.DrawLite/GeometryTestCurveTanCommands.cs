// Port of: src/Draw/TKTopTest/GeometryTest/GeometryTest_CurveTanCommands.cxx
// Port of: src/Draw/TKTopTest/GeometryTest/GeometryTest_DrawableQualifiedCurve2d.hxx
// Port of: src/Draw/TKTopTest/GeometryTest/GeometryTest_DrawableQualifiedCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// A qualified 2D circle or line (GeometryTest_DrawableQualifiedCurve2d, a DrawTrSurf_Curve2d with a GccEnt_Position),
/// made by <c>qcircle</c> and <c>qline</c>. <see cref="DrawTrSurf.GetCurve2d"/> reads its curve, as OCCT's does through
/// the base class.
/// </summary>
public sealed class DrawQualifiedCurve2d(Curve2D theCurve, global::Kerf.Engine.Construction.Constraints.Qualifier thePosition) : DrawDrawable
{
    /// <summary>The curve (frozen).</summary>
    public Curve2D Curve { get; } = theCurve;

    /// <summary>The qualifier.</summary>
    public global::Kerf.Engine.Construction.Constraints.Qualifier Position { get; } = thePosition;

    public override string TypeName => "GeometryTest_DrawableQualifiedCurve2d";

    /// <summary>GeometryTest_DrawableQualifiedCurve2d::Whatis: nothing for curves other than circles and lines.</summary>
    public override void Whatis(DrawResult theResult)
    {
        if (Curve is CircleCurve2D)
        {
            theResult.Append("qualified 2d Circle");
        }
        else if (Curve is LineCurve2D)
        {
            theResult.Append("qualified 2d Line");
        }
    }

    /// <summary>GeometryTest_DrawableQualifiedCurve2d::Dump: the qualifier, then the curve.</summary>
    public override void Dump(TextWriter theStream)
    {
        theStream.Write("Qualified curve 2D: \n");
        theStream.Write("Position :" + global::Kerf.Engine.Construction.Constraints.GccEnt.PositionToString(Position) + "\n");
        Curve2dSet.PrintCurve2d(Curve, theStream);
    }

    /// <summary>DrawTrSurf_Curve2d::Save: the curve.</summary>
    public override void Save(TextWriter theStream) => Curve2dSet.PrintCurve2d(Curve, theStream, true);

    /// <summary>DrawTrSurf_Curve2d::Copy: a plain curve drawable (the qualifier is not copied).</summary>
    public override DrawDrawable Copy() => DrawGeometry.Of(Curve);
}

/// <summary>
/// GeometryTest_CurveTanCommands.cxx (unit 5W): <c>qcircle</c> and <c>qline</c> (qualified circles and lines) and
/// <c>circ2d3Tan</c> (GccAna_Circ2d3Tan: circles tangent to three qualified circles, lines or points).
/// </summary>
/// <remarks>Message::SendFail writes a line on the interpreter's output (decision 405).</remarks>
internal static class GeometryTestCurveTanCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "qcircle", qcurve);
        DrawCxx.Add(theInterpreter, "qline", qcurve);
        DrawCxx.Add(theInterpreter, "circ2d3Tan", circ2d3Tan);
    }

    private static int Fail(DrawInterpreter theDI, string theMessage)
    {
        theDI.Output.WriteLine(theMessage);
        return 1;
    }

    // qcircle name x y [ux uy] radius [-unqualified|-enclosing|-enclosed|-outside|-noqualifier]
    private static int qcurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        int theArgsNb = theArgVec.Count;
        if (theArgsNb < 5)
        {
            return Fail(theDI, "Error: wrong number of argument");
        }

        Curve2D? aResult2d = null;
        string aPositionType = "";
        if (theArgVec[0] == "qcircle")
        {
            if (theArgsNb == 5 || theArgsNb == 6)
            {
                aResult2d = new CircleCurve2D(new Frame2D(new Point2D(theDI.Atof(theArgVec[2]), theDI.Atof(theArgVec[3])),
                                                           new Direction2D(Direction2D.D.X)),
                                              theDI.Atof(theArgVec[4]));
            }
            else if (theArgsNb == 7 || theArgsNb == 8)
            {
                aResult2d = new CircleCurve2D(new Frame2D(new Point2D(theDI.Atof(theArgVec[2]), theDI.Atof(theArgVec[3])),
                                                           new Direction2D(theDI.Atof(theArgVec[4]), theDI.Atof(theArgVec[5]))),
                                              theDI.Atof(theArgVec[6]));
            }

            if (theArgsNb == 6)
            {
                aPositionType = theArgVec[5];
            }
            else if (theArgsNb == 8)
            {
                aPositionType = theArgVec[7];
            }
        }
        else if (theArgVec[0] == "qline")
        {
            if (theArgsNb < 6)
            {
                return Fail(theDI, "Error: wrong number of arguments");
            }
            aResult2d = new LineCurve2D(new Point2D(theDI.Atof(theArgVec[2]), theDI.Atof(theArgVec[3])),
                                        new Direction2D(theDI.Atof(theArgVec[4]), theDI.Atof(theArgVec[5])));
            if (theArgsNb == 7)
            {
                aPositionType = theArgVec[6];
            }
        }
        else
        {
            return Fail(theDI, "Error: wrong command name");
        }

        global::Kerf.Engine.Construction.Constraints.Qualifier aKindOfPosition = global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified;
        if (aPositionType.Length != 0)
        {
            global::Kerf.Engine.Construction.Constraints.Qualifier aParameterPosition = global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified;
            if (global::Kerf.Engine.Construction.Constraints.GccEnt.PositionFromString(aPositionType, ref aParameterPosition))
            {
                aKindOfPosition = aParameterPosition;
            }
        }

        // OCCT makes a drawable of a null curve for qcircle with 9 or more arguments; Draw-lite binds nothing
        if (aResult2d is null)
        {
            theDI.SetDrawable(theArgVec[1], null);
            return 0;
        }
        aResult2d.Freeze();
        theDI.SetDrawable(theArgVec[1], new DrawQualifiedCurve2d(aResult2d, aKindOfPosition));
        return 0;
    }

    private static int solutions(DrawInterpreter theDI, DrawResult di, AnalyticCirc2d3Tan theCirTan3, string theName)
    {
        if (!theCirTan3.IsDone)
        {
            return Fail(theDI, "GccAna_Circ2d3Tan is not done");
        }

        string aName = theName + "_";
        for (int aSolId = 1; aSolId <= theCirTan3.SolutionCount; aSolId++)
        {
            var aCircle = new CircleCurve2D(theCirTan3.ThisSolution(aSolId));
            string aSolIdName = aName + aSolId;
            DrawTrSurf.Set(theDI, aSolIdName, aCircle);
            theCirTan3.WhichQualifier(aSolId, out global::Kerf.Engine.Construction.Constraints.Qualifier aQualifier1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQualifier2,
                                      out global::Kerf.Engine.Construction.Constraints.Qualifier aQualifier3);
            di.Append("circle: ").Append(aSolIdName).Append(", ")
              .Append("qualifiers: ").Append(global::Kerf.Engine.Construction.Constraints.GccEnt.PositionToString(aQualifier1)).Append(", ")
              .Append(global::Kerf.Engine.Construction.Constraints.GccEnt.PositionToString(aQualifier2)).Append(", ").Append(global::Kerf.Engine.Construction.Constraints.GccEnt.PositionToString(aQualifier3))
              .Append("\n");

            di.Append("  tangent points: point (parameter on solution, parameter on argument)\n");
            double aParSol, aParArg;
            Point2D aPntSol;
            // the first tangent point
            if (theCirTan3.IsTheSame1(aSolId))
            {
                di.Append("    ").Append("= the solution number ").Append(aSolId).Append(" is equal to the first argument\n");
            }
            else
            {
                theCirTan3.Tangency1(aSolId, out aParSol, out aParArg, out aPntSol);
                string aTanPntIdName = aSolIdName + "_tp_1";
                DrawTrSurf.Set(theDI, aTanPntIdName, aPntSol);
                di.Append("    ").Append(aTanPntIdName).Append(" (").Append(aParSol).Append(", ").Append(aParArg).Append(")\n");
            }
            // the second tangent point
            if (theCirTan3.IsTheSame2(aSolId))
            {
                di.Append("    ").Append("= the solution number ").Append(aSolId).Append(" is equal to the second argument\n");
            }
            else
            {
                theCirTan3.Tangency2(aSolId, out aParSol, out aParArg, out aPntSol);
                string aTanPntIdName = aSolIdName + "_tp_2";
                DrawTrSurf.Set(theDI, aTanPntIdName, aPntSol);
                di.Append("    ").Append(aTanPntIdName).Append(" (").Append(aParSol).Append(", ").Append(aParArg).Append(")\n");
            }
            // the third tangent point
            if (theCirTan3.IsTheSame3(aSolId))
            {
                di.Append("    ").Append("= the solution number ").Append(aSolId).Append(" is equal to the third argument\n");
            }
            else
            {
                theCirTan3.Tangency3(aSolId, out aParSol, out aParArg, out aPntSol);
                string aTanPntIdName = aSolIdName + "_tp_3";
                DrawTrSurf.Set(theDI, aTanPntIdName, aPntSol);
                di.Append("    ").Append(aTanPntIdName).Append(" (").Append(aParSol).Append(", ").Append(aParArg).Append(")");
            }
            if (aSolId != theCirTan3.SolutionCount)
            {
                di.Append("\n");
            }
        }
        return 0;
    }

    // circ2d3Tan cname qcicrle1/qlin1/point1 qcicrle2/qlin2/point2 qcicrle3/qlin3/point3 [tolerance]
    private static int circ2d3Tan(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        int theArgsNb = theArgVec.Count;
        if (theArgsNb < 5)
        {
            return Fail(theDI, "Error: wrong number of arguments");
        }

        var aQCurve1 = theDI.GetDrawable(theArgVec[2]) as DrawQualifiedCurve2d;
        var aQCurve2 = theDI.GetDrawable(theArgVec[3]) as DrawQualifiedCurve2d;
        var aQCurve3 = theDI.GetDrawable(theArgVec[4]) as DrawQualifiedCurve2d;

        Point2D aPoint1 = new(), aPoint2 = new(), aPoint3 = new();
        bool anIsPoint1 = DrawTrSurf.GetPoint2d(theDI, theArgVec[2], ref aPoint1);
        bool anIsPoint2 = DrawTrSurf.GetPoint2d(theDI, theArgVec[3], ref aPoint2);
        bool anIsPoint3 = DrawTrSurf.GetPoint2d(theDI, theArgVec[4], ref aPoint3);

        double aTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (theArgsNb > 5)
        {
            aTolerance = theDI.Atof(theArgVec[5]);
        }

        string aName = theArgVec[1];
        if (aQCurve1 is null) // <point, point, point>
        {
            if (!anIsPoint1 || !anIsPoint2 || !anIsPoint3)
            {
                return Fail(theDI, "Error: wrong points definition");
            }
            return solutions(theDI, di, new AnalyticCirc2d3Tan(aPoint1, aPoint2, aPoint3, aTolerance), aName);
        }

        // the first curve is not NULL
        if (aQCurve2 is null) // <qcircle, point, point> or <qlin, point, point>
        {
            if (!anIsPoint2 || !anIsPoint3)
            {
                return Fail(theDI, "Error: wrong points definition");
            }
            var anAdaptorCurve1 = new CurveAdaptor2D(aQCurve1.Curve);
            if (anAdaptorCurve1.GetType() == Circle)
            {
                var aQualifiedCircle1 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve1.Circle(), aQCurve1.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedCircle1, aPoint2, aPoint3, aTolerance), aName);
            }
            else if (anAdaptorCurve1.GetType() == Line)
            {
                var aQualifiedLin1 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve1.Line(), aQCurve1.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedLin1, aPoint2, aPoint3, aTolerance), aName);
            }
            return Fail(theDI, "Error: wrong curve type");
        }

        // the first and the second curves are not NULL
        if (aQCurve3 is null) // <qcircle, qcircle, point> or <qcircle, qlin, point> or <qlin, qlin, point>
        {
            if (!anIsPoint3)
            {
                return Fail(theDI, "Error: wrong point definition");
            }
            var anAdaptorCurve1 = new CurveAdaptor2D(aQCurve1.Curve);
            var anAdaptorCurve2 = new CurveAdaptor2D(aQCurve2.Curve);
            if (anAdaptorCurve1.GetType() == Circle && anAdaptorCurve2.GetType() == Circle)
            {
                var aQualifiedCircle1 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve1.Circle(), aQCurve1.Position);
                var aQualifiedCircle2 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve2.Circle(), aQCurve2.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedCircle1, aQualifiedCircle2, aPoint3, aTolerance), aName);
            }
            else if (anAdaptorCurve1.GetType() == Circle && anAdaptorCurve2.GetType() == Line)
            {
                var aQualifiedCircle1 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve1.Circle(), aQCurve1.Position);
                var aQualifiedLin2 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve2.Line(), aQCurve2.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedCircle1, aQualifiedLin2, aPoint3, aTolerance), aName);
            }
            else if (anAdaptorCurve1.GetType() == Line && anAdaptorCurve2.GetType() == Line)
            {
                var aQualifiedLin1 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve1.Line(), aQCurve1.Position);
                var aQualifiedLin2 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve2.Line(), aQCurve2.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedLin1, aQualifiedLin2, aPoint3, aTolerance), aName);
            }
            return Fail(theDI, "Error: wrong curve type");
        }

        // the first, the second and the third curves are not NULL
        // <qcircle, qcircle, qcircle> or <qcircle, qcircle, qlin>, <qcircle, qlin, qlin>, <qlin, qlin, qlin>
        {
            var anAdaptorCurve1 = new CurveAdaptor2D(aQCurve1.Curve);
            var anAdaptorCurve2 = new CurveAdaptor2D(aQCurve2.Curve);
            var anAdaptorCurve3 = new CurveAdaptor2D(aQCurve3.Curve);
            if (anAdaptorCurve1.GetType() == Circle && anAdaptorCurve2.GetType() == Circle
                && anAdaptorCurve3.GetType() == Circle)
            {
                var aQualifiedCircle1 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve1.Circle(), aQCurve1.Position);
                var aQualifiedCircle2 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve2.Circle(), aQCurve2.Position);
                var aQualifiedCircle3 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve3.Circle(), aQCurve3.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedCircle1, aQualifiedCircle2, aQualifiedCircle3, aTolerance),
                                 aName);
            }
            if (anAdaptorCurve1.GetType() == Circle && anAdaptorCurve2.GetType() == Circle
                && anAdaptorCurve3.GetType() == Line)
            {
                var aQualifiedCircle1 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve1.Circle(), aQCurve1.Position);
                var aQualifiedCircle2 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve2.Circle(), aQCurve2.Position);
                var aQualifiedLin3 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve3.Line(), aQCurve3.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedCircle1, aQualifiedCircle2, aQualifiedLin3, aTolerance),
                                 aName);
            }
            if (anAdaptorCurve1.GetType() == Circle && anAdaptorCurve2.GetType() == Line
                && anAdaptorCurve3.GetType() == Line)
            {
                var aQualifiedCircle1 = new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(anAdaptorCurve1.Circle(), aQCurve1.Position);
                var aQualifiedLin2 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve2.Line(), aQCurve2.Position);
                var aQualifiedLin3 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve3.Line(), aQCurve3.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedCircle1, aQualifiedLin2, aQualifiedLin3, aTolerance),
                                 aName);
            }
            if (anAdaptorCurve1.GetType() == Line && anAdaptorCurve2.GetType() == Line
                && anAdaptorCurve3.GetType() == Line)
            {
                var aQualifiedLin1 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve1.Line(), aQCurve1.Position);
                var aQualifiedLin2 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve2.Line(), aQCurve2.Position);
                var aQualifiedLin3 = new global::Kerf.Engine.Construction.Constraints.QualifiedLin(anAdaptorCurve3.Line(), aQCurve3.Position);
                return solutions(theDI, di, new AnalyticCirc2d3Tan(aQualifiedLin1, aQualifiedLin2, aQualifiedLin3, aTolerance), aName);
            }
        }

        return Fail(theDI, "Error: wrong curve type");
    }
}
