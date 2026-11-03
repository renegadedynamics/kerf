// Port of: src/Draw/TKTopTest/GeometryTest/GeometryTest_ContinuityCommands.cxx
// Port of: src/Draw/TKTopTest/GeometryTest/GeometryTest_CurveCommands.cxx (getcurvcontinuity)
// Port of: src/Draw/TKTopTest/GeometryTest/GeometryTest_SurfaceCommands.cxx (getsurfcontinuity)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// GeometryTest's continuity commands (unit 5W): <c>surfaceCcontinuity</c>, <c>surfaceGcontinuity</c>,
/// <c>curveCcontinuity</c> and <c>curveGcontinuity</c> (LocalAnalysis, unit 5P), and the continuity of one curve or
/// surface, <c>getcurvcontinuity</c> and <c>getsurfcontinuity</c>.
/// </summary>
internal static class GeometryTestContinuityCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "surfaceCcontinuity", surfaceCcontinuity);
        DrawCxx.Add(theInterpreter, "surfaceGcontinuity", surfaceGcontinuity);
        DrawCxx.Add(theInterpreter, "curveCcontinuity", curveCcontinuity);
        DrawCxx.Add(theInterpreter, "curveGcontinuity", curveGcontinuity);
        DrawCxx.Add(theInterpreter, "getcurvcontinuity", GetCurveContinuity);
        DrawCxx.Add(theInterpreter, "getsurfcontinuity", GetSurfaceContinuity);
    }

    // GeomAbs_Shape's names in its order
    private static readonly string[] aContName = ["C0", "G1", "C1", "G2", "C2", "C3", "CN"];

    private static void InitEpsSurf(out double epsnl, out double epsdis, out double epsangk1, out double epsangk2,
                                    out double epsangn1, out double perce, out double maxlen)
    {
        epsnl = 0.001;
        epsdis = 0.001;
        epsangk1 = 0.001;
        epsangk2 = 0.001;
        epsangn1 = 0.001;
        perce = 0.01;
        maxlen = 10000;
    }

    private static void InitEpsCurv(out double epsnl, out double epsdis, out double epsangk1, out double epsangk2,
                                    out double epsangg1, out double epsangg2, out double percent, out double maxlen)
    {
        epsnl = 0.001;
        epsdis = 0.001;
        epsangk1 = 0.001;
        epsangk2 = 0.001;
        epsangg1 = 0.001;
        epsangg2 = 0.001;
        percent = 0.01;
        maxlen = 10000;
    }

    // Message::SendFail: a line on the interpreter's output (decision 405)
    private static int Fail(DrawInterpreter theDI, string theMessage)
    {
        theDI.Output.WriteLine(theMessage);
        return 1;
    }

    // the parameter lies between the two bounds, whichever way round they are
    private static bool Within(double U, double u1, double u2) => ((U >= u1) && (U <= u2)) || ((U <= u1) && (U >= u2));

    private static int surfaceCcontinuity(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        double U1, U2, V1, V2, u1, u2, v1, v2;
        Continuity TypeCont;
        int ord;
        bool b1, b2, b3, b4;
        if (n < 8)
        {
            return 1;
        }

        InitEpsSurf(out double epsnl, out double epsC0, out double epsC1, out double epsC2, out double epsG1,
                    out double perce, out double maxlen);

        Surface? surf1 = DrawTrSurf.GetSurface(theDI, a[2]);
        if (surf1 is null)
        {
            return 1;
        }

        Surface? surf2 = DrawTrSurf.GetSurface(theDI, a[5]);
        if (surf2 is null)
        {
            return 1;
        }

        U1 = theDI.Atof(a[3]);
        U2 = theDI.Atof(a[6]);
        V1 = theDI.Atof(a[4]);
        V2 = theDI.Atof(a[7]);

        ord = theDI.Atoi(a[1]);

        surf1.Bounds(out u1, out u2, out v1, out v2);
        b1 = Within(U1, u1, u2);
        b3 = Within(V1, v1, v2);

        surf2.Bounds(out u1, out u2, out v1, out v2);
        b2 = Within(U2, u1, u2);
        b4 = Within(V2, v1, v2);

        if (!((b1 && b2) && (b3 && b4)))
        {
            return 1;
        }

        switch (ord)
        {
            case 0:
                TypeCont = C0;
                switch (n)
                {
                    case 10:
                        epsC0 = theDI.Atof(a[9]);
                        goto case 9;
                    case 9:
                        epsnl = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 8 - 10 parameters");
                }
                break;

            case 1:
                TypeCont = C1;
                switch (n)
                {
                    case 11:
                        epsC1 = theDI.Atof(a[10]);
                        goto case 10;
                    case 10:
                        epsC0 = theDI.Atof(a[9]);
                        goto case 9;
                    case 9:
                        epsnl = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 8 - 11 parameters");
                }
                break;

            case 2:
                TypeCont = C2;
                switch (n)
                {
                    case 12:
                        epsC2 = theDI.Atof(a[11]);
                        goto case 11;
                    case 11:
                        epsC1 = theDI.Atof(a[10]);
                        goto case 10;
                    case 10:
                        epsC0 = theDI.Atof(a[9]);
                        goto case 9;
                    case 9:
                        epsnl = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 8 - 12 parameters");
                }
                break;
            default:
                return Fail(theDI, $"Error: invalid value of parameter 1 ({a[1]}): should be 0, 1, or 2");
        }

        var RES = new global::Kerf.Engine.Measurement.SurfaceContinuity(surf1, U1, V1, surf2, U2, V2, TypeCont, epsnl, epsC0, epsC1, epsC2,
                                                      epsG1, perce, maxlen);
        var aSStream = new StringWriter { NewLine = "\n" };
        global::Kerf.Engine.Measurement.LocalAnalysis.Dump(RES, aSStream);
        di.Append(aSStream.ToString());
        return 0;
    }

    private static int surfaceGcontinuity(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        double U1, U2, V1, V2, u1, u2, v1, v2;
        Continuity TypeCont;
        int ord;
        bool b1, b2, b3, b4;

        if (n < 8)
        {
            return 1;
        }
        InitEpsSurf(out double epsnl, out double epsC0, out double epsC1, out double epsC2, out double epsG1,
                    out double perce, out double maxlen);

        Surface? surf1 = DrawTrSurf.GetSurface(theDI, a[2]);
        if (surf1 is null)
        {
            return 1;
        }

        Surface? surf2 = DrawTrSurf.GetSurface(theDI, a[5]);
        if (surf2 is null)
        {
            return 1;
        }

        U1 = theDI.Atof(a[3]);
        U2 = theDI.Atof(a[6]);
        V1 = theDI.Atof(a[4]);
        V2 = theDI.Atof(a[7]);

        ord = theDI.Atoi(a[1]);

        surf1.Bounds(out u1, out u2, out v1, out v2);
        b1 = Within(U1, u1, u2);
        b3 = Within(V1, v1, v2);

        surf2.Bounds(out u1, out u2, out v1, out v2);
        b2 = Within(U2, u1, u2);
        b4 = Within(V2, v1, v2);

        if (!((b1 && b2) && (b3 && b4)))
        {
            return 1;
        }

        switch (ord)
        {
            case 1:
                TypeCont = G1;
                switch (n)
                {
                    case 11:
                        epsG1 = theDI.Atof(a[10]);
                        goto case 10;
                    case 10:
                        epsC0 = theDI.Atof(a[9]);
                        goto case 9;
                    case 9:
                        epsnl = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 8 - 11 parameters");
                }
                break;

            case 2:
                TypeCont = G2;
                switch (n)
                {
                    case 13:
                        maxlen = theDI.Atof(a[12]);
                        goto case 12;
                    case 12:
                        perce = theDI.Atof(a[11]);
                        goto case 11;
                    case 11:
                        epsG1 = theDI.Atof(a[10]);
                        goto case 10;
                    case 10:
                        epsC0 = theDI.Atof(a[9]);
                        goto case 9;
                    case 9:
                        epsnl = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 8 - 13 parameters");
                }
                break;
            default:
                return Fail(theDI, $"Error: invalid value of parameter 1 ({a[1]}): should be 1 or 2");
        }

        var RES = new global::Kerf.Engine.Measurement.SurfaceContinuity(surf1, U1, V1, surf2, U2, V2, TypeCont, epsnl, epsC0, epsC1, epsC2,
                                                      epsG1, perce, maxlen);
        var aSStream = new StringWriter { NewLine = "\n" };
        global::Kerf.Engine.Measurement.LocalAnalysis.Dump(RES, aSStream);
        di.Append(aSStream.ToString());
        return 0;
    }

    private static int curveGcontinuity(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        double U1, U2, u1, u2;
        Continuity TypeCont;
        int ord;
        bool b1, b2;
        if (n < 6)
        {
            return 1;
        }

        Curve3D? curv1 = DrawTrSurf.GetCurve(theDI, a[2]);
        if (curv1 is null)
        {
            return 1;
        }
        Curve3D? curv2 = DrawTrSurf.GetCurve(theDI, a[4]);
        if (curv2 is null)
        {
            return 1;
        }

        U1 = theDI.Atof(a[3]);
        U2 = theDI.Atof(a[5]);

        ord = theDI.Atoi(a[1]);

        u1 = curv1.FirstParameter();
        u2 = curv1.LastParameter();
        b1 = Within(U1, u1, u2);

        u1 = curv2.FirstParameter();
        u2 = curv2.LastParameter();
        b2 = Within(U2, u1, u2);

        if (!(b1 && b2))
        {
            return 1;
        }

        InitEpsCurv(out double epsnl, out double epsC0, out double epsC1, out double epsC2, out double epsG1,
                    out double epsG2, out double percent, out double maxlen);
        switch (ord)
        {
            case 1:
                TypeCont = G1;
                switch (n)
                {
                    case 9:
                        epsG1 = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        epsC0 = theDI.Atof(a[7]);
                        goto case 7;
                    case 7:
                        epsnl = theDI.Atof(a[6]);
                        goto case 6;
                    case 6:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 6 - 8 parameters");
                }
                break;
            case 2:
                TypeCont = G2;
                switch (n)
                {
                    case 12:
                        maxlen = theDI.Atof(a[11]);
                        goto case 11;
                    case 11:
                        percent = theDI.Atof(a[10]);
                        goto case 10;
                    case 10:
                        epsG2 = theDI.Atof(a[9]);
                        goto case 9;
                    case 9:
                        epsG1 = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        epsC0 = theDI.Atof(a[7]);
                        goto case 7;
                    case 7:
                        epsnl = theDI.Atof(a[6]);
                        goto case 6;
                    case 6:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 6 - 12 parameters");
                }
                break;

            default:
                return Fail(theDI, $"Error: invalid value of parameter 1 ({a[1]}): should be 1 or 2");
        }

        var RES = new global::Kerf.Engine.Measurement.CurveContinuity(curv1, U1, curv2, U2, TypeCont, epsnl, epsC0, epsC1, epsC2, epsG1,
                                                    epsG2, percent, maxlen);
        var aSStream = new StringWriter { NewLine = "\n" };
        global::Kerf.Engine.Measurement.LocalAnalysis.Dump(RES, aSStream);
        di.Append(aSStream.ToString());
        return 0;
    }

    private static int curveCcontinuity(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        double U1, U2, u1, u2;
        Continuity TypeCont;
        int ord;
        bool b1, b2;
        if (n < 6)
        {
            return 1;
        }

        Curve3D? curv1 = DrawTrSurf.GetCurve(theDI, a[2]);
        if (curv1 is null)
        {
            return 1;
        }
        Curve3D? curv2 = DrawTrSurf.GetCurve(theDI, a[4]);
        if (curv2 is null)
        {
            return 1;
        }

        U1 = theDI.Atof(a[3]);
        U2 = theDI.Atof(a[5]);

        ord = theDI.Atoi(a[1]);

        u1 = curv1.FirstParameter();
        u2 = curv1.LastParameter();
        b1 = Within(U1, u1, u2);

        u1 = curv2.FirstParameter();
        u2 = curv2.LastParameter();
        b2 = Within(U2, u1, u2);

        if (!(b1 && b2))
        {
            return 1;
        }

        InitEpsCurv(out double epsnl, out double epsC0, out double epsC1, out double epsC2, out double epsG1,
                    out double epsG2, out double percent, out double maxlen);
        switch (ord)
        {
            case 0:
                TypeCont = C0;
                switch (n)
                {
                    case 8:
                        epsC0 = theDI.Atof(a[7]);
                        goto case 7;
                    case 7:
                        epsnl = theDI.Atof(a[6]);
                        goto case 6;
                    case 6:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 6 - 8 parameters");
                }
                break;
            case 1:
                TypeCont = C1;
                switch (n)
                {
                    case 9:
                        epsC1 = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        epsC0 = theDI.Atof(a[7]);
                        goto case 7;
                    case 7:
                        epsnl = theDI.Atof(a[6]);
                        goto case 6;
                    case 6:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 6 - 9 parameters");
                }
                break;

            case 2:
                TypeCont = C2;
                switch (n)
                {
                    case 10:
                        epsC2 = theDI.Atof(a[9]);
                        goto case 9;
                    case 9:
                        epsC1 = theDI.Atof(a[8]);
                        goto case 8;
                    case 8:
                        epsC0 = theDI.Atof(a[7]);
                        goto case 7;
                    case 7:
                        epsnl = theDI.Atof(a[6]);
                        goto case 6;
                    case 6:
                        break;
                    default:
                        return Fail(theDI, "Error: invalid number of arguments: expected to get 6 - 10 parameters");
                }
                break;

            default:
                return Fail(theDI, $"Error: invalid value of parameter 1 ({a[1]}): should be 0, 1, or 2");
        }

        var RES = new global::Kerf.Engine.Measurement.CurveContinuity(curv1, U1, curv2, U2, TypeCont, epsnl, epsC0, epsC1, epsC2, epsG1,
                                                    epsG2, percent, maxlen);
        var aSStream = new StringWriter { NewLine = "\n" };
        global::Kerf.Engine.Measurement.LocalAnalysis.Dump(RES, aSStream);
        di.Append(aSStream.ToString());
        return 0;
    }

    // GetCurveContinuity: the continuity of the given curve (GeometryTest_CurveCommands.cxx)
    private static int GetCurveContinuity(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgv)
    {
        if (theArgv.Count != 2)
        {
            theResult.Append("Use: getcurvcontinuity {curve or 2dcurve} \n");
            return 1;
        }

        Curve2D? GC2d;
        Curve3D? GC3d = DrawTrSurf.GetCurve(theDI, theArgv[1]);
        if (GC3d is null)
        {
            GC2d = DrawTrSurf.GetCurve2d(theDI, theArgv[1]);
            if (GC2d is null)
            {
                theResult.Append("Argument is not a 2D or 3D curve!\n");
                return 1;
            }
            else
            {
                theResult.Append(theArgv[1]).Append(" has ").Append(aContName[(int)GC2d.Continuity()]).Append(" continuity.\n");
            }
        }
        else
        {
            theResult.Append(theArgv[1]).Append(" has ").Append(aContName[(int)GC3d.Continuity()]).Append(" continuity.\n");
        }

        return 0;
    }

    // GetSurfaceContinuity: the continuity of the given surface (GeometryTest_SurfaceCommands.cxx)
    private static int GetSurfaceContinuity(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgv)
    {
        if (theArgv.Count != 2)
        {
            theResult.Append("Use: getsurfcontinuity surface\n");
            return 1;
        }

        Surface? GS1 = DrawTrSurf.GetSurface(theDI, theArgv[1]);
        if (GS1 is null)
        {
            theResult.Append("Argument is not a surface!\n");
            return 1;
        }

        theResult.Append(theArgv[1]).Append(" has ").Append(aContName[(int)GS1.Continuity()]).Append(" continuity.\n");

        return 0;
    }
}
