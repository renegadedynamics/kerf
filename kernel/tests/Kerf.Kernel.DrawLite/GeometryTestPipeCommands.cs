// Port of: src/Draw/TKTopTest/GeometryTest/GeometryTest_SurfaceCommands.cxx (sweep, tuyau, partuyau)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Construction.Surfaces.Trihedron;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// GeometryTest_SurfaceCommands.cxx's pipes (unit 5L2): <c>sweep</c> (a section swept along a path with a chosen
/// trihedron, GeomFill_Pipe) and <c>tuyau</c> (pipes of constant radius, of a constant or evolving section, through N
/// sections, or on an edge between two guide lines); <c>partuyau</c> is <c>tuyau</c> under another name (unit 5W).
/// </summary>
/// <remarks>
/// The file's other commands (<c>ruled</c>, <c>appsurf</c>, <c>fillcurves</c>, ...) are not registered here. A null
/// curve where OCCT dereferences it without a check (sweep -CN's path and section) is a .NET exception, a Tcl error.
/// </remarks>
internal static class GeometryTestPipeCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "sweep", Sweep);
        DrawCxx.Add(theInterpreter, "tuyau", Tuyau);
        DrawCxx.Add(theInterpreter, "partuyau", Tuyau);
    }

    private static int Sweep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        Trihedron Option = Trihedron.IsCorrectedFrenet;
        int ipath = 2, isection = 4, NbSeg = 30, MaxDegree = 10;
        double Tol = 1.0e-4;

        if (n < 4)
        {
            return 1;
        }

        if (a[2] == "-FX")
        {
            ipath = 3;
            Option = Trihedron.IsFixed;
        }
        if (a[2] == "-FR")
        {
            ipath = 3;
            Option = Trihedron.IsFrenet;
        }
        if (a[2] == "-CF")
        {
            ipath = 3;
            Option = Trihedron.IsCorrectedFrenet;
        }
        if (a[2] == "-CN")
        {
            ipath = 3;
            isection = 7;
            Option = Trihedron.IsConstantNormal;
        }
        if (a[2] == "-DX")
        {
            ipath = 3;
            isection = 5;
            Option = Trihedron.IsDarboux;
        }

        SurfacePipe Pipe = new();
        Pipe.GenerateParticularCase(true);

        if (Option == Trihedron.IsDarboux)
        {
            Curve2D? path = DrawTrSurf.GetCurve2d(theDI, a[ipath]);
            if (path is null)
            {
                return 1;
            }

            Surface? Support = DrawTrSurf.GetSurface(theDI, a[ipath + 1]);
            if (Support is null)
            {
                return 1;
            }

            Curve3D? firstS = DrawTrSurf.GetCurve(theDI, a[ipath + 2]);
            if (firstS is null)
            {
                return 1;
            }

            Pipe.Init(path, Support, firstS);
        }
        else if (Option == Trihedron.IsConstantNormal)
        {
            Direction3D D = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
            Curve3D path = DrawTrSurf.GetCurve(theDI, a[6])!;
            Curve3D firstS = DrawTrSurf.GetCurve(theDI, a[7])!;
            Pipe.Init(path, firstS, D);
        }
        else
        {
            Curve3D? path = DrawTrSurf.GetCurve(theDI, a[ipath]);
            if (path is null)
            {
                return 1;
            }

            Curve3D? firstS = DrawTrSurf.GetCurve(theDI, a[ipath + 1]);
            if (firstS is null)
            {
                return 1;
            }

            Pipe.Init(path, firstS, Option);
        }

        if (n >= isection + 2)
        {
            MaxDegree = theDI.Atoi(a[isection + 1]);
            if (n > isection + 2)
            {
                NbSeg = theDI.Atoi(a[isection + 2]);
            }
        }

        Pipe.Perform(Tol, false, C2, MaxDegree, NbSeg);

        if (!Pipe.IsDone)
        {
            di.Append("GeomFill_Pipe cannot make a surface\n");
            return 1;
        }

        double Accuracy = Pipe.ErrorOnSurf();
        di.Append("Accuracy of approximation = ").Append(Accuracy).Append("\n");

        DrawTrSurf.Set(theDI, a[1], Pipe.Surface());
        return 0;
    }

    private static int Tuyau(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Continuity Cont = C2;
        SurfacePipe Pipe = new();
        Pipe.GenerateParticularCase(true);

        bool Option_NS = false;
        int indice_path = 2, narg = n;
        if (a[1] == "-NS")
        {
            Option_NS = true;
            indice_path++;
            narg--;
        }
        Curve3D? path = DrawTrSurf.GetCurve(theDI, a[indice_path]);
        if (path is null)
        {
            return 1;
        }

        int isect = indice_path + 1;
        Curve3D? firstS = DrawTrSurf.GetCurve(theDI, a[isect]);
        if (firstS is null)
        {
            if (narg == 4)
            {
                // tuyau a rayon constant.
                Pipe.Init(path, theDI.Atof(a[isect]));
            }
            else
            {
                return 1;
            }
        }
        else
        {
            if (narg == 4)
            {
                // tuyau a section constante.
                Pipe.Init(path, firstS, Trihedron.IsCorrectedFrenet);
            }
            else
            {
                if (narg == 5 && !Option_NS)
                {
                    // tuyau a section evolutive
                    Curve3D lastS = DrawTrSurf.GetCurve(theDI, a[isect + 1])!;
                    Cont = C2;
                    Pipe.Init(path, firstS, lastS);
                }
                else
                {
                    if (narg == 6 && !Option_NS && theDI.Atof(a[5]) != 0)
                    {
                        Curve3D lastS = DrawTrSurf.GetCurve(theDI, a[isect + 1])!;
                        Cont = C2;
                        Pipe = new SurfacePipe(path, firstS, lastS, theDI.Atof(a[5]));
                        Pipe.Perform(true);
                        Surface? aSurface = null;
                        if (Pipe.IsDone)
                        {
                            aSurface = Pipe.Surface();
                        }
                        DrawTrSurf.Set(theDI, a[1], aSurface);
                        return 0;
                    }
                    // tuyau a N sections, N>=2
                    NCollection_Sequence<Curve3D> Seq = new();
                    Seq.Clear();
                    Seq.Append(firstS);
                    for (int i = isect + 1; i < n; i++)
                    {
                        Curve3D nextS = DrawTrSurf.GetCurve(theDI, a[i])!;
                        Seq.Append(nextS);
                    }
                    Cont = C2;
                    Pipe.Init(path, Seq);
                }
            }
        }

        Pipe.Perform(1.0e-4, false, Cont);
        if (!Pipe.IsDone)
        {
            di.Append("Error: GeomFill_Pipe cannot make a surface\n");
            return 0;
        }

        double Accuracy = Pipe.ErrorOnSurf();
        di.Append("Accuracy of approximation = ").Append(Accuracy).Append("\n");

        DrawTrSurf.Set(theDI, a[indice_path - 1], Pipe.Surface());

        return 0;
    }
}
