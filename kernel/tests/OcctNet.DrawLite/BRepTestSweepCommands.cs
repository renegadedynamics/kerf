// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>BRepTest's sweep commands (BRepTest_SweepCommands.cxx): prism, revol, pipe and evolved.</summary>
/// <remarks>
/// <para>Draw records the prism's, the revolution's and the pipe's history (BRepTest_Objects::SetHistory) for
/// <c>savehistory</c> (the pipe's since unit 5W, the prism's and the revolution's since unit 7W).</para>
/// <para><c>pipe</c> runs BRepOffsetAPI_MakePipe (unit 5S1, with unit 5S2's BRepFill_Sweep).</para>
/// <para><c>evolved</c> runs BRepOffsetAPI_MakeEvolved (units 5O2 and 5S2; <c>-v</c>, the volume mode, since unit
/// 7G).</para>
/// <para>The file's other commands (pruled, gener, thrusections, the sweep commands, geompipe and middlepath) are
/// <see cref="BRepTestPipeShellCommands"/>.</para>
/// </remarks>
internal static class BRepTestSweepCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "prism", Prism);
        DrawCxx.Add(theInterpreter, "revol", Revol);
        DrawCxx.Add(theInterpreter, "pipe", Pipe);
        DrawCxx.Add(theInterpreter, "evolved", Evolved);
    }

    // evolved result -s spine -p profile [-solid] [-v] [-a] [-t toler] [-parallel]
    private static int Evolved(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 1)
        {
            di.Append(" evolved result -s spine -p profile [-solid] [-v] [-a] [-t toler] [-parallel] : \n");
            di.Append("   Make evolved profile on spine.\n");
            di.Append("   -solid means make closed solid.\n");
            di.Append("   -v means use alternative algorithm (volume mode).\n");
            di.Append("   -a means referential CS is automatically computed, otherwise global CS is used. \n");
            di.Append("   -t sets the tolerance.\n");
            di.Append("   -parallel turns on parallel execution.\n");
            return 0;
        }

        if (n < 4)
        {
            return 1;
        }
        bool Solid = false;
        bool isVolume = false;
        bool hasToComputeAxes = false;
        double aTolerance = 0.0;
        Shape Base = new();
        Wire Prof = new();
        bool isParallel = true;

        for (int i = 2; i < n; i++)
        {
            if (First(a[i]) != '-')
            {
                di.Append("Error: wrong option!\n");
                return 1;
            }

            if (!Solid && a[i] == "-solid")
            {
                Solid = true;
                continue;
            }

            if (a[i] == "-stm")
            {
                isParallel = false;
                continue;
            }

            switch (a[i].Length > 1 ? a[i][1] : '\0')
            {
                case 's':
                    {
                        Base = theDI.GetShape(a[++i], ShapeType.Wire, false);
                        if (Base.IsNull)
                        {
                            Base = theDI.GetShape(a[i], ShapeType.Face, false);
                        }
                    }
                    break;

                case 'p':
                    {
                        Prof = ShapeCasts.Wire(theDI.GetShape(a[++i], ShapeType.Wire, false));
                    }
                    break;

                case 'v':
                    {
                        isVolume = true;
                    }
                    break;

                case 'a':
                    {
                        hasToComputeAxes = true;
                    }
                    break;

                case 't':
                    {
                        aTolerance = theDI.Atof(a[++i]);
                    }
                    break;

                default:
                    di.Append("Error: Unknown option!\n");
                    break;
            }
        }

        if (Base.IsNull || Prof.IsNull)
        {
            di.Append("spine (face or wire) and profile (wire) are expected\n");
            return 1;
        }

        Shape Volevo = (Shape)new MakeEvolved(Base,
                                                                           Prof,
                                                                           JoinKind.Arc,
                                                                           !hasToComputeAxes,
                                                                           Solid,
                                                                           false,
                                                                           aTolerance,
                                                                           isVolume,
                                                                           isParallel);

        theDI.SetShape(a[1], Volevo);

        return 0;
    }

    // *a[i]: the word's first character, '\0' for an empty word
    private static char First(string theWord) => theWord.Length > 0 ? theWord[0] : '\0';

    private static int Prism(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6)
        {
            return 1;
        }

        Shape @base = theDI.GetShape(a[2]);
        if (@base.IsNull)
        {
            return 1;
        }

        var V = new Vector3D(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));

        bool copy = false;
        bool inf = false;
        bool sinf = false;

        if (n > 6)
        {
            copy = First(a[6]) == 'c' || First(a[6]) == 'C';
            inf = First(a[6]) == 'i' || First(a[6]) == 'I';
            sinf = First(a[6]) == 's' || First(a[6]) == 'S';
        }

        Shape res;

        ApiMakePrism Prism;
        if (inf || sinf)
        {
            Prism = new ApiMakePrism(@base, new Direction3D(V), inf);
        }
        else
        {
            Prism = new ApiMakePrism(@base, V, copy);
        }

        res = Prism.Shape();

        theDI.SetShape(a[1], res);

        // History
        var anArgs = new NCollection_List<Shape>();
        anArgs.Append(@base);
        BRepTestHistoryCommands.SetHistory(theDI, anArgs, Prism);

        return 0;
    }

    private static int Revol(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 10)
        {
            return 1;
        }

        Shape @base = theDI.GetShape(a[2]);
        if (@base.IsNull)
        {
            return 1;
        }

        var P = new Point3D(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        var D = new Direction3D(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));
        var A = new Axis3D(P, D);

        double angle = theDI.Atof(a[9]) * (Math.PI / 180.0);

        bool copy = n > 10;

        var Revol = new ApiMakeRevol(@base, A, angle, copy);

        if (Revol.IsDone)
        {
            Shape res = Revol.Shape();

            theDI.SetShape(a[1], res);

            // History
            var anArgs = new NCollection_List<Shape>();
            anArgs.Append(@base);
            BRepTestHistoryCommands.SetHistory(theDI, anArgs, Revol);
        }
        else
        {
            di.Append("Revol not done \n");
        }

        return 0;
    }

    private static int Pipe(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 1)
        {
            di.Append("pipe result Wire_spine Profile [Mode [Approx]]\n");
            di.Append("Mode = 0 - CorrectedFrenet,\n");
            di.Append("     = 1 - Frenet,\n");
            di.Append("     = 2 - DiscreteTrihedron\n");
            di.Append("Approx - force C1-approximation if result is C0\n");
            return 0;
        }

        if (n > 1 && n < 4)
        {
            return 1;
        }

        Shape Spine = theDI.GetShape(a[2], ShapeType.Wire);
        if (Spine.IsNull)
        {
            return 1;
        }

        Shape Profile = theDI.GetShape(a[3]);
        if (Profile.IsNull)
        {
            return 1;
        }

        Trihedron Mode = Trihedron.IsCorrectedFrenet;
        if (n >= 5)
        {
            int iMode = theDI.Atoi(a[4]);
            if (iMode == 1)
            {
                Mode = Trihedron.IsFrenet;
            }
            else if (iMode == 2)
            {
                Mode = Trihedron.IsDiscreteTrihedron;
            }
        }

        bool ForceApproxC1 = false;
        if (n >= 6)
        {
            ForceApproxC1 = true;
        }

        var PipeBuilder = new ApiMakePipe(ShapeCasts.Wire(Spine), Profile, Mode, ForceApproxC1);
        Shape S = PipeBuilder.Shape();

        theDI.SetShape(a[1], S);

        // Save history of pipe
        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            var aList = new NCollection_List<Shape>();
            aList.Append(Profile);
            aList.Append(Spine);
            BRepTestHistoryCommands.SetHistory(theDI, aList, PipeBuilder);
        }

        return 0;
    }
}
