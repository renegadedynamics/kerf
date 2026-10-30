// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>BRepTest's sweep commands (BRepTest_SweepCommands.cxx): prism, revol and pipe.</summary>
/// <remarks>
/// <para>Draw records each sweep's history (BRepTest_Objects::SetHistory) for <c>savehistory</c>, which Draw-lite
/// doesn't have; the history isn't kept.</para>
/// <para><c>pipe</c> (BRepOffsetAPI_MakePipe, unit 5S1) throws Standard_NotImplemented until unit 5S2 ports
/// BRepFill_Sweep.</para>
/// <para>Not here: evolved, pruled, gener, thrusections, the sweep commands
/// (BRepOffsetAPI_MakePipeShell), geompipe and middlepath, which need TKOffset, TKFillet or GeomFill.</para>
/// </remarks>
internal static class BRepTestSweepCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "prism", Prism);
        DrawCxx.Add(theInterpreter, "revol", Revol);
        DrawCxx.Add(theInterpreter, "pipe", Pipe);
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

        TopoDS_Shape @base = theDI.GetShape(a[2]);
        if (@base.IsNull())
        {
            return 1;
        }

        var V = new gp_Vec(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));

        bool copy = false;
        bool inf = false;
        bool sinf = false;

        if (n > 6)
        {
            copy = First(a[6]) == 'c' || First(a[6]) == 'C';
            inf = First(a[6]) == 'i' || First(a[6]) == 'I';
            sinf = First(a[6]) == 's' || First(a[6]) == 'S';
        }

        TopoDS_Shape res;

        BRepPrimAPI_MakePrism Prism;
        if (inf || sinf)
        {
            Prism = new BRepPrimAPI_MakePrism(@base, new gp_Dir(V), inf);
        }
        else
        {
            Prism = new BRepPrimAPI_MakePrism(@base, V, copy);
        }

        res = Prism.Shape();

        theDI.SetShape(a[1], res);

        return 0;
    }

    private static int Revol(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 10)
        {
            return 1;
        }

        TopoDS_Shape @base = theDI.GetShape(a[2]);
        if (@base.IsNull())
        {
            return 1;
        }

        var P = new gp_Pnt(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        var D = new gp_Dir(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));
        var A = new gp_Ax1(P, D);

        double angle = theDI.Atof(a[9]) * (Math.PI / 180.0);

        bool copy = n > 10;

        var Revol = new BRepPrimAPI_MakeRevol(@base, A, angle, copy);

        if (Revol.IsDone())
        {
            TopoDS_Shape res = Revol.Shape();

            theDI.SetShape(a[1], res);
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

        TopoDS_Shape Spine = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_WIRE);
        if (Spine.IsNull())
        {
            return 1;
        }

        TopoDS_Shape Profile = theDI.GetShape(a[3]);
        if (Profile.IsNull())
        {
            return 1;
        }

        GeomFill_Trihedron Mode = GeomFill_Trihedron.GeomFill_IsCorrectedFrenet;
        if (n >= 5)
        {
            int iMode = theDI.Atoi(a[4]);
            if (iMode == 1)
            {
                Mode = GeomFill_Trihedron.GeomFill_IsFrenet;
            }
            else if (iMode == 2)
            {
                Mode = GeomFill_Trihedron.GeomFill_IsDiscreteTrihedron;
            }
        }

        bool ForceApproxC1 = false;
        if (n >= 6)
        {
            ForceApproxC1 = true;
        }

        var PipeBuilder = new BRepOffsetAPI_MakePipe(TopoDS.Wire(Spine), Profile, Mode, ForceApproxC1);
        TopoDS_Shape S = PipeBuilder.Shape();

        theDI.SetShape(a[1], S);

        // Draw saves the pipe's history (BRepTest_Objects::SetHistory), which Draw-lite doesn't keep

        return 0;
    }
}
