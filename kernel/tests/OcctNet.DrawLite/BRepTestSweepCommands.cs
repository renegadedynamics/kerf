// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>BRepTest's sweep commands (BRepTest_SweepCommands.cxx): prism and revol.</summary>
/// <remarks>
/// <para>Draw records each sweep's history (BRepTest_Objects::SetHistory) for <c>savehistory</c>, which Draw-lite
/// doesn't have; the history isn't kept.</para>
/// <para>Not here: pipe (BRepOffsetAPI_MakePipe), evolved, pruled, gener, thrusections, the sweep commands
/// (BRepOffsetAPI_MakePipeShell), geompipe and middlepath, which need TKOffset, TKFillet or GeomFill.</para>
/// </remarks>
internal static class BRepTestSweepCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "prism", Prism);
        DrawCxx.Add(theInterpreter, "revol", Revol);
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
}
