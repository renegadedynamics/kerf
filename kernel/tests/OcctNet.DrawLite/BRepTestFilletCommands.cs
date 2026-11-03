// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Fillets;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using Occt;
using static Kerf.Engine.Fillets.FilletShape;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's fillet commands (BRepTest_FilletCommands.cxx, unit 7N3): <c>continuityblend</c>, <c>tolblend</c>,
/// <c>blend</c>, <c>checkhist</c>, <c>mkevol</c>, <c>updatevol</c>, <c>buildevol</c>, <c>bfuseblend</c> and
/// <c>bcutblend</c>, written as OCCT writes them; the rolling-ball commands are in the file's other part (unit 7G).
/// </summary>
/// <remarks>
/// <para>OCCT keeps the tolerances, the continuity, the last <c>blend</c>'s maker (<c>Rakk</c>, read by
/// <c>checkhist</c>) and the evolving fillet's maker and result name (<c>Rake</c>, <c>name</c>) in file-scope
/// statics; Draw-lite keeps them per interpreter (decision 1144), so a corpus case starts from a fresh DRAWEXE's
/// state.</para>
/// <para>Left out: <c>blend1</c> (FilletSurf_Builder).</para>
/// </remarks>
internal static partial class BRepTestFilletCommands
{
    private sealed class FilletState
    {
        public double tesp = 1.0e-4;
        public double t3d = 1.0e-4;
        public double t2d = 1.0e-5;
        public double ta = 1.0e-2;
        public double fl = 1.0e-3;
        public double tapp_angle = 1.0e-2;
        public Continuity blend_cont = C1;
        public MakeFillet? Rakk;
        public MakeFillet? Rake;
        public string name = "";
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, FilletState> s_states = new();

    private static FilletState State(DrawInterpreter theDI) => s_states.GetValue(theDI, static _ => new FilletState());

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "continuityblend", contblend);
        DrawCxx.Add(theInterpreter, "tolblend", tolblend);
        DrawCxx.Add(theInterpreter, "blend", BLEND);
        DrawCxx.Add(theInterpreter, "checkhist", CheckHist);
        DrawCxx.Add(theInterpreter, "mkevol", MKEVOL);
        DrawCxx.Add(theInterpreter, "updatevol", UPDATEVOL);
        DrawCxx.Add(theInterpreter, "buildevol", BUILDEVOL);
        DrawCxx.Add(theInterpreter, "bfuseblend", boptopoblend);
        DrawCxx.Add(theInterpreter, "bcutblend", boptopoblend);
        RegisterRollingBallCommands(theInterpreter);
    }

    private static bool EqualsNoCase(string theA, string theB) => string.Equals(theA, theB, StringComparison.OrdinalIgnoreCase);

    private static int contblend(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        FilletState s = State(theDI);
        int narg = a.Count;
        if (narg == 1)
        {
            di.Append("tolerance angular of approximation  : ").Append(s.tapp_angle).Append("\n");
            di.Append("internal continuity                 : ");
            switch (s.blend_cont)
            {
                case C0:
                    di.Append("C0\n");
                    break;
                case C1:
                    di.Append("C1\n");
                    break;
                case C2:
                    di.Append("C2\n");
                    break;
                default:
                    break;
            }
            return 0;
        }
        if (narg > 3)
        {
            return 1;
        }
        if (narg == 3)
        {
            s.tapp_angle = Math.Abs(theDI.Atof(a[2]));
        }
        char c = a[1].Length > 1 ? a[1][1] : '\0';
        switch (c)
        {
            case '0':
                s.blend_cont = C0;
                break;
            case '2':
                s.blend_cont = C2;
                break;
            default:
                s.blend_cont = C1;
                break;
        }
        return 0;
    }

    private static void printtolblend(FilletState s, DrawResult di)
    {
        di.Append("tolerance ang : ").Append(s.ta).Append("\n");
        di.Append("tolerance 3d  : ").Append(s.t3d).Append("\n");
        di.Append("tolerance 2d  : ").Append(s.t2d).Append("\n");
        di.Append("fleche        : ").Append(s.fl).Append("\n");

        di.Append("tolblend ").Append(s.ta).Append(" ").Append(s.t3d).Append(" ").Append(s.t2d).Append(" ").Append(s.fl).Append("\n");
    }

    private static int tolblend(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        FilletState s = State(theDI);
        int narg = a.Count;
        if (narg == 1)
        {
            printtolblend(s, di);
            return 0;
        }
        if (narg == 5)
        {
            s.ta = theDI.Atof(a[1]);
            s.t3d = theDI.Atof(a[2]);
            s.t2d = theDI.Atof(a[3]);
            s.fl = theDI.Atof(a[4]);
            return 0;
        }
        return 1;
    }

    /// <summary>The 3D tolerance <c>tolblend</c> sets, which the rolling-ball commands read (OCCT's <c>t3d</c>).</summary>
    private static double T3d(DrawInterpreter theDI) => State(theDI).t3d;

    private static int BLEND(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        FilletState s = State(theDI);
        s.Rakk = null;
        printtolblend(s, di);
        int narg = a.Count;
        if (narg < 5)
        {
            return 1;
        }
        Shape V = theDI.GetShape(a[2]);
        if (V.IsNull)
        {
            return 1;
        }
        FilletShape FSh = FilletShape.Rational;
        if (narg % 2 == 0)
        {
            if (EqualsNoCase(a[narg - 1], "Q"))
            {
                FSh = FilletShape.QuasiAngular;
            }
            else if (EqualsNoCase(a[narg - 1], "P"))
            {
                FSh = FilletShape.Polynomial;
            }
        }
        MakeFillet Rakk = new(V, FSh);
        s.Rakk = Rakk;
        Rakk.SetParams(s.ta, s.tesp, s.t2d, s.t3d, s.t2d, s.fl);
        Rakk.SetContinuity(s.blend_cont, s.tapp_angle);
        int nbedge = 0;
        for (int ii = 1; ii < (narg - 1) / 2; ii++)
        {
            double Rad = theDI.Atof(a[2 * ii + 1]);
            Shape aLocalEdge = theDI.GetShape(a[2 * ii + 2], ShapeType.Edge);
            Edge E = ShapeCasts.Edge(aLocalEdge);
            if (!E.IsNull)
            {
                Rakk.Add(Rad, E);
                nbedge++;
            }
        }
        if (nbedge == 0)
        {
            return 1;
        }
        Rakk.Build();
        if (!Rakk.IsDone)
        {
            return 1;
        }

        // Save history for fillet
        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            NCollection_List<Shape> anArg = new();
            anArg.Append(V);
            BRepTestHistoryCommands.SetHistory(theDI, anArg, Rakk);
        }

        Shape res = Rakk.Shape();
        theDI.SetShape(a[1], res);
        return 0;
    }

    private static void PrintHist(DrawInterpreter theDI, in Shape S, IEnumerable<Shape> It, ref int nbgen)
    {
        Compound C = new();
        BrepBuilder B = new();
        B.MakeCompound(ref C);
        B.Add(C, S);
        string localname;
        if (nbgen < 10)
        {
            localname = "generated_00" + nbgen++;
        }
        else if (nbgen < 100)
        {
            localname = "generated_0" + nbgen++;
        }
        else
        {
            localname = "generated_" + nbgen++;
        }
        foreach (Shape aS in It)
        {
            B.Add(C, aS);
        }
        theDI.SetShape(localname, C);
    }

    private static int CheckHist(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        MakeFillet? Rakk = State(theDI).Rakk;
        if (Rakk is null)
        {
            di.Append("No active Builder\n");
            return 1;
        }
        if (!Rakk.IsDone)
        {
            di.Append("Active Builder Not Done\n");
            return 1;
        }
        int nbc = Rakk.ContourCount;
        int nbgen = 0;
        for (int i = 1; i <= nbc; i++)
        {
            Shape curshape = Rakk.FirstVertex(i);
            PrintHist(theDI, curshape, Rakk.Generated(curshape), ref nbgen);
            int nbe = Rakk.NbEdges(i);
            for (int j = 1; j <= nbe; j++)
            {
                curshape = Rakk.Edge(i, j);
                PrintHist(theDI, curshape, Rakk.Generated(curshape), ref nbgen);
            }
            curshape = Rakk.LastVertex(i);
            PrintHist(theDI, curshape, Rakk.Generated(curshape), ref nbgen);
        }
        di.Append("foreach g [lsort [dir gen*]] { wclick; puts [dname $g]; donl $g; }\n");
        return 0;
    }

    private static int MKEVOL(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        FilletState s = State(theDI);
        s.Rake = null;
        printtolblend(s, di);
        int narg = a.Count;
        if (narg < 3)
        {
            return 1;
        }
        Shape V = theDI.GetShape(a[2]);
        MakeFillet Rake = new(V);
        s.Rake = Rake;
        Rake.SetParams(s.ta, s.tesp, s.t2d, s.t3d, s.t2d, s.fl);
        Rake.SetContinuity(s.blend_cont, s.tapp_angle);
        if (narg == 4)
        {
            FilletShape FSh = FilletShape.Rational;
            if (EqualsNoCase(a[3], "Q"))
            {
                FSh = FilletShape.QuasiAngular;
            }
            else if (EqualsNoCase(a[3], "P"))
            {
                FSh = FilletShape.Polynomial;
            }
            Rake.SetFilletShape(FSh);
        }
        s.name = a[1];
        return 0;
    }

    private static int UPDATEVOL(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        MakeFillet? Rake = State(theDI).Rake;
        if (Rake is null)
        {
            di.Append("MakeFillet not initialized\n");
            return 1;
        }
        int narg = a.Count;
        if (narg % 2 != 0 || narg < 4)
        {
            return 1;
        }
        NCollection_Array1<Point2D> uandr = new(1, (narg / 2) - 1);
        Shape aLocalEdge = theDI.GetShape(a[1], ShapeType.Edge);
        Edge E = ShapeCasts.Edge(aLocalEdge);
        for (int ii = 1; ii <= (narg / 2) - 1; ii++)
        {
            double Par = theDI.Atof(a[2 * ii]);
            double Rad = theDI.Atof(a[2 * ii + 1]);
            uandr.SetValue(ii, new Point2D(Par, Rad));
        }
        Rake.Add(uandr, E);
        return 0;
    }

    private static int BUILDEVOL(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        FilletState s = State(theDI);
        MakeFillet? Rake = s.Rake;
        if (Rake is null)
        {
            di.Append("MakeFillet not initialized\n");
            return 1;
        }
        // OCCT deletes Rake after Build, done or not (an exception of Build leaves it)
        Rake.Build();
        s.Rake = null;
        if (Rake.IsDone)
        {
            Shape result = Rake.Shape();
            theDI.SetShape(s.name, result);
            return 0;
        }
        return 1;
    }

    //**********************************************
    // bfuse or bcut and then blend the section
    //**********************************************

    private static int boptopoblend(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        FilletState s = State(theDI);
        printtolblend(s, di);
        int narg = a.Count;
        if (narg < 5)
        {
            // Message::SendFail (decision 405)
            theDI.Output.WriteLine("Use <command name> result shape1 shape2 radius [-d]");
            return 1;
        }

        bool fuse = a[0] == "bfuseblend";
        Shape S1 = theDI.GetShape(a[2]);
        Shape S2 = theDI.GetShape(a[3]);
        if (S1.IsNull || S2.IsNull)
        {
            theDI.Output.WriteLine(" Null shapes are not allowed");
            return 1;
        }
        double Rad = theDI.Atof(a[4]);
        bool isDebug = narg == 6 && a[5] == "-d";

        PaveFiller theDSFiller = new();
        NCollection_List<Shape> aLS = new();
        aLS.Append(S1);
        aLS.Append(S2);
        theDSFiller.SetArguments(aLS);
        // the Draw_ProgressIndicator left out
        theDSFiller.Perform();
        if (theDSFiller.HasErrors)
        {
            theDI.Output.WriteLine("Check types of the arguments, please");
            return 1;
        }

        BooleanOperation pBuilder = fuse
            ? new Fuse(S1, S2, theDSFiller)
            : new Cut(S1, S2, theDSFiller, true);

        if (!pBuilder.IsDone)
        {
            theDI.Output.WriteLine("boolean operation not done HasErrors()=" + (pBuilder.HasErrors ? 1 : 0));
            return 1;
        }

        Shape ResultOfBop = pBuilder.Shape();

        if (isDebug)
        {
            string aBuff = a[1] + "_bop";
            theDI.SetShape(aBuff, ResultOfBop);
            di.Append("Intermediate result of BOP-operation is saved to \"").Append(aBuff).Append("\" variable\n");
        }

        SectionOperation aSection = new(S1, S2, theDSFiller);
        Shape theSection = aSection.Shape();

        if (isDebug)
        {
            string aBuff = a[1] + "_sec";
            theDI.SetShape(aBuff, theSection);
            di.Append("Intermediate bopsection result is saved to \"").Append(aBuff).Append("\" variable\n");
        }

        Compound result = new();
        BrepBuilder BB = new();
        BB.MakeCompound(ref result);

        Explorer Explo = new(ResultOfBop, ShapeType.Solid);
        for (; Explo.More(); Explo.Next())
        {
            Shape aSolid = Explo.Current();

            MakeFillet Blender = new(aSolid);
            Blender.SetParams(s.ta, s.tesp, s.t2d, s.t3d, s.t2d, s.fl);
            Blender.SetContinuity(s.blend_cont, s.tapp_angle);

            Explorer expsec = new(theSection, ShapeType.Edge);
            for (; expsec.More(); expsec.Next())
            {
                Edge anEdge = ShapeCasts.Edge(expsec.Current());
                Blender.Add(Rad, anEdge);
            }

            Blender.Build();
            if (Blender.IsDone)
            {
                BB.Add(result, Blender.Shape());
            }
            else
            {
                di.Append("Error: Cannot find the result of BLEND-operation."
                          + " The result of BOP operation will be returned.\n");
                BB.Add(result, aSolid);
            }
        }

        theDI.SetShape(a[1], result);
        return 0;
    }
}
