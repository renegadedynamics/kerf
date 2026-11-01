// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Occt;
using static Occt.BRepFeat_Status;
using static Occt.Standard;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's feature commands (BRepTest_FeatureCommands.cxx): <c>localope</c>, <c>hole</c>, <c>firsthole</c>,
/// <c>holend</c>, <c>blindhole</c>, <c>holecontrol</c>, <c>wprism</c>, <c>fprism</c>, <c>wrotate</c>,
/// <c>frotate</c>, <c>splitshape</c>, <c>glue</c>, <c>featprism</c>, <c>featrevol</c>, <c>featpipe</c>,
/// <c>featdprism</c>, <c>featlf</c>, <c>featrf</c>, <c>addslide</c>, <c>featperform</c>, <c>featperformval</c> and
/// <c>endedges</c>, as OCCT writes them.
/// </summary>
/// <remarks>
/// <para>OCCT keeps the feature makers (and the hole, its control flag, the prism's arguments and which features are
/// defined) in function-scope statics, from a <c>feat*</c> command to <c>featperform</c>; Draw-lite keeps them per
/// interpreter (decision 1055), so a corpus case starts from a fresh DRAWEXE's state.</para>
/// <para>Left out: <c>fillet</c> and <c>bossage</c> (BRepFilletAPI_MakeFillet, unit 7N3); the offset commands of the
/// same file (unit 7G); the interactive picks of <c>glue</c> and <c>splitshape</c> (names starting with a dot).</para>
/// </remarks>
internal static partial class BRepTestFeatureCommands
{
    private sealed class FeatureState
    {
        public readonly BRepFeat_MakeCylindricalHole Hole = new();
        public bool WithControl = true;
        public readonly BRepFeat_MakePrism Prism = new();
        public readonly BRepFeat_MakeDPrism DPrism = new();
        public readonly BRepFeat_MakeRevol Revol = new();
        public readonly BRepFeat_MakePipe Pipe = new();
        public readonly BRepFeat_MakeLinearForm LinearForm = new();
        public readonly BRepFeat_MakeRevolutionForm RevolutionForm = new();
        // Input shapes for Prism, DPrism, Revol, Pipe
        public TopoDS_Shape theSbase = new(), thePbase = new();
        public TopoDS_Face theSkface = new();
        public bool dprdef, prdef, rvdef, pidef, lfdef, rfdef;
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, FeatureState> s_states = new();

    private static FeatureState State(DrawInterpreter theDI) => s_states.GetValue(theDI, static _ => new FeatureState());

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "localope", Loc);
        DrawCxx.Add(theInterpreter, "hole", HOLE1);
        DrawCxx.Add(theInterpreter, "firsthole", HOLE2);
        DrawCxx.Add(theInterpreter, "holend", HOLE3);
        DrawCxx.Add(theInterpreter, "blindhole", HOLE4);
        DrawCxx.Add(theInterpreter, "holecontrol", CONTROL);
        DrawCxx.Add(theInterpreter, "wprism", PRW);
        DrawCxx.Add(theInterpreter, "fprism", PRF);
        DrawCxx.Add(theInterpreter, "wrotate", ROW);
        DrawCxx.Add(theInterpreter, "frotate", ROF);
        DrawCxx.Add(theInterpreter, "splitshape", SPLS);
        DrawCxx.Add(theInterpreter, "glue", GLU);
        DrawCxx.Add(theInterpreter, "featprism", DEFIN);
        DrawCxx.Add(theInterpreter, "featrevol", DEFIN);
        DrawCxx.Add(theInterpreter, "featpipe", DEFIN);
        DrawCxx.Add(theInterpreter, "featdprism", DEFIN);
        DrawCxx.Add(theInterpreter, "featlf", DEFIN);
        DrawCxx.Add(theInterpreter, "featrf", DEFIN);
        DrawCxx.Add(theInterpreter, "addslide", ADD);
        DrawCxx.Add(theInterpreter, "featperform", PERF);
        DrawCxx.Add(theInterpreter, "featperformval", PERF);
        DrawCxx.Add(theInterpreter, "endedges", BOSS);
    }

    private static bool EqualsNoCase(string theA, string theB) => string.Equals(theA, theB, StringComparison.OrdinalIgnoreCase);

    private static bool IsAlphabetic(char theChar) => theChar is >= 'a' and <= 'z' or >= 'A' and <= 'Z';

    private static char At(string theWord, int theIndex) => theIndex < theWord.Length ? theWord[theIndex] : '\0';

    private static void Print(DrawResult di, BRepFeat_Status St)
    {
        di.Append("  Error Status : ");
        switch (St)
        {
            case BRepFeat_NoError:
                di.Append("No error");
                break;
            case BRepFeat_InvalidPlacement:
                di.Append("Invalid placement");
                break;
            case BRepFeat_HoleTooLong:
                di.Append("Hole too long");
                break;
        }
    }

    private static int Loc(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 6)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[2]);
        TopoDS_Shape T = theDI.GetShape(a[3]);

        bool Fuse;
        if (EqualsNoCase("F", a[4]))
        {
            Fuse = true;
        }
        else if (EqualsNoCase("C", a[4]))
        {
            Fuse = false;
        }
        else
        {
            return 1;
        }

        NCollection_List<TopoDS_Shape> LF = new();
        for (int i = 0; i <= narg - 6; i++)
        {
            TopoDS_Shape aLocalShape = theDI.GetShape(a[i + 5], TopAbs_FACE);
            LF.Append(aLocalShape);
        }

        NCollection_List<TopoDS_Shape> parts = new();
        BRepFeat_Builder BLoc = new();
        BLoc.Init(S, T);
        BLoc.SetOperation(Fuse ? 1 : 0);
        BLoc.Perform();
        BLoc.PartsOfTool(parts);
        // the interactive choice of the parts (#if 0 in OCCT) left out

        BLoc.PerformResult();
        if (!BLoc.HasErrors())
        {
            theDI.SetShape(a[1], BLoc.Shape());
            return 0;
        }
        theCommands.Append("Local operation not done");
        return 1;
    }

    private static int HoleDone(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a, FeatureState theState)
    {
        theState.Hole.Build();
        if (!theState.Hole.HasErrors())
        {
            theDI.SetShape(a[1], theState.Hole.Shape());
            return 0;
        }
        theCommands.Append("Echec de MakeCylindricalHole");
        Print(theCommands, theState.Hole.Status());
        return 1;
    }

    private static int HOLE1(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 10 || narg == 11)
        {
            return 1;
        }
        FeatureState aState = State(theDI);
        TopoDS_Shape S = theDI.GetShape(a[2]);

        gp_Pnt Or = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        gp_Dir Di = new(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));

        double Radius = theDI.Atof(a[9]);

        aState.Hole.Init(S, new gp_Ax1(Or, Di));

        if (narg <= 10)
        {
            aState.Hole.Perform(Radius);
        }
        else
        {
            double pfrom = theDI.Atof(a[10]);
            double pto = theDI.Atof(a[11]);
            aState.Hole.Perform(Radius, pfrom, pto, aState.WithControl);
        }

        return HoleDone(theDI, theCommands, a, aState);
    }

    private static int HOLE2(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 10)
        {
            return 1;
        }
        FeatureState aState = State(theDI);
        TopoDS_Shape S = theDI.GetShape(a[2]);

        gp_Pnt Or = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        gp_Dir Di = new(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));

        double Radius = theDI.Atof(a[9]);

        aState.Hole.Init(S, new gp_Ax1(Or, Di));
        aState.Hole.PerformThruNext(Radius, aState.WithControl);

        return HoleDone(theDI, theCommands, a, aState);
    }

    private static int HOLE3(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 10)
        {
            return 1;
        }
        FeatureState aState = State(theDI);
        TopoDS_Shape S = theDI.GetShape(a[2]);

        gp_Pnt Or = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        gp_Dir Di = new(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));

        double Radius = theDI.Atof(a[9]);

        aState.Hole.Init(S, new gp_Ax1(Or, Di));
        aState.Hole.PerformUntilEnd(Radius, aState.WithControl);

        return HoleDone(theDI, theCommands, a, aState);
    }

    private static int HOLE4(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 11)
        {
            return 1;
        }
        FeatureState aState = State(theDI);
        TopoDS_Shape S = theDI.GetShape(a[2]);

        gp_Pnt Or = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        gp_Dir Di = new(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));

        double Radius = theDI.Atof(a[9]);
        double Length = theDI.Atof(a[10]);

        aState.Hole.Init(S, new gp_Ax1(Or, Di));
        aState.Hole.PerformBlind(Radius, Length, aState.WithControl);

        return HoleDone(theDI, theCommands, a, aState);
    }

    private static int CONTROL(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        FeatureState aState = State(theDI);
        if (a.Count >= 2)
        {
            aState.WithControl = a[1] != "0";
        }
        if (aState.WithControl)
        {
            theCommands.Append("Mode avec controle");
        }
        else
        {
            theCommands.Append("Mode sans controle");
        }
        return 0;
    }

    // The search of the sliding faces of wprism and fprism: the faces of the shape parallel to the prism's direction
    private static void AddSlidingFacesForPrism(in TopoDS_Shape S, in TopoDS_Shape ToPrism, in gp_Vec V,
                                                BRepFeat_MakePrism thePFace)
    {
        LocOpe_FindEdgesInFace FEIF = new();
        for (TopExp_Explorer exp = new(S, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face fac = TopoDS.Face(exp.Current());
            Geom_Surface Su = BRep_Tool.Surface(fac)!;
            if (Su.GetType() == typeof(Geom_RectangularTrimmedSurface))
            {
                Su = ((Geom_RectangularTrimmedSurface)Su).BasisSurface();
            }
            if (Su.GetType() == typeof(Geom_Plane))
            {
                gp_Pln pl = ((Geom_Plane)Su).Pln();
                if (pl.Contains(new gp_Lin(pl.Location(), new gp_Dir(V)), Precision.Confusion(), Precision.Angular()))
                {
                    FEIF.Set(ToPrism, fac);
                    for (FEIF.Init(); FEIF.More(); FEIF.Next())
                    {
                        thePFace.Add(FEIF.Edge(), fac);
                    }
                }
            }
            else if (Su.GetType() == typeof(Geom_CylindricalSurface))
            {
                gp_Cylinder cy = ((Geom_CylindricalSurface)Su).Cylinder();
                if (V.IsParallel(new gp_Vec(cy.Axis().Direction()), Precision.Angular()))
                {
                    FEIF.Set(ToPrism, fac);
                    for (FEIF.Init(); FEIF.More(); FEIF.Next())
                    {
                        thePFace.Add(FEIF.Edge(), fac);
                    }
                }
            }
        }
    }

    // The end of wprism and fprism: the prism by its bounds
    private static int PerformPrism(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a,
                                    BRepFeat_MakePrism thePFace, int borne, double Length, in TopoDS_Shape FFrom,
                                    in TopoDS_Shape FUntil)
    {
        if (borne == 7)
        {
            thePFace.Perform(Length);
        }
        else if (borne == 8)
        {
            thePFace.Perform(FUntil);
        }
        else if (borne == 9)
        {
            if (!(FFrom.IsNull() || FUntil.IsNull()))
            {
                thePFace.Perform(FFrom, FUntil);
            }
            else if (FFrom.IsNull())
            {
                if (!FUntil.IsNull())
                {
                    thePFace.PerformFromEnd(FUntil);
                }
                else
                {
                    thePFace.PerformThruAll();
                }
            }
            else
            {
                // il faudrait inverser V et appeler PerfomFromEnd...
                theCommands.Append("Not Implemented\n");
            }
        }
        if (!thePFace.IsDone())
        {
            theCommands.Append("Local operation not done");
            return 1;
        }

        theDI.SetShape(a[2], thePFace.Shape());
        return 0;
    }

    private static int PRW(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 9)
        {
            return 1;
        }
        TopoDS_Shape S = theDI.GetShape(a[3]);
        BRepFeat_MakePrism thePFace = new();
        gp_Vec V = new();
        TopoDS_Shape FFrom = new(), FUntil = new();
        int borne;
        bool fuse;
        if (At(a[1], 0) == 'f' || At(a[1], 0) == 'F')
        {
            fuse = true;
        }
        else if (At(a[1], 0) == 'c' || At(a[1], 0) == 'C')
        {
            fuse = false;
        }
        else
        {
            return 1;
        }

        if (At(a[4], 0) == '.' || IsAlphabetic(At(a[4], 0)))
        {
            if (narg < 10)
            {
                return 1;
            }
            if (At(a[5], 0) == '.' || IsAlphabetic(At(a[5], 0)))
            {
                if (narg < 11)
                {
                    return 1;
                }
                V.SetCoord(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));
                FFrom = theDI.GetShape(a[4], TopAbs_SHAPE);
                FUntil = theDI.GetShape(a[5], TopAbs_SHAPE);
                borne = 9;
            }
            else
            {
                V.SetCoord(theDI.Atof(a[5]), theDI.Atof(a[6]), theDI.Atof(a[7]));
                FUntil = theDI.GetShape(a[4], TopAbs_SHAPE);
                borne = 8;
            }
        }
        else
        {
            V.SetCoord(theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]));
            borne = 7;
        }
        double Length = V.Magnitude();
        if (Length < Precision.Confusion())
        {
            return 1;
        }

        TopoDS_Shape aLocalShape = theDI.GetShape(a[borne], TopAbs_FACE);
        TopoDS_Face F = TopoDS.Face(aLocalShape);
        BRepFeat_SplitShape Spls = new(F);
        for (int i = borne + 1; i < narg; i++)
        {
            TopoDS_Wire wir;
            if (At(a[i], 0) != '-')
            {
                aLocalShape = theDI.GetShape(a[i], TopAbs_WIRE);
                wir = TopoDS.Wire(aLocalShape);
            }
            else
            {
                if (At(a[i], 1) == '\0')
                {
                    return 1;
                }
                string Temp = a[i][1..];
                aLocalShape = theDI.GetShape(Temp, TopAbs_WIRE);
                wir = TopoDS.Wire(aLocalShape);
                wir.Reverse();
            }
            Spls.Add(wir, F);
        }
        Spls.Build();
        TopoDS_Shape ToPrism;
        IReadOnlyList<TopoDS_Shape> lleft = Spls.DirectLeft();
        if (lleft.Count == 1)
        {
            thePFace.Init(S, lleft[0], F, new gp_Dir(V), fuse ? 1 : 0, true);
            ToPrism = lleft[0];
        }
        else
        {
            BRep_Builder B = new();
            TopoDS_Shell Sh = new();
            B.MakeShell(ref Sh);
            foreach (TopoDS_Shape aLeft in lleft)
            {
                B.Add(Sh, TopoDS.Face(aLeft));
            }
            Sh.Closed(BRep_Tool.IsClosed(Sh));
            thePFace.Init(S, Sh, F, new gp_Dir(V), fuse ? 1 : 0, true);
            ToPrism = Sh;
        }

        // Recherche des faces de glissement, si on n`a pas sketche sur une face
        // du shape de depart
        TopExp_Explorer exp = new(S, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(F))
            {
                break;
            }
        }

        if (!exp.More())
        {
            AddSlidingFacesForPrism(S, ToPrism, V, thePFace);
        }

        return PerformPrism(theDI, theCommands, a, thePFace, borne, Length, FFrom, FUntil);
    }

    private static int PRF(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 8)
        {
            return 1;
        }
        TopoDS_Shape S = theDI.GetShape(a[3]);
        BRepFeat_MakePrism thePFace = new();
        int borne;
        gp_Vec V = new();
        TopoDS_Shape FFrom = new(), FUntil = new();
        bool fuse;
        if (At(a[1], 0) == 'f' || At(a[1], 0) == 'F')
        {
            fuse = true;
        }
        else if (At(a[1], 0) == 'c' || At(a[1], 0) == 'C')
        {
            fuse = false;
        }
        else
        {
            return 1;
        }

        if (At(a[4], 0) == '.' || IsAlphabetic(At(a[4], 0)))
        {
            if (narg < 9)
            {
                return 1;
            }
            if (At(a[5], 0) == '.' || IsAlphabetic(At(a[5], 0)))
            {
                if (narg < 10)
                {
                    return 1;
                }
                borne = 9;
                V.SetCoord(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));
                FFrom = theDI.GetShape(a[4], TopAbs_SHAPE);
                FUntil = theDI.GetShape(a[5], TopAbs_SHAPE);
            }
            else
            {
                borne = 8;
                V.SetCoord(theDI.Atof(a[5]), theDI.Atof(a[6]), theDI.Atof(a[7]));
                FUntil = theDI.GetShape(a[4], TopAbs_SHAPE);
            }
        }
        else
        {
            borne = 7;
            V.SetCoord(theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]));
        }
        double Length = V.Magnitude();
        if (Length < Precision.Confusion())
        {
            return 1;
        }

        TopoDS_Shape ToPrism;
        if (narg == borne + 1)
        {
            TopoDS_Shape aLocalShape = theDI.GetShape(a[borne], TopAbs_FACE);
            TopoDS_Face F = TopoDS.Face(aLocalShape);
            thePFace.Init(S, F, F, new gp_Dir(V), fuse ? 1 : 0, true);
            ToPrism = F;
        }
        else
        {
            TopoDS_Shell She = new();
            BRep_Builder B = new();
            B.MakeShell(ref She);
            for (int i = borne; i < narg; i++)
            {
                TopoDS_Shape aLocalShape = theDI.GetShape(a[i], TopAbs_FACE);
                TopoDS_Face F = TopoDS.Face(aLocalShape);
                if (!F.IsNull())
                {
                    B.Add(She, F);
                }
            }
            She.Closed(BRep_Tool.IsClosed(She));
            thePFace.Init(S, She, new TopoDS_Face(), new gp_Dir(V), fuse ? 1 : 0, false);
            ToPrism = She;
        }

        // Recherche des faces de glissement, on ne prisme pas une face
        // du shape de depart
        TopExp_Explorer exp = new(ToPrism, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            TopExp_Explorer exp2 = new(S, TopAbs_FACE);
            for (; exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(exp.Current()))
                {
                    break;
                }
            }
            if (exp2.More())
            {
                break;
            }
        }

        if (!exp.More())
        {
            AddSlidingFacesForPrism(S, ToPrism, V, thePFace);
        }

        return PerformPrism(theDI, theCommands, a, thePFace, borne, Length, FFrom, FUntil);
    }

    private static int SPLS(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        const string aUsage = "splitshape result shape [splitedges] "
                              + "[face wire/edge/compound [wire/edge/compound ...] "
                              + "[face wire/edge/compound [wire/edge/compound...] ...] "
                              + "[@ edgeonshape edgeonwire [edgeonshape edgeonwire...]]";
        int narg = a.Count;
        int newnarg;

        if (narg < 3)
        {
            // Message::SendFail (decision 405)
            theDI.Output.WriteLine("Invalid number of arguments. Should be : " + aUsage);
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[2]);
        if (S.IsNull())
        {
            theDI.Output.WriteLine("Invalid input shape " + a[2]);
            return 1;
        }
        BRepFeat_SplitShape Spls = new(S);
        TopoDS_Shape EF = new();
        int i = 3;

        for (newnarg = 3; newnarg < narg; newnarg++)
        {
            if (At(a[newnarg], 0) == '@')
            {
                break;
            }
        }

        if (newnarg == 3 || (newnarg != narg && ((narg - newnarg) <= 2 || (narg - newnarg) % 2 != 1)))
        {
            return 1;
        }
        bool isSplittingEdges = false;
        NCollection_Sequence<TopoDS_Shape> aSplitEdges = new();
        if (i < newnarg)
        {
            // the interactive pick of a name starting with a dot is left out
            TopoDS_Shape aSh = theDI.GetShape(a[i]);
            if (aSh.IsNull())
            {
                theDI.Output.WriteLine("Invalid input shape " + a[i]);
                return 1;
            }
            if (aSh.ShapeType() == TopAbs_FACE)
            {
                EF = TopoDS.Face(aSh);
            }
            else
            {
                if (aSh.ShapeType() == TopAbs_COMPOUND || aSh.ShapeType() == TopAbs_WIRE || aSh.ShapeType() == TopAbs_EDGE)
                {
                    TopExp_Explorer aExpE = new(aSh, TopAbs_EDGE, TopAbs_FACE);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        aSplitEdges.Append(aExpE.Current());
                    }

                    isSplittingEdges = !aSplitEdges.IsEmpty();
                }
            }
        }
        i++;
        while (i < newnarg)
        {
            if (!isSplittingEdges && !EF.IsNull() && EF.ShapeType() == TopAbs_FACE)
            {
                // face wire/edge ...

                while (i < newnarg)
                {
                    TopoDS_Shape W;
                    bool rever = false;
                    if (At(a[i], 0) == '-')
                    {
                        if (At(a[i], 1) == '\0')
                        {
                            return 1;
                        }
                        string Temp = a[i][1..];
                        W = theDI.GetShape(Temp, TopAbs_SHAPE, false);
                        rever = true;
                    }
                    else
                    {
                        W = theDI.GetShape(a[i], TopAbs_SHAPE, false);
                    }

                    if (W.IsNull())
                    {
                        return 1; // on n`a rien recupere
                    }
                    TopAbs_ShapeEnum wtyp = W.ShapeType();
                    if (wtyp != TopAbs_WIRE && wtyp != TopAbs_EDGE && wtyp != TopAbs_COMPOUND)
                    {
                        EF = theDI.GetShape(a[i]);
                        break;
                    }
                    else
                    {
                        if (rever)
                        {
                            W.Reverse();
                        }
                        if (wtyp == TopAbs_WIRE)
                        {
                            Spls.Add(TopoDS.Wire(W), TopoDS.Face(EF));
                        }
                        else if (wtyp == TopAbs_EDGE)
                        {
                            Spls.Add(TopoDS.Edge(W), TopoDS.Face(EF));
                        }
                        else
                        {
                            Spls.Add(TopoDS.Compound(W), TopoDS.Face(EF));
                        }
                    }
                    i++;
                }
            }
            else
            {
                if (isSplittingEdges)
                {
                    TopoDS_Shape aSh = theDI.GetShape(a[i]);
                    if (aSh.IsNull())
                    {
                        theDI.Output.WriteLine("Invalid input shape " + a[i]);
                        return 1;
                    }
                    TopExp_Explorer aExpE = new(aSh, TopAbs_EDGE, TopAbs_FACE);
                    for (; aExpE.More(); aExpE.Next())
                    {
                        aSplitEdges.Append(aExpE.Current());
                    }
                }
                else
                {
                    theDI.Output.WriteLine("Invalid input arguments. Should be : " + aUsage);
                    return 1;
                }
            }
            i++;
        }

        if (isSplittingEdges)
        {
            Spls.Add(aSplitEdges);
        }

        // ici, i vaut newnarg
        for (; i < narg; i += 2)
        {
            TopoDS_Shape Ew, Es;
            TopoDS_Shape aLocalShape = theDI.GetShape(a[i], TopAbs_EDGE);
            Es = TopoDS.Edge(aLocalShape);
            if (Es.IsNull())
            {
                return 1;
            }
            aLocalShape = theDI.GetShape(a[i + 1], TopAbs_EDGE);
            Ew = TopoDS.Edge(aLocalShape);
            if (Ew.IsNull())
            {
                theDI.Output.WriteLine("Invalid input shape " + a[i + 1]);
                return 1;
            }
            Spls.Add(TopoDS.Edge(Ew), TopoDS.Edge(Es));
        }

        theDI.SetShape(a[1], Spls.Shape());
        return 0;
    }

    // The angle or the faces of wrotate and frotate: returns the index of the axis' first word
    private static int RotateBounds(DrawInterpreter theDI, IReadOnlyList<string> a, ref double Angle,
                                    ref TopoDS_Shape FFrom, ref TopoDS_Shape FUntil, int theMinArgs)
    {
        int i;
        FFrom = theDI.GetShape(a[4], TopAbs_SHAPE);
        if (FFrom.IsNull())
        {
            Angle = theDI.Atof(a[4]);
            Angle *= M_PI / 180.0;
            i = 5;
        }
        else
        {
            FUntil = theDI.GetShape(a[5], TopAbs_SHAPE);
            if (FUntil.IsNull())
            {
                i = 5;
                FUntil = FFrom;
                FFrom.Nullify();
            }
            else
            {
                if (a.Count < theMinArgs)
                {
                    return -1;
                }
                i = 6;
            }
        }
        return i;
    }

    // The search of the sliding faces of wrotate and frotate: the faces of the shape around the axis
    private static void AddSlidingFacesForRevol(in TopoDS_Shape S, in TopoDS_Shape ToRotate, in gp_Ax1 theAxis,
                                                BRepFeat_MakeRevol theRFace)
    {
        LocOpe_FindEdgesInFace FEIF = new();
        for (TopExp_Explorer exp = new(S, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face fac = TopoDS.Face(exp.Current());
            Geom_Surface Su = BRep_Tool.Surface(fac)!;
            if (Su.GetType() == typeof(Geom_RectangularTrimmedSurface))
            {
                Su = ((Geom_RectangularTrimmedSurface)Su).BasisSurface();
            }
            if (Su.GetType() == typeof(Geom_Plane))
            {
                gp_Pln pl = ((Geom_Plane)Su).Pln();
                if (pl.Axis().IsParallel(theAxis, Precision.Angular()))
                {
                    FEIF.Set(ToRotate, fac);
                    for (FEIF.Init(); FEIF.More(); FEIF.Next())
                    {
                        theRFace.Add(FEIF.Edge(), fac);
                    }
                }
            }
            else if (Su.GetType() == typeof(Geom_CylindricalSurface))
            {
                gp_Cylinder cy = ((Geom_CylindricalSurface)Su).Cylinder();
                if (cy.Axis().IsCoaxial(theAxis, Precision.Angular(), Precision.Confusion()))
                {
                    FEIF.Set(ToRotate, fac);
                    for (FEIF.Init(); FEIF.More(); FEIF.Next())
                    {
                        theRFace.Add(FEIF.Edge(), fac);
                    }
                }
            }
        }
    }

    // The end of wrotate and frotate: the revolution by its bounds
    private static int PerformRevol(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a,
                                    BRepFeat_MakeRevol theRFace, int borne, double Angle, in TopoDS_Shape FFrom,
                                    in TopoDS_Shape FUntil)
    {
        if (borne == 11)
        {
            if (FUntil.IsNull())
            {
                theRFace.Perform(Angle);
            }
            else
            {
                theRFace.Perform(FUntil);
            }
        }
        else
        { // borne == 12
            theRFace.Perform(FFrom, FUntil);
        }

        if (!theRFace.IsDone())
        {
            theCommands.Append("Local operation not done");
            return 1;
        }
        theDI.SetShape(a[2], theRFace.Shape());
        return 0;
    }

    private static int ROW(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 13)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[3]);
        BRepFeat_MakeRevol theRFace = new();
        double Angle = 0;
        TopoDS_Shape FFrom = new(), FUntil = new();
        int i, borne;
        bool fuse;

        if (At(a[1], 0) == 'f' || At(a[1], 0) == 'F')
        {
            fuse = true;
        }
        else if (At(a[1], 0) == 'c' || At(a[1], 0) == 'C')
        {
            fuse = false;
        }
        else
        {
            return 1;
        }

        i = RotateBounds(theDI, a, ref Angle, ref FFrom, ref FUntil, 14);
        if (i < 0)
        {
            return 1;
        }

        borne = i + 6;
        gp_Pnt Or = new(theDI.Atof(a[i]), theDI.Atof(a[i + 1]), theDI.Atof(a[i + 2]));
        gp_Dir D = new(theDI.Atof(a[i + 3]), theDI.Atof(a[i + 4]), theDI.Atof(a[i + 5]));
        gp_Ax1 theAxis = new(Or, D);

        TopoDS_Shape aLocalShape = theDI.GetShape(a[borne], TopAbs_FACE);
        TopoDS_Face F = TopoDS.Face(aLocalShape);
        BRepFeat_SplitShape Spls = new(F);
        for (i = borne + 1; i < narg; i++)
        {
            TopoDS_Wire wir;
            if (At(a[i], 0) != '-')
            {
                aLocalShape = theDI.GetShape(a[i], TopAbs_WIRE);
                wir = TopoDS.Wire(aLocalShape);
            }
            else
            {
                if (At(a[i], 1) == '\0')
                {
                    return 1;
                }
                string Temp = a[i][1..];
                aLocalShape = theDI.GetShape(Temp, TopAbs_WIRE);
                wir = TopoDS.Wire(aLocalShape);
                wir.Reverse();
            }
            Spls.Add(wir, F);
        }
        Spls.Build();
        TopoDS_Shape ToRotate;
        IReadOnlyList<TopoDS_Shape> lleft = Spls.DirectLeft();
        if (lleft.Count == 1)
        {
            theRFace.Init(S, lleft[0], F, theAxis, fuse ? 1 : 0, true);
            ToRotate = lleft[0];
        }
        else
        {
            BRep_Builder B = new();
            TopoDS_Shell Sh = new();
            B.MakeShell(ref Sh);
            foreach (TopoDS_Shape aLeft in lleft)
            {
                B.Add(Sh, TopoDS.Face(aLeft));
            }
            Sh.Closed(BRep_Tool.IsClosed(Sh));
            theRFace.Init(S, Sh, F, theAxis, fuse ? 1 : 0, true);
            ToRotate = Sh;
        }

        // Recherche des faces de glissement
        TopExp_Explorer exp = new(S, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(F))
            {
                break;
            }
        }

        if (!exp.More())
        {
            AddSlidingFacesForRevol(S, ToRotate, theAxis, theRFace);
        }

        return PerformRevol(theDI, theCommands, a, theRFace, borne, Angle, FFrom, FUntil);
    }

    private static int ROF(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 12)
        {
            return 1;
        }

        TopoDS_Shape S = theDI.GetShape(a[3]);
        BRepFeat_MakeRevol theRFace = new();
        double Angle = 0;
        TopoDS_Shape FFrom = new(), FUntil = new();
        int i, borne;
        bool fuse;

        if (At(a[1], 0) == 'f' || At(a[1], 0) == 'F')
        {
            fuse = true;
        }
        else if (At(a[1], 0) == 'c' || At(a[1], 0) == 'C')
        {
            fuse = false;
        }
        else
        {
            return 1;
        }

        i = RotateBounds(theDI, a, ref Angle, ref FFrom, ref FUntil, 13);
        if (i < 0)
        {
            return 1;
        }

        borne = i + 6;
        gp_Pnt Or = new(theDI.Atof(a[i]), theDI.Atof(a[i + 1]), theDI.Atof(a[i + 2]));
        gp_Dir D = new(theDI.Atof(a[i + 3]), theDI.Atof(a[i + 4]), theDI.Atof(a[i + 5]));
        gp_Ax1 theAxis = new(Or, D);

        TopoDS_Shape ToRotate;
        if (narg == borne + 1)
        {
            TopoDS_Shape aLocalShape = theDI.GetShape(a[borne], TopAbs_FACE);
            TopoDS_Face F = TopoDS.Face(aLocalShape);
            theRFace.Init(S, F, F, theAxis, fuse ? 1 : 0, true);
            ToRotate = F;
        }
        else
        {
            TopoDS_Shell She = new();
            BRep_Builder B = new();
            B.MakeShell(ref She);

            for (i = borne; i < narg; i++)
            {
                TopoDS_Shape aLocalShape = theDI.GetShape(a[i], TopAbs_FACE);
                TopoDS_Face F = TopoDS.Face(aLocalShape);
                if (!F.IsNull())
                {
                    B.Add(She, F);
                }
            }
            She.Closed(BRep_Tool.IsClosed(She));
            theRFace.Init(S, She, new TopoDS_Face(), theAxis, fuse ? 1 : 0, false);
            ToRotate = She;
        }

        TopExp_Explorer exp = new(ToRotate, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            TopExp_Explorer exp2 = new(S, TopAbs_FACE);
            for (; exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(exp.Current()))
                {
                    break;
                }
            }
            if (exp2.More())
            {
                break;
            }
        }

        if (!exp.More())
        {
            AddSlidingFacesForRevol(S, ToRotate, theAxis, theRFace);
        }

        return PerformRevol(theDI, theCommands, a, theRFace, borne, Angle, FFrom, FUntil);
    }

    private static int GLU(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 6 || narg % 2 != 0)
        {
            return 1;
        }
        TopoDS_Shape Sne = theDI.GetShape(a[2]);
        TopoDS_Shape Sba = theDI.GetShape(a[3]);

        BRepFeat_Gluer theGl = new(Sne, Sba);
        TopoDS_Shape Fne, Fba;

        LocOpe_FindEdges fined = new();

        int i = 4;
        bool first = true;
        while (i < narg)
        {
            // the interactive picks of names starting with a dot are left out
            Fne = theDI.GetShape(a[i]);
            if (Fne.IsNull())
            {
                return 1;
            }
            TopAbs_ShapeEnum sht = Fne.ShapeType();
            if (first && sht != TopAbs_FACE)
            {
                return 1;
            }
            first = false;
            Fba = theDI.GetShape(a[i + 1]);
            if (Fba.IsNull())
            {
                return 1;
            }
            if (Fba.ShapeType() != sht)
            {
                return 1;
            }
            if (sht == TopAbs_FACE)
            {
                TopoDS_Face f1 = TopoDS.Face(Fne);
                TopoDS_Face f2 = TopoDS.Face(Fba);
                theGl.Bind(f1, f2);
                fined.Set(Fne, Fba);
                for (fined.InitIterator(); fined.More(); fined.Next())
                {
                    theGl.Bind(fined.EdgeFrom(), fined.EdgeTo());
                }
            }
            else
            {
                theGl.Bind(TopoDS.Edge(Fne), TopoDS.Edge(Fba));
            }
            i += 2;
        }

        theDI.SetShape(a[1], theGl.Shape());
        return 0;
    }

    private static int DEFIN(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        string a0 = a[0];
        if (!EqualsNoCase(a0, "FEATPRISM") && !EqualsNoCase(a0, "FEATDPRISM") && !EqualsNoCase(a0, "FEATREVOL")
            && !EqualsNoCase(a0, "FEATPIPE") && !EqualsNoCase(a0, "FEATLF") && !EqualsNoCase(a0, "FEATRF"))
        {
            return 1;
        }

        if ((EqualsNoCase(a0, "FEATPRISM") && narg != 9) || (EqualsNoCase(a0, "FEATREVOL") && narg != 12)
            || (EqualsNoCase(a0, "FEATDPRISM") && narg != 7) || (EqualsNoCase(a0, "FEATPIPE") && narg != 7)
            || (EqualsNoCase(a0, "FEATLF") && narg != 12) || (EqualsNoCase(a0, "FEATRF") && narg != 14))
        {
            theCommands.Append("invalid number of arguments");
            return 1;
        }

        FeatureState aState = State(theDI);
        TopoDS_Shape Sbase = theDI.GetShape(a[1]);
        if (Sbase.IsNull())
        {
            theCommands.Append("null basis shape");
            return 1;
        }
        int Ifuse = theDI.Atoi(a[narg - 2]);
        int Imodif = theDI.Atoi(a[narg - 1]);

        int Fuse = Ifuse;
        bool Modify = Imodif != 0;

        TopoDS_Shape Pbase = new();
        TopoDS_Face Skface = new();
        TopoDS_Wire W = new();

        Geom_Plane? P = null;

        BRepFeat_StatusError se;

        if (!EqualsNoCase(a0, "FEATLF") && !EqualsNoCase(a0, "FEATRF"))
        {
            Pbase = theDI.GetShape(a[2]);
            if (Pbase.IsNull())
            {
                theCommands.Append("null shape to transform");
                return 1;
            }
            TopoDS_Shape aLocalShape = theDI.GetShape(a[3], TopAbs_FACE);
            Skface = TopoDS.Face(aLocalShape);
            if (Skface.IsNull())
            {
                theCommands.Append("null face of Sketch");
                return 1;
            }
        }
        else
        {
            TopoDS_Shape aLocalShape = theDI.GetShape(a[2], TopAbs_WIRE);
            W = TopoDS.Wire(aLocalShape);
            if (W.IsNull())
            {
                theCommands.Append("null profile for rib or slot");
                return 1;
            }
            Geom_Surface? s = DrawTrSurf.GetSurface(theDI, a[3]);
            P = s as Geom_Plane;
            if (P is null)
            {
                theCommands.Append("null plane to transform");
                return 1;
            }
        }
        if (narg == 9 || narg == 12 || narg == 14)
        {
            double X, Y, Z;
            X = theDI.Atof(a[4]);
            Y = theDI.Atof(a[5]);
            Z = theDI.Atof(a[6]);

            if (narg == 9)
            { // prism
                aState.prdef = true;
                aState.theSbase = Sbase;
                aState.thePbase = Pbase;
                aState.theSkface = Skface;
                aState.Prism.Init(Sbase, Pbase, Skface, new gp_Dir(X, Y, Z), Fuse, Modify);
            }
            else if (narg == 14)
            {
                aState.rfdef = true;
                gp_Pnt Or = new(X, Y, Z);
                X = theDI.Atof(a[7]);
                Y = theDI.Atof(a[8]);
                Z = theDI.Atof(a[9]);
                double H1 = theDI.Atof(a[10]);
                double H2 = theDI.Atof(a[11]);
                gp_Ax1 ax1 = new(Or, new gp_Dir(X, Y, Z));
                aState.RevolutionForm.Init(Sbase, W, P!, ax1, H1, H2, Fuse, Modify);
                if (!aState.RevolutionForm.IsDone())
                {
                    se = aState.RevolutionForm.CurrentStatusError();
                    theCommands.Append(BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
            }
            else if (narg == 12 && !EqualsNoCase(a0, "FEATLF"))
            {
                aState.rvdef = true;
                gp_Pnt Or = new(X, Y, Z);
                X = theDI.Atof(a[7]);
                Y = theDI.Atof(a[8]);
                Z = theDI.Atof(a[9]);
                aState.theSbase = Sbase;
                aState.thePbase = Pbase;
                aState.theSkface = Skface;
                aState.Revol.Init(Sbase, Pbase, Skface, new gp_Ax1(Or, new gp_Dir(X, Y, Z)), Fuse, Modify);
            }
            else
            {
                aState.lfdef = true;
                gp_Vec Direct = new(X, Y, Z);
                X = theDI.Atof(a[7]);
                Y = theDI.Atof(a[8]);
                Z = theDI.Atof(a[9]);
                aState.LinearForm.Init(Sbase, W, P!, Direct, new gp_Vec(X, Y, Z), Fuse, Modify);
                if (!aState.LinearForm.IsDone())
                {
                    se = aState.LinearForm.CurrentStatusError();
                    theCommands.Append(BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
            }
        }
        else if (narg == 7)
        {
            if (EqualsNoCase(a0, "FEATDPRISM"))
            {
                if (Pbase.ShapeType() != TopAbs_FACE)
                {
                    theCommands.Append("Invalid DPrism base");
                    return 1;
                }
                double Angle = theDI.Atof(a[4]) * M_PI / 360;
                aState.dprdef = true;
                aState.theSbase = Sbase;
                aState.thePbase = Pbase;
                aState.theSkface = Skface;
                aState.DPrism.Init(Sbase, TopoDS.Face(Pbase), Skface, Angle, Fuse, Modify);
            }
            else
            { // FEATPIPE
                TopoDS_Shape aLocalShape = theDI.GetShape(a[4], TopAbs_WIRE);
                TopoDS_Wire Spine = TopoDS.Wire(aLocalShape);
                if (Spine.IsNull())
                {
                    TopoDS_Shape Edspine = theDI.GetShape(a[4], TopAbs_EDGE);
                    if (Edspine.IsNull())
                    {
                        theCommands.Append("null spine");
                        return 1;
                    }
                    BRep_Builder B = new();
                    B.MakeWire(ref Spine);
                    B.Add(Spine, Edspine);
                }
                aState.pidef = true;
                aState.theSbase = Sbase;
                aState.thePbase = Pbase;
                aState.theSkface = Skface;
                aState.Pipe.Init(Sbase, Pbase, Skface, Spine, Fuse, Modify);
            }
        }
        return 0;
    }

    private static int ADD(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        int i;
        if (narg < 4 || narg % 2 != 0)
        {
            return 1;
        }
        FeatureState aState = State(theDI);
        if (EqualsNoCase("PRISM", a[1]) || EqualsNoCase("REVOL", a[1]) || EqualsNoCase("PIPE", a[1]))
        {
            bool aDefined = EqualsNoCase("PRISM", a[1]) ? aState.prdef
                          : EqualsNoCase("REVOL", a[1]) ? aState.rvdef : aState.pidef;
            if (!aDefined)
            {
                return 1;
            }
            for (i = 2; i < narg; i += 2)
            {
                TopoDS_Shape aLocalShape = theDI.GetShape(a[i], TopAbs_EDGE);
                TopoDS_Edge edg = TopoDS.Edge(aLocalShape);
                if (edg.IsNull())
                {
                    return 1;
                }
                aLocalShape = theDI.GetShape(a[i + 1], TopAbs_FACE);
                TopoDS_Face fac = TopoDS.Face(aLocalShape);
                if (fac.IsNull())
                {
                    return 1;
                }
                if (EqualsNoCase("PRISM", a[1]))
                {
                    aState.Prism.Add(edg, fac);
                }
                else if (EqualsNoCase("REVOL", a[1]))
                {
                    aState.Revol.Add(edg, fac);
                }
                else
                {
                    aState.Pipe.Add(edg, fac);
                }
            }
        }
        else
        {
            return 1;
        }
        return 0;
    }

    private static int PERF(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 3)
        {
            return 1;
        }
        string a0 = a[0];
        if (!EqualsNoCase(a0, "FEATPERFORM") && !EqualsNoCase(a0, "FEATPERFORMVAL"))
        {
            return 1;
        }

        FeatureState aState = State(theDI);
        NCollection_List<TopoDS_Shape> anArgs = new();

        int Kas;
        if (EqualsNoCase("PRISM", a[1]))
        {
            Kas = 1;
            if (!aState.prdef)
            {
                theCommands.Append("prism not defined");
                return 1;
            }
        }
        else if (EqualsNoCase("REVOL", a[1]))
        {
            Kas = 2;
            if (!aState.rvdef)
            {
                theCommands.Append("revol not defined");
                return 1;
            }
        }
        else if (EqualsNoCase("PIPE", a[1]))
        {
            Kas = 3;
            if (!aState.pidef)
            {
                theCommands.Append("pipe not defined");
                return 1;
            }
            if (EqualsNoCase(a0, "FEATPERFORMVAL"))
            {
                theCommands.Append("invalid command for pipe");
                return 1;
            }
        }
        else if (EqualsNoCase("DPRISM", a[1]))
        {
            Kas = 4;
            if (!aState.dprdef)
            {
                theCommands.Append("dprism not defined");
                return 1;
            }
        }
        else if (EqualsNoCase("LF", a[1]))
        {
            Kas = 5;
            if (!aState.lfdef)
            {
                theCommands.Append("lf not defined");
                return 1;
            }
            if (EqualsNoCase(a0, "FEATPERFORMVAL"))
            {
                theCommands.Append("invalid command for lf");
                return 1;
            }
        }
        else if (EqualsNoCase("RF", a[1]))
        {
            Kas = 6;
            if (!aState.rfdef)
            {
                theCommands.Append("rf not defined");
                return 1;
            }
            if (EqualsNoCase(a0, "FEATPERFORMVAL"))
            {
                theCommands.Append("invalid command for rf");
                return 1;
            }
        }
        else
        {
            theCommands.Append("unknown argument : ").Append(a[1]);
            return 1;
        }

        if (EqualsNoCase(a0, "FEATPERFORMVAL"))
        {
            if (narg != 4 && narg != 5)
            {
                theCommands.Append("invalid number of arguments");
                return 1;
            }
            if (narg == 4)
            {
                double Val = theDI.Atof(a[3]);
                if (Kas == 1)
                {
                    aState.Prism.Perform(Val);
                }
                else if (Kas == 2)
                {
                    Val *= M_PI / 180.0;
                    aState.Revol.Perform(Val);
                }
                else if (Kas == 4)
                {
                    aState.DPrism.Perform(Val);
                }
                else if (Kas == 5)
                {
                    theCommands.Append("invalid command for lf");
                    return 1;
                }
                else if (Kas == 6)
                {
                    theCommands.Append("invalid command for rf");
                    return 1;
                }
            }
            else if (narg == 5)
            {
                double Val = theDI.Atof(a[3]);
                TopoDS_Shape FUntil = theDI.GetShape(a[4], TopAbs_SHAPE);
                if (Kas == 1)
                {
                    aState.Prism.PerformUntilHeight(FUntil, Val);
                }
                else if (Kas == 2)
                {
                    Val *= M_PI / 180.0;
                    aState.Revol.PerformUntilAngle(FUntil, Val);
                }
                else if (Kas == 4)
                {
                    aState.DPrism.PerformUntilHeight(FUntil, Val);
                }
                else
                {
                    theCommands.Append("invalid command for ribs or slots");
                    return 1;
                }
            }
        }
        else if (EqualsNoCase(a0, "FEATPERFORM"))
        {
            if (narg == 3)
            { // Thru all
                switch (Kas)
                {
                    case 1:
                        aState.Prism.PerformThruAll();
                        break;
                    case 2:
                        aState.Revol.PerformThruAll();
                        break;
                    case 3:
                        aState.Pipe.Perform();
                        break;
                    case 4:
                        aState.DPrism.PerformThruAll();
                        break;
                    case 5:
                        aState.LinearForm.Perform();
                        break;
                    case 6:
                        aState.RevolutionForm.Perform();
                        break;
                    default:
                        return 1;
                }
            }
            else if (narg == 4)
            { // Until
                TopoDS_Shape Funtil = theDI.GetShape(a[3], TopAbs_SHAPE);
                switch (Kas)
                {
                    case 1:
                    {
                        if (Funtil.IsNull())
                        {
                            aState.Prism.PerformUntilEnd();
                        }
                        else
                        {
                            aState.Prism.Perform(Funtil);
                        }
                    }
                    break;
                    case 2:
                    {
                        if (!Funtil.IsNull())
                        {
                            aState.Revol.Perform(Funtil);
                        }
                        else
                        {
                            return 1;
                        }
                    }
                    break;
                    case 3:
                    {
                        if (!Funtil.IsNull())
                        {
                            aState.Pipe.Perform(Funtil);
                        }
                        else
                        {
                            theCommands.Append("invalid command for ribs pipe");
                            return 1;
                        }
                    }
                    break;
                    case 4:
                    {
                        if (!Funtil.IsNull())
                        {
                            aState.DPrism.Perform(Funtil);
                        }
                        else
                        {
                            aState.DPrism.PerformUntilEnd();
                        }
                    }
                    break;
                    case 5:
                    {
                        theCommands.Append("invalid command for lf");
                        return 1;
                    }
                    case 6:
                    {
                        theCommands.Append("invalid command for rf");
                        return 1;
                    }
                    default:
                        return 1;
                }
            }
            else if (narg == 5)
            {
                TopoDS_Shape Ffrom = theDI.GetShape(a[3], TopAbs_SHAPE);
                TopoDS_Shape Funtil = theDI.GetShape(a[4], TopAbs_SHAPE);
                if (Funtil.IsNull())
                {
                    return 1;
                }
                switch (Kas)
                {
                    case 1:
                    {
                        if (Ffrom.IsNull())
                        {
                            aState.Prism.PerformFromEnd(Funtil);
                        }
                        else
                        {
                            aState.Prism.Perform(Ffrom, Funtil);
                        }
                    }
                    break;
                    case 2:
                    {
                        if (Ffrom.IsNull())
                        {
                            return 1;
                        }
                        aState.Revol.Perform(Ffrom, Funtil);
                    }
                    break;
                    case 3:
                    {
                        if (Ffrom.IsNull())
                        {
                            return 1;
                        }
                        aState.Pipe.Perform(Ffrom, Funtil);
                    }
                    break;
                    case 4:
                    {
                        if (Ffrom.IsNull())
                        {
                            aState.DPrism.PerformFromEnd(Funtil);
                        }
                        else
                        {
                            aState.DPrism.Perform(Ffrom, Funtil);
                        }
                    }
                    break;
                    default:
                        return 1;
                }
            }
        }

        BRepFeat_StatusError se;
        switch (Kas)
        {
            case 1:
                if (!aState.Prism.IsDone())
                {
                    se = aState.Prism.CurrentStatusError();
                    theCommands.Append(BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
                theDI.SetShape(a[2], aState.Prism.Shape());
                // History
                if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
                {
                    anArgs.Clear();
                    anArgs.Append(aState.theSbase);
                    anArgs.Append(aState.thePbase);
                    anArgs.Append(aState.theSkface);
                    BRepTestHistoryCommands.SetHistory(theDI, anArgs, aState.Prism);
                }
                return 0;
            case 2:
                if (!aState.Revol.IsDone())
                {
                    se = aState.Revol.CurrentStatusError();
                    theCommands.Append(BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
                // History
                if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
                {
                    anArgs.Clear();
                    anArgs.Append(aState.theSbase);
                    anArgs.Append(aState.thePbase);
                    anArgs.Append(aState.theSkface);
                    BRepTestHistoryCommands.SetHistory(theDI, anArgs, aState.Revol);
                }
                theDI.SetShape(a[2], aState.Revol.Shape());
                return 0;
            case 3:
                if (!aState.Pipe.IsDone())
                {
                    se = aState.Pipe.CurrentStatusError();
                    theCommands.Append(BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
                // History
                if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
                {
                    anArgs.Clear();
                    anArgs.Append(aState.theSbase);
                    anArgs.Append(aState.thePbase);
                    anArgs.Append(aState.theSkface);
                    BRepTestHistoryCommands.SetHistory(theDI, anArgs, aState.Pipe);
                }
                theDI.SetShape(a[2], aState.Pipe.Shape());
                return 0;
            case 4:
                if (!aState.DPrism.IsDone())
                {
                    se = aState.DPrism.CurrentStatusError();
                    theCommands.Append(BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
                // History
                if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
                {
                    anArgs.Clear();
                    anArgs.Append(aState.theSbase);
                    anArgs.Append(aState.thePbase);
                    anArgs.Append(aState.theSkface);
                    BRepTestHistoryCommands.SetHistory(theDI, anArgs, aState.DPrism);
                }
                theDI.SetShape(a[2], aState.DPrism.Shape());
                return 0;
            case 5:
                if (!aState.LinearForm.IsDone())
                {
                    se = aState.LinearForm.CurrentStatusError();
                    theCommands.Append(BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
                theDI.SetShape(a[2], aState.LinearForm.Shape());
                return 0;
            case 6:
                if (!aState.RevolutionForm.IsDone())
                {
                    se = aState.RevolutionForm.CurrentStatusError();
                    theCommands.Append(BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
                theDI.SetShape(a[2], aState.RevolutionForm.Shape());
                return 0;
            default:
                return 1;
        }
    }

    // BOSS, for endedges only: fillet and bossage need BRepFilletAPI_MakeFillet (unit 7N3)
    private static int BOSS(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (!EqualsNoCase(a[0], "ENDEDGES"))
        {
            return 1;
        }

        if (narg != 5)
        {
            // theCommands.PrintHelp(a[0])
            theCommands.Append(a[0]).Append(" :  Return top and bottom edges of dprism :endedges dprism shapetop shapebottom First/LastShape (1/2)\n");
            return 1;
        }

        FeatureState aState = State(theDI);
        int dprsig = theDI.Atoi(a[4]);

        TopoDS_Compound theShapeTop = new();
        TopoDS_Compound theShapeBottom = new();

        if (EqualsNoCase("DPRISM", a[1]))
        {
            if (!aState.dprdef)
            {
                theCommands.Append("dprism not defined");
                return 1;
            }
        }
        else
        {
            theCommands.Append("unknown argument : ").Append(a[1]);
            return 1;
        }

        aState.DPrism.BossEdges(dprsig);

        NCollection_List<TopoDS_Shape> theTopEdges = new(aState.DPrism.InternalTopEdges());
        NCollection_List<TopoDS_Shape> theLatEdges = new(aState.DPrism.InternalLatEdges());

        NCollection_List<TopoDS_Shape>.Iterator it = new();
        BRep_Builder B = new();

        B.MakeCompound(ref theShapeTop);
        it.Initialize(theTopEdges);
        for (; it.More(); it.Next())
        {
            TopExp_Explorer exp = new();
            for (exp.Init(it.Value(), TopAbs_EDGE); exp.More(); exp.Next())
            {
                B.Add(theShapeTop, exp.Current());
            }
        }
        theDI.SetShape(a[2], theShapeTop);

        B.MakeCompound(ref theShapeBottom);
        it.Initialize(theLatEdges);
        for (; it.More(); it.Next())
        {
            B.Add(theShapeBottom, it.Value());
        }
        theDI.SetShape(a[3], theShapeBottom);
        return 0;
    }
}
