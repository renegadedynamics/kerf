// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Features.Local;
using Kerf.Engine.Fillets;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using Kerf.Engine;
using static Kerf.Engine.Features.FeatureStatus;
using static Kerf.Engine.Foundation.Standard;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest's feature commands (BRepTest_FeatureCommands.cxx): <c>localope</c>, <c>hole</c>, <c>firsthole</c>,
/// <c>holend</c>, <c>blindhole</c>, <c>holecontrol</c>, <c>wprism</c>, <c>fprism</c>, <c>wrotate</c>,
/// <c>frotate</c>, <c>splitshape</c>, <c>glue</c>, <c>featprism</c>, <c>featrevol</c>, <c>featpipe</c>,
/// <c>featdprism</c>, <c>featlf</c>, <c>featrf</c>, <c>addslide</c>, <c>featperform</c>, <c>featperformval</c>,
/// <c>endedges</c>, <c>fillet</c> and <c>bossage</c>, as OCCT writes them.
/// </summary>
/// <remarks>
/// <para>OCCT keeps the feature makers (and the hole, its control flag, the prism's arguments and which features are
/// defined) in function-scope statics, from a <c>feat*</c> command to <c>featperform</c>; Draw-lite keeps them per
/// interpreter (decision 1055), so a corpus case starts from a fresh DRAWEXE's state.</para>
/// <para>Left out: the offset commands of the same file (unit 7G, in the file's other part); the interactive picks of <c>glue</c> and <c>splitshape</c> (names starting with a dot).</para>
/// </remarks>
internal static partial class BRepTestFeatureCommands
{
    private sealed class FeatureState
    {
        public readonly global::Kerf.Engine.Features.MakeCylindricalHole Hole = new();
        public bool WithControl = true;
        public readonly global::Kerf.Engine.Features.MakePrism Prism = new();
        public readonly global::Kerf.Engine.Features.MakeDraftPrism DPrism = new();
        public readonly global::Kerf.Engine.Features.MakeRevol Revol = new();
        public readonly global::Kerf.Engine.Features.MakePipe Pipe = new();
        public readonly global::Kerf.Engine.Features.MakeLinearForm LinearForm = new();
        public readonly global::Kerf.Engine.Features.MakeRevolutionForm RevolutionForm = new();
        // Input shapes for Prism, DPrism, Revol, Pipe
        public Shape theSbase = new(), thePbase = new();
        public Face theSkface = new();
        public bool dprdef, prdef, rvdef, pidef, lfdef, rfdef;
        // the fillet of fillet and bossage (OCCT's file-scope Rakk)
        public MakeFillet? Rakk;
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
        DrawCxx.Add(theInterpreter, "fillet", BOSS);
        DrawCxx.Add(theInterpreter, "bossage", BOSS);
    }

    private static bool EqualsNoCase(string theA, string theB) => string.Equals(theA, theB, StringComparison.OrdinalIgnoreCase);

    private static bool IsAlphabetic(char theChar) => theChar is >= 'a' and <= 'z' or >= 'A' and <= 'Z';

    private static char At(string theWord, int theIndex) => theIndex < theWord.Length ? theWord[theIndex] : '\0';

    private static void Print(DrawResult di, global::Kerf.Engine.Features.FeatureStatus St)
    {
        di.Append("  Error Status : ");
        switch (St)
        {
            case global::Kerf.Engine.Features.FeatureStatus.NoError:
                di.Append("No error");
                break;
            case global::Kerf.Engine.Features.FeatureStatus.InvalidPlacement:
                di.Append("Invalid placement");
                break;
            case global::Kerf.Engine.Features.FeatureStatus.HoleTooLong:
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

        Shape S = theDI.GetShape(a[2]);
        Shape T = theDI.GetShape(a[3]);

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

        NCollection_List<Shape> LF = new();
        for (int i = 0; i <= narg - 6; i++)
        {
            Shape aLocalShape = theDI.GetShape(a[i + 5], ShapeType.Face);
            LF.Append(aLocalShape);
        }

        NCollection_List<Shape> parts = new();
        global::Kerf.Engine.Features.FeatureBuilder BLoc = new();
        BLoc.Init(S, T);
        BLoc.SetOperation(Fuse ? 1 : 0);
        BLoc.Perform();
        BLoc.PartsOfTool(parts);
        // the interactive choice of the parts (#if 0 in OCCT) left out

        BLoc.PerformResult();
        if (!BLoc.HasErrors)
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
        if (!theState.Hole.HasErrors)
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
        Shape S = theDI.GetShape(a[2]);

        Point3D Or = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        Direction3D Di = new(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));

        double Radius = theDI.Atof(a[9]);

        aState.Hole.Init(S, new Axis3D(Or, Di));

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
        Shape S = theDI.GetShape(a[2]);

        Point3D Or = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        Direction3D Di = new(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));

        double Radius = theDI.Atof(a[9]);

        aState.Hole.Init(S, new Axis3D(Or, Di));
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
        Shape S = theDI.GetShape(a[2]);

        Point3D Or = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        Direction3D Di = new(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));

        double Radius = theDI.Atof(a[9]);

        aState.Hole.Init(S, new Axis3D(Or, Di));
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
        Shape S = theDI.GetShape(a[2]);

        Point3D Or = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));
        Direction3D Di = new(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));

        double Radius = theDI.Atof(a[9]);
        double Length = theDI.Atof(a[10]);

        aState.Hole.Init(S, new Axis3D(Or, Di));
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
    private static void AddSlidingFacesForPrism(in Shape S, in Shape ToPrism, in Vector3D V,
                                                global::Kerf.Engine.Features.MakePrism thePFace)
    {
        FindEdgesInFace FEIF = new();
        for (global::Kerf.Engine.Topology.Explorer exp = new(S, ShapeType.Face); exp.More(); exp.Next())
        {
            Face fac = global::Kerf.Engine.Topology.ShapeCasts.Face(exp.Current());
            Surface Su = Tool.Surface(fac)!;
            if (Su.GetType() == typeof(TrimmedSurface))
            {
                Su = ((TrimmedSurface)Su).BasisSurface;
            }
            if (Su.GetType() == typeof(PlaneSurface))
            {
                Plane3D pl = ((PlaneSurface)Su).Plane;
                if (pl.Contains(new Line3D(pl.Location, new Direction3D(V)), global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    FEIF.Set(ToPrism, fac);
                    for (FEIF.Init(); FEIF.More(); FEIF.Next())
                    {
                        thePFace.Add(FEIF.Edge(), fac);
                    }
                }
            }
            else if (Su.GetType() == typeof(CylindricalSurface))
            {
                Cylinder3D cy = ((CylindricalSurface)Su).Cylinder;
                if (V.IsParallel(new Vector3D(cy.Axis.Direction), global::Kerf.Engine.Foundation.Precision.Angular()))
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
                                    global::Kerf.Engine.Features.MakePrism thePFace, int borne, double Length, in Shape FFrom,
                                    in Shape FUntil)
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
            if (!(FFrom.IsNull || FUntil.IsNull))
            {
                thePFace.Perform(FFrom, FUntil);
            }
            else if (FFrom.IsNull)
            {
                if (!FUntil.IsNull)
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
        if (!thePFace.IsDone)
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
        Shape S = theDI.GetShape(a[3]);
        global::Kerf.Engine.Features.MakePrism thePFace = new();
        Vector3D V = new();
        Shape FFrom = new(), FUntil = new();
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
                V = V.WithCoord(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));
                FFrom = theDI.GetShape(a[4], ShapeType.Shape);
                FUntil = theDI.GetShape(a[5], ShapeType.Shape);
                borne = 9;
            }
            else
            {
                V = V.WithCoord(theDI.Atof(a[5]), theDI.Atof(a[6]), theDI.Atof(a[7]));
                FUntil = theDI.GetShape(a[4], ShapeType.Shape);
                borne = 8;
            }
        }
        else
        {
            V = V.WithCoord(theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]));
            borne = 7;
        }
        double Length = V.Magnitude;
        if (Length < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return 1;
        }

        Shape aLocalShape = theDI.GetShape(a[borne], ShapeType.Face);
        Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(aLocalShape);
        global::Kerf.Engine.Features.SplitShape Spls = new(F);
        for (int i = borne + 1; i < narg; i++)
        {
            Wire wir;
            if (At(a[i], 0) != '-')
            {
                aLocalShape = theDI.GetShape(a[i], ShapeType.Wire);
                wir = global::Kerf.Engine.Topology.ShapeCasts.Wire(aLocalShape);
            }
            else
            {
                if (At(a[i], 1) == '\0')
                {
                    return 1;
                }
                string Temp = a[i][1..];
                aLocalShape = theDI.GetShape(Temp, ShapeType.Wire);
                wir = global::Kerf.Engine.Topology.ShapeCasts.Wire(aLocalShape);
                wir.Reverse();
            }
            Spls.Add(wir, F);
        }
        Spls.Build();
        Shape ToPrism;
        IReadOnlyList<Shape> lleft = Spls.DirectLeft();
        if (lleft.Count == 1)
        {
            thePFace.Init(S, lleft[0], F, new Direction3D(V), fuse ? 1 : 0, true);
            ToPrism = lleft[0];
        }
        else
        {
            BrepBuilder B = new();
            Shell Sh = new();
            B.MakeShell(ref Sh);
            foreach (Shape aLeft in lleft)
            {
                B.Add(Sh, global::Kerf.Engine.Topology.ShapeCasts.Face(aLeft));
            }
            Sh.Closed(Tool.IsClosed(Sh));
            thePFace.Init(S, Sh, F, new Direction3D(V), fuse ? 1 : 0, true);
            ToPrism = Sh;
        }

        // Recherche des faces de glissement, si on n`a pas sketche sur une face
        // du shape de depart
        global::Kerf.Engine.Topology.Explorer exp = new(S, ShapeType.Face);
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
        Shape S = theDI.GetShape(a[3]);
        global::Kerf.Engine.Features.MakePrism thePFace = new();
        int borne;
        Vector3D V = new();
        Shape FFrom = new(), FUntil = new();
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
                V = V.WithCoord(theDI.Atof(a[6]), theDI.Atof(a[7]), theDI.Atof(a[8]));
                FFrom = theDI.GetShape(a[4], ShapeType.Shape);
                FUntil = theDI.GetShape(a[5], ShapeType.Shape);
            }
            else
            {
                borne = 8;
                V = V.WithCoord(theDI.Atof(a[5]), theDI.Atof(a[6]), theDI.Atof(a[7]));
                FUntil = theDI.GetShape(a[4], ShapeType.Shape);
            }
        }
        else
        {
            borne = 7;
            V = V.WithCoord(theDI.Atof(a[4]), theDI.Atof(a[5]), theDI.Atof(a[6]));
        }
        double Length = V.Magnitude;
        if (Length < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return 1;
        }

        Shape ToPrism;
        if (narg == borne + 1)
        {
            Shape aLocalShape = theDI.GetShape(a[borne], ShapeType.Face);
            Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(aLocalShape);
            thePFace.Init(S, F, F, new Direction3D(V), fuse ? 1 : 0, true);
            ToPrism = F;
        }
        else
        {
            Shell She = new();
            BrepBuilder B = new();
            B.MakeShell(ref She);
            for (int i = borne; i < narg; i++)
            {
                Shape aLocalShape = theDI.GetShape(a[i], ShapeType.Face);
                Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(aLocalShape);
                if (!F.IsNull)
                {
                    B.Add(She, F);
                }
            }
            She.Closed(Tool.IsClosed(She));
            thePFace.Init(S, She, new Face(), new Direction3D(V), fuse ? 1 : 0, false);
            ToPrism = She;
        }

        // Recherche des faces de glissement, on ne prisme pas une face
        // du shape de depart
        global::Kerf.Engine.Topology.Explorer exp = new(ToPrism, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            global::Kerf.Engine.Topology.Explorer exp2 = new(S, ShapeType.Face);
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

        Shape S = theDI.GetShape(a[2]);
        if (S.IsNull)
        {
            theDI.Output.WriteLine("Invalid input shape " + a[2]);
            return 1;
        }
        global::Kerf.Engine.Features.SplitShape Spls = new(S);
        Shape EF = new();
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
        NCollection_Sequence<Shape> aSplitEdges = new();
        if (i < newnarg)
        {
            // the interactive pick of a name starting with a dot is left out
            Shape aSh = theDI.GetShape(a[i]);
            if (aSh.IsNull)
            {
                theDI.Output.WriteLine("Invalid input shape " + a[i]);
                return 1;
            }
            if (aSh.Type == ShapeType.Face)
            {
                EF = global::Kerf.Engine.Topology.ShapeCasts.Face(aSh);
            }
            else
            {
                if (aSh.Type == ShapeType.Compound || aSh.Type == ShapeType.Wire || aSh.Type == ShapeType.Edge)
                {
                    global::Kerf.Engine.Topology.Explorer aExpE = new(aSh, ShapeType.Edge, ShapeType.Face);
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
            if (!isSplittingEdges && !EF.IsNull && EF.Type == ShapeType.Face)
            {
                // face wire/edge ...

                while (i < newnarg)
                {
                    Shape W;
                    bool rever = false;
                    if (At(a[i], 0) == '-')
                    {
                        if (At(a[i], 1) == '\0')
                        {
                            return 1;
                        }
                        string Temp = a[i][1..];
                        W = theDI.GetShape(Temp, ShapeType.Shape, false);
                        rever = true;
                    }
                    else
                    {
                        W = theDI.GetShape(a[i], ShapeType.Shape, false);
                    }

                    if (W.IsNull)
                    {
                        return 1; // on n`a rien recupere
                    }
                    ShapeType wtyp = W.Type;
                    if (wtyp != ShapeType.Wire && wtyp != ShapeType.Edge && wtyp != ShapeType.Compound)
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
                        if (wtyp == ShapeType.Wire)
                        {
                            Spls.Add(global::Kerf.Engine.Topology.ShapeCasts.Wire(W), global::Kerf.Engine.Topology.ShapeCasts.Face(EF));
                        }
                        else if (wtyp == ShapeType.Edge)
                        {
                            Spls.Add(global::Kerf.Engine.Topology.ShapeCasts.Edge(W), global::Kerf.Engine.Topology.ShapeCasts.Face(EF));
                        }
                        else
                        {
                            Spls.Add(global::Kerf.Engine.Topology.ShapeCasts.Compound(W), global::Kerf.Engine.Topology.ShapeCasts.Face(EF));
                        }
                    }
                    i++;
                }
            }
            else
            {
                if (isSplittingEdges)
                {
                    Shape aSh = theDI.GetShape(a[i]);
                    if (aSh.IsNull)
                    {
                        theDI.Output.WriteLine("Invalid input shape " + a[i]);
                        return 1;
                    }
                    global::Kerf.Engine.Topology.Explorer aExpE = new(aSh, ShapeType.Edge, ShapeType.Face);
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
            Shape Ew, Es;
            Shape aLocalShape = theDI.GetShape(a[i], ShapeType.Edge);
            Es = global::Kerf.Engine.Topology.ShapeCasts.Edge(aLocalShape);
            if (Es.IsNull)
            {
                return 1;
            }
            aLocalShape = theDI.GetShape(a[i + 1], ShapeType.Edge);
            Ew = global::Kerf.Engine.Topology.ShapeCasts.Edge(aLocalShape);
            if (Ew.IsNull)
            {
                theDI.Output.WriteLine("Invalid input shape " + a[i + 1]);
                return 1;
            }
            Spls.Add(global::Kerf.Engine.Topology.ShapeCasts.Edge(Ew), global::Kerf.Engine.Topology.ShapeCasts.Edge(Es));
        }

        theDI.SetShape(a[1], Spls.Shape());
        return 0;
    }

    // The angle or the faces of wrotate and frotate: returns the index of the axis' first word
    private static int RotateBounds(DrawInterpreter theDI, IReadOnlyList<string> a, ref double Angle,
                                    ref Shape FFrom, ref Shape FUntil, int theMinArgs)
    {
        int i;
        FFrom = theDI.GetShape(a[4], ShapeType.Shape);
        if (FFrom.IsNull)
        {
            Angle = theDI.Atof(a[4]);
            Angle *= M_PI / 180.0;
            i = 5;
        }
        else
        {
            FUntil = theDI.GetShape(a[5], ShapeType.Shape);
            if (FUntil.IsNull)
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
    private static void AddSlidingFacesForRevol(in Shape S, in Shape ToRotate, in Axis3D theAxis,
                                                global::Kerf.Engine.Features.MakeRevol theRFace)
    {
        FindEdgesInFace FEIF = new();
        for (global::Kerf.Engine.Topology.Explorer exp = new(S, ShapeType.Face); exp.More(); exp.Next())
        {
            Face fac = global::Kerf.Engine.Topology.ShapeCasts.Face(exp.Current());
            Surface Su = Tool.Surface(fac)!;
            if (Su.GetType() == typeof(TrimmedSurface))
            {
                Su = ((TrimmedSurface)Su).BasisSurface;
            }
            if (Su.GetType() == typeof(PlaneSurface))
            {
                Plane3D pl = ((PlaneSurface)Su).Plane;
                if (pl.Axis.IsParallel(theAxis, global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    FEIF.Set(ToRotate, fac);
                    for (FEIF.Init(); FEIF.More(); FEIF.Next())
                    {
                        theRFace.Add(FEIF.Edge(), fac);
                    }
                }
            }
            else if (Su.GetType() == typeof(CylindricalSurface))
            {
                Cylinder3D cy = ((CylindricalSurface)Su).Cylinder;
                if (cy.Axis.IsCoaxial(theAxis, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion()))
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
                                    global::Kerf.Engine.Features.MakeRevol theRFace, int borne, double Angle, in Shape FFrom,
                                    in Shape FUntil)
    {
        if (borne == 11)
        {
            if (FUntil.IsNull)
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

        if (!theRFace.IsDone)
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

        Shape S = theDI.GetShape(a[3]);
        global::Kerf.Engine.Features.MakeRevol theRFace = new();
        double Angle = 0;
        Shape FFrom = new(), FUntil = new();
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
        Point3D Or = new(theDI.Atof(a[i]), theDI.Atof(a[i + 1]), theDI.Atof(a[i + 2]));
        Direction3D D = new(theDI.Atof(a[i + 3]), theDI.Atof(a[i + 4]), theDI.Atof(a[i + 5]));
        Axis3D theAxis = new(Or, D);

        Shape aLocalShape = theDI.GetShape(a[borne], ShapeType.Face);
        Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(aLocalShape);
        global::Kerf.Engine.Features.SplitShape Spls = new(F);
        for (i = borne + 1; i < narg; i++)
        {
            Wire wir;
            if (At(a[i], 0) != '-')
            {
                aLocalShape = theDI.GetShape(a[i], ShapeType.Wire);
                wir = global::Kerf.Engine.Topology.ShapeCasts.Wire(aLocalShape);
            }
            else
            {
                if (At(a[i], 1) == '\0')
                {
                    return 1;
                }
                string Temp = a[i][1..];
                aLocalShape = theDI.GetShape(Temp, ShapeType.Wire);
                wir = global::Kerf.Engine.Topology.ShapeCasts.Wire(aLocalShape);
                wir.Reverse();
            }
            Spls.Add(wir, F);
        }
        Spls.Build();
        Shape ToRotate;
        IReadOnlyList<Shape> lleft = Spls.DirectLeft();
        if (lleft.Count == 1)
        {
            theRFace.Init(S, lleft[0], F, theAxis, fuse ? 1 : 0, true);
            ToRotate = lleft[0];
        }
        else
        {
            BrepBuilder B = new();
            Shell Sh = new();
            B.MakeShell(ref Sh);
            foreach (Shape aLeft in lleft)
            {
                B.Add(Sh, global::Kerf.Engine.Topology.ShapeCasts.Face(aLeft));
            }
            Sh.Closed(Tool.IsClosed(Sh));
            theRFace.Init(S, Sh, F, theAxis, fuse ? 1 : 0, true);
            ToRotate = Sh;
        }

        // Recherche des faces de glissement
        global::Kerf.Engine.Topology.Explorer exp = new(S, ShapeType.Face);
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

        Shape S = theDI.GetShape(a[3]);
        global::Kerf.Engine.Features.MakeRevol theRFace = new();
        double Angle = 0;
        Shape FFrom = new(), FUntil = new();
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
        Point3D Or = new(theDI.Atof(a[i]), theDI.Atof(a[i + 1]), theDI.Atof(a[i + 2]));
        Direction3D D = new(theDI.Atof(a[i + 3]), theDI.Atof(a[i + 4]), theDI.Atof(a[i + 5]));
        Axis3D theAxis = new(Or, D);

        Shape ToRotate;
        if (narg == borne + 1)
        {
            Shape aLocalShape = theDI.GetShape(a[borne], ShapeType.Face);
            Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(aLocalShape);
            theRFace.Init(S, F, F, theAxis, fuse ? 1 : 0, true);
            ToRotate = F;
        }
        else
        {
            Shell She = new();
            BrepBuilder B = new();
            B.MakeShell(ref She);

            for (i = borne; i < narg; i++)
            {
                Shape aLocalShape = theDI.GetShape(a[i], ShapeType.Face);
                Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(aLocalShape);
                if (!F.IsNull)
                {
                    B.Add(She, F);
                }
            }
            She.Closed(Tool.IsClosed(She));
            theRFace.Init(S, She, new Face(), theAxis, fuse ? 1 : 0, false);
            ToRotate = She;
        }

        global::Kerf.Engine.Topology.Explorer exp = new(ToRotate, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            global::Kerf.Engine.Topology.Explorer exp2 = new(S, ShapeType.Face);
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
        Shape Sne = theDI.GetShape(a[2]);
        Shape Sba = theDI.GetShape(a[3]);

        global::Kerf.Engine.Features.Gluer theGl = new(Sne, Sba);
        Shape Fne, Fba;

        FindEdges fined = new();

        int i = 4;
        bool first = true;
        while (i < narg)
        {
            // the interactive picks of names starting with a dot are left out
            Fne = theDI.GetShape(a[i]);
            if (Fne.IsNull)
            {
                return 1;
            }
            ShapeType sht = Fne.Type;
            if (first && sht != ShapeType.Face)
            {
                return 1;
            }
            first = false;
            Fba = theDI.GetShape(a[i + 1]);
            if (Fba.IsNull)
            {
                return 1;
            }
            if (Fba.Type != sht)
            {
                return 1;
            }
            if (sht == ShapeType.Face)
            {
                Face f1 = global::Kerf.Engine.Topology.ShapeCasts.Face(Fne);
                Face f2 = global::Kerf.Engine.Topology.ShapeCasts.Face(Fba);
                theGl.Bind(f1, f2);
                fined.Set(Fne, Fba);
                for (fined.InitIterator(); fined.More(); fined.Next())
                {
                    theGl.Bind(fined.EdgeFrom(), fined.EdgeTo());
                }
            }
            else
            {
                theGl.Bind(global::Kerf.Engine.Topology.ShapeCasts.Edge(Fne), global::Kerf.Engine.Topology.ShapeCasts.Edge(Fba));
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
        Shape Sbase = theDI.GetShape(a[1]);
        if (Sbase.IsNull)
        {
            theCommands.Append("null basis shape");
            return 1;
        }
        int Ifuse = theDI.Atoi(a[narg - 2]);
        int Imodif = theDI.Atoi(a[narg - 1]);

        int Fuse = Ifuse;
        bool Modify = Imodif != 0;

        Shape Pbase = new();
        Face Skface = new();
        Wire W = new();

        PlaneSurface? P = null;

        global::Kerf.Engine.Features.StatusError se;

        if (!EqualsNoCase(a0, "FEATLF") && !EqualsNoCase(a0, "FEATRF"))
        {
            Pbase = theDI.GetShape(a[2]);
            if (Pbase.IsNull)
            {
                theCommands.Append("null shape to transform");
                return 1;
            }
            Shape aLocalShape = theDI.GetShape(a[3], ShapeType.Face);
            Skface = global::Kerf.Engine.Topology.ShapeCasts.Face(aLocalShape);
            if (Skface.IsNull)
            {
                theCommands.Append("null face of Sketch");
                return 1;
            }
        }
        else
        {
            Shape aLocalShape = theDI.GetShape(a[2], ShapeType.Wire);
            W = global::Kerf.Engine.Topology.ShapeCasts.Wire(aLocalShape);
            if (W.IsNull)
            {
                theCommands.Append("null profile for rib or slot");
                return 1;
            }
            Surface? s = DrawTrSurf.GetSurface(theDI, a[3]);
            P = s as PlaneSurface;
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
                aState.Prism.Init(Sbase, Pbase, Skface, new Direction3D(X, Y, Z), Fuse, Modify);
            }
            else if (narg == 14)
            {
                aState.rfdef = true;
                Point3D Or = new(X, Y, Z);
                X = theDI.Atof(a[7]);
                Y = theDI.Atof(a[8]);
                Z = theDI.Atof(a[9]);
                double H1 = theDI.Atof(a[10]);
                double H2 = theDI.Atof(a[11]);
                Axis3D ax1 = new(Or, new Direction3D(X, Y, Z));
                aState.RevolutionForm.Init(Sbase, W, P!, ax1, H1, H2, Fuse, Modify);
                if (!aState.RevolutionForm.IsDone)
                {
                    se = aState.RevolutionForm.CurrentStatusError();
                    theCommands.Append(global::Kerf.Engine.Features.BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
            }
            else if (narg == 12 && !EqualsNoCase(a0, "FEATLF"))
            {
                aState.rvdef = true;
                Point3D Or = new(X, Y, Z);
                X = theDI.Atof(a[7]);
                Y = theDI.Atof(a[8]);
                Z = theDI.Atof(a[9]);
                aState.theSbase = Sbase;
                aState.thePbase = Pbase;
                aState.theSkface = Skface;
                aState.Revol.Init(Sbase, Pbase, Skface, new Axis3D(Or, new Direction3D(X, Y, Z)), Fuse, Modify);
            }
            else
            {
                aState.lfdef = true;
                Vector3D Direct = new(X, Y, Z);
                X = theDI.Atof(a[7]);
                Y = theDI.Atof(a[8]);
                Z = theDI.Atof(a[9]);
                aState.LinearForm.Init(Sbase, W, P!, Direct, new Vector3D(X, Y, Z), Fuse, Modify);
                if (!aState.LinearForm.IsDone)
                {
                    se = aState.LinearForm.CurrentStatusError();
                    theCommands.Append(global::Kerf.Engine.Features.BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
            }
        }
        else if (narg == 7)
        {
            if (EqualsNoCase(a0, "FEATDPRISM"))
            {
                if (Pbase.Type != ShapeType.Face)
                {
                    theCommands.Append("Invalid DPrism base");
                    return 1;
                }
                double Angle = theDI.Atof(a[4]) * M_PI / 360;
                aState.dprdef = true;
                aState.theSbase = Sbase;
                aState.thePbase = Pbase;
                aState.theSkface = Skface;
                aState.DPrism.Init(Sbase, global::Kerf.Engine.Topology.ShapeCasts.Face(Pbase), Skface, Angle, Fuse, Modify);
            }
            else
            { // FEATPIPE
                Shape aLocalShape = theDI.GetShape(a[4], ShapeType.Wire);
                Wire Spine = global::Kerf.Engine.Topology.ShapeCasts.Wire(aLocalShape);
                if (Spine.IsNull)
                {
                    Shape Edspine = theDI.GetShape(a[4], ShapeType.Edge);
                    if (Edspine.IsNull)
                    {
                        theCommands.Append("null spine");
                        return 1;
                    }
                    BrepBuilder B = new();
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
                Shape aLocalShape = theDI.GetShape(a[i], ShapeType.Edge);
                Edge edg = global::Kerf.Engine.Topology.ShapeCasts.Edge(aLocalShape);
                if (edg.IsNull)
                {
                    return 1;
                }
                aLocalShape = theDI.GetShape(a[i + 1], ShapeType.Face);
                Face fac = global::Kerf.Engine.Topology.ShapeCasts.Face(aLocalShape);
                if (fac.IsNull)
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
        NCollection_List<Shape> anArgs = new();

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
                Shape FUntil = theDI.GetShape(a[4], ShapeType.Shape);
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
                Shape Funtil = theDI.GetShape(a[3], ShapeType.Shape);
                switch (Kas)
                {
                    case 1:
                    {
                        if (Funtil.IsNull)
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
                        if (!Funtil.IsNull)
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
                        if (!Funtil.IsNull)
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
                        if (!Funtil.IsNull)
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
                Shape Ffrom = theDI.GetShape(a[3], ShapeType.Shape);
                Shape Funtil = theDI.GetShape(a[4], ShapeType.Shape);
                if (Funtil.IsNull)
                {
                    return 1;
                }
                switch (Kas)
                {
                    case 1:
                    {
                        if (Ffrom.IsNull)
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
                        if (Ffrom.IsNull)
                        {
                            return 1;
                        }
                        aState.Revol.Perform(Ffrom, Funtil);
                    }
                    break;
                    case 3:
                    {
                        if (Ffrom.IsNull)
                        {
                            return 1;
                        }
                        aState.Pipe.Perform(Ffrom, Funtil);
                    }
                    break;
                    case 4:
                    {
                        if (Ffrom.IsNull)
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

        global::Kerf.Engine.Features.StatusError se;
        switch (Kas)
        {
            case 1:
                if (!aState.Prism.IsDone)
                {
                    se = aState.Prism.CurrentStatusError();
                    theCommands.Append(global::Kerf.Engine.Features.BRepFeat.Description(se)).Append("\n");
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
                if (!aState.Revol.IsDone)
                {
                    se = aState.Revol.CurrentStatusError();
                    theCommands.Append(global::Kerf.Engine.Features.BRepFeat.Description(se)).Append("\n");
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
                if (!aState.Pipe.IsDone)
                {
                    se = aState.Pipe.CurrentStatusError();
                    theCommands.Append(global::Kerf.Engine.Features.BRepFeat.Description(se)).Append("\n");
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
                if (!aState.DPrism.IsDone)
                {
                    se = aState.DPrism.CurrentStatusError();
                    theCommands.Append(global::Kerf.Engine.Features.BRepFeat.Description(se)).Append("\n");
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
                if (!aState.LinearForm.IsDone)
                {
                    se = aState.LinearForm.CurrentStatusError();
                    theCommands.Append(global::Kerf.Engine.Features.BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
                theDI.SetShape(a[2], aState.LinearForm.Shape());
                return 0;
            case 6:
                if (!aState.RevolutionForm.IsDone)
                {
                    se = aState.RevolutionForm.CurrentStatusError();
                    theCommands.Append(global::Kerf.Engine.Features.BRepFeat.Description(se)).Append("\n");
                    return 1;
                }
                theDI.SetShape(a[2], aState.RevolutionForm.Shape());
                return 0;
            default:
                return 1;
        }
    }

    // BOSS: endedges, fillet and bossage
    private static int BOSS(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (!EqualsNoCase(a[0], "ENDEDGES") && !EqualsNoCase(a[0], "FILLET") && !EqualsNoCase(a[0], "BOSSAGE"))
        {
            return 1;
        }

        if ((EqualsNoCase(a[0], "ENDEDGES") && narg != 5) || (EqualsNoCase(a[0], "FILLET") && (narg < 5 || narg % 2 != 1))
            || (EqualsNoCase(a[0], "BOSSAGE") && narg != 6))
        {
            // theCommands.PrintHelp(a[0])
            theCommands.Append(a[0]).Append(" : ").Append(BossHelp(a[0])).Append("\n");
            return 1;
        }

        FeatureState aState = State(theDI);
        int Kas = 0;
        int dprsig = 0;
        if (EqualsNoCase("ENDEDGES", a[0]))
        {
            Kas = 1;
            dprsig = theDI.Atoi(a[4]);
        }
        else if (EqualsNoCase("FILLET", a[0]))
        {
            Kas = 2;
        }
        else if (EqualsNoCase("BOSSAGE", a[0]))
        {
            Kas = 3;
            dprsig = theDI.Atoi(a[5]);
        }

        Compound theShapeTop = new();
        Compound theShapeBottom = new();

        if (Kas == 1 || Kas == 3)
        {
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

            NCollection_List<Shape> theTopEdges = new(aState.DPrism.InternalTopEdges());
            NCollection_List<Shape> theLatEdges = new(aState.DPrism.InternalLatEdges());

            NCollection_List<Shape>.Iterator it = new();
            BrepBuilder B = new();

            B.MakeCompound(ref theShapeTop);
            it.Initialize(theTopEdges);
            for (; it.More(); it.Next())
            {
                global::Kerf.Engine.Topology.Explorer exp = new();
                for (exp.Init(it.Value(), ShapeType.Edge); exp.More(); exp.Next())
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
            if (Kas == 1)
            {
                return 0;
            }
        }

        // Kas == 2 || Kas == 3
        Shape V = new();
        if (Kas == 2)
        {
            V = theDI.GetShape(a[2], ShapeType.Shape);
        }
        else
        {
            V = aState.DPrism.Shape();
        }

        if (V.IsNull)
        {
            return 1;
        }
        global::Kerf.Fillets.FilletShape FSh = global::Kerf.Fillets.FilletShape.Rational;
        // the file's own tolerances (OCCT's file-scope statics, which no command of this file changes)
        const double tesp = 1.0e-4, t3d = 1.0e-4, t2d = 1.0e-5, ta = 1.0e-2, fl = 1.0e-3, tapp_angle = 1.0e-2;
        MakeFillet Rakk = new(V, FSh);
        aState.Rakk = Rakk;
        Rakk.SetParams(ta, tesp, t2d, t3d, t2d, fl);
        Rakk.SetContinuity(Continuity.C1, tapp_angle);
        int nbedge = 0;

        if (Kas == 2)
        {
            for (int ii = 1; ii < (narg - 1) / 2; ii++)
            {
                double Rad = theDI.Atof(a[2 * ii + 1]);
                if (Rad == 0.0)
                {
                    continue;
                }
                Shape S = theDI.GetShape(a[2 * ii + 2], ShapeType.Shape);
                global::Kerf.Engine.Topology.Explorer exp = new();
                for (exp.Init(S, ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(exp.Current());
                    if (!E.IsNull)
                    {
                        Rakk.Add(Rad, E);
                        nbedge++;
                    }
                }
            }
        }
        else
        {
            double Rad = theDI.Atof(a[3]);
            if (Rad != 0.0)
            {
                global::Kerf.Engine.Topology.Explorer exp = new();
                for (exp.Init(theShapeTop, ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(exp.Current());
                    if (!E.IsNull)
                    {
                        Rakk.Add(Rad, E);
                        nbedge++;
                    }
                }
            }
            Rad = theDI.Atof(a[4]);
            if (Rad != 0.0)
            {
                global::Kerf.Engine.Topology.Explorer exp = new();
                for (exp.Init(theShapeBottom, ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(exp.Current());
                    if (!E.IsNull)
                    {
                        Rakk.Add(Rad, E);
                        nbedge++;
                    }
                }
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
        Shape res = Rakk.Shape();

        theDI.SetShape(Kas == 2 ? a[1] : a[2], res);

        // Save history for fillet
        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            NCollection_List<Shape> anArg = new();
            anArg.Append(V);
            BRepTestHistoryCommands.SetHistory(theDI, anArg, Rakk);
        }

        return 0;
    }

    private static string BossHelp(string theName) => theName.ToUpperInvariant() switch
    {
        "ENDEDGES" => " Return top and bottom edges of dprism :endedges dprism shapetop shapebottom First/LastShape (1/2)",
        "FILLET" => " Perform fillet on compounds of edges :fillet result object rad1 comp1 rad2 comp2 ...",
        _ => " Perform fillet on top and bottom edges of dprism :bossage dprism result radtop radbottom First/LastShape (1/2)",
    };
}
