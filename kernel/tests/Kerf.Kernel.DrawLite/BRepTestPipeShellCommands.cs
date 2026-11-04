// Port of: src/Draw/TKTopTest/BRepTest/BRepTest_SweepCommands.cxx (pruled, gener, thrusections, the sweep commands, geompipe, middlepath)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using Occt;
using static Kerf.Engine.Construction.Api.PipeError;
using static Kerf.Construction.TransitionMode;
using static Kerf.Engine.Construction.Filling.ThruSectionErrorStatus;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The rest of BRepTest_SweepCommands.cxx (unit 5W): <c>pruled</c> (BRepFill::Face and Shell), <c>gener</c>
/// (BRepFill_Generator), <c>thrusections</c> (BRepOffsetAPI_ThruSections), the sweep commands <c>mksweep</c>,
/// <c>setsweep</c>, <c>addsweep</c>, <c>deletesweep</c>, <c>buildsweep</c>, <c>errorsweep</c> and <c>simulsweep</c>
/// (BRepOffsetAPI_MakePipeShell), <c>geompipe</c> (GeomFill_Pipe along a guide) and <c>middlepath</c>
/// (BRepOffsetAPI_MiddlePath). <c>prism</c>, <c>revol</c>, <c>pipe</c> and <c>evolved</c> are
/// <see cref="BRepTestSweepCommands"/>.
/// </summary>
/// <remarks>
/// <para>OCCT keeps the sweep maker and the loft of the last <c>thrusections</c> in two process-wide statics
/// (<c>Sweep</c>, <c>Generator</c>); Draw-lite keeps them per interpreter, as <see cref="BRepTestHistoryCommands"/> keeps
/// its session, so the corpus's cases can run side by side.</para>
/// <para><c>errorsweep</c> and <c>deletesweep</c> without a <c>mksweep</c> dereference the null maker natively (an access
/// violation, which DRAWEXE's signal handler reports); in Draw-lite it is a .NET exception, a Tcl error.</para>
/// </remarks>
internal static class BRepTestPipeShellCommands
{
    private sealed class Session
    {
        public MakePipeShell? Sweep;
        public ThruSections? Generator;
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, Session> theSessions = new();

    private static Session Of(DrawInterpreter theDI) => theSessions.GetValue(theDI, static _ => new Session());

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "pruled", pruled);
        DrawCxx.Add(theInterpreter, "gener", gener);
        DrawCxx.Add(theInterpreter, "thrusections", thrusections);
        DrawCxx.Add(theInterpreter, "mksweep", mksweep);
        DrawCxx.Add(theInterpreter, "setsweep", setsweep);
        DrawCxx.Add(theInterpreter, "addsweep", addsweep);
        DrawCxx.Add(theInterpreter, "deletesweep", deletesweep);
        DrawCxx.Add(theInterpreter, "buildsweep", buildsweep);
        DrawCxx.Add(theInterpreter, "errorsweep", errorsweep);
        DrawCxx.Add(theInterpreter, "simulsweep", simulsweep);
        DrawCxx.Add(theInterpreter, "geompipe", geompipe);
        DrawCxx.Add(theInterpreter, "middlepath", middlepath);
    }

    private static int geompipe(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        Shape Spine = theDI.GetShape(a[2], ShapeType.Edge);
        if (Spine.IsNull)
        {
            return 1;
        }
        if (n < 5)
        {
            return 1;
        }
        Shape Profile = theDI.GetShape(a[3], ShapeType.Edge);
        if (Profile.IsNull)
        {
            return 1;
        }
        Curve3D? SpineCurve = Tool.Curve(global::Kerf.Engine.Topology.ShapeCasts.Edge(Spine), out double aSpFirst, out double aSpLast);
        Curve3D? ProfileCurve = Tool.Curve(global::Kerf.Engine.Topology.ShapeCasts.Edge(Profile), out _, out _);
        var aAdaptCurve = new CurveAdaptor(SpineCurve!, aSpFirst, aSpLast);
        bool ByACR = false;
        bool rotate = false;
        double Radius = theDI.Atof(a[4]);
        ProfileCurve!.D1(aSpFirst, out Point3D ctr, out Vector3D norm);
        var xAxisStart = new Vector3D(ctr, SpineCurve!.Value(aSpFirst));
        var aAx2Start = new Frame3D(ctr, new Direction3D(norm), new Direction3D(xAxisStart));
        var cStart = new CircleCurve3D(aAx2Start, Radius);
        int k = 5;
        if (n > k)
        {
            ByACR = theDI.Atoi(a[k++]) == 1;
        }
        if (n > k)
        {
            rotate = theDI.Atoi(a[k++]) == 1;
        }
        var aPipe = new SurfacePipe(ProfileCurve, aAdaptCurve, cStart, ByACR, rotate);
        aPipe.Perform(true);
        if (!aPipe.IsDone)
        {
            theDI.Output.WriteLine("GeomFill_Pipe cannot make a surface");
            return 1;
        }

        double Accuracy = aPipe.ErrorOnSurf();
        theDI.Output.WriteLine("Accuracy of approximation = " + global::Kerf.Engine.Foundation.StreamUtilities.FormatReal(Accuracy, 6));

        Surface? Sur = aPipe.Surface();
        Face F = new();
        if (Sur is not null)
        {
            F = new MakeFace(Sur, global::Kerf.Engine.Foundation.Precision.Confusion()).Face();
        }
        theDI.SetShape(a[1], F);
        return 0;
    }

    private static int pruled(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 4)
        {
            return 1;
        }

        bool YaWIRE = false;
        Shape S1 = theDI.GetShape(a[2], ShapeType.Edge);
        if (S1.IsNull)
        {
            S1 = theDI.GetShape(a[2], ShapeType.Wire);
            if (S1.IsNull)
            {
                return 1;
            }
            YaWIRE = true;
        }

        Shape S2 = theDI.GetShape(a[3], ShapeType.Edge);
        if (S2.IsNull)
        {
            S2 = theDI.GetShape(a[3], ShapeType.Wire);
            if (S2.IsNull)
            {
                return 1;
            }
            if (!YaWIRE)
            {
                S1 = new LibraryMakeWire(global::Kerf.Engine.Topology.ShapeCasts.Edge(S1)).Wire();
                YaWIRE = true;
            }
        }
        else if (YaWIRE)
        {
            S2 = new LibraryMakeWire(global::Kerf.Engine.Topology.ShapeCasts.Edge(S2)).Wire();
        }

        Shape Result;
        if (YaWIRE)
        {
            Result = Fill.Shell(global::Kerf.Engine.Topology.ShapeCasts.Wire(S1), global::Kerf.Engine.Topology.ShapeCasts.Wire(S2));
        }
        else
        {
            Result = Fill.Face(global::Kerf.Engine.Topology.ShapeCasts.Edge(S1), global::Kerf.Engine.Topology.ShapeCasts.Edge(S2));
        }

        theDI.SetShape(a[1], Result);
        return 0;
    }

    // gener: a surface between generating wires
    private static int gener(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        Shape Shape;

        var aGenerator = new Generator();

        for (int i = 2; i <= n - 1; i++)
        {
            Shape = theDI.GetShape(a[i], ShapeType.Wire);
            if (Shape.IsNull)
            {
                return 1;
            }

            aGenerator.AddWire(global::Kerf.Engine.Topology.ShapeCasts.Wire(Shape));
        }

        aGenerator.Perform();

        Shell Shell = aGenerator.Shell();

        theDI.SetShape(a[1], Shell);

        return 0;
    }

    private static int thrusections(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6)
        {
            return 1;
        }

        bool check = true;
        bool samenumber = true;
        int index = 2;
        // Lecture option
        if (a[1] == "-N")
        {
            if (n < 7)
            {
                return 1;
            }
            check = false;
            index++;
        }

        Shape Shape;

        bool issolid = theDI.Atoi(a[index]) == 1;
        bool isruled = theDI.Atoi(a[index + 1]) == 1;

        Session aSession = Of(theDI);
        aSession.Generator = new ThruSections(issolid, isruled);
        ThruSections Generator = aSession.Generator;
        bool IsMutableInput = true;
        int NbEdges = 0;
        bool IsFirstWire = false;
        for (int i = index + 2; i <= n - 1; i++)
        {
            if (a[i] == "-safe")
            {
                IsMutableInput = false;
                continue;
            }
            bool IsWire = true;
            Shape = theDI.GetShape(a[i], ShapeType.Wire);
            if (!Shape.IsNull)
            {
                Generator.AddWire(global::Kerf.Engine.Topology.ShapeCasts.Wire(Shape));
                IsFirstWire = !IsFirstWire;
            }
            else
            {
                Shape = theDI.GetShape(a[i], ShapeType.Vertex);
                IsWire = false;
                if (!Shape.IsNull)
                {
                    Generator.AddVertex(global::Kerf.Engine.Topology.ShapeCasts.Vertex(Shape));
                }
                else
                {
                    return 1;
                }
            }

            int cpt = 0;
            var PE = new global::Kerf.Engine.Topology.Explorer();
            for (PE.Init(Shape, ShapeType.Edge); PE.More(); PE.Next())
            {
                cpt++;
            }
            if (IsFirstWire)
            {
                NbEdges = cpt;
            }
            else if (IsWire && cpt != NbEdges)
            {
                samenumber = false;
            }
        }

        Generator.SetMutableInput(IsMutableInput);

        check = check || !samenumber;
        Generator.CheckCompatibility(check);

        Generator.Build();

        if (Generator.IsDone)
        {
            Shape Shell = Generator.Shape();
            theDI.SetShape(a[index - 1], Shell);
            // Save history of the lofting
            if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
            {
                var aList = new NCollection_List<Shape>();
                foreach (Shape aWire in Generator.Wires())
                {
                    aList.Append(aWire);
                }
                BRepTestHistoryCommands.SetHistory(theDI, aList, Generator);
            }
        }
        else
        {
            ThruSectionErrorStatus aStatus = Generator.GetStatus();
            switch (aStatus)
            {
                case ThruSectionErrorStatus.ThruSectionErrorStatus_NotDone:
                    di.Append("Algorithm is not done\n");
                    break;
                case ThruSectionErrorStatus.ThruSectionErrorStatus_NotSameTopology:
                    di.Append("The input profiles should be all closed or all opened\n");
                    break;
                case ThruSectionErrorStatus.ThruSectionErrorStatus_ProfilesInconsistent:
                    di.Append("Profiles inconsistent\n");
                    break;
                case ThruSectionErrorStatus.ThruSectionErrorStatus_WrongUsage:
                    di.Append("Wrong usage of punctual sections\n");
                    break;
                case ThruSectionErrorStatus.ThruSectionErrorStatus_Null3DCurve:
                    di.Append("Some edges have null 3d curve");
                    break;
                case ThruSectionErrorStatus.ThruSectionErrorStatus_Failed:
                    di.Append("Algorithm has failed\n");
                    break;
                default:
                    break;
            }
        }

        return 0;
    }

    private static int mksweep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 2 && n != 5)
        {
            return 1;
        }
        Shape Spine = theDI.GetShape(a[1], ShapeType.Wire);
        if (Spine.IsNull)
        {
            return 1;
        }
        Session aSession = Of(theDI);
        aSession.Sweep = null;

        if (n > 2 && n <= 5)
        {
            if (a[2] == "-C")
            {
                // the unifier's safe input mode copies the frozen spine
                var aUnif = new UnifySameDomain(Spine, true, false, true);

                double anAngTol = 5.0;
                double aLinTol = 0.1;

                if (n == 5)
                {
                    anAngTol = theDI.Atof(a[3]);
                    aLinTol = theDI.Atof(a[4]);
                }

                aUnif.SetAngularTolerance(anAngTol * Math.PI / 180.0);
                aUnif.SetLinearTolerance(aLinTol);
                aUnif.Build();
                Spine = aUnif.Shape();
                if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
                {
                    BRepTestHistoryCommands.SetHistory(theDI, aUnif.History());
                }
            }
            else
            {
                di.Append("To correct input spine use 'mksweep wire -C [AngTol LinTol]'\n");
                di.Append("By default, AngTol = 5, LinTol = 0.1");
                return 1;
            }
        }

        aSession.Sweep = new MakePipeShell(global::Kerf.Engine.Topology.ShapeCasts.Wire(Spine));
        return 0;
    }

    private static int setsweep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 1)
        {
            di.Append("setsweep options [arg1 [arg2 [...]]] : options are :\n");
            di.Append("   -FR : Tangent and Normal are given by Frenet trihedron\n");
            di.Append("   -CF : Tangente is given by Frenet,\n");
            di.Append("         the Normal is computed to minimize the torsion \n");
            di.Append("   -DT : discrete trihedron\n");
            di.Append("   -DX Surf : Tangent and Normal are given by Darboux trihedron,\n");
            di.Append("       Surf have to be a shell or a face\n");
            di.Append("   -CN dx dy dz : BiNormal is given by dx dy dz\n");
            di.Append("   -FX Tx Ty TZ [Nx Ny Nz] : Tangent and Normal are fixed\n");
            di.Append("   -G guide  0|1(Plan|ACR)  0|1|2(no contact|contact|contact on border) : with guide\n");
            di.Append("   -SM : Set the maximum degree of approximation\n");
            di.Append("         paramvalue more then 0 (100 by default)\n");
            di.Append("   -DM : Set the maximum number of span of approximation\n");
            di.Append("         paramvalue [1; 14] (11 by default)\n");
            return 0;
        }

        MakePipeShell? Sweep = Of(theDI).Sweep;
        if (Sweep is null)
        {
            di.Append("You have forgotten the <<mksweep>> command  !\n");
            return 1;
        }
        if (a[1] == "-FR")
        {
            Sweep.SetMode(true);
        }
        else if (a[1] == "-CF")
        {
            Sweep.SetMode(false);
        }
        else if (a[1] == "-DT")
        {
            Sweep.SetDiscreteMode();
        }
        else if (a[1] == "-DX")
        {
            if (n != 3)
            {
                di.Append("bad arguments !\n");
                return 1;
            }
            Shape Surf = theDI.GetShape(a[2], ShapeType.Shape);
            if (Surf.IsNull)
            {
                di.Append(a[2]).Append("is not a shape !\n");
                return 1;
            }
            Sweep.SetMode(Surf);
        }
        else if (a[1] == "-CN")
        {
            if (n != 5)
            {
                di.Append("bad arguments !\n");
                return 1;
            }
            var D = new Direction3D(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]));
            Sweep.SetMode(D);
        }
        else if (a[1] == "-FX")
        {
            if (n != 5 && n != 8)
            {
                di.Append("bad arguments !\n");
                return 1;
            }
            var D = new Direction3D(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]));
            if (n == 8)
            {
                var DN = new Direction3D(theDI.Atof(a[5]), theDI.Atof(a[6]), theDI.Atof(a[7]));
                var Axe = new Frame3D(new Point3D(0.0, 0.0, 0.0), D, DN);
                Sweep.SetMode(Axe);
            }
            else
            {
                var Axe = new Frame3D(new Point3D(0.0, 0.0, 0.0), D);
                Sweep.SetMode(Axe);
            }
        }
        else if (a[1] == "-G") // contour guide
        {
            if (n != 5)
            {
                di.Append("bad arguments !\n");
                return 1;
            }
            else
            {
                Shape Guide = theDI.GetShape(a[2], ShapeType.Wire);
                bool CurvilinearEquivalence = theDI.Atoi(a[3]) != 0;
                int KeepContact = theDI.Atoi(a[4]);
                Sweep.SetMode(global::Kerf.Engine.Topology.ShapeCasts.Wire(Guide), CurvilinearEquivalence, (TypeOfContact)KeepContact);
            }
        }
        else if (a[1] == "-DM")
        {
            if (n != 3)
            {
                di.Append("bad arguments !\n");
                return 1;
            }

            if (theDI.Atoi(a[2]) > 0 && theDI.Atoi(a[2]) < 15)
            {
                Sweep.SetMaxDegree(theDI.Atoi(a[2]));
            }
            else
            {
                di.Append(" -DM paramvalue must be [1; 14]\n");
                return 1;
            }
        }
        else if (a[1] == "-SM")
        {
            if (n != 3)
            {
                di.Append("bad arguments !\n");
                return 1;
            }

            if (theDI.Atoi(a[2]) > 0)
            {
                Sweep.SetMaxSegments(theDI.Atoi(a[2]));
            }
            else
            {
                di.Append(" -SM paramvalue must be more then 0\n");
                return 1;
            }
        }
        else
        {
            di.Append("The option ").Append(a[1]).Append(" is unknown !\n");
            return 1;
        }
        return 0;
    }

    private static int addsweep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 1)
        {
            di.Append("addsweep wire/vertex [Vertex] [-T] [-R] [u0 v0 u1 v1 [...[uN vN]]] : options are :\n");
            di.Append("   -T : the wire/vertex have to be translated to assume contact\n");
            di.Append("        with the spine\n");
            di.Append("   -R : the wire have to be rotated to assume orthogonality\n");
            di.Append("        with the spine's tangent\n");
            return 0;
        }

        MakePipeShell? Sweep = Of(theDI).Sweep;
        if (Sweep is null)
        {
            di.Append("You have forgotten the <<mksweep>> command  !\n");
            return 1;
        }

        Shape Section;
        Vertex Vertex = new();
        Interpol? thelaw = null;

        Section = theDI.GetShape(a[1], ShapeType.Shape);
        if (Section.IsNull
            || (Section.Type != ShapeType.Wire && Section.Type != ShapeType.Vertex))
        {
            di.Append(a[1]).Append(" is not a wire and is not a vertex!\n");
            return 1;
        }

        bool HasVertex = false, isT = false, isR = false;

        if (n > 2)
        {
            int cur = 2;
            // Reading of Vertex
            Shape InputVertex = theDI.GetShape(a[cur], ShapeType.Vertex);
            Vertex = global::Kerf.Engine.Topology.ShapeCasts.Vertex(InputVertex);
            if (!Vertex.IsNull)
            {
                cur++;
                HasVertex = true;
            }

            // Reading of the translation option
            if (n > cur && a[cur] == "-T")
            {
                cur++;
                isT = true;
            }

            // Reading of the rotation option
            if (n > cur && a[cur] == "-R")
            {
                cur++;
                isR = true;
            }

            // law ?
            if (n > cur)
            {
                int nbreal = n - cur;
                if (nbreal < 4 || nbreal % 2 != 0)
                {
                    di.Append("bad arguments ! :").Append(a[cur]).Append("\n");
                }
                else
                { // law of interpolation
                    int ii, L = nbreal / 2;
                    var ParAndRad = new NCollection_Array1<Point2D>(1, L);
                    for (ii = 1; ii <= L; ii++, cur += 2)
                    {
                        ParAndRad[ii] = ParAndRad[ii].WithX(theDI.Atof(a[cur]));
                        ParAndRad[ii] = ParAndRad[ii].WithY(theDI.Atof(a[cur + 1]));
                    }
                    thelaw = new Interpol();
                    thelaw.Set(ParAndRad, Math.Abs(ParAndRad[1].Y - ParAndRad[L].Y) < global::Kerf.Engine.Foundation.Precision.Confusion());
                }
            }
        }

        if (thelaw is null)
        {
            if (HasVertex)
            {
                Sweep.Add(Section, Vertex, isT, isR);
            }
            else
            {
                Sweep.Add(Section, isT, isR);
            }
        }
        else
        {
            if (HasVertex)
            {
                Sweep.SetLaw(Section, thelaw, Vertex, isT, isR);
            }
            else
            {
                Sweep.SetLaw(Section, thelaw, isT, isR);
            }
        }

        return 0;
    }

    private static int deletesweep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 2)
        {
            return 1;
        }
        Shape InputShape = theDI.GetShape(a[1], ShapeType.Shape);
        Wire Section = global::Kerf.Engine.Topology.ShapeCasts.Wire(InputShape);
        if (Section.IsNull)
        {
            di.Append(a[1]).Append("is not a wire !\n");
            return 1;
        }

        // OCCT dereferences the maker without a check
        Of(theDI).Sweep!.Delete(Section);
        return 0;
    }

    private static int buildsweep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 1)
        {
            di.Append("build sweep result [-M/-C/-R] [-S] [tol] : options are\n");
            di.Append("   -M : Discontinuities are treated by Modification of\n");
            di.Append("        the sweeping mode : it is the default\n");
            di.Append("   -C : Discontinuities are treated like Right Corner\n");
            di.Append("        Treatment is Extent && Intersect\n");
            di.Append("   -R : Discontinuities are treated like Round Corner\n");
            di.Append("        Treatment is Intersect and Fill\n");
            di.Append("   -S : To build a Solid\n");
            return 0;
        }

        bool mksolid = false;
        MakePipeShell? Sweep = Of(theDI).Sweep;
        if (Sweep is null)
        {
            di.Append("You have forgotten the <<mksweep>> command  !\n");
            return 1;
        }

        if (!Sweep.IsReady)
        {
            di.Append("You have forgotten the <<addsweep>> command  !\n");
            return 1;
        }

        Shape result;
        int cur = 2;
        if (n > cur)
        {
            global::Kerf.Construction.TransitionMode Transition = global::Kerf.Construction.TransitionMode.Transformed;

            // Reading Transition
            if (a[cur] == "-C")
            {
                Transition = global::Kerf.Construction.TransitionMode.RightCorner;
                cur++;
            }
            else if (a[cur] == "-R")
            {
                Transition = global::Kerf.Construction.TransitionMode.RoundCorner;
                cur++;
            }
            Sweep.SetTransitionMode(Transition);
        }
        // Reading solid ?
        if (n > cur && a[cur] == "-S")
        {
            mksolid = true;
        }

        // Calcul le resultat
        Sweep.Build();
        if (!Sweep.IsDone)
        {
            di.Append("Buildsweep : Not Done\n");
            PipeError Stat = Sweep.GetStatus();
            if (Stat == PipeError.PlaneNotIntersectGuide)
            {
                di.Append("Buildsweep : One Plane not intersect the guide\n");
            }
            if (Stat == PipeError.ImpossibleContact)
            {
                di.Append("BuildSweep : One section can not be in contact with the guide\n");
            }
        }
        else
        {
            if (mksolid)
            {
                bool B;
                B = Sweep.MakeSolid();
                if (!B)
                {
                    di.Append(" BuildSweep : It is impossible to make a solid !\n");
                }
            }
            result = Sweep.Shape();
            theDI.SetShape(a[1], result);
            // Save history of sweep
            if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
            {
                var aList = new NCollection_List<Shape>();
                Sweep.Profiles(aList);
                Shape aSpine = Sweep.Spine();
                aList.Append(aSpine);
                BRepTestHistoryCommands.SetHistory(theDI, aList, Sweep);
            }
        }

        return 0;
    }

    // errorsweep: the summary error on the surfaces the sweep reached
    private static int errorsweep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        // OCCT dereferences the maker without a check
        MakePipeShell Sweep = Of(theDI).Sweep!;
        if (!Sweep.IsDone)
        {
            di.Append("Sweep is not done\n");
            return 1;
        }
        double ErrorOnSurfaces = Sweep.ErrorOnSurface();
        di.Append("Tolerance on surfaces = ").Append(ErrorOnSurfaces).Append("\n");
        return 0;
    }

    private static int simulsweep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 3 && n != 4)
        {
            return 1;
        }

        MakePipeShell? Sweep = Of(theDI).Sweep;
        if (Sweep is null)
        {
            di.Append("You have forgotten the <<mksweep>> command  !\n");
            return 1;
        }

        if (!Sweep.IsReady)
        {
            di.Append("You have forgotten the <<addsweep>> command  !\n");
            return 1;
        }

        var List = new NCollection_List<Shape>();
        int N, ii;
        N = theDI.Atoi(a[2]);

        if (n > 3)
        {
            global::Kerf.Construction.TransitionMode Transition = global::Kerf.Construction.TransitionMode.Transformed;
            // Lecture Transition
            if (a[3] == "-C")
            {
                Transition = global::Kerf.Construction.TransitionMode.RightCorner;
            }
            else if (a[3] == "-R")
            {
                Transition = global::Kerf.Construction.TransitionMode.RoundCorner;
            }
            Sweep.SetTransitionMode(Transition);
        }

        // Calculate the result
        Sweep.Simulate(N, List);
        ii = 1;
        for (NCollection_List<Shape>.Iterator it = new(List); it.More(); it.Next(), ii++)
        {
            theDI.SetShape($"{a[1]}_{ii}", it.Value());
        }

        return 0;
    }

    private static int middlepath(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        Shape aShape = theDI.GetShape(a[2]);
        if (aShape.IsNull)
        {
            return 1;
        }

        Shape StartShape = theDI.GetShape(a[3]);
        if (StartShape.IsNull)
        {
            return 1;
        }

        Shape EndShape = theDI.GetShape(a[4]);
        if (EndShape.IsNull)
        {
            return 1;
        }

        var Builder = new MiddlePath(aShape, StartShape, EndShape);
        Builder.Build();

        Shape Result = Builder.Shape();
        theDI.SetShape(a[1], Result);

        return 0;
    }
}
