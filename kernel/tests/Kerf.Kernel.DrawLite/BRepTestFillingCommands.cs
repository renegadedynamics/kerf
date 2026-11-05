// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest's filling commands (BRepTest_FillingCommands.cxx): plate, gplate, approxplate (GeomPlate), filling and
/// fillingparam (BRepOffsetAPI_MakeFilling).
/// </summary>
/// <remarks>
/// <para>The parameters fillingparam sets are process-wide in Draw (file statics); here they belong to the
/// interpreter, as Draw's other settings do.</para>
/// <para>Draw records filling's history (BRepTest_Objects::SetHistory) for <c>savehistory</c>, which Draw-lite
/// doesn't have; the history isn't kept. Draw's progress indicator (UserBreak) isn't, either: the builds aren't
/// interrupted.</para>
/// <para>Where Draw reads an argument past the end of the command (a missing word), Draw-lite reads an empty word.</para>
/// </remarks>
internal static class BRepTestFillingCommands
{
    private const int defDegree = 3;
    private const int defNbPtsOnCur = 10;
    private const int defNbIter = 3;
    private const bool defAnisotropie = false;
    private const double defTol2d = 0.00001;
    private const double defTol3d = 0.0001;
    private const double defTolAng = 0.01;
    private const double defTolCurv = 0.1;
    private const int defMaxDeg = 8;
    private const int defMaxSegments = 9;

    /// <summary>The filling parameters, one set per interpreter (Draw's file statics).</summary>
    private sealed class Parameters
    {
        public int Degree = defDegree;
        public int NbPtsOnCur = defNbPtsOnCur;
        public int NbIter = defNbIter;
        public bool Anisotropie = defAnisotropie;
        public double Tol2d = defTol2d;
        public double Tol3d = defTol3d;
        public double TolAng = defTolAng;
        public double TolCurv = defTolCurv;
        public int MaxDeg = defMaxDeg;
        public int MaxSegments = defMaxSegments;
    }

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<DrawInterpreter, Parameters> s_parameters = new();

    private static Parameters Params(DrawInterpreter theDI) => s_parameters.GetOrCreateValue(theDI);

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "plate", Plate);
        DrawCxx.Add(theInterpreter, "gplate", Gplate);
        DrawCxx.Add(theInterpreter, "approxplate", Approxplate);
        DrawCxx.Add(theInterpreter, "filling", Filling);
        DrawCxx.Add(theInterpreter, "fillingparam", Fillingparam);
    }

    // a[i], or an empty word past the end
    private static string At(IReadOnlyList<string> a, int i) => i < a.Count ? a[i] : "";

    // commande plate : resultat face sur surface plate
    private static int Plate(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 8)
        {
            return 1;
        }
        int NbCurFront = theDI.Atoi(a[3]);
        var Fronts = new NCollection_HArray1<CurveAdaptorBase>(1, NbCurFront);
        var Tang = new NCollection_HArray1<int>(1, NbCurFront);
        var NbPtsCur = new NCollection_HArray1<int>(1, NbCurFront);
        var B = new BrepBuilder();

        var Henri = new BuildPlateSurface(3, 15, 2);

        int i;
        for (i = 1; i <= NbCurFront; i++)
        {
            Shape aLocalEdge = theDI.GetShape(At(a, 3 * i + 1), ShapeType.Edge);
            Edge E = ShapeCasts.Edge(aLocalEdge);
            if (E.IsNull)
            {
                return 1;
            }
            Shape aLocalFace = theDI.GetShape(At(a, 3 * i + 2), ShapeType.Face);
            Face F = ShapeCasts.Face(aLocalFace);
            if (F.IsNull)
            {
                return 1;
            }
            int T = theDI.Atoi(At(a, 3 * i + 3));
            Tang.SetValue(i, T);
            NbPtsCur.SetValue(i, theDI.Atoi(a[2]));
            var S = new FaceSurfaceAdaptor();
            S.Initialize(F);
            var C = new EdgeCurve2DAdaptor();
            C.Initialize(E, F);
            var HConS = new CurveOnSurface(C, S);
            Fronts.SetValue(i, HConS);
            CurveConstraint Cont = new FillingCurveConstraint(HConS, Tang.Value(i), NbPtsCur.Value(i));
            Henri.Add(Cont);
        }
        Henri.Perform();

        double ErrG0 = 1.1 * Henri.G0Error();
        di.Append(" dist. max = ").Append(Henri.G0Error()).Append(" ; angle max = ").Append(Henri.G1Error()).Append("\n");

        var MW = new MakeWire();
        for (i = 1; i <= NbCurFront; i++)
        {
            int iInOrder = Henri.Order().Value(i);
            Edge E;
            if (Henri.Sense().Value(iInOrder) == 1)
            {
                var ME = new MakeEdge(Henri.Curves2d().Value(iInOrder)!, Henri.Surface()!,
                                                     Fronts.Value(iInOrder).LastParameter(), Fronts.Value(iInOrder).FirstParameter());
                E = ME.InternalEdge();
            }
            else
            {
                var ME = new MakeEdge(Henri.Curves2d().Value(iInOrder)!, Henri.Surface()!,
                                                     Fronts.Value(iInOrder).FirstParameter(), Fronts.Value(iInOrder).LastParameter());
                E = ME.InternalEdge();
            }
            B.UpdateVertex(Exploration.FirstVertex(E), ErrG0);
            B.UpdateVertex(Exploration.LastVertex(E), ErrG0);
            BrepLibrary.BuildCurve3d(E);
            string name = $"Edge_{i}";
            theDI.SetShape(name, E);
            MW.Add(E);
            if (!MW.IsDone)
            {
                throw new global::Kerf.Engine.Foundation.FailureException("mkWire is over ");
            }
        }
        Wire W;
        W = MW.Wire();
        if (!(W.ClosedFlag))
        {
            throw new global::Kerf.Engine.Foundation.FailureException("Wire is not closed");
        }
        var MF = new MakeFace(Henri.Surface()!, W, true);
        theDI.SetShape(a[1], MF.Face());
        return 0;
    }

    // commande gplate : resultat face egale a la surface approchee
    private static int Gplate(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6)
        {
            return 1;
        }
        int NbCurFront = theDI.Atoi(a[2]), NbPointConstraint = theDI.Atoi(a[3]);

        var Henri = new BuildPlateSurface(3, 15, 2);

        int i;
        int Conti;
        int Indice = 4;
        // Surface d'init
        Shape aLocalFace = theDI.GetShape(At(a, Indice++), ShapeType.Face);
        Face SI = ShapeCasts.Face(aLocalFace);
        if (SI.IsNull)
        {
            Indice--;
        }
        else
        {
            var HSI = new FaceSurfaceAdaptor();
            HSI.Initialize(SI);
            Henri.LoadInitSurface(Tool.Surface(HSI.Face())!);
        }
        for (i = 1; i <= NbCurFront; i++)
        {
            Shape aLocalShape = theDI.GetShape(At(a, Indice++), ShapeType.Edge);
            Edge E = ShapeCasts.Edge(aLocalShape);
            if (E.IsNull)
            {
                return 1;
            }
            Conti = theDI.Atoi(At(a, Indice++));
            if ((Conti == 0) || (Conti == -1))
            {
                var C = new EdgeCurveAdaptor();
                C.Initialize(E);
                CurveAdaptorBase aC = C; // to avoid ambiguity
                CurveConstraint Cont = new FillingCurveConstraint(aC, Conti);
                Henri.Add(Cont);
            }
            else
            {
                aLocalFace = theDI.GetShape(At(a, Indice++), ShapeType.Face);
                Face F = ShapeCasts.Face(aLocalFace);
                if (F.IsNull)
                {
                    return 1;
                }
                var S = new FaceSurfaceAdaptor();
                S.Initialize(F);
                var C = new EdgeCurve2DAdaptor();
                C.Initialize(E, F);
                var HConS = new CurveOnSurface(C, S);
                CurveConstraint Cont = new FillingCurveConstraint(HConS, Conti);
                Henri.Add(Cont);
            }
        }

        for (i = 1; i <= NbPointConstraint; i++)
        {
            var P1 = new Point3D();

            if (DrawTrSurf.GetPoint(theDI, At(a, Indice), ref P1))
            {
                Conti = 0;
                var PCont = new PointConstraint(P1, 0);
                Henri.Add(PCont);
                Indice++;
            }
            else
            {
                double u = theDI.Atof(At(a, Indice++)), v = theDI.Atof(At(a, Indice++));

                Conti = theDI.Atoi(At(a, Indice++));
                aLocalFace = theDI.GetShape(At(a, Indice++), ShapeType.Face);
                Face F = ShapeCasts.Face(aLocalFace);
                if (F.IsNull)
                {
                    return 1;
                }
                var HF = new FaceSurfaceAdaptor();
                HF.Initialize(F);
                var PCont = new PointConstraint(u, v, Tool.Surface(HF.Face())!, Conti, 0.001, 0.001, 0.001);
                Henri.Add(PCont);
            }
        }
        Henri.Perform();
        int nbcarreau = 9;
        int degmax = 8;
        double seuil;

        PlateSurface gpPlate = Henri.Surface()!;
        var S2d = new NCollection_Sequence<Xy>();
        var S3d = new NCollection_Sequence<Xyz>();
        S2d.Clear();
        S3d.Clear();
        Henri.Disc2dContour(4, S2d);
        Henri.Disc3dContour(4, 0, S3d);
        seuil = Math.Max(0.0001, 10 * Henri.G0Error());
        var critere = new PlateG0Criterion(S2d, S3d, seuil);
        var Mapp = new MakeApprox(gpPlate, critere, 0.0001, nbcarreau, degmax);
        Surface Surf = Mapp.Surface()!;

        Henri.Surface()!.ParameterBounds(out double Umin, out double Umax, out double Vmin, out double Vmax);

        var MF = new MakeFace(Surf, Umin, Umax, Vmin, Vmax, global::Kerf.Engine.Foundation.Precision.Confusion());

        theDI.SetShape(a[1], MF.Face());
        return 0;
    }

    // commande approxplate : resultat face sur surface approchee
    private static int Approxplate(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 9)
        {
            return 1;
        }
        int NbMedium = theDI.Atoi(a[2]);
        int NbCurFront = theDI.Atoi(a[3]);
        var Fronts = new NCollection_HArray1<CurveAdaptorBase>(1, NbCurFront);
        var Tang = new NCollection_HArray1<int>(1, NbCurFront);
        var NbPtsCur = new NCollection_HArray1<int>(1, NbCurFront);

        var Henri = new BuildPlateSurface(3, 15, 2);

        int i;
        for (i = 1; i <= NbCurFront; i++)
        {
            Shape aLocalShape = theDI.GetShape(At(a, 3 * i + 1), ShapeType.Edge);
            Edge E = ShapeCasts.Edge(aLocalShape);
            if (E.IsNull)
            {
                return 1;
            }
            Shape aLocalFace = theDI.GetShape(At(a, 3 * i + 2), ShapeType.Face);
            Face F = ShapeCasts.Face(aLocalFace);
            if (F.IsNull)
            {
                return 1;
            }
            int T = theDI.Atoi(At(a, 3 * i + 3));
            Tang.SetValue(i, T);
            NbPtsCur.SetValue(i, NbMedium);
            var S = new FaceSurfaceAdaptor();
            S.Initialize(F);
            var C = new EdgeCurve2DAdaptor();
            C.Initialize(E, F);
            var HConS = new CurveOnSurface(C, S);
            Fronts.SetValue(i, HConS);
            CurveConstraint Cont = new FillingCurveConstraint(HConS, Tang.Value(i), NbPtsCur.Value(i));
            Henri.Add(Cont);
        }

        Henri.Perform();

        double dmax = Henri.G0Error(), anmax = Henri.G1Error();
        di.Append(" dist. max = ").Append(dmax).Append(" ; angle max = ").Append(anmax).Append("\n");

        double Tol3d = theDI.Atof(At(a, 3 * NbCurFront + 4));
        int Nbmax = theDI.Atoi(At(a, 3 * NbCurFront + 5));
        int degmax = theDI.Atoi(At(a, 3 * NbCurFront + 6));
        int CritOrder = theDI.Atoi(At(a, 3 * NbCurFront + 7));
        PlateSurface surf = Henri.Surface()!;
        BSplineSurface? support = null;

        if (CritOrder == -1)
        {
            var MApp = new MakeApprox(surf, Tol3d, Nbmax, degmax, dmax, -1);
            support = MApp.Surface();
        }
        else if (CritOrder >= 0)
        {
            var S2d = new NCollection_Sequence<Xy>();
            var S3d = new NCollection_Sequence<Xyz>();
            S2d.Clear();
            S3d.Clear();
            double seuil;
            if (CritOrder == 0)
            {
                Henri.Disc2dContour(4, S2d);
                Henri.Disc3dContour(4, 0, S3d);
                seuil = Math.Max(Tol3d, dmax * 10);
                var Crit0 = new PlateG0Criterion(S2d, S3d, seuil);
                var MApp = new MakeApprox(surf, Crit0, Tol3d, Nbmax, degmax);
                support = MApp.Surface();
            }
            else if (CritOrder == 1)
            {
                Henri.Disc2dContour(4, S2d);
                Henri.Disc3dContour(4, 1, S3d);
                seuil = Math.Max(Tol3d, anmax * 10);
                var Crit1 = new PlateG1Criterion(S2d, S3d, seuil);
                var MApp = new MakeApprox(surf, Crit1, Tol3d, Nbmax, degmax);
                support = MApp.Surface();
            }
        }

        var MW = new MakeWire();
        var B = new BrepBuilder();
        for (i = 1; i <= NbCurFront; i++)
        {
            int iInOrder = Henri.Order().Value(i);
            Edge E;
            if (Henri.Sense().Value(iInOrder) == 1)
            {
                var ME = new MakeEdge(Henri.Curves2d().Value(iInOrder)!, support!,
                                                     Fronts.Value(iInOrder).LastParameter(), Fronts.Value(iInOrder).FirstParameter());
                E = ME.InternalEdge();
            }
            else
            {
                var ME = new MakeEdge(Henri.Curves2d().Value(iInOrder)!, support!,
                                                     Fronts.Value(iInOrder).FirstParameter(), Fronts.Value(iInOrder).LastParameter());
                E = ME.InternalEdge();
            }
            B.UpdateVertex(Exploration.FirstVertex(E), dmax);
            B.UpdateVertex(Exploration.LastVertex(E), dmax);
            BrepLibrary.BuildCurve3d(E);
            MW.Add(E);
            if (!MW.IsDone)
            {
                throw new global::Kerf.Engine.Foundation.FailureException("mkWire is over ");
            }
        }
        Wire W;
        W = MW.Wire();
        if (!(W.ClosedFlag))
        {
            throw new global::Kerf.Engine.Foundation.FailureException("Wire is not closed");
        }
        var MF = new MakeFace(support!, W, true);
        theDI.SetShape(a[1], MF.Face());

        return 0;
    }

    private static int Filling(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 7)
        {
            // di.PrintHelp(a[0]): Draw-lite has no help texts
            return 1;
        }

        Parameters p = Params(theDI);
        int NbBounds = theDI.Atoi(a[2]);
        int NbConstraints = theDI.Atoi(a[3]);
        int NbPoints = theDI.Atoi(a[4]);

        var MakeFilling = new MakeFilling(p.Degree, p.NbPtsOnCur, p.NbIter, p.Anisotropie, p.Tol2d, p.Tol3d,
                                                        p.TolAng, p.TolCurv, p.MaxDeg, p.MaxSegments);
        Face InitFace = ShapeCasts.Face(theDI.GetShape(a[5], ShapeType.Face));
        if (!InitFace.IsNull)
        {
            MakeFilling.LoadInitSurface(InitFace);
        }

        int i = (InitFace.IsNull) ? 5 : 6, k;
        Edge E;
        Face F;
        var Point = new Point3D();
        int Order;
        for (k = 1; k <= NbBounds; k++)
        {
            E = ShapeCasts.Edge(theDI.GetShape(At(a, i), ShapeType.Edge));
            if (!E.IsNull)
            {
                i++;
            }
            F = ShapeCasts.Face(theDI.GetShape(At(a, i), ShapeType.Face));
            if (!F.IsNull)
            {
                i++;
            }

            Order = theDI.Atoi(At(a, i++));

            if (!E.IsNull && !F.IsNull)
            {
                MakeFilling.Add(E, F, (Continuity)Order);
            }
            else if (E.IsNull)
            {
                if (F.IsNull)
                {
                    di.Append("\nWrong parameters\n\n");
                    return 1;
                }
                else
                {
                    MakeFilling.Add(F, (Continuity)Order);
                }
            }
            else
            {
                MakeFilling.Add(E, (Continuity)Order);
            }
        }
        for (k = 1; k <= NbConstraints; k++)
        {
            E = ShapeCasts.Edge(theDI.GetShape(At(a, i++), ShapeType.Edge));
            if (E.IsNull)
            {
                di.Append("Wrong parameters\n");
                return 1;
            }
            F = ShapeCasts.Face(theDI.GetShape(At(a, i), ShapeType.Face));
            if (!F.IsNull)
            {
                i++;
            }

            Order = theDI.Atoi(At(a, i++));

            if (F.IsNull)
            {
                MakeFilling.Add(E, (Continuity)Order, false);
            }
            else
            {
                MakeFilling.Add(E, F, (Continuity)Order, false);
            }
        }
        for (k = 1; k <= NbPoints; k++)
        {
            if (DrawTrSurf.GetPoint(theDI, At(a, i), ref Point))
            {
                MakeFilling.Add(Point);
                i++;
            }
            else
            {
                double U = theDI.Atof(At(a, i++)), V = theDI.Atof(At(a, i++));
                F = ShapeCasts.Face(theDI.GetShape(At(a, i++), ShapeType.Face));
                if (F.IsNull)
                {
                    di.Append("Wrong parameters\n");
                    return 1;
                }
                Order = theDI.Atoi(At(a, i++));

                MakeFilling.Add(U, V, F, (Continuity)Order);
            }
        }

        MakeFilling.Build();
        if (!MakeFilling.IsDone)
        {
            di.Append("filling failed\n");
            return 0;
        }

        double dmax = MakeFilling.G0Error(), angmax = MakeFilling.G1Error(), curvmax = MakeFilling.G2Error();
        di.Append(" dist. max = ").Append(dmax).Append(" ; angle max = ").Append(angmax).Append(" ; diffcurv max = ")
          .Append(curvmax).Append("\n");

        Face ResFace = ShapeCasts.Face(MakeFilling.Shape());
        theDI.SetShape(a[1], ResFace);

        return 0;
    }

    private static int Fillingparam(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        Parameters p = Params(theDI);
        if (n == 1)
        {
            di.Append("fillingparam : options are").Append("\n");
            di.Append("-l : to list current values\n");
            di.Append("-i : to set default values").Append("\n");
            di.Append("-r deg nbPonC nbIt anis : to set filling options\n");
            di.Append("-c t2d t3d tang tcur : to set tolerances\n");
            di.Append("-a maxdeg maxseg : Approximation option\n");
        }
        else if (n > 1)
        {
            string flag = a[1].ToLowerInvariant();
            if (flag == "-l" && n == 2)
            {
                di.Append("\n");
                di.Append("Degree = ").Append(p.Degree).Append("\n");
                di.Append("NbPtsOnCur = ").Append(p.NbPtsOnCur).Append("\n");
                di.Append("NbIter = ").Append(p.NbIter).Append("\n");
                di.Append("Anisotropie = ").Append(p.Anisotropie ? 1 : 0).Append("\n\n");

                di.Append("Tol2d = ").Append(p.Tol2d).Append("\n");
                di.Append("Tol3d = ").Append(p.Tol3d).Append("\n");
                di.Append("TolAng = ").Append(p.TolAng).Append("\n");
                di.Append("TolCurv = ").Append(p.TolCurv).Append("\n\n");

                di.Append("MaxDeg = ").Append(p.MaxDeg).Append("\n");
                di.Append("MaxSegments = ").Append(p.MaxSegments).Append("\n\n");
            }
            else if (flag == "-i" && n == 2)
            {
                s_parameters.AddOrUpdate(theDI, new Parameters());
            }
            else if (flag == "-r" && n == 6)
            {
                p.Degree = theDI.Atoi(a[2]);
                p.NbPtsOnCur = theDI.Atoi(a[3]);
                p.NbIter = theDI.Atoi(a[4]);
                p.Anisotropie = theDI.Atoi(a[5]) != 0;
            }
            else if (flag == "-c" && n == 6)
            {
                p.Tol2d = theDI.Atof(a[2]);
                p.Tol3d = theDI.Atof(a[3]);
                p.TolAng = theDI.Atof(a[4]);
                p.TolCurv = theDI.Atof(a[5]);
            }
            else if (flag == "-a" && n == 4)
            {
                p.MaxDeg = theDI.Atoi(a[2]);
                p.MaxSegments = theDI.Atoi(a[3]);
            }
            else
            {
                di.Append("Wrong parameters\n");
                return 1;
            }
        }
        return 0;
    }
}
