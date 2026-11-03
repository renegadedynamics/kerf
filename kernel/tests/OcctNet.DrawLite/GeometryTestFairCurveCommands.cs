// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Fairing;
using Kerf.Engine.Exchange.Brep;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Foundation.Standard;

namespace OcctNet.DrawLite;

/// <summary>
/// A batten or minimal variation curve (DrawFairCurve_Batten, DrawFairCurve_MinimalVariation): Draw keeps the
/// FairCurve object with its computed curve, and the editing commands change the object and compute again. Like
/// Draw's, which derive from DrawTrSurf_BSplineCurve2d, it is a 2D B-spline to the other commands
/// (<see cref="DrawTrSurf.GetCurve2d"/>).
/// </summary>
public sealed class DrawFairCurve : DrawDrawable
{
    internal DrawFairCurve(Batten theBatten)
    {
        Batten = theBatten;
        Curve = theBatten.Curve();
        Compute();
    }

    /// <summary>The batten (a FairCurve_MinimalVariation for minvarcurve).</summary>
    public Batten Batten { get; }

    /// <summary>The curve of the latest computation (frozen).</summary>
    public BSplineCurve2D Curve { get; private set; }

    public override string TypeName => Batten is MinimalVariation ? "DrawFairCurve_MinimalVariation" : "DrawFairCurve_Batten";

    /// <summary>DrawTrSurf_Curve2d::Whatis.</summary>
    public override void Whatis(DrawResult theResult) => theResult.Append("2d curve");

    /// <summary>DrawFairCurve_Batten::Dump: the curve, then the batten's state.</summary>
    public override void Dump(TextWriter theStream)
    {
        Curve2dSet.PrintCurve2d(Curve, theStream);
        Batten.Dump(theStream);
    }

    /// <summary>DrawTrSurf_Curve2d::Save: the curve.</summary>
    public override void Save(TextWriter theStream) => Curve2dSet.PrintCurve2d(Curve, theStream, true);

    /// <summary>DrawTrSurf_BSplineCurve2d::Copy: a plain B-spline drawable (the batten is not copied).</summary>
    public override DrawDrawable Copy() => DrawGeometry.Of(Curve);

    /// <summary>DrawFairCurve_Batten::Compute: 50 iterations, tolerance 1.0e-2.</summary>
    internal void Compute()
    {
        Batten.Compute(out _, 50, 1.0e-2);
        Curve = Batten.Curve();
    }
}

/// <summary>
/// GeometryTest_FairCurveCommands.cxx: <c>battencurve</c>, <c>minvarcurve</c> and the editing commands
/// (<c>setpoint</c>, <c>setangle</c>, <c>setslide</c>, <c>freeangle</c>, <c>freeslide</c>, <c>setheight</c>,
/// <c>setslope</c>, <c>setcurvature</c>, <c>freecurvature</c>, <c>setphysicalratio</c>) over DrawFairCurve's methods.
/// </summary>
internal static class GeometryTestFairCurveCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "battencurve", BattenCurve);
        DrawCxx.Add(theInterpreter, "minvarcurve", MVCurve);
        DrawCxx.Add(theInterpreter, "setpoint", SetPoint);
        DrawCxx.Add(theInterpreter, "setangle", SetAngle);
        DrawCxx.Add(theInterpreter, "setslide", SetSlide);
        DrawCxx.Add(theInterpreter, "freeangle", FreeAngle);
        DrawCxx.Add(theInterpreter, "freeslide", FreeSlide);
        DrawCxx.Add(theInterpreter, "setheight", SetHeight);
        DrawCxx.Add(theInterpreter, "setslope", SetSlope);
        DrawCxx.Add(theInterpreter, "setcurvature", SetCurvature);
        DrawCxx.Add(theInterpreter, "freecurvature", FreeCurvature);
        DrawCxx.Add(theInterpreter, "setphysicalratio", SetPhysicalRatio);
    }

    private static bool IsGoodNumber(int argc, int waiting, DrawResult di)
    {
        // argc vaut 1 de plus, puisque argv[0] contient le nom de la commande
        if (argc != (waiting + 1))
        {
            di.Append("Waiting " + waiting + " arguments\n");
            return false;
        }
        else
        {
            return true;
        }
    }

    private static DrawFairCurve? GetBatten(DrawInterpreter theDI, string theName) => theDI.GetDrawable(theName) as DrawFairCurve;

    private static MinimalVariation? GetMVC(DrawInterpreter theDI, string theName, out DrawFairCurve? theDrawable)
    {
        theDrawable = GetBatten(theDI, theName);
        return theDrawable?.Batten as MinimalVariation;
    }

    /// <summary>battencurve P1 P2 Angle1 Angle2 Heigth BattenName (angles in degrees).</summary>
    private static int BattenCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 6, di))
        {
            return 1;
        }

        double a1 = theDI.Atof(argv[3]), a2 = theDI.Atof(argv[4]), h = theDI.Atof(argv[5]);
        Point2D P1 = new(), P2 = new();

        if (!DrawTrSurf.GetPoint2d(theDI, argv[1], ref P1))
        {
            return 1;
        }
        if (!DrawTrSurf.GetPoint2d(theDI, argv[2], ref P2))
        {
            return 1;
        }

        Batten Bat = new(P1, P2, h);
        Bat.SetAngle1(a1 * M_PI / 180);
        Bat.SetAngle2(a2 * M_PI / 180);

        Bat.Compute(out _);

        // DrawFairCurve_Batten's constructor computes again (50, 1.0e-2)
        theDI.SetDrawable(argv[6], new DrawFairCurve(Bat));
        return 0;
    }

    /// <summary>minvarcurve P1 P2 Angle1 Angle2 Heigth MVCName (angles in degrees).</summary>
    private static int MVCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 6, di))
        {
            return 1;
        }

        double a1 = theDI.Atof(argv[3]), a2 = theDI.Atof(argv[4]), h = theDI.Atof(argv[5]);
        Point2D P1 = new(), P2 = new();

        if (!DrawTrSurf.GetPoint2d(theDI, argv[1], ref P1))
        {
            return 1;
        }
        if (!DrawTrSurf.GetPoint2d(theDI, argv[2], ref P2))
        {
            return 1;
        }

        MinimalVariation MVC = new(P1, P2, h);
        MVC.SetAngle1(a1 * M_PI / 180);
        MVC.SetAngle2(a2 * M_PI / 180);

        MVC.Compute(out _);

        theDI.SetDrawable(argv[6], new DrawFairCurve(MVC));
        return 0;
    }

    /// <summary>setpoint side point BattenName: Draw takes a DrawTrSurf_Point's Point2d.</summary>
    private static int SetPoint(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 3, di))
        {
            return 1;
        }

        int cote = theDI.Atoi(argv[1]);

        Point2D Pnt;
        switch ((theDI.GetDrawable(argv[2]) as DrawGeometry)?.Geometry)
        {
            case Point2D aPnt2d:
                Pnt = aPnt2d;
                break;
            case Point3D aPnt:
                Pnt = new Point2D(aPnt.X(), aPnt.Y());
                break;
            default:
                return 1;
        }

        DrawFairCurve? Bat = GetBatten(theDI, argv[3]);
        if (Bat is null)
        {
            return 1;
        }

        if (cote == 1)
        {
            Bat.Batten.SetP1(Pnt);
        }
        else
        {
            Bat.Batten.SetP2(Pnt);
        }
        Bat.Compute();
        return 0;
    }

    /// <summary>setangle side angle BattenName (degrees); a free angle becomes constrained.</summary>
    private static int SetAngle(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 3, di))
        {
            return 1;
        }

        double angle = theDI.Atof(argv[2]);
        int cote = theDI.Atoi(argv[1]);

        DrawFairCurve? Bat = GetBatten(theDI, argv[3]);
        if (Bat is null)
        {
            return 1;
        }

        if (cote == 1)
        {
            Bat.Batten.SetAngle1(angle * M_PI / 180);
            if (Bat.Batten.GetConstraintOrder1() == 0)
            {
                Bat.Batten.SetConstraintOrder1(1);
            }
        }
        else
        {
            Bat.Batten.SetAngle2(angle * M_PI / 180);
            if (Bat.Batten.GetConstraintOrder2() == 0)
            {
                Bat.Batten.SetConstraintOrder2(1);
            }
        }
        Bat.Compute();
        return 0;
    }

    /// <summary>setslide slidingfactor BattenName: imposed sliding.</summary>
    private static int SetSlide(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 2, di))
        {
            return 1;
        }

        double slide = theDI.Atof(argv[1]);

        DrawFairCurve? Bat = GetBatten(theDI, argv[2]);
        if (Bat is null)
        {
            return 1;
        }

        Bat.Batten.SetFreeSliding(false);
        Bat.Batten.SetSlidingFactor(slide);
        Bat.Compute();
        return 0;
    }

    /// <summary>freeangle side BattenName.</summary>
    private static int FreeAngle(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 2, di))
        {
            return 1;
        }

        int cote = theDI.Atoi(argv[1]);

        DrawFairCurve? Bat = GetBatten(theDI, argv[2]);
        if (Bat is null)
        {
            return 1;
        }

        if (cote == 1)
        {
            Bat.Batten.SetConstraintOrder1(0);
        }
        else
        {
            Bat.Batten.SetConstraintOrder2(0);
        }
        Bat.Compute();
        return 0;
    }

    /// <summary>freeslide BattenName.</summary>
    private static int FreeSlide(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 1, di))
        {
            return 1;
        }

        DrawFairCurve? Bat = GetBatten(theDI, argv[1]);
        if (Bat is null)
        {
            return 1;
        }

        Bat.Batten.SetFreeSliding(true);
        Bat.Compute();
        return 0;
    }

    /// <summary>setheight height BattenName.</summary>
    private static int SetHeight(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 2, di))
        {
            return 1;
        }

        double Height = theDI.Atof(argv[1]);

        DrawFairCurve? Bat = GetBatten(theDI, argv[2]);
        if (Bat is null)
        {
            return 1;
        }

        Bat.Batten.SetHeight(Height);
        Bat.Compute();
        return 0;
    }

    /// <summary>setslope slope BattenName.</summary>
    private static int SetSlope(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 2, di))
        {
            return 1;
        }

        double Slope = theDI.Atof(argv[1]);

        DrawFairCurve? Bat = GetBatten(theDI, argv[2]);
        if (Bat is null)
        {
            return 1;
        }

        Bat.Batten.SetSlope(Slope);
        Bat.Compute();
        return 0;
    }

    /// <summary>setcurvature side rho MVCName: constraint order 2 at that side.</summary>
    private static int SetCurvature(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 3, di))
        {
            return 1;
        }

        double rho = theDI.Atof(argv[2]);
        int cote = theDI.Atoi(argv[1]);

        MinimalVariation? MVC = GetMVC(theDI, argv[3], out DrawFairCurve? aDrawable);
        if (MVC is null)
        {
            return 1;
        }

        if (cote == 1)
        {
            MVC.SetCurvature1(rho);
            MVC.SetConstraintOrder1(2);
        }
        else
        {
            MVC.SetCurvature2(rho);
            MVC.SetConstraintOrder2(2);
        }
        aDrawable!.Compute();
        return 0;
    }

    /// <summary>freecurvature side MVCName: back to constraint order 1 at that side.</summary>
    private static int FreeCurvature(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 2, di))
        {
            return 1;
        }

        int cote = theDI.Atoi(argv[1]);

        MinimalVariation? MVC = GetMVC(theDI, argv[2], out DrawFairCurve? aDrawable);
        if (MVC is null)
        {
            return 1;
        }

        if (cote == 1)
        {
            if (MVC.GetConstraintOrder1() > 1)
            {
                MVC.SetConstraintOrder1(1);
            }
        }
        else
        {
            if (MVC.GetConstraintOrder2() > 1)
            {
                MVC.SetConstraintOrder2(1);
            }
        }
        aDrawable!.Compute();
        return 0;
    }

    /// <summary>setphysicalratio ratio MVCName.</summary>
    private static int SetPhysicalRatio(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (!IsGoodNumber(argv.Count, 2, di))
        {
            return 1;
        }

        double ratio = theDI.Atof(argv[1]);

        MinimalVariation? MVC = GetMVC(theDI, argv[2], out DrawFairCurve? aDrawable);
        if (MVC is null)
        {
            return 1;
        }

        MVC.SetPhysicalRatio(ratio);
        aDrawable!.Compute();
        return 0;
    }
}
