// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Topology;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Runtime.CompilerServices;
using Occt;
using static Kerf.Engine.HiddenLines.TypeOfResultingEdge;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// HLRTest's commands (HLRTest.cxx, unit 7Q2): projectors (<c>hprj</c>, <c>hsetprj</c>), outliners (<c>houtl</c>,
/// <c>hfill</c>, <c>hsin</c>, <c>hsout</c>), the exact hidden-line removal of the loaded outliners (<c>hload</c>,
/// <c>hremove</c>, <c>hupdate</c>, <c>hhide</c>, <c>hshowall</c>, <c>hdebug</c>, <c>hnullify</c>, <c>hres2d</c>), and
/// HLRAppli_ReflectLines (<c>reflectlines</c>, <c>hlrin3d</c>, <c>hlrin2d</c>); and BRepTest's <c>build3d</c>
/// (BRepLib::BuildCurves3d, which HLR's tests run on their 2D results).
/// </summary>
/// <remarks>
/// OCCT keeps the hidden-line algorithm the commands share in a file-scope static (<c>hider</c>, made when the commands
/// are registered); Draw-lite keeps one per interpreter. The projector and outliner variables are Draw values
/// (HLRTest_Projector, HLRTest_OutLiner) with nothing to draw. <c>build3d</c> writes into its shape in OCCT; Draw-lite
/// writes into a copy and rebinds the name, as <c>mkedgecurve</c> does.
/// </remarks>
internal static class HLRTestCommands
{
    private static readonly ConditionalWeakTable<DrawInterpreter, HiddenLineAlgo> s_hiders = new();

    private static HiddenLineAlgo Hider(DrawInterpreter theDI) => s_hiders.GetValue(theDI, static _ => new HiddenLineAlgo());

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "hprj", Hprj);
        DrawCxx.Add(theInterpreter, "houtl", Hout);
        DrawCxx.Add(theInterpreter, "hfill", Hfil);
        DrawCxx.Add(theInterpreter, "hsin", Sori);
        DrawCxx.Add(theInterpreter, "hsout", Sout);
        DrawCxx.Add(theInterpreter, "hload", Hloa);
        DrawCxx.Add(theInterpreter, "hremove", Hrem);
        DrawCxx.Add(theInterpreter, "hsetprj", Sprj);
        DrawCxx.Add(theInterpreter, "hupdate", static (i, _, _) =>
        {
            Hider(i).Update();
            return 0;
        });
        DrawCxx.Add(theInterpreter, "hhide", static (i, _, _) =>
        {
            Hider(i).Hide();
            return 0;
        });
        DrawCxx.Add(theInterpreter, "hshowall", static (i, _, _) =>
        {
            Hider(i).ShowAll();
            return 0;
        });
        DrawCxx.Add(theInterpreter, "hdebug", static (i, di, _) =>
        {
            HiddenLineAlgo aHider = Hider(i);
            aHider.Debug(!aHider.Debug());
            di.Append(aHider.Debug() ? "debug\n" : "no debug\n");
            return 0;
        });
        DrawCxx.Add(theInterpreter, "hnullify", static (i, _, _) =>
        {
            Hider(i).OutLinedShapeNullify();
            return 0;
        });
        DrawCxx.Add(theInterpreter, "hres2d", Hres);
        DrawCxx.Add(theInterpreter, "reflectlines", Reflectlines);
        DrawCxx.Add(theInterpreter, "hlrin3d", Hlrin3d);
        DrawCxx.Add(theInterpreter, "hlrin2d", Hlrin2d);
        DrawCxx.Add(theInterpreter, "build3d", Build3d);
        _ = Hider(theInterpreter); // HLRTest::Commands makes the hider
    }

    // HLRTest::GetProjector
    private static bool GetProjector(DrawInterpreter theDI, string theName, out AlgorithmProjector P)
    {
        if (theDI.GetDrawable(theName) is DrawHlrProjector aHP)
        {
            P = aHP.Projector;
            return true;
        }
        P = new AlgorithmProjector();
        return false;
    }

    // HLRTest::GetOutLiner
    private static OutLiner? GetOutLiner(DrawInterpreter theDI, string theName)
        => theDI.GetDrawable(theName) is DrawHlrOutLiner anHS ? anHS.OutLiner : null;

    private static double Atof(string theText) => double.TryParse(theText, NumberStyles.Float, CultureInfo.InvariantCulture, out double aValue) ? aValue : 0.0;

    //=================================================================================================

    private static int Hprj(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }
        //
        Frame3D anAx2 = Constants.XOY();
        if (n == 11)
        {
            double x = theDI.Atof(a[2]);
            double y = theDI.Atof(a[3]);
            double z = theDI.Atof(a[4]);

            double dx = theDI.Atof(a[5]);
            double dy = theDI.Atof(a[6]);
            double dz = theDI.Atof(a[7]);

            double dx1 = theDI.Atof(a[8]);
            double dy1 = theDI.Atof(a[9]);
            double dz1 = theDI.Atof(a[10]);

            Point3D anOrigin = new(x, y, z);
            Direction3D aNormal = new(dx, dy, dz);
            Direction3D aDX = new(dx1, dy1, dz1);
            anAx2 = new Frame3D(anOrigin, aNormal, aDX);
        }

        AlgorithmProjector P = new(anAx2);
        theDI.SetDrawable(a[1], new DrawHlrProjector(P));
        return 0;
    }

    private static int Hout(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        string name = a.Count > 2 ? a[2] : "";
        Shape S = theDI.GetShape(name);
        if (S.IsNull)
        {
            di.Append(name).Append(" is not a shape.\n");
            return 1;
        }
        theDI.SetDrawable(a[1], new DrawHlrOutLiner(new OutLiner(S)));
        return 0;
    }

    private static int Hfil(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        int nbIso = 0;
        if (n < 3)
        {
            return 1;
        }
        if (n > 3)
        {
            nbIso = theDI.Atoi(a[3]);
        }
        string name1 = a[1];
        OutLiner? HS = GetOutLiner(theDI, name1);
        if (HS is null)
        {
            di.Append(name1).Append(" is not an OutLiner.\n");
            return 1;
        }
        string name2 = a[2];
        if (!GetProjector(theDI, name2, out AlgorithmProjector P))
        {
            di.Append(name2).Append(" is not a projector.\n");
            return 1;
        }
        Dictionary<Shape, AdaptorTool> MST = new(ShapeHasher.Instance);
        HS.Fill(P, MST, nbIso);
        return 0;
    }

    private static int Sori(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }
        string name1 = a[1];
        string name2 = a[2];
        OutLiner? HS = GetOutLiner(theDI, name2);
        if (HS is null)
        {
            di.Append(name2).Append(" is not an OutLiner.\n");
            return 1;
        }
        theDI.SetShape(name1, HS.OriginalShape());
        return 0;
    }

    private static int Sout(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }
        string name1 = a[1];
        string name2 = a[2];
        OutLiner? HS = GetOutLiner(theDI, name2);
        if (HS is null)
        {
            di.Append(name2).Append(" is not an OutLiner.\n");
            return 1;
        }
        if (HS.OutLinedShape().IsNull)
        {
            di.Append(name2).Append(" has no OutLinedShape.\n");
            return 1;
        }
        theDI.SetShape(name1, HS.OutLinedShape());
        return 0;
    }

    private static int Hloa(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        string name1 = a[1];
        OutLiner? HS = GetOutLiner(theDI, name1);
        if (HS is null)
        {
            di.Append(name1).Append(" is not an OutLiner.\n");
            return 1;
        }
        Hider(theDI).Load(HS);
        return 0;
    }

    private static int Hrem(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        HiddenLineAlgo hider = Hider(theDI);
        if (a.Count > 1)
        {
            string name = a[1];
            int index;
            OutLiner? HS = GetOutLiner(theDI, name);
            if (HS is null)
            {
                Shape S = theDI.GetShape(name);
                if (S.IsNull)
                {
                    di.Append(name).Append(" is not an OutLiner and not a shape.\n");
                    return 1;
                }
                else
                {
                    index = hider.Index(S);
                    if (index == 0)
                    {
                        di.Append(name).Append(" not loaded shape.\n");
                        return 1;
                    }
                }
            }
            else
            {
                index = hider.Index(HS.OriginalShape());
                if (index == 0)
                {
                    di.Append(name).Append(" not loaded outliner.\n");
                    return 1;
                }
            }
            hider.Remove(index);
            di.Append(name).Append(" removed\n");
        }
        else
        {
            while (hider.ShapeCount > 0)
            {
                hider.Remove(1);
            }
            di.Append(" all shapes removed\n");
        }
        return 0;
    }

    private static int Sprj(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        string name = a[1];
        if (!GetProjector(theDI, name, out AlgorithmProjector P))
        {
            di.Append(name).Append(" is not a projector.\n");
            return 1;
        }
        Hider(theDI).Projector(P);
        return 0;
    }

    private static int Hres(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        Shape S = new(), V, V1, VN, VO, VI, H, H1, HN, HO, HI;
        if (a.Count > 1)
        {
            string name = a[1];
            S = theDI.GetShape(name);
        }
        HlrToShape HS = new(Hider(theDI));

        if (S.IsNull)
        {
            V = HS.VCompound();
            V1 = HS.Rg1LineVCompound();
            VN = HS.RgNLineVCompound();
            VO = HS.OutLineVCompound();
            VI = HS.IsoLineVCompound();
            H = HS.HCompound();
            H1 = HS.Rg1LineHCompound();
            HN = HS.RgNLineHCompound();
            HO = HS.OutLineHCompound();
            HI = HS.IsoLineHCompound();
        }
        else
        {
            V = HS.VCompound(S);
            V1 = HS.Rg1LineVCompound(S);
            VN = HS.RgNLineVCompound(S);
            VO = HS.OutLineVCompound(S);
            VI = HS.IsoLineVCompound(S);
            H = HS.HCompound(S);
            H1 = HS.Rg1LineHCompound(S);
            HN = HS.RgNLineHCompound(S);
            HO = HS.OutLineHCompound(S);
            HI = HS.IsoLineHCompound(S);
        }
        (string Name, Shape Shape)[] aResults =
            [("vl", V), ("v1l", V1), ("vnl", VN), ("vol", VO), ("vil", VI), ("hl", H), ("h1l", H1), ("hnl", HN), ("hol", HO), ("hil", HI)];
        foreach ((string aName, Shape aShape) in aResults)
        {
            if (!aShape.IsNull)
            {
                theDI.SetShape(aName, aShape);
            }
        }
        return 0;
    }

    // reflectlines, hlrin3d and hlrin2d: HLRAppli_ReflectLines for the view direction (a[3], a[4], a[5])
    private static ReflectLines? MakeReflector(DrawInterpreter theDI, IReadOnlyList<string> a, bool theEye)
    {
        Shape aShape = theDI.GetShape(a[2]);
        if (aShape.IsNull)
        {
            return null;
        }

        // atof, not Draw::Atof
        double anAISViewProjX = Atof(a[3]);
        double anAISViewProjY = Atof(a[4]);
        double anAISViewProjZ = Atof(a[5]);

        Point3D anOrigin = new(0.0, 0.0, 0.0);
        Direction3D aNormal = new(anAISViewProjX, anAISViewProjY, anAISViewProjZ);
        Direction3D aDX;
        if (theEye)
        {
            aDX = new Direction3D(Atof(a[6]), Atof(a[7]), Atof(a[8]));
        }
        else
        {
            Frame3D theAxes = new(anOrigin, aNormal);
            aDX = theAxes.XDirection;
        }

        ReflectLines Reflector = new(aShape);
        Reflector.SetAxes(aNormal.X, aNormal.Y, aNormal.Z, anOrigin.X, anOrigin.Y, anOrigin.Z, aDX.X, aDX.Y,
                          aDX.Z);
        Reflector.Perform();
        return Reflector;
    }

    private static int Reflectlines(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 6)
        {
            return 1;
        }
        ReflectLines? Reflector = MakeReflector(theDI, a, false);
        if (Reflector is null)
        {
            return 1;
        }
        theDI.SetShape(a[1], Reflector.GetResult());
        return 0;
    }

    // hlrin3d and hlrin2d: the visible sharp edges, outlines and smooth edges in one compound
    private static int ReflectCompound(DrawInterpreter theDI, IReadOnlyList<string> a, int theMinArgs, bool theEye, bool theIn3d)
    {
        if (a.Count < theMinArgs)
        {
            return 1;
        }
        ReflectLines? Reflector = MakeReflector(theDI, a, theEye);
        if (Reflector is null)
        {
            return 1;
        }

        Shape[] aParts =
        [
            Reflector.GetCompoundOf3dEdges(TypeOfResultingEdge.Sharp, true, theIn3d),
            Reflector.GetCompoundOf3dEdges(TypeOfResultingEdge.OutLine, true, theIn3d),
            Reflector.GetCompoundOf3dEdges(TypeOfResultingEdge.Rg1Line, true, theIn3d),
        ];
        theDI.SetShape(a[1], Builder.MakeCompound(aParts.Where(s => !s.IsNull).ToArray()));
        return 0;
    }

    private static int Hlrin3d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
        => ReflectCompound(theDI, a, 6, false, true);

    private static int Hlrin2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
        => ReflectCompound(theDI, a, 9, true, false);

    // BRepTest_CurveCommands.cxx: build3d S [tol]
    private static int Build3d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if ((n < 2) || (n > 3))
        {
            di.Append(" 1 or 2 arguments expected\n");
            return 1;
        }

        bool Ok;
        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
        {
            return 1;
        }

        // in place in OCCT: on a copy here, rebound to the name (as mkedgecurve)
        Shape aCopy = DrawShapeCopy.Copy(S);
        if (n == 2)
        {
            Ok = BrepLibrary.BuildCurves3d(aCopy);
        }
        else
        {
            Ok = BrepLibrary.BuildCurves3d(aCopy, theDI.Atof(a[2]));
        }
        theDI.SetShape(a[1], aCopy);
        if (!Ok)
        {
            di.Append(" one of the computation failed\n");
        }

        return 0;
    }
}

/// <summary>A projector (HLRTest_Projector): <c>hprj</c>'s value.</summary>
internal sealed class DrawHlrProjector(in AlgorithmProjector theProjector) : DrawDrawable
{
    /// <summary>The projector.</summary>
    public AlgorithmProjector Projector { get; } = theProjector;

    public override string TypeName => "HLRTest_Projector";

    public override void Whatis(DrawResult theResult) => theResult.Append("projector");

    public override void Dump(TextWriter theStream)
    {
        theStream.Write("Projector : \n");
        if (Projector.Perspective())
        {
            theStream.Write("perspective, focal = " + Projector.Focus().ToString(CultureInfo.InvariantCulture) + "\n");
        }
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                theStream.Write(Projector.Transformation().Value(i, j).ToString(CultureInfo.InvariantCulture).PadLeft(15));
            }
            theStream.Write("\n");
        }
        theStream.Write('\n');
    }

    public override void Save(TextWriter theStream)
    {
        theStream.Write((Projector.Perspective() ? "1" : "0") + "\n");
        if (Projector.Perspective())
        {
            theStream.Write(Projector.Focus().ToString(CultureInfo.InvariantCulture) + "\n");
        }
        Transform3D aTransformation = Projector.Transformation();
        Xyz aTranslationVector = aTransformation.TranslationXyz();
        Matrix3x3 aMatrix = aTransformation.VectorialPart;
        for (int r = 1; r <= 3; r++)
        {
            for (int c = 1; c <= 3; c++)
            {
                theStream.Write(aMatrix.Value(r, c).ToString(CultureInfo.InvariantCulture) + " ");
            }
            theStream.Write(aTranslationVector.Coord(r).ToString(CultureInfo.InvariantCulture) + " ");
            theStream.Write("\n");
        }
    }

    public override DrawDrawable Copy() => new DrawHlrProjector(Projector);
}

/// <summary>An outliner (HLRTest_OutLiner): <c>houtl</c>'s value, a working object the HLR commands fill and load.</summary>
internal sealed class DrawHlrOutLiner(OutLiner theOutLiner) : DrawDrawable
{
    /// <summary>The outliner.</summary>
    public OutLiner OutLiner { get; } = theOutLiner;

    public override string TypeName => "HLRTest_OutLiner";

    public override void Whatis(DrawResult theResult) => theResult.Append("outliner");

    public override void Dump(TextWriter theStream) => theStream.Write("This is an outliner" + '\n');

    /// <summary>HLRTest_OutLiner has no Save; nothing is written.</summary>
    public override void Save(TextWriter theStream)
    {
    }

    /// <summary>A new outliner on the original shape, as HLRTest_OutLiner::Copy.</summary>
    public override DrawDrawable Copy() => new DrawHlrOutLiner(new OutLiner(OutLiner.OriginalShape()));
}
