// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Occt.BRepBuilderAPI_TransitionMode;
using static Occt.Draft_ErrorStatus;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's draft commands (BRepTest_DraftAngleCommands.cxx, unit 7G): <c>depouille</c> and <c>ndepouille</c>
/// (BRepOffsetAPI_DraftAngle) and <c>draft</c> (BRepOffsetAPI_MakeDraft), written as OCCT writes them.
/// </summary>
internal static class BRepTestDraftAngleCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "depouille", DEP);
        DrawCxx.Add(theInterpreter, "ndepouille", NDEP);
        DrawCxx.Add(theInterpreter, "draft", Draft);
    }

    private static void Print(DrawResult di, Draft_ErrorStatus St)
    {
        di.Append("  Error Status : ");
        switch (St)
        {
            case Draft_NoError:
                di.Append("No error");
                break;

            case Draft_FaceRecomputation:
                di.Append("Impossible face recomputation");
                break;

            case Draft_EdgeRecomputation:
                di.Append("Impossible edge recomputation");
                break;

            case Draft_VertexRecomputation:
                di.Append("Impossible vertex recomputation");
                break;
        }
    }

    private static int DEP(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if ((narg < 14) || (narg % 8 != 6))
        {
            return 1;
        }
        TopoDS_Shape V = theDI.GetShape(a[2]);
        BRepOffsetAPI_DraftAngle drft = new(V);

        gp_Dir Dirextract = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));

        TopoDS_Face F;
        double Angle;
        gp_Pnt Pax = new();
        gp_Dir Dax = new();
        for (int ii = 0; ii < (narg - 6) / 8; ii++)
        {
            TopoDS_Shape aLocalShape = theDI.GetShape(a[8 * ii + 6], TopAbs_FACE);
            F = TopoDS.Face(aLocalShape);
            //    F = TopoDS::Face(DBRep::Get(a[8*ii+6],TopAbs_FACE));
            Angle = theDI.Atof(a[8 * ii + 7]) * Math.PI / 180.0;
            Pax.SetCoord(theDI.Atof(a[8 * ii + 8]), theDI.Atof(a[8 * ii + 9]), theDI.Atof(a[8 * ii + 10]));
            Dax.SetCoord(theDI.Atof(a[8 * ii + 11]), theDI.Atof(a[8 * ii + 12]), theDI.Atof(a[8 * ii + 13]));
            drft.Add(F, Dirextract, Angle, new gp_Pln(Pax, Dax));
            if (!drft.AddDone())
            {
                break;
            }
        }

        if (!drft.AddDone())
        {
            theDI.SetShape("bugdep", drft.ProblematicShape());
            theCommands.Append("Bad shape in variable bugdep ");
            Print(theCommands, drft.Status());
            return 1;
        }
        drft.Build();
        if (drft.IsDone())
        {
            theDI.SetShape(a[1], drft.Shape());
            return 0;
        }
        theDI.SetShape("bugdep", drft.ProblematicShape());
        theCommands.Append("Problem encountered during the reconstruction : ");
        theCommands.Append("bad shape in variable bugdep; ");
        Print(theCommands, drft.Status());
        return 1;
    }

    private static int NDEP(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if ((narg < 15) || ((narg) % 9 != 6))
        {
            return 1;
        }
        TopoDS_Shape V = theDI.GetShape(a[2]);
        if (V.IsNull())
        {
            theCommands.Append(a[2]).Append(" is not a Shape\n");
            return 1;
        }

        BRepOffsetAPI_DraftAngle drft = new(V);

        gp_Dir Dirextract = new(theDI.Atof(a[3]), theDI.Atof(a[4]), theDI.Atof(a[5]));

        TopoDS_Face F;
        double Angle;
        gp_Pnt Pax = new();
        gp_Dir Dax = new();
        bool Flag;
        for (int ii = 0; ii < (narg - 6) / 9; ii++)
        {
            TopoDS_Shape aLocalFace = theDI.GetShape(a[9 * ii + 6], TopAbs_FACE);
            F = TopoDS.Face(aLocalFace);
            //    F = TopoDS::Face(DBRep::Get(a[9*ii+6],TopAbs_FACE));

            if (F.IsNull())
            {
                theCommands.Append(a[9 * ii + 6]).Append(" is not a face\n");
                return 1;
            }

            Flag = theDI.Atoi(a[9 * ii + 7]) != 0;
            Angle = theDI.Atof(a[9 * ii + 8]) * Math.PI / 180.0;
            Pax.SetCoord(theDI.Atof(a[9 * ii + 9]), theDI.Atof(a[9 * ii + 10]), theDI.Atof(a[9 * ii + 11]));
            Dax.SetCoord(theDI.Atof(a[9 * ii + 12]), theDI.Atof(a[9 * ii + 13]), theDI.Atof(a[9 * ii + 14]));
            drft.Add(F, Dirextract, Angle, new gp_Pln(Pax, Dax), Flag);
            if (!drft.AddDone())
            {
                break;
            }
        }

        if (!drft.AddDone())
        {
            theDI.SetShape("bugdep", drft.ProblematicShape());
            theCommands.Append("Bad shape in variable bugdep ");
            Print(theCommands, drft.Status());
            return 1;
        }
        drft.Build();
        if (drft.IsDone())
        {
            theDI.SetShape(a[1], drft.Shape());
            return 0;
        }
        theDI.SetShape("bugdep", drft.ProblematicShape());
        theCommands.Append("Problem encountered during the reconstruction : ");
        theCommands.Append("bad shape in variable bugdep; ");
        Print(theCommands, drft.Status());
        return 1;
    }

    private static int Draft(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        int Inside = -1;
        bool Internal = false;
        if (n < 8)
        {
            return 1;
        }

        double x, y, z, teta;
        TopoDS_Shape SInit = theDI.GetShape(a[2]); // shape d'arret

        x = theDI.Atof(a[3]);
        y = theDI.Atof(a[4]); // direction de depouille
        z = theDI.Atof(a[5]);
        teta = theDI.Atof(a[6]); // angle de depouille (teta)

        gp_Dir D = new(x, y, z);

        BRepOffsetAPI_MakeDraft MkDraft = new(SInit, D, teta);

        if (n > 8)
        {
            int cur = 8;
            if (a[cur] == "-IN")
            {
                Inside = 1;
                cur++;
            }
            else if (a[cur] == "-OUT")
            {
                Inside = 0;
                cur++;
            }

            if (cur < n)
            {
                if (a[cur] == "-Ri")
                {
                    MkDraft.SetOptions(BRepBuilderAPI_RightCorner);
                    cur++;
                }
                else if (a[cur] == "-Ro")
                {
                    MkDraft.SetOptions(BRepBuilderAPI_RoundCorner);
                    cur++;
                }
            }
            if (cur < n)
            {
                if (a[cur] == "-Internal")
                {
                    Internal = true;
                    cur++;
                }
            }
        }

        if (Internal)
        {
            MkDraft.SetDraft(Internal);
            di.Append("Internal Draft : \n");
        }
        else
        {
            di.Append("External Draft : \n");
        }

        TopoDS_Shape Stop = theDI.GetShape(a[7]); // shape d'arret
        if (!Stop.IsNull())
        {
            bool KeepOutside = true;
            if (Inside == 0)
            {
                KeepOutside = false;
            }
            MkDraft.Perform(Stop, KeepOutside);
        }
        else
        {
            Geom_Surface? Surf = DrawTrSurf.GetSurface(theDI, a[7]);
            if (Surf is not null)
            { // surface d'arret
                bool KeepInside = true;
                if (Inside == 1)
                {
                    KeepInside = false;
                }
                MkDraft.Perform(Surf, KeepInside);
            }
            else
            { // by Length
                double L = theDI.Atof(a[7]);
                if (L > 1.0e-7)
                {
                    MkDraft.Perform(L);
                }
                else
                {
                    return 1;
                }
            }
        }

        theDI.SetShape(a[1], MkDraft.Shape());
        theDI.SetShape("DraftShell", MkDraft.Shell());

        return 0;
    }
}
