// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Offsets.Blend;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest's rolling-ball commands (BRepTest_FilletCommands.cxx, unit 7G): <c>rollingball</c>, <c>brollingball</c>
/// and <c>trollingball</c> (BiTgte_Blend), written as OCCT writes them. The file's fillet commands are the fillet
/// units'.
/// </summary>
/// <remarks>
/// OCCT's tolerance is the file-scope <c>t3d</c> that <c>tolblend</c> sets (1e-4 by default), kept per interpreter
/// (decision 1144).
/// </remarks>
internal static partial class BRepTestFilletCommands
{
    private static void RegisterRollingBallCommands(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "rollingball", Rollingball);
        DrawCxx.Add(theInterpreter, "brollingball", Rollingball);
        DrawCxx.Add(theInterpreter, "trollingball", Rollingball);
    }

    //=================================================================================================

    private static int Rollingball(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        // OCCT reads a[2] and a[3] whatever n is (past the arguments for n < 4); Draw-lite reads empty words.
        Shape S = theDI.GetShape(n > 2 ? a[2] : "");
        if (S.IsNull)
        {
            return 1;
        }
        double Rad = theDI.Atof(n > 3 ? a[3] : "");

        double Tol = T3d(theDI); // the same as blend ! 1.e-7;

        OffsetBlend Roll = new();
        Roll.Init(S, Rad, Tol, false);

        int Nb = 0;
        for (int i = 4; i <= n - 1; i++)
        {
            if (a[i] == "@")
            {
                Nb++;
                continue;
            }

            if (Nb == 0)
            { // return stop faces.
                Shape aLocalFace = theDI.GetShape(a[i], ShapeType.Face);
                Face F1 = ShapeCasts.Face(aLocalFace);
                if (F1.IsNull)
                {
                    di.Append(" Stop face not referenced.\n");
                    return 1;
                }
                Roll.SetStoppingFace(F1);
            }
            else if (Nb == 1)
            { // return faces on which the ball rotates
                Shape aLocalFace = theDI.GetShape(a[i], ShapeType.Face);
                Face F1 = ShapeCasts.Face(aLocalFace);
                i++;
                // OCCT reads a[n] (the null terminator) when the support faces are odd in number at the end.
                if (i >= n || a[i] == "@")
                {
                    di.Append(" Even number of ball support faces is required \n");
                    return 1;
                }
                aLocalFace = theDI.GetShape(a[i], ShapeType.Face);
                Face F2 = ShapeCasts.Face(aLocalFace);
                if (F1.IsNull || F2.IsNull)
                {
                    di.Append(" Support face not referenced.\n");
                    return 1;
                }
                Roll.SetFaces(F1, F2);
            }
            else if (Nb == 2)
            { // return the edge on which the ball rotates
                Shape aLocalShape = theDI.GetShape(a[i], ShapeType.Edge);
                Edge E = ShapeCasts.Edge(aLocalShape);
                if (E.IsNull)
                {
                    di.Append(" Edge not referenced.\n");
                    return 1;
                }
                Roll.SetEdge(E);
            }
        }
        bool BuildShape = a[0] == "brollingball";

        Roll.Perform(BuildShape);

        bool ComputeBranches = a[0] == "trollingball";
        if (ComputeBranches)
        {
            int NbBranches = Roll.NbBranches();
            for (int i = 1; i <= NbBranches; i++)
            {
                Roll.IndicesOfBranche(i, out int From, out int To);
                di.Append(" Indexes of the ").Append(i).Append("th Branch : ");
                di.Append("   ").Append(From).Append("     ").Append(To).Append("\n");
                for (int j = From; j <= To; j++)
                {
                    Shape CurF = Roll.Face(j);
                    string localname = a[1] + "_" + i + "_" + j;
                    theDI.SetShape(localname, CurF);
                }
            }
        }
        else
        {
            theDI.SetShape(a[1], Roll.Shape());
        }

        return 0;
    }
}
