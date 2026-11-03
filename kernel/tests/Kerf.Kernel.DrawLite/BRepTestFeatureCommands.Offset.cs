// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using Occt;
using static Kerf.Engine.Offsets.Error;
using static Kerf.Engine.Offsets.OffsetMode;
using static Kerf.Geometry.JoinKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// BRepTest's 3D offset commands (BRepTest_FeatureCommands.cxx, unit 7G): <c>thickshell</c>, <c>mkoffsetshape</c>,
/// <c>offsetshape</c>, <c>offsetcompshape</c>, <c>offsetparameter</c>, <c>offsetload</c>, <c>offsetonface</c>,
/// <c>offsetperform</c> and <c>offsetshapesimple</c>, written as OCCT writes them. The file's feature commands are
/// BRepFeat's (unit 7I).
/// </summary>
/// <remarks>
/// OCCT keeps one BRepOffset_MakeOffset and its parameters in file-scope statics between <c>offsetparameter</c>,
/// <c>offsetload</c>, <c>offsetonface</c> and <c>offsetperform</c>; Draw-lite keeps them per interpreter (as the helix
/// axis, decision 995), so a corpus case starts from OCCT's defaults, as a fresh DRAWEXE does (decision 1048). The
/// progress indicator OCCT passes is left out.
/// </remarks>
internal static partial class BRepTestFeatureCommands
{
    // OCCT's statics TheOffset, TheRadius, theYaBouchon, TheTolerance, TheInter, TheJoin and RemoveIntEdges.
    private sealed class OffsetState
    {
        public readonly MakeOffset TheOffset = new();
        public double TheRadius;
        public bool theYaBouchon;
        public double TheTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        public bool TheInter;
        public JoinKind TheJoin = Arc;
        public bool RemoveIntEdges;
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, OffsetState> s_offsetStates = new();

    private static OffsetState Offset(DrawInterpreter theDI) => s_offsetStates.GetValue(theDI, static _ => new OffsetState());

    public static void RegisterOffsetCommands(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "thickshell", Thickshell);
        DrawCxx.Add(theInterpreter, "mkoffsetshape", Mkoffsetshape);
        DrawCxx.Add(theInterpreter, "offsetshape", Offsetshape);
        DrawCxx.Add(theInterpreter, "offsetcompshape", Offsetshape);
        DrawCxx.Add(theInterpreter, "offsetparameter", Offsetparameter);
        DrawCxx.Add(theInterpreter, "offsetload", Offsetload);
        DrawCxx.Add(theInterpreter, "offsetonface", Offsetonface);
        DrawCxx.Add(theInterpreter, "offsetperform", Offsetperform);
        DrawCxx.Add(theInterpreter, "offsetshapesimple", ComputeSimpleOffset);
    }

    private static void reportOffsetState(DrawResult theCommands, Error theErrorCode)
    {
        switch (theErrorCode)
        {
            case Error.NoError:
                theCommands.Append("OK. Offset performed successfully.");
                break;
            case Error.BadNormalsOnGeometry:
                theCommands.Append("ERROR. Degenerated normal on input data.");
                break;
            case Error.C0Geometry:
                theCommands.Append("ERROR. C0 continuity of input data.");
                break;
            case Error.NullOffset:
                theCommands.Append("ERROR. Null offset of all faces.");
                break;
            case Error.NotConnectedShell:
                theCommands.Append("ERROR. Incorrect set of faces to remove, the remaining shell is not connected.");
                break;
            case Error.CannotTrimEdges:
                theCommands.Append("ERROR. Can not trim edges.");
                break;
            case Error.CannotFuseVertices:
                theCommands.Append("ERROR. Can not fuse vertices.");
                break;
            case Error.CannotExtentEdge:
                theCommands.Append("ERROR. Can not extent edge.");
                break;
            case Error.MixedConnectivity:
                theCommands.Append("ERROR. Mixed connectivity of faces.");
                break;
            default:
                theCommands.Append("ERROR. offsetperform operation not done.");
                break;
        }
    }

    //=================================================================================================

    private static int Thickshell(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }
        Shape S = theDI.GetShape(a[2]);
        if (S.IsNull)
        {
            return 1;
        }

        double Of = theDI.Atof(a[3]);

        JoinKind JT = Arc;
        if (n > 4)
        {
            if (a[4] == "i")
            {
                JT = Intersection;
            }
            if (a[4] == "t")
            {
                JT = Tangent;
            }
        }

        bool Inter = false; // true;
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (n > 5)
        {
            Tol = theDI.Atof(a[5]);
        }

        MakeOffset B = new();
        B.Initialize(S, Of, Tol, OffsetMode.Skin, Inter, false, JT, true);

        B.MakeOffsetShape();

        Error aRetCode = B.Error();
        reportOffsetState(theCommands, aRetCode);

        if (!B.Shape().IsNull)
        {
            theDI.SetShape(a[1], B.Shape());
        }
        return 0;
    }

    //=================================================================================================

    private static int Mkoffsetshape(DrawInterpreter theDI, DrawResult theDIResult, IReadOnlyList<string> theArgVec)
    {
        int theArgNb = theArgVec.Count;
        if (theArgNb < 4)
        {
            return 0;
        }
        Shape aShape = theDI.GetShape(theArgVec[2]);
        if (aShape.IsNull)
        {
            theDIResult.Append("Shape is null");
            return 1;
        }
        double anOffVal = theDI.Atof(theArgVec[3]);
        MakeOffsetShape aMaker = new();
        if (theArgNb == 4)
        {
            aMaker.PerformBySimple(aShape, anOffVal);
        }
        else
        {
            double aTol = theDI.Atof(theArgVec[4]);

            bool anInt = false;
            if (theArgNb > 5)
            {
                if (theDI.Atof(theArgVec[5]) == 1)
                {
                    anInt = true;
                }
            }

            bool aSelfInt = false;
            if (theArgNb > 6)
            {
                if (theDI.Atof(theArgVec[6]) == 1)
                {
                    aSelfInt = true;
                }
            }

            JoinKind aJoin = Arc;
            if (theArgNb > 7)
            {
                if (theArgVec[7] == "i")
                {
                    aJoin = Intersection;
                }
            }

            bool aRemIntEdges = false;
            if (theArgNb > 8)
            {
                if (theDI.Atof(theArgVec[8]) == 1)
                {
                    aRemIntEdges = true;
                }
            }
            aMaker.PerformByJoin(aShape, anOffVal, aTol, OffsetMode.Skin, anInt, aSelfInt, aJoin, aRemIntEdges);
        }

        if (!aMaker.IsDone)
        {
            theDIResult.Append(" Error: Offset is not done.\n");
            return 1;
        }
        theDI.SetShape(theArgVec[1], aMaker.Shape());
        return 0;
    }

    //=================================================================================================

    private static int Offsetshape(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }
        Shape S = theDI.GetShape(a[2]);
        if (S.IsNull)
        {
            return 1;
        }

        double Of = theDI.Atof(a[3]);
        bool Inter = a[0] == "offsetcompshape";
        JoinKind JT = Arc;
        if (a[0] == "offsetinter")
        {
            JT = Intersection;
            Inter = true;
        }

        MakeOffset B = new();
        int IB = 4;
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (n > 4)
        {
            Shape SF = theDI.GetShape(a[4], ShapeType.Face);
            if (SF.IsNull)
            {
                IB = 5;
                Tol = theDI.Atof(a[4]);
            }
        }
        B.Initialize(S, Of, Tol, OffsetMode.Skin, Inter, false, JT);
        //------------------------------------------
        // recuperation et chargement des bouchons.
        //----------------------------------------
        bool YaBouchon = false;

        for (int i = IB; i < n; i++)
        {
            Shape SF = theDI.GetShape(a[i], ShapeType.Face);
            if (!SF.IsNull)
            {
                YaBouchon = true;
                B.AddFace(ShapeCasts.Face(SF));
            }
        }

        if (!YaBouchon)
        {
            B.MakeOffsetShape();
        }
        else
        {
            B.MakeThickSolid();
        }

        Error aRetCode = B.Error();
        reportOffsetState(theCommands, aRetCode);

        if (!B.Shape().IsNull)
        {
            theDI.SetShape(a[1], B.Shape());
        }

        return 0;
    }

    //=================================================================================================

    private static int Offsetparameter(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        OffsetState s = Offset(theDI);
        if (n == 1)
        {
            di.Append(" offsetparameter Tol Inter(c/p) JoinType(a/i/t) [RemoveInternalEdges(r/k)]\n");
            di.Append(" Current Values\n");
            di.Append("   --> Tolerance : ").Append(s.TheTolerance).Append("\n");
            di.Append("   --> TheInter  : ");
            if (s.TheInter)
            {
                di.Append("Complete");
            }
            else
            {
                di.Append("Partial");
            }
            di.Append("\n   --> TheJoin   : ");

            switch (s.TheJoin)
            {
                case Arc:
                    di.Append("Arc");
                    break;
                case Intersection:
                    di.Append("Intersection");
                    break;
                default:
                    break;
            }
            //
            di.Append("\n   --> Internal Edges : ");
            if (s.RemoveIntEdges)
            {
                di.Append("Remove");
            }
            else
            {
                di.Append("Keep");
            }
            di.Append("\n");
            //
            return 0;
        }

        if (n < 4)
        {
            return 1;
        }
        //
        s.TheTolerance = theDI.Atof(a[1]);
        s.TheInter = a[2] != "p";
        //
        if (a[3] == "a")
        {
            s.TheJoin = Arc;
        }
        else if (a[3] == "i")
        {
            s.TheJoin = Intersection;
        }
        else if (a[3] == "t")
        {
            s.TheJoin = Tangent;
        }
        //
        s.RemoveIntEdges = (n >= 5) && a[4] == "r";
        //
        return 0;
    }

    //=================================================================================================

    private static int Offsetload(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }
        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
        {
            return 1;
        }

        OffsetState s = Offset(theDI);
        // OCCT reads a[2] even when n == 2 (past the arguments: Atof of a null pointer); Draw-lite reads an empty word.
        double Of = theDI.Atof(n > 2 ? a[2] : "");
        s.TheRadius = Of;
        //  bool Inter = true;

        s.TheOffset.Initialize(S, Of, s.TheTolerance, OffsetMode.Skin, s.TheInter, false, s.TheJoin, false, s.RemoveIntEdges);
        //------------------------------------------
        // recuperation et chargement des bouchons.
        //----------------------------------------
        for (int i = 3; i < n; i++)
        {
            Shape SF = theDI.GetShape(a[i], ShapeType.Face);
            if (!SF.IsNull)
            {
                s.TheOffset.AddFace(ShapeCasts.Face(SF));
            }
        }
        s.theYaBouchon = n >= 4;

        return 0;
    }

    //=================================================================================================

    private static int Offsetonface(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        OffsetState s = Offset(theDI);
        for (int i = 1; i < n; i += 2)
        {
            Shape SF = theDI.GetShape(a[i], ShapeType.Face);
            if (!SF.IsNull)
            {
                // OCCT reads a[i + 1] past the arguments for an odd count; Draw-lite reads an empty word.
                double Of = theDI.Atof(i + 1 < n ? a[i + 1] : "");
                s.TheOffset.SetOffsetOnFace(ShapeCasts.Face(SF), Of);
            }
        }

        return 0;
    }

    //=================================================================================================

    private static int Offsetperform(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int theNArg = a.Count;
        if (theNArg < 2)
        {
            return 1;
        }

        OffsetState s = Offset(theDI);
        if (s.theYaBouchon)
        {
            s.TheOffset.MakeThickSolid();
        }
        else
        {
            s.TheOffset.MakeOffsetShape();
        }

        if (s.TheOffset.IsDone)
        {
            theDI.SetShape(a[1], s.TheOffset.Shape());
        }
        else
        {
            Error aRetCode = s.TheOffset.Error();
            reportOffsetState(theCommands, aRetCode);
        }

        // Store the history of Boolean operation into the session
        if (BRepTestHistoryCommands.IsHistoryNeeded(theDI))
        {
            NCollection_List<Shape> aLA = new();
            aLA.Append(s.TheOffset.InitShape());
            BRepTestHistoryCommands.SetHistory(theDI, aLA, new OffsetHistory(s.TheOffset));
        }

        return 0;
    }

    // OCCT's BRepTest_Objects::SetHistory<BRepOffset_MakeOffset> reads the algorithm's Generated, Modified and
    // IsDeleted through a template; the port's history takes the interface.
    private sealed class OffsetHistory(MakeOffset theAlgo) : History.IAlgorithm
    {
        public bool IsDeleted(in Shape theS) => theAlgo.IsDeleted(theS);

        public IEnumerable<Shape> Modified(in Shape theS) => theAlgo.Modified(theS);

        public IEnumerable<Shape> Generated(in Shape theS) => theAlgo.Generated(theS);
    }

    //=================================================================================================

    private static int ComputeSimpleOffset(DrawInterpreter theDI, DrawResult theCommands, IReadOnlyList<string> a)
    {
        int narg = a.Count;
        if (narg < 4)
        {
            theCommands.Append("offsetshapesimple result shape offsetvalue [solid] [tolerance=1e-7]\n");
            return 1;
        }

        // Input data.
        Shape aShape = theDI.GetShape(a[2]);
        if (aShape.IsNull)
        {
            theCommands.Append("Input shape is null");
            return 0;
        }
        double anOffsetValue = theDI.Atof(a[3]);
        if (Math.Abs(anOffsetValue) < Constants.Resolution())
        {
            theCommands.Append("Null offset value");
            return 0;
        }

        bool makeSolid = narg > 4 && string.Equals(a[4], "solid", StringComparison.OrdinalIgnoreCase);
        int iTolArg = (makeSolid ? 5 : 4);
        double aTol = (narg > iTolArg ? theDI.Atof(a[iTolArg]) : global::Kerf.Engine.Foundation.Precision.Confusion());

        MakeSimpleOffset aMaker = new(aShape, anOffsetValue);
        aMaker.SetTolerance(aTol);
        aMaker.SetBuildSolidFlag(makeSolid);
        aMaker.Perform();

        if (!aMaker.IsDone)
        {
            theCommands.Append("ERROR:").Append(aMaker.GetErrorMessage()).Append("\n");
            return 0;
        }

        theDI.SetShape(a[1], aMaker.GetResultShape());

        return 0;
    }
}
