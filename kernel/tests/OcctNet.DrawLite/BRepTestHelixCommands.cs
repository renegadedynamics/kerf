// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using System.Runtime.CompilerServices;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's helix commands (BRepTest_HelixCommands.cxx): <c>setaxis</c>, <c>comphelix</c>, <c>helix</c>,
/// <c>spiral</c>, <c>comphelix2</c>, <c>helix2</c> and <c>spiral2</c>, building wires with HelixBRep_BuilderHelix,
/// written as OCCT writes them.
/// </summary>
/// <remarks>
/// OCCT keeps the axis <c>setaxis</c> sets in a file-scope static; Draw-lite keeps it per interpreter (as the viewer's
/// context, decision 995), so a corpus case starts from OCCT's default axis, as a fresh DRAWEXE does.
/// </remarks>
internal static class BRepTestHelixCommands
{
    private static readonly ConditionalWeakTable<DrawInterpreter, StrongBox<CoordinateSystem>> s_axes = new();

    // theHelixAxis: gp_Ax3(gp_Pnt(0., 0., 0), gp_Dir(gp_Dir::D::Z), gp_Dir(gp_Dir::D::X))
    private static StrongBox<CoordinateSystem> HelixAxis(DrawInterpreter theDI)
        => s_axes.GetValue(theDI, static _ => new StrongBox<CoordinateSystem>(
            new CoordinateSystem(new Point3D(0.0, 0.0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X))));

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "setaxis", Setaxis);
        DrawCxx.Add(theInterpreter, "comphelix", Comphelix);
        DrawCxx.Add(theInterpreter, "helix", Helix);
        DrawCxx.Add(theInterpreter, "spiral", Spiral);
        DrawCxx.Add(theInterpreter, "comphelix2", Comphelix2);
        DrawCxx.Add(theInterpreter, "helix2", Helix2);
        DrawCxx.Add(theInterpreter, "spiral2", Spiral2);
    }

    // Helper function to display helix results
    private static void DisplayHelixResult(DrawInterpreter theDI, DrawResult di, HelixBRep_BuilderHelix theBuilder,
                                           string theName)
    {
        if (theBuilder.ErrorStatus() == 0)
        {
            double aMaxError = theBuilder.ToleranceReached();
            di.Append("WarningStatus = ").Append(theBuilder.WarningStatus()).Append("\n");
            di.Append("ToleranceReached = ").Append(aMaxError).Append("\n");
            TopoDS_Shape aW = theBuilder.Shape();
            theDI.SetShape(theName, aW);
        }
        else
        {
            di.Append("ErrorStatus = ").Append(theBuilder.ErrorStatus()).Append("\n");
        }
    }

    //=================================================================================================

    private static int Setaxis(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 10)
        {
            di.Append("Usage : ").Append(a[0]).Append(" x y z Nx Ny Nz Xx Xy Xz").Append("\n");
            di.Append("        ").Append("x y z - location").Append("\n");
            di.Append("        ").Append("Nx Ny Nz - direction").Append("\n");
            di.Append("        ").Append("Xx Xy Xz - X direction").Append("\n");
            return 1;
        }
        double[] xx = new double[9];
        int i;
        for (i = 0; i < 9; ++i)
        {
            xx[i] = theDI.Atof(a[i + 1]);
        }

        StrongBox<CoordinateSystem> theHelixAxis = HelixAxis(theDI);
        theHelixAxis.Value.SetLocation(new Point3D(xx[0], xx[1], xx[2]));
        theHelixAxis.Value.SetDirection(new Direction3D(xx[3], xx[4], xx[5]));
        theHelixAxis.Value.SetXDirection(new Direction3D(xx[6], xx[7], xx[8]));

        return 0;
    }

    //=================================================================================================

    private static int ComphelixUsage(DrawResult di, IReadOnlyList<string> a)
    {
        di.Append("Usage : ").Append(a[0]).Append(" name np D1 D2 [Di...] H1 [Hi...] P1 [Pi...] PF1 [PFi...]").Append("\n");
        di.Append("        ").Append("name - name of result").Append("\n");
        di.Append("        ").Append("np - number of helix parts").Append("\n");
        di.Append("        ").Append("D1 D2 ... (must be np+1 values) - diameters").Append("\n");
        di.Append("        ").Append("H1, H2 ... (must be np values) - heights").Append("\n");
        di.Append("        ").Append("P1, P2, ...  (must be np values) - pitches or numbers of turns").Append("\n");
        di.Append("        ")
          .Append("PF1, PF2 ... (must be np values) - 0 or 1, if PFi = 1, Pi is pitch, otherwise Pi is "
                  + "number of turns")
          .Append("\n");
        return 1;
    }

    private static int Comphelix(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 8)
        {
            return ComphelixUsage(di, a);
        }
        //
        int i, aNb, ic;
        HelixBRep_BuilderHelix aBH = new();
        //
        aNb = theDI.Atoi(a[2]);
        if (n != 3 + (aNb + 1) + aNb * 3)
        {
            return ComphelixUsage(di, a);
        }

        NCollection_Array1<double> aDiams = new(1, aNb + 1);
        NCollection_Array1<double> aHeights = new(1, aNb);
        NCollection_Array1<double> aPitches = new(1, aNb);
        NCollection_Array1<bool> bIsPitches = new(1, aNb);

        ic = 3;
        for (i = 1; i <= aNb + 1; ++i)
        {
            aDiams[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aHeights[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aPitches[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            bIsPitches[i] = theDI.Atoi(a[ic]) != 0;
            ++ic;
        }

        aBH.SetParameters(HelixAxis(theDI).Value, aDiams, aHeights, aPitches, bIsPitches);

        aBH.Perform();
        DisplayHelixResult(theDI, di, aBH, a[1]);

        return 0;
    }

    //=================================================================================================

    private static int HelixUsage(DrawResult di, IReadOnlyList<string> a)
    {
        di.Append("Usage : ").Append(a[0]).Append(" name np D1 H1 [Hi...] P1 [Pi...] PF1 [PFi...]").Append("\n");
        di.Append("        ").Append("name - name of result").Append("\n");
        di.Append("        ").Append("np - number of helix parts").Append("\n");
        di.Append("        ").Append("D1 - diameter").Append("\n");
        di.Append("        ").Append("H1, H2 ... (must be np values) - heights").Append("\n");
        di.Append("        ").Append("P1, P2, ...  (must be np values) - pitches or numbers of turns").Append("\n");
        di.Append("        ")
          .Append("PF1, PF2 ... (must be np values) - 0 or 1, if PFi = 1, Pi is pitch, otherwise Pi is "
                  + "number of turns")
          .Append("\n");
        return 1;
    }

    private static int Helix(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 7)
        {
            return HelixUsage(di, a);
        }
        //
        int i, aNb, ic;
        HelixBRep_BuilderHelix aBH = new();
        //
        aNb = theDI.Atoi(a[2]);
        if (n != 3 + 1 + aNb * 3)
        {
            return HelixUsage(di, a);
        }

        NCollection_Array1<double> aDiams = new(1, 1);
        NCollection_Array1<double> aHeights = new(1, aNb);
        NCollection_Array1<double> aPitches = new(1, aNb);
        NCollection_Array1<bool> bIsPitches = new(1, aNb);

        ic = 3;
        aDiams[1] = theDI.Atof(a[ic]);
        ++ic;

        for (i = 1; i <= aNb; ++i)
        {
            aHeights[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aPitches[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            bIsPitches[i] = theDI.Atoi(a[ic]) != 0;
            ++ic;
        }

        aBH.SetParameters(HelixAxis(theDI).Value, aDiams[1], aHeights, aPitches, bIsPitches);

        aBH.Perform();
        DisplayHelixResult(theDI, di, aBH, a[1]);

        return 0;
    }

    //=================================================================================================

    private static int SpiralUsage(DrawResult di, IReadOnlyList<string> a)
    {
        di.Append("Usage : ").Append(a[0]).Append(" name np D1 D2 H1 [Hi...] P1 [Pi...] PF1 [PFi...]").Append("\n");
        di.Append("        ").Append("name - name of result").Append("\n");
        di.Append("        ").Append("np - number of helix parts").Append("\n");
        di.Append("        ").Append("D1 D2 - first and last diameters").Append("\n");
        di.Append("        ").Append("H1, H2 ... (must be np values) - heights").Append("\n");
        di.Append("        ").Append("P1, P2, ...  (must be np values) - pitches or numbers of turns").Append("\n");
        di.Append("        ")
          .Append("PF1, PF2 ... (must be np values) - 0 or 1, if PFi = 1, Pi is pitch, otherwise Pi is "
                  + "number of turns")
          .Append("\n");
        return 1;
    }

    private static int Spiral(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 8)
        {
            return SpiralUsage(di, a);
        }
        //
        int i, aNb, ic;
        HelixBRep_BuilderHelix aBH = new();
        //
        aNb = theDI.Atoi(a[2]);
        if (n != 3 + 2 + aNb * 3)
        {
            return SpiralUsage(di, a);
        }

        NCollection_Array1<double> aDiams = new(1, 2);
        NCollection_Array1<double> aHeights = new(1, aNb);
        NCollection_Array1<double> aPitches = new(1, aNb);
        NCollection_Array1<bool> bIsPitches = new(1, aNb);

        ic = 3;
        for (i = 1; i <= 2; ++i)
        {
            aDiams[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aHeights[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aPitches[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            bIsPitches[i] = theDI.Atoi(a[ic]) != 0;
            ++ic;
        }

        aBH.SetParameters(HelixAxis(theDI).Value, aDiams[1], aDiams[2], aHeights, aPitches, bIsPitches);

        aBH.Perform();
        DisplayHelixResult(theDI, di, aBH, a[1]);

        return 0;
    }

    //=================================================================================================

    private static int Comphelix2Usage(DrawResult di, IReadOnlyList<string> a)
    {
        di.Append("Usage : ").Append(a[0]).Append(" name np D1 D2 [Di...] P1 [Pi...] N1 [Ni...]").Append("\n");
        di.Append("        ").Append("name - name of result").Append("\n");
        di.Append("        ").Append("np - number of helix parts").Append("\n");
        di.Append("        ").Append("D1 D2, ... (must be np+1 values) - diameters").Append("\n");
        di.Append("        ").Append("P1, P2, ...  (must be np values) - pitches").Append("\n");
        di.Append("        ").Append("N1, N2, ...  (must be np values) - numbers of turns").Append("\n");
        return 1;
    }

    private static int Comphelix2(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 7)
        {
            return Comphelix2Usage(di, a);
        }
        //
        int i, aNb, ic;
        HelixBRep_BuilderHelix aBH = new();
        //
        aNb = theDI.Atoi(a[2]);
        if (n != 3 + (aNb + 1) + aNb * 2)
        {
            return Comphelix2Usage(di, a);
        }

        NCollection_Array1<double> aDiams = new(1, aNb + 1);
        NCollection_Array1<double> aPitches = new(1, aNb);
        NCollection_Array1<double> aNbTurns = new(1, aNb);

        ic = 3;
        for (i = 1; i <= aNb + 1; ++i)
        {
            aDiams[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aPitches[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aNbTurns[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        //

        aBH.SetParameters(HelixAxis(theDI).Value, aDiams, aPitches, aNbTurns);

        aBH.Perform();
        DisplayHelixResult(theDI, di, aBH, a[1]);

        return 0;
    }

    //=================================================================================================

    private static int Helix2(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 6)
        {
            di.Append("Usage : ").Append(a[0]).Append(" name np D1 P1 [Pi...] N1 [Ni...]").Append("\n");
            di.Append("        ").Append("name - name of result").Append("\n");
            di.Append("        ").Append("np - number of helix parts").Append("\n");
            di.Append("        ").Append("D1 - diameter").Append("\n");
            di.Append("        ").Append("P1, P2, ...  (must be np values) - pitches").Append("\n");
            di.Append("        ").Append("N1, N2, ...  (must be np values) - numbers of turns").Append("\n");
            return 1;
        }
        //
        int i, aNb, ic;
        HelixBRep_BuilderHelix aBH = new();
        //
        aNb = theDI.Atoi(a[2]);
        if (n != 3 + 1 + aNb * 2)
        {
            // OCCT: the usage line is spiral's and helix's (H and PF), not helix2's
            di.Append("Usage : ").Append(a[0]).Append(" name np D1 H1 [Hi...] P1 [Pi...] PF1 [PFi...]").Append("\n");
            di.Append("        ").Append("name - name of result").Append("\n");
            di.Append("        ").Append("np - number of helix parts").Append("\n");
            di.Append("        ").Append("D1 - diameter").Append("\n");
            di.Append("        ").Append("P1, P2, ...  (must be np values) - pitches").Append("\n");
            di.Append("        ").Append("N1, N2, ...  (must be np values) - numbers of turns").Append("\n");
            return 1;
        }

        NCollection_Array1<double> aDiams = new(1, 1);
        NCollection_Array1<double> aPitches = new(1, aNb);
        NCollection_Array1<double> aNbTurns = new(1, aNb);

        ic = 3;
        aDiams[1] = theDI.Atof(a[ic]);
        ++ic;

        for (i = 1; i <= aNb; ++i)
        {
            aPitches[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aNbTurns[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        aBH.SetParameters(HelixAxis(theDI).Value, aDiams[1], aPitches, aNbTurns);

        aBH.Perform();
        DisplayHelixResult(theDI, di, aBH, a[1]);

        return 0;
    }

    //=================================================================================================

    private static int Spiral2(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 7)
        {
            di.Append("Usage : ").Append(a[0]).Append(" name np D1 D2 P1 [Pi...] N1 [Ni...]").Append("\n");
            di.Append("        ").Append("name - name of result").Append("\n");
            di.Append("        ").Append("np - number of helix parts").Append("\n");
            di.Append("        ").Append("D1 D2 - first and last diameters").Append("\n");
            di.Append("        ").Append("P1, P2, ...  (must be np values) - pitches").Append("\n");
            di.Append("        ").Append("N1, N2, ...  (must be np values) - numbers of turns").Append("\n");
            return 1;
        }
        //
        int i, aNb, ic;
        HelixBRep_BuilderHelix aBH = new();
        //
        aNb = theDI.Atoi(a[2]);
        if (n != 3 + 2 + aNb * 2)
        {
            // OCCT: the usage line is spiral's (H and PF), not spiral2's
            di.Append("Usage : ").Append(a[0]).Append(" name np D1 D2 H1 [Hi...] P1 [Pi...] PF1 [PFi...]").Append("\n");
            di.Append("        ").Append("name - name of result").Append("\n");
            di.Append("        ").Append("np - number of helix parts").Append("\n");
            di.Append("        ").Append("D1 D2 - first and last diameters").Append("\n");
            di.Append("        ").Append("P1, P2, ...  (must be np values) - pitches").Append("\n");
            di.Append("        ").Append("N1, N2, ...  (must be np values) - numbers of turns").Append("\n");
            return 1;
        }

        NCollection_Array1<double> aDiams = new(1, 2);
        NCollection_Array1<double> aPitches = new(1, aNb);
        NCollection_Array1<double> aNbTurns = new(1, aNb);

        ic = 3;
        for (i = 1; i <= 2; ++i)
        {
            aDiams[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aPitches[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        for (i = 1; i <= aNb; ++i)
        {
            aNbTurns[i] = theDI.Atof(a[ic]);
            ++ic;
        }

        aBH.SetParameters(HelixAxis(theDI).Value, aDiams[1], aDiams[2], aPitches, aNbTurns);

        aBH.Perform();
        DisplayHelixResult(theDI, di, aBH, a[1]);

        return 0;
    }
}
