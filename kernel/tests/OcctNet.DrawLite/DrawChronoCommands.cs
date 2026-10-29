// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Text;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// OSD_Timer: wall-clock time (Stopwatch) and the process's CPU times, as OCCT's timer (OSD_Chronometer with
/// theThisThreadOnly false, its default).
/// </summary>
internal sealed class DrawTimer
{
    private readonly Stopwatch myWall = new();
    private TimeSpan myCumulUser;
    private TimeSpan myCumulSys;
    private TimeSpan myStartUser;
    private TimeSpan myStartSys;

    public bool IsStopped => !myWall.IsRunning;

    public void Reset()
    {
        myWall.Reset();
        myCumulUser = myCumulSys = TimeSpan.Zero;
    }

    public void Restart()
    {
        Reset();
        Start();
    }

    public void Start()
    {
        if (IsStopped)
        {
            (myStartUser, myStartSys) = ProcessCpu();
            myWall.Start();
        }
    }

    public void Stop()
    {
        if (!IsStopped)
        {
            myWall.Stop();
            (TimeSpan aUser, TimeSpan aSys) = ProcessCpu();
            myCumulUser += aUser - myStartUser;
            myCumulSys += aSys - myStartSys;
        }
    }

    public double ElapsedTime() => myWall.Elapsed.TotalSeconds;

    public double UserTimeCPU() => Cpu().User;

    public double SystemTimeCPU() => Cpu().Sys;

    private (double User, double Sys) Cpu()
    {
        TimeSpan aUser = myCumulUser, aSys = myCumulSys;
        if (!IsStopped)
        {
            (TimeSpan aCurrUser, TimeSpan aCurrSys) = ProcessCpu();
            aUser += aCurrUser - myStartUser;
            aSys += aCurrSys - myStartSys;
        }

        return (aUser.TotalSeconds, aSys.TotalSeconds);
    }

    private static (TimeSpan User, TimeSpan Sys) ProcessCpu()
    {
        using Process aProcess = Process.GetCurrentProcess();
        return (aProcess.UserProcessorTime, aProcess.PrivilegedProcessorTime);
    }

    /// <summary>OSD_Timer::Show(ostream): the elapsed time, then OSD_Chronometer::Show, at precision 12.</summary>
    public string Show()
    {
        double aTimeCumul = ElapsedTime();
        int aSec = (int)aTimeCumul;
        int anHours = aSec / 3600;
        int aMinutes = (aSec - anHours * 3600) / 60;
        double aSeconds = aTimeCumul - anHours * 3600 - aMinutes * 60;
        (double aUser, double aSys) = Cpu();
        return new StringBuilder()
               .Append($"Elapsed time: {anHours} Hours {aMinutes} Minutes {Standard_Stream.FormatReal(aSeconds, 12)} Seconds\n")
               .Append($"CPU user time: {Standard_Stream.FormatReal(aUser, 12)} seconds\n")
               .Append($"CPU system time: {Standard_Stream.FormatReal(aSys, 12)} seconds\n")
               .ToString();
    }
}

/// <summary>A chronometer (Draw_Chronometer): what <c>dchrono name</c> binds.</summary>
public sealed class DrawChronometer : DrawDrawable
{
    internal DrawTimer Timer { get; } = new();

    public override string TypeName => "Draw_Chronometer";

    public override void Whatis(DrawResult theResult) => theResult.Append("chronometer");

    public override void Dump(TextWriter theStream) => theStream.Write("Chronometer, " + Timer.Show());

    public override void Save(TextWriter theStream) => throw new Standard_NotImplemented("Draw_Drawable3D::Save");

    public override DrawDrawable Copy() => new DrawChronometer();
}

/// <summary>
/// Draw_BasicCommands' dchrono and chrono (timers on Draw variables) and dlog (the command log); they were no-ops
/// before unit 3L2a.
/// </summary>
internal static class DrawChronoCommands
{
    private static bool s_drawChrono;

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "dchrono", DChronom);
        DrawCxx.Add(theInterpreter, "chrono", DChronom);
        DrawCxx.Add(theInterpreter, "dlog", DLog);
    }

    private static int DChronom(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> theArgVec)
    {
        int theNbArgs = theArgVec.Count;
        if (theNbArgs == 1 || (theNbArgs == 2 && theArgVec[1].Length > 0 && theArgVec[1][0] is '0' or '1'))
        {
            s_drawChrono = theNbArgs == 1 ? !s_drawChrono : theArgVec[1][0] == '1';
            di.Append(s_drawChrono ? "Chronometers activated.\n" : "Chronometers deactivated.\n");
            return 0;
        }

        string aTimerName = theArgVec[1];
        if (theDI.GetDrawable(aTimerName) is not DrawChronometer aChronom)
        {
            aChronom = new DrawChronometer();
            theDI.SetDrawable(aTimerName, aChronom);
        }

        if (theNbArgs <= 2)
        {
            aChronom.Timer.Reset();
            return 0;
        }

        bool toShowCout = theArgVec[0] == "chrono";
        int aNbPuts = 0;
        for (int anIter = 2; anIter < theNbArgs; ++anIter)
        {
            string anArg = theArgVec[anIter].ToLowerInvariant();
            if (anArg is "-reset" or "reset")
            {
                aChronom.Timer.Reset();
            }
            else if (anArg is "-restart" or "restart")
            {
                aChronom.Timer.Restart();
            }
            else if (anArg is "-start" or "-resume" or "start")
            {
                aChronom.Timer.Start();
            }
            else if (anArg is "-stop" or "-pause" or "stop")
            {
                aChronom.Timer.Stop();
            }
            else if (anArg is "-show" or "show")
            {
                if (toShowCout)
                {
                    theDI.Output.Write(aChronom.Timer.Show());
                }
                else
                {
                    di.Append(aChronom.Timer.Show());
                }
            }
            else if (anIter + 1 < theNbArgs && anArg is "-counter" or "counter")
            {
                string aLine = $"COUNTER {theArgVec[++anIter]}: {Standard_Stream.FormatReal(aChronom.Timer.UserTimeCPU(), 6)}\n";
                if (toShowCout)
                {
                    theDI.Output.Write(aLine);
                }
                else
                {
                    // Draw_Interpretor's << of a real is %.17g
                    di.Append($"COUNTER {theArgVec[anIter]}: ").Append(aChronom.Timer.UserTimeCPU()).Append("\n");
                }
            }
            else if (anArg is "-elapsed" or "-cpu" or "-usercpu" or "-cpuuser" or "-systemcpu" or "-syscpu" or "-cpusystem" or "-cpusys")
            {
                if (++aNbPuts > 1)
                {
                    di.Append(" ");
                }

                di.Append(anArg == "-elapsed" ? aChronom.Timer.ElapsedTime()
                          : anArg is "-cpu" or "-usercpu" or "-cpuuser" ? aChronom.Timer.UserTimeCPU()
                          : aChronom.Timer.SystemTimeCPU());
            }
            else if (anArg is "-thread" or "-threadonly" or "-process")
            {
                // OCCT switches between the thread's and the process's CPU times; the port measures the process's
                if (anIter + 1 < theNbArgs && theArgVec[anIter + 1] is "on" or "off" or "1" or "0")
                {
                    anIter++;
                }

                aChronom.Timer.Stop();
                aChronom.Timer.Reset();
            }
            else
            {
                di.Append($"Syntax error at '{theArgVec[anIter]}'\n");
                return 1;
            }
        }

        return 0;
    }

    private static int DLog(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 2 && n != 3)
        {
            theDI.Output.WriteLine($"Enable or disable logging: {a[0]} {{on|off}}\nReset log: {a[0]} reset\nGet log content: {a[0]} get");
            return 1;
        }

        if (a[1] == "on" && n == 2)
        {
            theDI.DoLog = true;
        }
        else if (a[1] == "off" && n == 2)
        {
            theDI.DoLog = false;
        }
        else if (a[1] == "reset" && n == 2)
        {
            theDI.ResetLog();
        }
        else if (a[1] == "get" && n == 2)
        {
            di.Append(theDI.GetLog());
        }
        else if (a[1] == "add" && n == 3)
        {
            theDI.Output.Write(a[2] + "\n");
        }
        else if (a[1] == "status" && n == 2)
        {
            di.Append(theDI.DoLog ? "on" : "off");
        }
        else
        {
            theDI.Output.WriteLine($"Unrecognized option(s): {a[1]}");
            return 1;
        }

        return 0;
    }
}
