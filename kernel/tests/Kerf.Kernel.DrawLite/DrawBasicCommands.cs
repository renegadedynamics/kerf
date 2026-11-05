// Port of: src/Draw/TKDraw/Draw/Draw_BasicCommands.cxx (dversion, meminfo)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Kerf.Kernel.DrawLite;

/// <summary>Draw's basic commands (Draw_BasicCommands.cxx) that the corpus calls: <c>dversion</c> and <c>meminfo</c>.</summary>
/// <remarks>
/// <para><c>dversion</c> prints what DRAWEXE prints for OCCT 8.0.1 built without the optional libraries, with
/// exceptions enabled (the port keeps OCCT's argument checks, decision 9), "Debug mode" when Draw-lite itself is a
/// Debug build (the corpus relaxes its time limits on that line, as for a Debug DRAWEXE), and the architecture and
/// OS .NET runs on. The compiler line is OCCT's fallback, "Compiler: unrecognized": no C++ compiler built the port.</para>
/// <para><c>meminfo</c> reads OSD_MemInfo's counters from the process (System.Diagnostics.Process); the heap counter,
/// which OCCT sums from the C heap, is the managed heap's live bytes after a full collection (GC.GetTotalMemory), so
/// that the corpus's leak checks compare what the commands keep alive.</para>
/// </remarks>
internal static class DrawBasicCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "dversion", DVersion);
        DrawCxx.Add(theInterpreter, "meminfo", DMemInfo);
    }

    private static int DVersion(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        // print OCCT version and OCCTY-specific macros used
        di.Append("Open CASCADE Technology 8.0.1\n");
        // Kerf: no "Debug mode" line in either configuration. The port leaves out OCCT_DEBUG code, so it is always
        // OCCT's release build, and the corpus scripts that read the line run the same in Debug and Release builds
        di.Append("Tk disabled\n");
        if (!OperatingSystem.IsWindows())
        {
            di.Append("Xlib disabled\n");
        }
        di.Append("TBB disabled\n");
        di.Append("FreeType disabled\n");
        di.Append("FreeImage disabled\n");
        di.Append("FFmpeg disabled\n");
        di.Append("OpenVR disabled\n");
        di.Append("RapidJSON disabled\n");
        di.Append("Draco disabled\n");
        di.Append("VTK disabled\n");
        di.Append("Exceptions enabled\n");

        // check compiler, OS, etc. using pre-processor macros provided by compiler: no C++ compiler here
        di.Append("Compiler: unrecognized\n");

        // architecture
        di.Append(RuntimeInformation.ProcessArchitecture switch
        {
            Architecture.X64 => "Architecture: AMD64\n",
            Architecture.X86 => "Architecture: Intel x86\n",
            Architecture.Arm64 => "Architecture: ARM 64-bit\n",
            Architecture.Arm => "Architecture: ARM 32-bit\n",
            Architecture.Wasm => "Architecture: WASM 32-bit\n",
            _ => "Architecture: unrecognized\n",
        });

        // OS
        if (OperatingSystem.IsWindows())
        {
            di.Append("OS: Windows\n");
        }
        else if (OperatingSystem.IsMacOS())
        {
            di.Append("OS: Mac OS X\n");
        }
        else if (OperatingSystem.IsAndroid())
        {
            di.Append("OS: Android\n");
        }
        else if (OperatingSystem.IsLinux())
        {
            di.Append("OS: Linux\n");
        }
        else if (OperatingSystem.IsFreeBSD())
        {
            di.Append("OS: BSD\n");
        }
        else
        {
            di.Append("OS: unrecognized\n");
        }

        return 0;
    }

    // OSD_MemInfo::Counter
    private enum MemCounter
    {
        MemPrivate = 0,
        MemVirtual,
        MemWorkingSet,
        MemWorkingSetPeak,
        MemSwapUsage,
        MemSwapUsagePeak,
        MemHeapUsage,
    }

    // OSD_MemInfo::Value, in bytes
    private static double Value(MemCounter theCounter)
    {
        using Process aProcess = Process.GetCurrentProcess();
        return theCounter switch
        {
            MemCounter.MemPrivate => aProcess.PrivateMemorySize64,
            MemCounter.MemVirtual => aProcess.VirtualMemorySize64,
            MemCounter.MemWorkingSet => aProcess.WorkingSet64,
            MemCounter.MemWorkingSetPeak => aProcess.PeakWorkingSet64,
            MemCounter.MemSwapUsage => aProcess.PagedMemorySize64,
            MemCounter.MemSwapUsagePeak => aProcess.PeakPagedMemorySize64,
            _ => GC.GetTotalMemory(true),
        };
    }

    // OSD_MemInfo::ToString
    private static string MemInfoText()
    {
        static long MiB(MemCounter theCounter) => (long)Value(theCounter) / (1024 * 1024);

        return $"  Private memory:     {MiB(MemCounter.MemPrivate)} MiB\n"
               + $"  Working Set:        {MiB(MemCounter.MemWorkingSet)} MiB (peak: {MiB(MemCounter.MemWorkingSetPeak)} MiB)\n"
               + $"  Pagefile usage:     {MiB(MemCounter.MemSwapUsage)} MiB (peak: {MiB(MemCounter.MemSwapUsagePeak)} MiB)\n"
               + $"  Virtual memory:     {MiB(MemCounter.MemVirtual)} MiB\n"
               + $"  Heap memory:     {MiB(MemCounter.MemHeapUsage)} MiB\n";
    }

    private static int DMemInfo(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        int theArgNb = theArgVec.Count;
        if (theArgNb <= 1)
        {
            theResult.Append(MemInfoText());
            return 0;
        }

        var aCounters = new SortedSet<MemCounter>();
        for (int anIter = 1; anIter < theArgNb; ++anIter)
        {
            string anArg = theArgVec[anIter].ToLowerInvariant();
            if (anArg == "virt" || anArg == "v")
            {
                aCounters.Add(MemCounter.MemVirtual);
            }
            else if (anArg == "heap" || anArg == "h")
            {
                aCounters.Add(MemCounter.MemHeapUsage);
            }
            else if (anArg == "wset" || anArg == "w")
            {
                aCounters.Add(MemCounter.MemWorkingSet);
            }
            else if (anArg == "wsetpeak")
            {
                aCounters.Add(MemCounter.MemWorkingSetPeak);
            }
            else if (anArg == "swap")
            {
                aCounters.Add(MemCounter.MemSwapUsage);
            }
            else if (anArg == "swappeak")
            {
                aCounters.Add(MemCounter.MemSwapUsagePeak);
            }
            else if (anArg == "private")
            {
                aCounters.Add(MemCounter.MemPrivate);
            }
            else
            {
                theResult.Append("Syntax error at '").Append(theArgVec[anIter]).Append("'!\n");
                return 1;
            }
        }

        // NCollection_Map of the counters iterates them in their order (an integer hashes to itself)
        foreach (MemCounter aCounter in aCounters)
        {
            theResult.Append(Value(aCounter)).Append(" ");
        }
        theResult.Append("\n");
        return 0;
    }
}
