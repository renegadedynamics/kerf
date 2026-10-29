// Port of: src/FoundationClasses/TKMath/BVH/BVH_BuildThread.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_BuildThread.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A tool to call BVH builder subroutines.</summary>
public abstract class BVH_BuildTool
{
    /// <summary>Splits the given BVH node.</summary>
    public abstract void Perform(int theNode);
}

/// <summary>A BVH build thread: takes nodes from the queue and splits them until no thread has work left.</summary>
/// <remarks>OCCT's <c>OSD_Thread</c> is a <see cref="Thread"/>.</remarks>
public class BVH_BuildThread : Standard_Transient
{
    /// <summary>Data needed to build the BVH.</summary>
    protected BVH_BuildTool myBuildTool;

    /// <summary>The BVH build queue.</summary>
    protected BVH_BuildQueue myBuildQueue;

    /// <summary>Thread to execute work items.</summary>
    protected Thread? myWorkThread;

    /// <summary>Creates a new BVH build thread.</summary>
    public BVH_BuildThread(BVH_BuildTool theBuildTool, BVH_BuildQueue theBuildQueue)
    {
        myBuildTool = theBuildTool;
        myBuildQueue = theBuildQueue;
    }

    /// <summary>Starts the thread.</summary>
    public void Run()
    {
        myWorkThread = new Thread(execute) { IsBackground = true };
        myWorkThread.Start();
    }

    /// <summary>Waits till the thread finishes.</summary>
    public void Wait() => myWorkThread?.Join();

    /// <summary>Executes the work loop (in the calling thread, when not started with <see cref="Run"/>).</summary>
    protected internal void execute()
    {
        for (bool wasBusy = false; /**/; /**/)
        {
            int aNode = myBuildQueue.Fetch(ref wasBusy);

            if (aNode == -1) // queue is empty
            {
                if (!myBuildQueue.HasBusyThreads())
                {
                    break; // no active threads
                }
            }
            else
            {
                myBuildTool.Perform(aNode);
            }
        }
    }
}
