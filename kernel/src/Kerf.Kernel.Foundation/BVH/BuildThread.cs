// Port of: src/FoundationClasses/TKMath/BVH/BVH_BuildThread.hxx
// Port of: src/FoundationClasses/TKMath/BVH/BVH_BuildThread.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A tool to call BVH builder subroutines.</summary>
internal abstract class BuildTool
{
    /// <summary>Splits the given BVH node.</summary>
    public abstract void Perform(int node);
}

/// <summary>A BVH build thread: takes nodes from the queue and splits them until no thread has work left.</summary>
/// <remarks>OCCT's <c>OSD_Thread</c> is a <see cref="Thread"/>.</remarks>
internal class BuildThread : Transient
{
    /// <summary>Data needed to build the BVH.</summary>
    protected BuildTool _buildTool;

    /// <summary>The BVH build queue.</summary>
    protected BuildQueue _buildQueue;

    /// <summary>Thread to execute work items.</summary>
    protected Thread? _workThread;

    /// <summary>Creates a new BVH build thread.</summary>
    public BuildThread(BuildTool buildTool, BuildQueue buildQueue)
    {
        _buildTool = buildTool;
        _buildQueue = buildQueue;
    }

    /// <summary>Starts the thread.</summary>
    public void Run()
    {
        _workThread = new Thread(Execute) { IsBackground = true };
        _workThread.Start();
    }

    /// <summary>Waits till the thread finishes.</summary>
    public void Wait() => _workThread?.Join();

    /// <summary>Executes the work loop (in the calling thread, when not started with <see cref="Run"/>).</summary>
    protected internal void Execute()
    {
        for (bool wasBusy = false; /**/; /**/)
        {
            int node = _buildQueue.Fetch(ref wasBusy);

            if (node == -1) // queue is empty
            {
                if (!_buildQueue.HasBusyThreads)
                {
                    break; // no active threads
                }
            }
            else
            {
                _buildTool.Perform(node);
            }
        }
    }
}
