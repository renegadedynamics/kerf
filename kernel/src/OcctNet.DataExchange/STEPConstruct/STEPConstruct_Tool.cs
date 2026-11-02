// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Tool.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Tool.cxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Tool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Provides basic functionalities for tools which are intended for encoding/decoding specific STEP constructs.
/// </summary>
/// <remarks>It is initialized by WorkSession and allows easy access to its fields and internal data such as Model,
/// TransferReader and FinderProcess. A C++ value class, a C# class here (decision 1185).</remarks>
public class STEPConstruct_Tool
{
    /// <summary>Creates an empty tool</summary>
    public STEPConstruct_Tool()
    {
    }

    /// <summary>Creates a tool and loads it with worksession</summary>
    public STEPConstruct_Tool(XSControl_WorkSession WS)
    {
        SetWS(WS);
    }

    /// <summary>Returns currently loaded WorkSession</summary>
    public XSControl_WorkSession WS()
    {
        return myWS;
    }

    /// <summary>Returns current model (Null if not loaded)</summary>
    public Interface_InterfaceModel Model()
    {
        return myWS.Model();
    }

    /// <summary>Returns current graph (recomputing if necessary)</summary>
    public Interface_Graph Graph(bool recompute = false)
    {
        // Note: myWS->Graph() recomputes graph each time!
        return recompute ? myWS.Graph() : myHGraph.Graph();
    }

    /// <summary>Returns TransientProcess (reading; Null if not loaded)</summary>
    public Transfer_TransientProcess TransientProcess()
    {
        return myTransientProcess;
    }

    /// <summary>Returns FinderProcess (writing; Null if not loaded)</summary>
    public Transfer_FinderProcess FinderProcess()
    {
        return myFinderProcess;
    }

    /// <summary>
    /// Load worksession; returns True if succeeded Returns False if either FinderProcess of TransientProcess cannot be
    /// obtained or are Null
    /// </summary>
    protected bool SetWS(XSControl_WorkSession WS)
    {
        myWS = null;
        myTransientProcess = null;
        myFinderProcess = null;

        if (WS is null)
        {
            return false;
        }
        myWS = WS;
        myHGraph = myWS.HGraph();

        // collect data on reading process
        XSControl_TransferReader TR = WS.TransferReader();
        if (TR is not null)
        {
            myTransientProcess = TR.TransientProcess();
        }

        // collect data on writing process
        XSControl_TransferWriter TW = myWS.TransferWriter();
        if (TW is not null)
        {
            myFinderProcess = TW.FinderProcess();
        }

        return myTransientProcess is not null && myFinderProcess is not null;
    }

    private XSControl_WorkSession myWS;
    private Transfer_FinderProcess myFinderProcess;
    private Transfer_TransientProcess myTransientProcess;
    private Interface_HGraph myHGraph;
}
