// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Tool.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Tool.cxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Tool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;

using Kerf.Engine.Exchange.Framework.Transfer;

using Occt;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// Provides basic functionalities for tools which are intended for encoding/decoding specific STEP constructs.
/// </summary>
/// <remarks>It is initialized by WorkSession and allows easy access to its fields and internal data such as Model,
/// TransferReader and FinderProcess. A C++ value class, a C# class here (decision 1185).</remarks>
internal class ConstructTool
{
    /// <summary>Creates an empty tool</summary>
    public ConstructTool()
    {
    }

    /// <summary>Creates a tool and loads it with worksession</summary>
    public ConstructTool(ExchangeWorkSession ws)
    {
        SetWS(ws);
    }

    /// <summary>Returns currently loaded WorkSession</summary>
    public ExchangeWorkSession WS()
    {
        return _ws;
    }

    /// <summary>Returns current model (Null if not loaded)</summary>
    public InterfaceModel Model()
    {
        return _ws.Model();
    }

    /// <summary>Returns current graph (recomputing if necessary)</summary>
    public EntityGraph Graph(bool recompute = false)
    {
        // Note: myWS->Graph() recomputes graph each time!
        return recompute ? _ws.Graph() : _hGraph.Graph();
    }

    /// <summary>Returns TransientProcess (reading; Null if not loaded)</summary>
    public TransientProcess TransientProcess()
    {
        return _transientProcess;
    }

    /// <summary>Returns FinderProcess (writing; Null if not loaded)</summary>
    public FinderProcess FinderProcess()
    {
        return _finderProcess;
    }

    /// <summary>
    /// Load worksession; returns True if succeeded Returns False if either FinderProcess of TransientProcess cannot be
    /// obtained or are Null
    /// </summary>
    protected bool SetWS(ExchangeWorkSession ws)
    {
        _ws = null;
        _transientProcess = null;
        _finderProcess = null;

        if (ws is null)
        {
            return false;
        }
        _ws = ws;
        _hGraph = _ws.HGraph();

        // collect data on reading process
        TransferReader tr = ws.TransferReader();
        if (tr is not null)
        {
            _transientProcess = tr.TransientProcess();
        }

        // collect data on writing process
        TransferWriter tw = _ws.TransferWriter();
        if (tw is not null)
        {
            _finderProcess = tw.FinderProcess();
        }

        return _transientProcess is not null && _finderProcess is not null;
    }

    private ExchangeWorkSession _ws;
    private FinderProcess _finderProcess;
    private TransientProcess _transientProcess;
    private HGraph _hGraph;
}
