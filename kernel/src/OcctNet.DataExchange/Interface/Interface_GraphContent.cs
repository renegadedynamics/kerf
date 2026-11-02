// Port of: src/DataExchange/TKXSBase/Interface/Interface_GraphContent.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_GraphContent.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Defines general form for classes of graph algorithms on Interfaces, this form is that of EntityIterator
/// Each sub-class fills it according to its own algorithm This also allows to combine any graph result to
/// others, all being given under one unique form
/// </summary>
public class Interface_GraphContent : Interface_EntityIterator
{

    /// <summary>
    /// Creates an empty GraphContent, ready to be filled
    /// </summary>
    public Interface_GraphContent()
    {
    }

    /// <summary>
    /// Creates with all entities designated by a Graph
    /// </summary>
    public Interface_GraphContent(Interface_Graph agraph)
    {
        GetFromGraph(agraph);
    }

    /// <summary>
    /// Creates with entities having specific Status value in a Graph
    /// </summary>
    public Interface_GraphContent(Interface_Graph agraph, int stat)
    {
        GetFromGraph(agraph, stat);
    }

    /// <summary>
    /// Creates an Iterator with Shared entities of an entity (equivalente to EntityIterator but with a Graph)
    /// </summary>
    public Interface_GraphContent(Interface_Graph agraph, Standard_Transient? ent)
    {
        Interface_EntityIterator list = agraph.Shareds(ent);
        int nb = list.NbEntities();
        if (nb == 0)
        {
            return; // Liste redefinie a VIDE
        }
        for (; list.More(); list.Next())
        {
            Standard_Transient curent = list.Value();
            if (agraph.IsPresent(agraph.EntityNumber(curent)))
            {
                GetOneItem(curent);
            }
        }
    }

    /// <summary>
    /// Gets all Entities designated by a Graph (once created), adds them to those already recorded
    /// </summary>
    public void GetFromGraph(Interface_Graph agraph)
    {
        int nb = agraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (agraph.IsPresent(i))
            {
                GetOneItem(agraph.Entity(i));
            }
        }
    }

    /// <summary>
    /// Gets entities from a graph which have a specific Status value (one created), adds them to those already
    /// recorded
    /// </summary>
    public void GetFromGraph(Interface_Graph agraph, int stat)
    {
        int nb = agraph.Size();
        for (int i = 1; i <= nb; i++)
        {
            if (agraph.IsPresent(i) && agraph.Status(i) == stat)
            {
                GetOneItem(agraph.Entity(i));
            }
        }
    }

    /// <summary>
    /// Returns Result under the exact form of an EntityIterator : Can be used when EntityIterator itself is
    /// required (as a returned value for instance), without way for a sub-class
    /// </summary>
    public Interface_EntityIterator Result()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator(); // On transvase ...
        for (Begin(); More(); Next())
        {
            iter.GetOneItem(Value());
        }
        return iter;
    }

    /// <summary>
    /// Does the Evaluation before starting the iteration itself (in out)
    /// </summary>
    public void Begin()
    {
        Evaluate();
        base.Start();
    }

    /// <summary>
    /// Evaluates list of Entities to be iterated. Called by Start Default is set to doing nothing : intended to
    /// be redefined by each sub-class
    /// </summary>
    public virtual void Evaluate()
    {

    }
}
