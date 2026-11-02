// Port of: src/DataExchange/TKXSBase/Interface/Interface_ShareFlags.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ShareFlags.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class only says for each Entity of a Model, if it is Shared or not by one or more other(s) of this
/// Model It uses the General Service "Shared".
/// </summary>
public class Interface_ShareFlags
{

    /// <summary>
    /// Creates a ShareFlags from a Model and builds required data (flags) by calling the General Service Library
    /// given as argument &lt;lib&gt;
    /// </summary>
    public Interface_ShareFlags(Interface_InterfaceModel? amodel, Interface_GeneralLib lib)
    {
        theflags = new Interface_BitMap(amodel.NbEntities());
        Interface_GTool gtool = null; // null
        themodel = amodel;
        Evaluate(lib, gtool);
    }

    /// <summary>
    /// Same as above, but GeneralLib is detained by a GTool
    /// </summary>
    public Interface_ShareFlags(Interface_InterfaceModel? amodel, Interface_GTool? gtool)
    {
        theflags = new Interface_BitMap(amodel.NbEntities());
        themodel = amodel;
        Evaluate(gtool.Lib(), gtool);
    }

    /// <summary>
    /// Same as above, but GeneralLib is defined through a Protocol
    /// </summary>
    public Interface_ShareFlags(Interface_InterfaceModel? amodel, Interface_Protocol? protocol)
    {
        theflags = new Interface_BitMap(amodel.NbEntities());
        Interface_GTool gtool = null; // null
        themodel = amodel;
        Evaluate(new Interface_GeneralLib(protocol), gtool);
    }

    /// <summary>
    /// Same as above, but works with the GTool of the Model
    /// </summary>
    public Interface_ShareFlags(Interface_InterfaceModel? amodel)
    {
        theflags = new Interface_BitMap(amodel.NbEntities());
        Interface_GTool gtool = themodel.GTool();
        gtool.Reservate(amodel.NbEntities());
        themodel = amodel;
        Evaluate(gtool.Lib(), gtool);
    }

    /// <summary>
    /// Creates a ShareFlags by querying information from a Graph (remark that Graph also has a method IsShared)
    /// </summary>
    public Interface_ShareFlags(Interface_Graph agraph)
    {
        theflags = new Interface_BitMap(agraph.Model().NbEntities());
        themodel = agraph.Model();
        int nb = themodel.NbEntities();
        if (nb == 0)
        {
            return;
        }
        theroots = new NCollection_HSequence<Standard_Transient>();
        for (int i = 1; i <= nb; i++)
        {
            //    Result obtained from the Graph
            Standard_Transient ent = themodel.Value(i);
            NCollection_HSequence<Standard_Transient> list =
                agraph.GetSharings(ent);

            if (list is not null && list.Length() > 0)
            {
                theflags.SetTrue(i);
            }
            else
            {
                theroots.Append(ent);
            }
        }
    }

    /// <summary>
    /// Returns the Model used for the evaluation
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return themodel;
    }

    /// <summary>
    /// Returns True if &lt;ent&gt; is Shared by one or more other Entity(ies) of the Model
    /// </summary>
    public bool IsShared(Standard_Transient? ent)
    {
        int num = themodel.Number(ent);
        if (num == 0 || num > themodel.NbEntities())
        {
            throw new Standard_DomainError("Interface ShareFlags : IsShared");
        }
        return theflags.Value(num);
    }

    /// <summary>
    /// Returns the Entities which are not Shared (see their flags)
    /// </summary>
    public Interface_EntityIterator RootEntities()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator(theroots);
        return iter;
    }

    /// <summary>
    /// Returns the count of root entities
    /// </summary>
    public int NbRoots()
    {
        return (theroots is null ? 0 : theroots.Length());
    }

    /// <summary>
    /// Returns a root entity according its rank in the list of roots By default, it returns the first one
    /// </summary>
    public Standard_Transient? Root(int num = 1)
    {
        return theroots.Value(num);
    }

    /// <summary>
    /// Computes flags at Creation time Normally, gtool suffices. But if a Graph is created from a GeneralLib
    /// directly, it cannot be used If &lt;gtool&gt; is defined, it has priority
    /// </summary>
    private void Evaluate(Interface_GeneralLib lib, Interface_GTool? gtool)
    {
        bool patool = gtool is null;
        int nb = themodel.NbEntities();
        if (nb == 0)
        {
            return;
        }
        theroots = new NCollection_HSequence<Standard_Transient>();
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= nb; i++)
        {

            //    WARNING: If Entity not loaded hence unreadable, switch to its
            //    equivalent "Content"
            Standard_Transient ent = themodel.Value(i);
            if (themodel.IsRedefinedContent(i))
            {
                ent = themodel.ReportEntity(i).Content();
            }

            //    Result obtained via GeneralLib
            Interface_EntityIterator iter = new Interface_EntityIterator();
            Interface_GeneralModule module = null;
            int CN;
            if (patool)
            {
                if (lib.Select(ent, out module, out CN))
                {
                    module.FillShared(themodel, CN, ent, iter);
                }
            }
            else
            {
                if (gtool.Select(ent, out module, out CN))
                {
                    module.FillShared(themodel, CN, ent, iter);
                }
            }

            //    Entities shared by <ent>: need to mark each one as "Shared"
            for (iter.Start(); iter.More(); iter.Next())
            {
                int num = themodel.Number(iter.Value());
                theflags.SetTrue(num); // Et Voila
            }
        }
        for (i = 1; i <= nb; i++)
        {
            if (!theflags.Value(i))
            {
                theroots.Append(themodel.Value(i));
            }
        }
    }

    private Interface_InterfaceModel? themodel;
    private Interface_BitMap theflags = new Interface_BitMap();
    private NCollection_HSequence<Standard_Transient>? theroots;
}
