// Port of: src/DataExchange/TKXSBase/Interface/Interface_ShareFlags.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ShareFlags.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class only says for each Entity of a Model, if it is Shared or not by one or more other(s) of this
/// Model It uses the General Service "Shared".
/// </summary>
public class ShareFlags
{

    /// <summary>
    /// Creates a ShareFlags from a Model and builds required data (flags) by calling the General Service Library
    /// given as argument &lt;lib&gt;
    /// </summary>
    public ShareFlags(InterfaceModel? amodel, GeneralLib lib)
    {
        _theflags = new BitMap(amodel.EntityCount);
        GTool gtool = null; // null
        _themodel = amodel;
        Evaluate(lib, gtool);
    }

    /// <summary>
    /// Same as above, but GeneralLib is detained by a GTool
    /// </summary>
    public ShareFlags(InterfaceModel? amodel, GTool? gtool)
    {
        _theflags = new BitMap(amodel.EntityCount);
        _themodel = amodel;
        Evaluate(gtool.Lib(), gtool);
    }

    /// <summary>
    /// Same as above, but GeneralLib is defined through a Protocol
    /// </summary>
    public ShareFlags(InterfaceModel? amodel, Protocol? protocol)
    {
        _theflags = new BitMap(amodel.EntityCount);
        GTool gtool = null; // null
        _themodel = amodel;
        Evaluate(new GeneralLib(protocol), gtool);
    }

    /// <summary>
    /// Same as above, but works with the GTool of the Model
    /// </summary>
    public ShareFlags(InterfaceModel? amodel)
    {
        _theflags = new BitMap(amodel.EntityCount);
        GTool gtool = _themodel.GTool();
        gtool.Reservate(amodel.EntityCount);
        _themodel = amodel;
        Evaluate(gtool.Lib(), gtool);
    }

    /// <summary>
    /// Creates a ShareFlags by querying information from a Graph (remark that Graph also has a method IsShared)
    /// </summary>
    public ShareFlags(EntityGraph agraph)
    {
        _theflags = new BitMap(agraph.Model().EntityCount);
        _themodel = agraph.Model();
        int nb = _themodel.EntityCount;
        if (nb == 0)
        {
            return;
        }
        _theroots = new NCollection_HSequence<Transient>();
        for (int i = 1; i <= nb; i++)
        {
            //    Result obtained from the Graph
            Transient ent = _themodel.Value(i);
            NCollection_HSequence<Transient> list =
                agraph.GetSharings(ent);

            if (list is not null && list.Length() > 0)
            {
                _theflags.SetTrue(i);
            }
            else
            {
                _theroots.Append(ent);
            }
        }
    }

    /// <summary>
    /// Returns the Model used for the evaluation
    /// </summary>
    public InterfaceModel? Model()
    {
        return _themodel;
    }

    /// <summary>
    /// Returns True if &lt;ent&gt; is Shared by one or more other Entity(ies) of the Model
    /// </summary>
    public bool IsShared(Transient? ent)
    {
        int num = _themodel.Number(ent);
        if (num == 0 || num > _themodel.EntityCount)
        {
            throw new DomainErrorException("Interface ShareFlags : IsShared");
        }
        return _theflags.Value(num);
    }

    /// <summary>
    /// Returns the Entities which are not Shared (see their flags)
    /// </summary>
    public EntityIterator RootEntities()
    {
        EntityIterator iter = new EntityIterator(_theroots);
        return iter;
    }

    /// <summary>
    /// Returns the count of root entities
    /// </summary>
    public int RootCount
    {
        get
        {
            return (_theroots is null ? 0 : _theroots.Length());
        }
    }

    /// <summary>
    /// Returns a root entity according its rank in the list of roots By default, it returns the first one
    /// </summary>
    public Transient? Root(int num = 1)
    {
        return _theroots.Value(num);
    }

    /// <summary>
    /// Computes flags at Creation time Normally, gtool suffices. But if a Graph is created from a GeneralLib
    /// directly, it cannot be used If &lt;gtool&gt; is defined, it has priority
    /// </summary>
    private void Evaluate(GeneralLib lib, GTool? gtool)
    {
        bool patool = gtool is null;
        int nb = _themodel.EntityCount;
        if (nb == 0)
        {
            return;
        }
        _theroots = new NCollection_HSequence<Transient>();
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= nb; i++)
        {

            //    WARNING: If Entity not loaded hence unreadable, switch to its
            //    equivalent "Content"
            Transient ent = _themodel.Value(i);
            if (_themodel.IsRedefinedContent(i))
            {
                ent = _themodel.ReportEntity(i).Content();
            }

            //    Result obtained via GeneralLib
            EntityIterator iter = new EntityIterator();
            GeneralModule module = null;
            int cn;
            if (patool)
            {
                if (lib.Select(ent, out module, out cn))
                {
                    module.FillShared(_themodel, cn, ent, iter);
                }
            }
            else
            {
                if (gtool.Select(ent, out module, out cn))
                {
                    module.FillShared(_themodel, cn, ent, iter);
                }
            }

            //    Entities shared by <ent>: need to mark each one as "Shared"
            for (iter.Start(); iter.More(); iter.Next())
            {
                int num = _themodel.Number(iter.Value());
                _theflags.SetTrue(num); // Et Voila
            }
        }
        for (i = 1; i <= nb; i++)
        {
            if (!_theflags.Value(i))
            {
                _theroots.Append(_themodel.Value(i));
            }
        }
    }

    private InterfaceModel? _themodel;
    private BitMap _theflags = new BitMap();
    private NCollection_HSequence<Transient>? _theroots;
}
