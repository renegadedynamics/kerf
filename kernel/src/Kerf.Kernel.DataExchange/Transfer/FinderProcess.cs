// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_FinderProcess.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_FinderProcess.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.Transfer.StatusExec;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>Adds specific features to the generic definition : PrintTrace is adapted.</summary>
internal class FinderProcess : ProcessForFinder
{
    /// <summary>Sets FinderProcess at initial state, with an initial size.</summary>
    public FinderProcess(int nb = 10000)
        : base(nb)
    {
    }

    /// <summary>
    /// Sets an InterfaceModel, which can be used during transfer for instance if a context must be managed, it is in
    /// the Model.
    /// </summary>
    public void SetModel(InterfaceModel? model)
    {
        _themodel = model;
    }

    /// <summary>Returns the Model which can be used for context.</summary>
    public InterfaceModel? Model()
    {
        return _themodel;
    }

    /// <summary>
    /// In the list of mapped items (between 1 and NbMapped), searches for the first mapped item which follows
    /// <paramref name="num0"/> (not included) and which has an attribute named <paramref name="name"/>. The considered
    /// Attributes are those brought by Finders, i.e. by Input data.
    /// </summary>
    public int NextMappedWithAttribute(string name, int num0)
    {
        int num, nb = MappedCount;
        for (num = num0 + 1; num <= nb; num++)
        {
            Finder? fnd = Mapped(num);
            if (fnd is null)
            {
                continue;
            }
            if (fnd.Attribute(name) is not null)
            {
                return num;
            }
        }
        return 0;
    }

    /// <summary>
    /// Returns a TransientMapper for a given Transient Object. Either <paramref name="obj"/> is already mapped, then
    /// its Mapper is returned. Or it is not, then a new one is created then returned, BUT it is not mapped here (use
    /// Bind or FindElseBind to do this).
    /// </summary>
    public TransientMapper TransientMapper(Transient? obj)
    {
        var mapper = new TransientMapper(obj);
        int index = MapIndex(mapper);
        if (index == 0)
        {
            return mapper;
        }
        return Mapped(index) as TransientMapper;
    }

    /// <summary>Specific printing to trace a Finder (by its method ValueType).</summary>
    public override void PrintTrace(Finder? start, TextWriter s)
    {
        if (start is not null)
        {
            s.Write(" Type:");
            s.Write(start.ValueTypeName());
        }
    }

    /// <summary>Prints statistics on a given output, according mode.</summary>
    public void PrintStats(int mode, TextWriter s)
    {
        s.Write("\n*******************************************************************\n");
        if (mode == 1)
        { //  Basic statistics
            s.Write("********                 Basic Statistics                  ********\n");

            int nbr = 0, nbe = 0, nbw = 0;
            int i, max = MappedCount, nbroots = RootCount;
            s.Write("****        Nb Final Results    : " + nbroots + "\n");

            for (i = 1; i <= max; i++)
            {
                Binder? binder = MapItem(i);
                if (binder is null)
                {
                    continue;
                }
                FrameworkCheck ach = binder.Check();
                StatusExec stat = binder.StatusExec();
                if (stat != StatusExec.StatusInitial && stat != StatusExec.StatusDone)
                {
                    nbe++;
                }
                else
                {
                    if (ach.WarningCount > 0)
                    {
                        nbw++;
                    }
                    if (binder.HasResult)
                    {
                        nbr++;
                    }
                }
            }
            if (nbr > nbroots)
            {
                s.Write("****      ( Itermediate Results : " + (nbr - nbroots) + " )\n");
            }
            if (nbe > 0)
            {
                s.Write("****                  Errors on :" + FrameworkMsg.Blanks(nbe, 4) + nbe + " Entities\n");
            }
            if (nbw > 0)
            {
                s.Write("****                Warnings on : " + FrameworkMsg.Blanks(nbw, 4) + nbw + " Entities\n");
            }
            s.Write("*******************************************************************");
        }
        s.Write('\n');
    }

    private InterfaceModel? _themodel;
}
