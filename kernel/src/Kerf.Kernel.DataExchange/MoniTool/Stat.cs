// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_Stat.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_Stat.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Monitoring;

/// <summary>
/// This class manages Statistics to be queried asynchronously. It is organized as a stack of counters,
/// identified by their levels, from one to ... . Each one has a total account of items to be counted, a
/// count of already passed items, plus a count of "current items". The counters of higher level play on
/// these current items. For instance, if a counter has been opened for 100 items, 40 already passed, 20
/// current, its own percent is 40, but there is the contribution of higher level counters, rated for 20 % of
/// this counter. Hence, a counter is opened, items are added. Also items can be add for sub-counter (of
/// higher level), they will be added definitively when the sub-counter will be closed. When the count has
/// ended, this counter is closed, the counter of lower level cumulates it and goes on. As follows : Way of
/// use : Open(nbitems); Add(..) : direct adding Add(..) AddSub (nsub) : for sub-counter Open (nbsubs) :
/// nbsubs for this sub-counter Add (..) Close : the sub-counter AddEnd() etc... Close : the starting counter
/// This means that a counter can be opened in a Stat, regardless to the already opened ones :: this will be
/// cumulated A Current Stat is available, but it is possible to have others
/// </summary>
internal class Stat
{

    /// <summary>
    /// Creates a Stat form. At start, one default phase is defined, with one default step. Then, it suffises to
    /// start with a count of items (and cycles if several) then record items, to have a queryable report.
    /// </summary>
    public Stat(string title = "")
    {
        _thetit = new TCollection_HAsciiString(title);
        _thelev = 0;
        _thetot = new NCollection_HArray1<int>(1, 20);
        _thetot.Init(0);
        _thedone = new NCollection_HArray1<int>(1, 20);
        _thetot.Init(0); // OCCT: initialises thetot again, not thedone
        _thecurr = new NCollection_HArray1<int>(1, 20);
        _thetot.Init(0);
    }

    /// <summary>
    /// used when starting
    /// </summary>
    public Stat(Stat other)
    {
        _thelev = 0; // OCCT: the copy has no title and no counters (null arrays): Open() on it fails

    }

    // Kerf: OCCT's static Current() (one process-wide statistic) is left out: process-wide state that changes,
    // with no caller (decision 1187).

    /// <summary>
    /// Opens a new counter with a starting count of items
    /// </summary>
    public int Open(int nb = 100)
    {
        _thelev++;
        _thetot.SetValue(_thelev, nb);
        _thedone.SetValue(_thelev, 0);
        _thecurr.SetValue(_thelev, 0);
        return _thelev;
    }

    /// <summary>
    /// Adds more items to be counted by Add... on current level
    /// </summary>
    public void OpenMore(int id, int nb)
    {
        if (id <= 0 || id > _thelev)
        {
            return;
        }
        _thetot.SetValue(id, _thetot.Value(id) + nb);
    }

    /// <summary>
    /// Directly adds items
    /// </summary>
    public void Add(int nb = 1)
    {
        _thedone.SetValue(_thelev, _thedone.Value(_thelev) + nb);
        _thecurr.SetValue(_thelev, 0);
    }

    /// <summary>
    /// Declares a count of items to be added later. If a sub-counter is opened, its percentage multiplies this
    /// sub-count to compute the percent of current level
    /// </summary>
    public void AddSub(int nb = 1)
    {
        _thecurr.SetValue(_thelev, nb);
    }

    /// <summary>
    /// Ends the AddSub and cumulates the sub-count to current level
    /// </summary>
    public void AddEnd()
    {
        _thedone.SetValue(_thelev, _thedone.Value(_thelev) + _thecurr.Value(_thelev));
        _thecurr.SetValue(_thelev, 0);
    }

    public void Close(int id)
    {
        if (id < _thelev)
        {
            Close(id + 1);
        }
        AddEnd();
        _thelev--;
    }

    public int Level()
    {
        return _thelev;
    }

    public double Percent(int fromlev = 0)
    {
        if (fromlev > _thelev)
        {
            return 0;
        }
        double r1, r2, r3;
        int tot = _thetot.Value(fromlev);
        int done = _thedone.Value(fromlev);
        if (done >= tot)
        {
            return 100.0;
        }
        if (fromlev == _thelev)
        {
            r1 = tot;
            r2 = done;
            return (r2 * 100) / r1;
        }
        int cur = _thecurr.Value(fromlev);
        r1 = tot;
        r2 = done;
        r3 = 0;
        if (cur > 0)
        {
            r3 = cur;
            r3 = cur / tot; // OCCT: an integer division (0 unless cur >= tot)
            r3 = r3 * Percent(fromlev + 1);
        }
        if (r1 == 0)
        {
            return 1;
        }
        return (r2 * 100) / r1 + r3;
    }

    private TCollection_HAsciiString? _thetit;
    private int _thelev;
    private NCollection_HArray1<int>? _thetot;
    private NCollection_HArray1<int>? _thedone;
    private NCollection_HArray1<int>? _thecurr;
}
