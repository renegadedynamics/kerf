// Port of: src/FoundationClasses/TKernel/Message/Message_CompositeAlerts.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_CompositeAlerts.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Foundation;

/// <summary>A container of alerts, one list per gravity.</summary>
/// <remarks>
/// OCCT's <c>Standard_ASSERT_RETURN</c> checks are kept (they are compiled in release builds too): a null alert or a
/// gravity out of range gives an empty list or false. <c>DumpJson</c> is left out.
/// </remarks>
internal class CompositeAlerts : Transient
{
    // store messages in a lists sorted by gravity;
    // here we rely on knowledge that Message_Fail is the last element of the enum
    protected readonly NCollection_List<Alert>[] _alerts = NewLists(); //!< container of child alert for each type of gravity

    private static NCollection_List<Alert>[] NewLists()
    {
        var lists = new NCollection_List<Alert>[(int)Fail + 1];
        for (int i = 0; i < lists.Length; ++i)
        {
            lists[i] = new NCollection_List<Alert>();
        }
        return lists;
    }

    private bool IsValid(Gravity gravity) => gravity >= 0 && (int)gravity < _alerts.Length;

    /// <summary>
    /// The alerts of the gravity: the container's own list, which callers only read (a new empty list for a gravity
    /// out of range).
    /// </summary>
    public NCollection_List<Alert> Alerts(Gravity gravity)
    {
        if (!IsValid(gravity))
        {
            // Standard_ASSERT_RETURN: "Requesting alerts for gravity not in valid range"
            return new NCollection_List<Alert>();
        }
        return _alerts[(int)gravity];
    }

    /// <summary>
    /// Adds the alert with the gravity, or merges it into an alert of exactly the same type if it supports merging.
    /// True if the alert was appended, false if it was merged (or is invalid).
    /// </summary>
    public bool AddAlert(Gravity gravity, Alert? alert)
    {
        if (alert is null || !IsValid(gravity))
        {
            // Standard_ASSERT_RETURN: "Attempt to add null alert", "Adding alert with gravity not in valid range"
            return false;
        }

        NCollection_List<Alert> list = _alerts[(int)gravity];
        if (alert.SupportsMerge() && !list.IsEmpty())
        {
            // merge is performed only for alerts of exactly same type
            Type type = alert.DynamicType();
            for (NCollection_List<Alert>.Iterator it = new(list); it.More(); it.Next())
            {
                // if merged successfully, just return
                if (type == it.Value().DynamicType() && alert.Merge(it.Value()))
                {
                    return false;
                }
            }
        }

        // if not merged, just add to the list
        list.Append(alert);
        return true;
    }

    /// <summary>Removes the alert from the gravity's list; true if it was there.</summary>
    public bool RemoveAlert(Gravity gravity, Alert? alert)
    {
        if (alert is null || !IsValid(gravity))
        {
            return false;
        }
        NCollection_List<Alert> alerts = _alerts[(int)gravity];
        if (!alerts.Contains(alert))
        {
            return false;
        }
        return alerts.Remove(alert);
    }

    /// <summary>True if the alert is in one of the lists.</summary>
    public bool HasAlert(Alert alert)
    {
        for (int gravIter = (int)Trace; gravIter <= (int)Fail; ++gravIter)
        {
            NCollection_List<Alert> alerts = Alerts((Gravity)gravIter);
            if (alerts.Contains(alert))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>True if an alert of exactly the type (<c>IsInstance</c>: not a derived type) has the gravity.</summary>
    public bool HasAlert(Type type, Gravity gravity)
    {
        if (!IsValid(gravity))
        {
            return false;
        }
        for (NCollection_List<Alert>.Iterator it = new(_alerts[(int)gravity]); it.More(); it.Next())
        {
            if (it.Value().IsInstance(type))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>Removes all the alerts.</summary>
    public void Clear()
    {
        for (int i = 0; i < _alerts.Length; ++i)
        {
            _alerts[i].Clear();
        }
    }

    /// <summary>Removes the alerts of the gravity.</summary>
    public void Clear(Gravity gravity)
    {
        if (!IsValid(gravity))
        {
            return;
        }
        _alerts[(int)gravity].Clear();
    }

    /// <summary>Removes the alerts of exactly the type (and null ones).</summary>
    public void Clear(Type type)
    {
        for (int i = 0; i < _alerts.Length; ++i)
        {
            for (NCollection_List<Alert>.Iterator it = new(_alerts[i]); it.More();)
            {
                if (it.Value() is null || it.Value().IsInstance(type))
                {
                    _alerts[i].Remove(it);
                }
                else
                {
                    // OCCT calls anIt.More() here, not anIt.Next(): an alert of another type makes the loop run
                    // forever. The port advances (occt-issues.md, unit 6A).
                    it.Next();
                }
            }
        }
    }
}
