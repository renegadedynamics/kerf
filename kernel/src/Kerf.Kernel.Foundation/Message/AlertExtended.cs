// Port of: src/FoundationClasses/TKernel/Message/Message_AlertExtended.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_AlertExtended.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Foundation;

/// <summary>
/// An alert with an attribute (its name and other data) and, optionally, child alerts
/// (<see cref="CompositeAlerts"/>): the alerts of a report's levels (<see cref="Level"/>).
/// </summary>
internal class AlertExtended : Alert
{
    protected CompositeAlerts? _compositAlerts; //!< class provided hierarchical structure of alerts
    protected MessageAttribute? _attribute;           //!< container of the alert attributes

    /// <summary>
    /// Makes an extended alert with the attribute and adds it to the report with the gravity; returns the alert.
    /// </summary>
    public static Alert AddAlert(Report report, MessageAttribute? attribute, Gravity gravity)
    {
        AlertExtended alert = new();
        alert.SetAttribute(attribute);
        report.AddAlert(gravity, alert);
        return alert;
    }

    /// <summary>The key of the alert's message: its attribute's key, or the class name without an attribute.</summary>
    public override string GetMessageKey()
    {
        if (_attribute is null)
        {
            return base.GetMessageKey();
        }
        return _attribute.GetMessageKey();
    }

    /// <summary>The attribute of the alert.</summary>
    public MessageAttribute? Attribute() => _attribute;

    /// <summary>Sets the attribute of the alert.</summary>
    public void SetAttribute(MessageAttribute? attribute) => _attribute = attribute;

    /// <summary>The child alerts, made first if <paramref name="toCreate"/> is true; null if there are none.</summary>
    public CompositeAlerts? CompositeAlerts(bool toCreate = false)
    {
        if (_compositAlerts is null && toCreate)
        {
            _compositAlerts = new CompositeAlerts();
        }
        return _compositAlerts;
    }

    /// <summary>True unless the alert has child alerts: hierarchical alerts can't be merged.</summary>
    public override bool SupportsMerge()
    {
        if (_compositAlerts is null)
        {
            return true;
        }

        // hierarchical alerts can not be merged
        for (int gravIter = (int)Trace; gravIter <= (int)Fail; ++gravIter)
        {
            if (!_compositAlerts.Alerts((Gravity)gravIter).IsEmpty())
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>Never merges (false).</summary>
    public override bool Merge(Alert target)
    {
        // by default, merge trivially
        return false;
    }
}
