// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#if KERF_0_5_OR_LATER
using System.Collections;
using System.Reflection;
using Kerf.Modeling;
using Renegade.ShaftGauge;

namespace Kerf.UserCases;

/// <summary>The shaft gauges (shaft-gauge/README.md), every size built again.</summary>
internal static class ShaftGaugeModel
{
    /// <summary>Each size's blank and gauge, built again: the model's caches emptied first, so nothing is kept from a last run.</summary>
    public static object Build()
    {
        Forget();
        var shapes = new List<(string, Part)>();
        foreach (var size in Gauge.Nominals)
        {
            Gauge.Use(size);
            shapes.Add(($"blank {size}", Gauge.Blank()));
            shapes.Add(($"gauge {size}", Gauge.Make()));
        }
        return shapes;
    }

    // The model keeps each size's blank and gauge in private static dictionaries, blanks and gauges, as the script builds
    // each once; emptied here, as the model can't be changed to say so
    private static void Forget()
    {
        foreach (var name in new[] { "blanks", "gauges" })
        {
            var field = typeof(Gauge).GetField(name, BindingFlags.Static | BindingFlags.NonPublic)
                ?? throw new InvalidOperationException($"the shaft gauge model has no cache called {name}");
            ((IDictionary)field.GetValue(null)!).Clear();
        }
    }
}
#endif
