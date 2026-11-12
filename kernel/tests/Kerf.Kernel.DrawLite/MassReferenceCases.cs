// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// Cases whose script holds an area or a volume from OCCT's precise mass properties, which never refine along U (OCCT #1675,
/// decision 1784). Kerf's figures are right: each area here is within about 2e-3 of a fine mesh's, where the scripts' are up to
/// 22% off. Each case is pinned to Kerf's figure, so a change to its shape still fails.
/// </summary>
internal static class MassReferenceCases
{
    public static readonly IReadOnlyDictionary<string, string[]> Figures = new Dictionary<string, string[]>(StringComparer.Ordinal)
    {
        // B4, F1 and F4 at Kerf's figures since its evolving fillets are the envelopes of their balls (unit 15AK: 11734.1, which
        // is OCCT's, 76962 and 49499.6 before, on the balls' great circles, decision 2217; 76963.4 and 49513.7 before that)
        ["blend/buildevol/B4"] = ["The area of result shape is 11859.9"],
        ["blend/buildevol/F1"] = ["The area of result shape is 76963.2"],
        ["blend/buildevol/F4"] = ["The area of result shape is 49558.1"],
        // J6 and J8 at Kerf's figures since its fillets are on the rolling ball (decision 2157: 77933.5 and 51088.7 before)
        ["blend/simple/J6"] = ["The area of result shape is 77933.3"],
        ["blend/simple/J8"] = ["The area of result shape is 51087.3"],
        ["boolean/removefeatures/C6"] = ["The area of result shape is 441103", "The volume of result shape is 8.18198e+06"],
        ["boolean/removefeatures/C7"] = ["The area of result shape is 439514", "The volume of result shape is 8.11394e+06"],
        ["boolean/removefeatures/C8"] = ["The area of result shape is 441341", "The volume of result shape is 8.18281e+06"],
        ["boolean/removefeatures/C9"] = ["The area of result shape is 422115", "The volume of result shape is 8.34107e+06"],
        ["boolean/removefeatures/D1"] = ["The area of result shape is 415107", "The volume of result shape is 8.28187e+06"],
        ["boolean/removefeatures/D2"] = ["The area of result shape is 440694", "The volume of result shape is 8.1858e+06"],
        ["boolean/removefeatures/D4"] = ["The area of result shape is 439077", "The volume of result shape is 8.2738e+06"],
        ["boolean/removefeatures/D5"] = ["The area of result shape is 410702", "The volume of result shape is 8.98857e+06"],
        ["boolean/removefeatures/D6"] = ["The area of result shape is 427465", "The volume of result shape is 8.04839e+06"],
        ["boolean/removefeatures/G2"] = ["The area of result shape is 501834", "The volume of result shape is 8.6075e+06"],
        ["bugs/modalg_2/bug5805_10"] = ["The area of result shape is 1041.34"],
        ["bugs/modalg_2/bug5805_11"] = ["The area of result shape is 1041.34"],
        ["bugs/modalg_2/bug5805_12"] = ["The area of result shape is 982.243"],
        ["bugs/modalg_2/bug5805_2"] = ["The area of result shape is 982.243"],
        ["bugs/modalg_2/bug5805_21"] = ["The area of result shape is 244.195"],
        ["bugs/modalg_2/bug5805_22"] = ["The area of result shape is 492.69"],
        ["bugs/modalg_2/bug5805_23"] = ["The area of result shape is 492.69"],
        ["bugs/modalg_2/bug5805_24"] = ["The area of result shape is 479.705"],
        ["bugs/modalg_2/bug5805_33"] = ["The area of result shape is 418.365"],
        ["bugs/modalg_2/bug5805_34"] = ["The area of result shape is 881.52"],
        ["bugs/modalg_2/bug5805_35"] = ["The area of result shape is 915.397"],
        ["bugs/modalg_2/bug5805_36"] = ["The area of result shape is 856.848"],
        ["bugs/modalg_2/bug5805_45"] = ["The area of result shape is 744.187"],
        ["bugs/modalg_2/bug5805_46"] = ["The area of result shape is 1461.96"],
        ["bugs/modalg_2/bug5805_47"] = ["The area of result shape is 1507.77"],
        ["bugs/modalg_2/bug5805_48"] = ["The area of result shape is 1448.94"],
        ["bugs/modalg_3/bug5805_9"] = ["The area of result shape is 495.651"],
        ["bugs/modalg_5/bug23706_2"] = ["The area of result shape is 241.61"],
        ["bugs/modalg_7/bug28828_21"] = ["The volume of result shape is 446299"],
        ["bugs/modalg_7/bug28828_6"] = ["The volume of result shape is 73604.4"],
        ["bugs/modalg_7/bug31845_2"] = ["The area of result shape is 213.285"],
        ["feat/featprism/M1"] = ["The area of result shape is 1147.87"],
        ["offset/faces_type_i/F6"] = ["The volume of result shape is 911.97"],
        ["offset/faces_type_i/F8"] = ["The volume of result shape is 798.638"],
        ["offset/faces_type_i/L3"] = ["The volume of result shape is 330.72"],
        ["offset/faces_type_i/L4"] = ["The volume of result shape is 252.144"],
        ["offset/faces_type_i/L5"] = ["The volume of result shape is 347.677"],
        ["pipe/standard/B5"] = ["The area of result shape is 4069.1"],
        ["pipe/standard/E6"] = ["The area of result shape is 4698.81"],
        ["pipe/standard/F4"] = ["The area of result shape is 4583.21"],
    };

    /// <summary>Whether the only errors in the log are the pinned figures of the case.</summary>
    public static bool OnlyThePinnedFiguresDiffer(string theId, string theLog)
    {
        string[] aErrors = theLog.Split('\n').Where(static l => l.StartsWith("Error", StringComparison.Ordinal)).ToArray();
        string[] aFigures = Figures[theId];
        return aErrors.Length == aFigures.Length
               && aErrors.Zip(aFigures).All(static p => p.First.StartsWith("Error : " + p.Second + ",", StringComparison.Ordinal));
    }
}
