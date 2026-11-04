// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Messages;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Extend;

/// <summary>General tools of Shape Healing: the coding of <see cref="ExtendStatus"/> flags into bit masks.</summary>
internal static class ShapeExtend
{
    // OCCT.NET: the messages go into the message table's defaults once per process (unit 10I)
    private static readonly Lazy<bool> Loaded = new(() =>
    {
        // OCCT.NET: OCCT's Message_MsgFile::LoadFromEnv("CSF_SHMessage", "SHAPE") left out (decision 975)
        MsgFile.AddDefaults(ShapeMessages.Text);
        if (!MsgFile.HasMsg("ShapeFix.FixSmallSolid.MSG0"))
        {
            throw new ProgramErrorException("Critical Error - message resources for ShapeExtend are invalid or undefined!");
        }
        return true;
    });

    /// <summary>
    /// Inits using of ShapeExtend: loads the messages output by Shape Healing's algorithms (SHMessage's texts) into
    /// the defaults of <see cref="MsgFile"/>, once per process.
    /// </summary>
    /// <remarks>
    /// OCCT first tries the message file that the environment variable CSF_SHMessage names; the port loads only the
    /// text compiled in, OCCT's SHAPE.us (decision 975), once (a one-time initialisation, unit 10I).
    /// </remarks>
    /// <exception cref="ProgramErrorException">The messages can't be loaded (never with the compiled-in text).</exception>
    public static void Init() => _ = Loaded.Value;

    /// <summary>Encodes a status (enumeration) to a bit flag.</summary>
    public static int EncodeStatus(ExtendStatus status)
    {
        switch (status)
        {
            case ExtendStatus.OK:
                return 0x0000;
            case ExtendStatus.DONE1:
                return 0x0001;
            case ExtendStatus.DONE2:
                return 0x0002;
            case ExtendStatus.DONE3:
                return 0x0004;
            case ExtendStatus.DONE4:
                return 0x0008;
            case ExtendStatus.DONE5:
                return 0x0010;
            case ExtendStatus.DONE6:
                return 0x0020;
            case ExtendStatus.DONE7:
                return 0x0040;
            case ExtendStatus.DONE8:
                return 0x0080;
            case ExtendStatus.Done:
                return 0x00ff;
            case ExtendStatus.FAIL1:
                return 0x0100;
            case ExtendStatus.FAIL2:
                return 0x0200;
            case ExtendStatus.FAIL3:
                return 0x0400;
            case ExtendStatus.FAIL4:
                return 0x0800;
            case ExtendStatus.FAIL5:
                return 0x1000;
            case ExtendStatus.FAIL6:
                return 0x2000;
            case ExtendStatus.FAIL7:
                return 0x4000;
            case ExtendStatus.FAIL8:
                return 0x8000;
            case ExtendStatus.Fail:
                return 0xff00;
        }

        return 0;
    }

    /// <summary>Tells if a bit flag contains the bit of an enumerated status.</summary>
    public static bool DecodeStatus(int flag, ExtendStatus status)
    {
        if (status == ExtendStatus.OK)
        {
            return (flag == 0);
        }

        return ((flag & global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(status)) != 0);
    }
}
