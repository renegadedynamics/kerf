// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>General tools of Shape Healing: the coding of <see cref="ShapeExtend_Status"/> flags into bit masks.</summary>
public static class ShapeExtend
{
    /// <summary>Inits using of ShapeExtend: in OCCT, loads the messages output by Shape Healing's algorithms.</summary>
    /// <remarks>
    /// Message_MsgFile and Shape Healing's message resources (SHMessage) aren't ported yet (phase 7), so this does
    /// nothing; no ported class reads those messages.
    /// </remarks>
    public static void Init()
    {
        // TODO(phase 7): load SHMessage_SHAPE_us through Message_MsgFile, as OCCT does.
    }

    /// <summary>Encodes a status (enumeration) to a bit flag.</summary>
    public static int EncodeStatus(ShapeExtend_Status status)
    {
        switch (status)
        {
            case ShapeExtend_OK:
                return 0x0000;
            case ShapeExtend_DONE1:
                return 0x0001;
            case ShapeExtend_DONE2:
                return 0x0002;
            case ShapeExtend_DONE3:
                return 0x0004;
            case ShapeExtend_DONE4:
                return 0x0008;
            case ShapeExtend_DONE5:
                return 0x0010;
            case ShapeExtend_DONE6:
                return 0x0020;
            case ShapeExtend_DONE7:
                return 0x0040;
            case ShapeExtend_DONE8:
                return 0x0080;
            case ShapeExtend_DONE:
                return 0x00ff;
            case ShapeExtend_FAIL1:
                return 0x0100;
            case ShapeExtend_FAIL2:
                return 0x0200;
            case ShapeExtend_FAIL3:
                return 0x0400;
            case ShapeExtend_FAIL4:
                return 0x0800;
            case ShapeExtend_FAIL5:
                return 0x1000;
            case ShapeExtend_FAIL6:
                return 0x2000;
            case ShapeExtend_FAIL7:
                return 0x4000;
            case ShapeExtend_FAIL8:
                return 0x8000;
            case ShapeExtend_FAIL:
                return 0xff00;
        }

        return 0;
    }

    /// <summary>Tells if a bit flag contains the bit of an enumerated status.</summary>
    public static bool DecodeStatus(int flag, ShapeExtend_Status status)
    {
        if (status == ShapeExtend_OK)
        {
            return (flag == 0);
        }

        return ((flag & ShapeExtend.EncodeStatus(status)) != 0);
    }
}
