// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>General tools of Shape Healing: the coding of <see cref="ShapeExtend_Status"/> flags into bit masks.</summary>
public static class ShapeExtend
{
    private static bool init = false;
    private static readonly Lock initLock = new();

    /// <summary>
    /// Inits using of ShapeExtend: loads the messages output by Shape Healing's algorithms (SHMessage's texts) into
    /// <see cref="Message_MsgFile"/>, once per process, unless they are there already.
    /// </summary>
    /// <remarks>
    /// OCCT first tries the message file that the environment variable CSF_SHMessage names; the port loads only the
    /// text compiled in, OCCT's SHAPE.us (decision 8103). Process-wide state, as OCCT's, behind a lock.
    /// </remarks>
    /// <exception cref="Standard_ProgramError">The messages can't be loaded (never with the compiled-in text).</exception>
    public static void Init()
    {
        lock (initLock)
        {
            if (init)
            {
                return;
            }

            init = true;

            // load Message File for Shape Healing
            if (!Message_MsgFile.HasMsg("ShapeFix.FixSmallSolid.MSG0"))
            {
                // OCCT.NET: OCCT's Message_MsgFile::LoadFromEnv("CSF_SHMessage", "SHAPE") left out (decision 8103)
                Message_MsgFile.LoadFromString(SHMessage_SHAPE_us.Text);
                if (!Message_MsgFile.HasMsg("ShapeFix.FixSmallSolid.MSG0"))
                {
                    throw new Standard_ProgramError(
                        "Critical Error - message resources for ShapeExtend are invalid or undefined!");
                }
            }
        }
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
