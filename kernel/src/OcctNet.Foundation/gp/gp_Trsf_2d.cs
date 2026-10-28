// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

// The constructor of gp_Trsf from a gp_Trsf2d; the rest of gp_Trsf is in gp_Trsf.cs.
public partial struct gp_Trsf
{
    /// <summary>The 3D transformation that applies <paramref name="T"/> in the XY plane and leaves Z unchanged.</summary>
    public gp_Trsf(in gp_Trsf2d T)
    {
        scale = T.ScaleFactor();
        shape = T.Form();
        matrix = new gp_Mat();
        loc = new gp_XYZ(T.TranslationPart().X(), T.TranslationPart().Y(), 0.0);
        gp_Mat2d M = T.HVectorialPart();
        matrix.ChangeValue(1, 1) = M[1, 1];
        matrix.ChangeValue(1, 2) = M[1, 2];
        matrix.ChangeValue(2, 1) = M[2, 1];
        matrix.ChangeValue(2, 2) = M[2, 2];
        matrix.ChangeValue(3, 3) = 1.0;
        if (shape == gp_Ax1Mirror)
        {
            scale = 1;
            matrix.Multiply(-1);
        }
    }
}
