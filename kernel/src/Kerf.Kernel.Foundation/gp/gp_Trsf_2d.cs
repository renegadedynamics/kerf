// Port of: src/FoundationClasses/TKMath/gp/gp_Trsf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Geometry;

// The constructor of gp_Trsf from a gp_Trsf2d; the rest of gp_Trsf is in gp_Trsf.cs.
public readonly partial struct Transform3D
{
    /// <summary>The 3D transformation that applies <paramref name="transform"/> in the XY plane and leaves Z unchanged.</summary>
    public Transform3D(in Transform2D transform)
    {
        _scale = transform.ScaleFactor;
        _shape = transform.Form;
        _loc = new Xyz(transform.TranslationXy().X(), transform.TranslationXy().Y(), 0.0);
        Matrix2x2 m = transform.RotationPart;
        _matrix = new Matrix3x3(m[1, 1], m[1, 2], 0.0,
                                m[2, 1], m[2, 2], 0.0,
                                0.0, 0.0, 1.0);
        if (_shape == AxisMirror)
        {
            _scale = 1;
            _matrix = _matrix.Multiplied(-1);
        }
    }
}
