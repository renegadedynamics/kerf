// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GlobalTranslationConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GlobalTranslationConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GlobalTranslationConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>A constraint that the plate's deformation is the same translation at every given 2D point.</summary>
/// <remarks>A class: OCCT's copies share the linear constraint's arrays (see <see cref="LinearXyzConstraint"/>).</remarks>
public class GlobalTranslationConstraint
{
    private readonly LinearXyzConstraint _lxyzc;

    public GlobalTranslationConstraint(NCollection_Sequence<Xy> sOfXY)
    {
        _lxyzc = new LinearXyzConstraint(sOfXY.Length() - 1, sOfXY.Length());
        int i;
        for (i = 1; i <= sOfXY.Length(); i++)
        {
            _lxyzc.SetPPC(i, new PinpointConstraint(sOfXY.Value(i), new Xyz(0.0, 0.0, 0.0), 0, 0));
        }
        for (i = 1; i <= sOfXY.Length() - 1; i++)
        {
            _lxyzc.SetCoeff(i, 1, -1.0);
            for (int j = 2; j <= sOfXY.Length(); j++)
            {
                if (j == (i + 1))
                {
                    _lxyzc.SetCoeff(i, j, 1.0);
                }
                else
                {
                    _lxyzc.SetCoeff(i, j, 0.0);
                }
            }
        }
    }

    public LinearXyzConstraint LXYZC() => _lxyzc;
}
