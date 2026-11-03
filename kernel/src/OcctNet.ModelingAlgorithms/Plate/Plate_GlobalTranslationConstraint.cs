// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GlobalTranslationConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GlobalTranslationConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_GlobalTranslationConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>A constraint that the plate's deformation is the same translation at every given 2D point.</summary>
/// <remarks>A class: OCCT's copies share the linear constraint's arrays (see <see cref="Plate_LinearXYZConstraint"/>).</remarks>
public class Plate_GlobalTranslationConstraint
{
    private readonly Plate_LinearXYZConstraint myLXYZC;

    public Plate_GlobalTranslationConstraint(NCollection_Sequence<Xy> SOfXY)
    {
        myLXYZC = new Plate_LinearXYZConstraint(SOfXY.Length() - 1, SOfXY.Length());
        int i;
        for (i = 1; i <= SOfXY.Length(); i++)
        {
            myLXYZC.SetPPC(i, new Plate_PinpointConstraint(SOfXY.Value(i), new Xyz(0.0, 0.0, 0.0), 0, 0));
        }
        for (i = 1; i <= SOfXY.Length() - 1; i++)
        {
            myLXYZC.SetCoeff(i, 1, -1.0);
            for (int j = 2; j <= SOfXY.Length(); j++)
            {
                if (j == (i + 1))
                {
                    myLXYZC.SetCoeff(i, j, 1.0);
                }
                else
                {
                    myLXYZC.SetCoeff(i, j, 0.0);
                }
            }
        }
    }

    public Plate_LinearXYZConstraint LXYZC() => myLXYZC;
}
