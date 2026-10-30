// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// SKELETON (5G): the public API, filled by the GeomInt fork.
public class GeomInt_IntSS
{
    public GeomInt_IntSS() { }

    public GeomInt_IntSS(Geom_Surface S1, Geom_Surface S2, double Tol, bool Approx = true, bool ApproxS1 = false,
                         bool ApproxS2 = false)
        => throw new NotImplementedException();

    public void Perform(Geom_Surface S1, Geom_Surface S2, double Tol, bool Approx = true, bool ApproxS1 = false,
                        bool ApproxS2 = false)
        => throw new NotImplementedException();

    public void Perform(GeomAdaptor_Surface HS1, GeomAdaptor_Surface HS2, double Tol, bool Approx = true,
                        bool ApproxS1 = false, bool ApproxS2 = false)
        => throw new NotImplementedException();

    public void Perform(Geom_Surface S1, Geom_Surface S2, double Tol, double U1, double V1, double U2, double V2,
                        bool Approx = true, bool ApproxS1 = false, bool ApproxS2 = false)
        => throw new NotImplementedException();

    public void Perform(GeomAdaptor_Surface HS1, GeomAdaptor_Surface HS2, double Tol, double U1, double V1, double U2,
                        double V2, bool Approx = true, bool ApproxS1 = false, bool ApproxS2 = false)
        => throw new NotImplementedException();

    public bool IsDone() => throw new NotImplementedException();
    public double TolReached3d() => throw new NotImplementedException();
    public double TolReached2d() => throw new NotImplementedException();
    public int NbLines() => throw new NotImplementedException();
    public Geom_Curve Line(int Index) => throw new NotImplementedException();
    public bool HasLineOnS1(int Index) => throw new NotImplementedException();
    public Geom2d_Curve LineOnS1(int Index) => throw new NotImplementedException();
    public bool HasLineOnS2(int Index) => throw new NotImplementedException();
    public Geom2d_Curve LineOnS2(int Index) => throw new NotImplementedException();
    public int NbBoundaries() => throw new NotImplementedException();
    public Geom_Curve Boundary(int Index) => throw new NotImplementedException();
    public int NbPoints() => throw new NotImplementedException();
    public gp_Pnt Point(int Index) => throw new NotImplementedException();
    public gp_Pnt2d Pnt2d(int Index, bool OnFirst) => throw new NotImplementedException();
}
