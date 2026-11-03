// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_TopolTool.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_TopolTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// The topological tool of the surface/surface intersections of the Boolean component: Adaptor3d_TopolTool with its
/// own sample points (a regular grid sized by the surface type, strictly inside the parametric range).
/// </summary>
/// <remarks>A working object (decision 21): the sample grid is computed on first use and kept.</remarks>
public class IntersectionTopolTool : TopolTool
{
    private int _nbSmplU;
    private int _nbSmplV;
    private double _u0;
    private double _v0;
    private double _du;
    private double _dv;

    /// <summary>An empty tool.</summary>
    public IntersectionTopolTool()
    {
        _nbSmplU = 0;
        _nbSmplV = 0;
        _du = 1.0;
        _dv = 1.0;
    }

    /// <summary>The tool of the surface.</summary>
    public IntersectionTopolTool(SurfaceAdaptorBase surface)
    {
        Initialize(surface);
        _nbSmplU = 0;
        _nbSmplV = 0;
        _du = 1.0;
        _dv = 1.0;
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as in OCCT.</summary>
    public override void Initialize() => throw new MethodNotImplementedException("IntTools_TopolTool::Initialize ()");

    /// <summary>Initializes the tool with the surface.</summary>
    public override void Initialize(SurfaceAdaptorBase surface)
    {
        base.Initialize(surface);
        // myS = theSurface;
        _nbSmplU = 0;
        _nbSmplV = 0;
        _du = 1.0;
        _dv = 1.0;
    }

    /// <summary>Computes the sample grid: its size by the surface type, at most 50 per direction for most types.</summary>
    public override void ComputeSamplePoints()
    {
        double uinf, usup, vinf, vsup;
        uinf = _s!.FirstUParameter();
        usup = _s.LastUParameter();
        vinf = _s.FirstVParameter();
        vsup = _s.LastVParameter();
        const int maxNbSample = 50;

        if (usup < uinf)
        {
            double temp = uinf;
            uinf = usup;
            usup = temp;
        }
        if (vsup < vinf)
        {
            double temp = vinf;
            vinf = vsup;
            vsup = temp;
        }
        bool isbiguinf, isbigusup, isbigvinf, isbigvsup;
        isbiguinf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(uinf);
        isbigusup = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(usup);
        isbigvinf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vinf);
        isbigvsup = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vsup);

        if (isbiguinf && isbigusup)
        {
            uinf = -1.0e5;
            usup = 1.0e5;
        }
        else if (isbiguinf)
        {
            uinf = usup - 2.0e5;
        }
        else if (isbigusup)
        {
            usup = uinf + 2.0e5;
        }

        if (isbigvinf && isbigvsup)
        {
            vinf = -1.0e5;
            vsup = 1.0e5;
        }
        else if (isbigvinf)
        {
            vinf = vsup - 2.0e5;
        }
        else if (isbigvsup)
        {
            vsup = vinf + 2.0e5;
        }
        _u0 = uinf;
        _v0 = vinf;

        int nbsu = 0, nbsv = 0;
        SurfaceKind typS = _s.GetType();

        switch (typS)
        {
            case Plane:
                {
                    nbsu = 10;
                    nbsv = 10;
                }
                break;
            case SurfaceKind.Cylinder:
                {
                    double radius = _s.Cylinder().Radius();
                    double maxAngle = M_PI * 0.5;
                    double deflection = 1.0e-02;

                    if (radius > deflection)
                    {
                        maxAngle = Math.Acos(1.0 - deflection / radius) * 2.0;
                    }
                    if (maxAngle > global::Kerf.Engine.Foundation.Precision.Angular())
                    {
                        nbsu = (int)((usup - uinf) / maxAngle);
                    }
                    nbsv = (int)(vsup - vinf);
                    nbsv /= 10;

                    if (nbsu < 2)
                    {
                        nbsu = 2;
                    }
                    if (nbsv < 2)
                    {
                        nbsv = 2;
                    }

                    //    if(nbsu < 10) nbsu = 10;
                    //    if(nbsv < 10) nbsv = 10;
                    if (nbsu > maxNbSample)
                    {
                        nbsu = maxNbSample;
                    }
                    if (nbsv > maxNbSample)
                    {
                        nbsv = maxNbSample;
                    }
                }
                break;
            case SurfaceKind.Cone:
                {
                    Cone cone = _s.Cone();
                    Circle3D circle = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(cone.Position(), cone.RefRadius(), cone.SemiAngle(), vinf);
                    double radius = circle.Radius();
                    circle = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(cone.Position(), cone.RefRadius(), cone.SemiAngle(), vsup);

                    if (radius < circle.Radius())
                    {
                        radius = circle.Radius();
                    }
                    double maxAngle = M_PI * 0.5;
                    double deflection = 1.0e-02;

                    if (radius > deflection)
                    {
                        maxAngle = Math.Acos(1.0 - deflection / radius) * 2.0;
                    }

                    if (maxAngle > global::Kerf.Engine.Foundation.Precision.Angular())
                    {
                        nbsu = (int)((usup - uinf) / maxAngle);
                    }
                    nbsv = (int)(vsup - vinf);
                    nbsv /= 10;

                    //     if(nbsu < 2) nbsu = 2;
                    //     if(nbsv < 2) nbsv = 2;

                    if (nbsu < 10)
                    {
                        nbsu = 10;
                    }
                    if (nbsv < 10)
                    {
                        nbsv = 10;
                    }
                    if (nbsu > maxNbSample)
                    {
                        nbsu = maxNbSample;
                    }
                    if (nbsv > maxNbSample)
                    {
                        nbsv = maxNbSample;
                    }
                }
                break;
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                {
                    Circle3D circle;
                    double radius1, radius2;

                    if (typS == SurfaceKind.Torus)
                    {
                        Torus torus = _s.Torus();
                        circle = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(torus.Position(), torus.MajorRadius(), torus.MinorRadius(), uinf);
                        radius2 = circle.Radius();
                        circle = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(torus.Position(), torus.MajorRadius(), torus.MinorRadius(), usup);
                        radius2 = (radius2 < circle.Radius()) ? circle.Radius() : radius2;

                        circle = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(torus.Position(), torus.MajorRadius(), torus.MinorRadius(), vinf);
                        radius1 = circle.Radius();
                        circle = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(torus.Position(), torus.MajorRadius(), torus.MinorRadius(), vsup);
                        radius1 = (radius1 < circle.Radius()) ? circle.Radius() : radius1;
                    }
                    else
                    {
                        Sphere sphere = _s.Sphere();
                        radius1 = sphere.Radius();
                        radius2 = sphere.Radius();
                    }
                    double maxAngle = M_PI * 0.5;
                    double deflection = 1.0e-02;

                    if (radius1 > deflection)
                    {
                        maxAngle = Math.Acos(1.0 - deflection / radius1) * 2.0;
                    }

                    if (maxAngle > global::Kerf.Engine.Foundation.Precision.Angular())
                    {
                        nbsu = (int)((usup - uinf) / maxAngle);
                    }
                    maxAngle = M_PI * 0.5;

                    if (radius2 > deflection)
                    {
                        maxAngle = Math.Acos(1.0 - deflection / radius2) * 2.0;
                    }

                    if (maxAngle > global::Kerf.Engine.Foundation.Precision.Angular())
                    {
                        nbsv = (int)((vsup - vinf) / maxAngle);
                    }
                    if (nbsu < 10)
                    {
                        nbsu = 10;
                    }
                    if (nbsv < 10)
                    {
                        nbsv = 10;
                    }
                    if (nbsu > maxNbSample)
                    {
                        nbsu = maxNbSample;
                    }
                    if (nbsv > maxNbSample)
                    {
                        nbsv = maxNbSample;
                    }
                }
                break;
            case SurfaceKind.BezierSurface:
                {
                    nbsv = 3 + _s.VPoleCount;
                    nbsu = 3 + _s.UPoleCount;

                    if (nbsu > 10 || nbsv > 10)
                    {
                        NCollection_Array2<Point3D> array2 = _s.Bezier().InternalPoles();
                        Analyse(array2, out nbsu, out nbsv);
                    }

                    if (nbsu < 10)
                    {
                        nbsu = 10;
                    }
                    if (nbsv < 10)
                    {
                        nbsv = 10;
                    }
                }
                break;
            case SurfaceKind.BSplineSurface:
                {
                    nbsv = _s.VKnotCount;
                    nbsv *= _s.VDegree();
                    if (nbsv < 4)
                    {
                        nbsv = 4;
                    }
                    nbsu = _s.UKnotCount;
                    nbsu *= _s.UDegree();
                    if (nbsu < 4)
                    {
                        nbsu = 4;
                    }

                    if (nbsu > 10 || nbsv > 10)
                    {
                        NCollection_Array2<Point3D> array2 = _s.BSpline().InternalPoles();
                        Analyse(array2, out nbsu, out nbsv);
                    }
                    if (nbsu < 10)
                    {
                        nbsu = 10;
                    }
                    if (nbsv < 10)
                    {
                        nbsv = 10;
                    }
                    // Check anisotropy
                    double uLen = (usup - uinf) / _s.UResolution(1.0);
                    double vLen = (vsup - vinf) / _s.VResolution(1.0);
                    double ratio = uLen / vLen;
                    if (ratio >= 10.0)
                    {
                        nbsu *= 2;
                        nbsu = Min(nbsu, maxNbSample);
                    }
                    else if (ratio <= 0.1)
                    {
                        nbsv *= 2;
                        nbsv = Min(nbsv, maxNbSample);
                    }
                }
                break;
            case SurfaceOfExtrusion:
                {
                    nbsu = 15;
                    nbsv = (int)(vsup - vinf);
                    nbsv /= 10;
                    if (nbsv < 15)
                    {
                        nbsv = 15;
                    }
                    if (nbsv > maxNbSample)
                    {
                        nbsv = maxNbSample;
                    }
                }
                break;
            case SurfaceOfRevolution:
                {
                    nbsv = 15;
                    nbsu = 15;
                }
                break;
            default:
                {
                    nbsu = 10;
                    nbsv = 10;
                }
                break;
        }
        _nbSmplU = nbsu;
        _nbSmplV = nbsv;

        _nbSamplesU = _nbSmplU;
        _nbSamplesV = _nbSmplV;

        _du = (usup - uinf) / (_nbSmplU + 1);
        _dv = (vsup - vinf) / (_nbSmplV + 1);
    }

    /// <summary>The number of samples in U (computing the grid on first use).</summary>
    public override int NbSamplesU()
    {
        if (_nbSmplU <= 0)
        {
            ComputeSamplePoints();
        }
        return _nbSmplU;
    }

    /// <summary>The number of samples in V (computing the grid on first use).</summary>
    public override int NbSamplesV()
    {
        if (_nbSmplV <= 0)
        {
            ComputeSamplePoints();
        }
        return _nbSmplV;
    }

    /// <summary>The number of sample points (computing the grid on first use).</summary>
    public override int NbSamples()
    {
        if (_nbSmplU <= 0)
        {
            ComputeSamplePoints();
        }
        return (_nbSmplU * _nbSmplV);
    }

    /// <summary>The sample point of the index (1 to NbSamples) in the parameter space and in 3D.</summary>
    public override void SamplePoint(int index, out Point2D p2d, out Point3D p3d)
    {
        if (_uPars is null)
        {
            if (_nbSmplU <= 0)
            {
                ComputeSamplePoints();
            }
            int iv = 1 + index / _nbSmplU;
            int iu = 1 + index - (iv - 1) * _nbSmplU;
            double u = _u0 + iu * _du;
            double v = _v0 + iv * _dv;
            p2d = new Point2D(u, v);
            p3d = _s!.Value(u, v);
        }
        else
        {
            base.SamplePoint(index, out p2d, out p3d);
        }
    }

    // OCCT's file-static Analyse: the number of curvature sign changes along the pole rows and columns, plus 5.
    // As OCCT, the first three poles of each row are read at columns 1 to 3 whatever the array's lower column.
    private static void Analyse(NCollection_Array2<Point3D> array2, out int nbSamplesU, out int nbSamplesV)
    {
        Vector3D vi = new(), vip1 = new();
        int sh, nbch, i, j;
        int nbup = array2.UpperRow() - array2.LowerRow() + 1;
        int nbvp = array2.UpperCol() - array2.LowerCol() + 1;

        sh = 1;
        nbch = 0;
        if (nbvp > 2)
        {

            for (i = array2.LowerRow(); i <= array2.UpperRow(); i++)
            {
                Point3D a = array2.Value(i, 1);
                Point3D b = array2.Value(i, 2);
                Point3D c = array2.Value(i, 3);
                vi = vi.WithCoord(c.X() - b.X() - b.X() + a.X(),
                            c.Y() - b.Y() - b.Y() + a.Y(),
                            c.Z() - b.Z() - b.Z() + a.Z());
                int locnbch = 0;

                for (j = array2.LowerCol() + 2; j < array2.UpperCol(); j++)
                { //-- essai
                    Point3D ax = array2.Value(i, j - 1);
                    Point3D bx = array2.Value(i, j);
                    Point3D cx = array2.Value(i, j + 1);
                    vip1 = vip1.WithCoord(cx.X() - bx.X() - bx.X() + ax.X(),
                                  cx.Y() - bx.Y() - bx.Y() + ax.Y(),
                                  cx.Z() - bx.Z() - bx.Z() + ax.Z());
                    double pd = vi.Dot(vip1);
                    vi = vip1;
                    if (pd > 1.0e-7 || pd < -1.0e-7)
                    {
                        if (pd > 0)
                        {
                            if (sh == -1)
                            {
                                sh = 1;
                                locnbch++;
                            }
                        }
                        else
                        {
                            if (sh == 1)
                            {
                                sh = -1;
                                locnbch++;
                            }
                        }
                    }
                }
                if (locnbch > nbch)
                {
                    nbch = locnbch;
                }
            }
        }
        nbSamplesV = nbch + 5;

        nbch = 0;
        if (nbup > 2)
        {
            for (j = array2.LowerCol(); j <= array2.UpperCol(); j++)
            {
                Point3D a = array2.Value(array2.LowerRow(), j);
                Point3D b = array2.Value(array2.LowerRow() + 1, j);
                Point3D c = array2.Value(array2.LowerRow() + 2, j);
                vi = vi.WithCoord(c.X() - b.X() - b.X() + a.X(),
                            c.Y() - b.Y() - b.Y() + a.Y(),
                            c.Z() - b.Z() - b.Z() + a.Z());
                int locnbch = 0;

                for (i = array2.LowerRow() + 2; i < array2.UpperRow(); i++)
                { //-- essai
                    Point3D ax = array2.Value(i - 1, j);
                    Point3D bx = array2.Value(i, j);
                    Point3D cx = array2.Value(i + 1, j);
                    vip1 = vip1.WithCoord(cx.X() - bx.X() - bx.X() + ax.X(),
                                  cx.Y() - bx.Y() - bx.Y() + ax.Y(),
                                  cx.Z() - bx.Z() - bx.Z() + ax.Z());
                    double pd = vi.Dot(vip1);
                    vi = vip1;
                    if (pd > 1.0e-7 || pd < -1.0e-7)
                    {
                        if (pd > 0)
                        {
                            if (sh == -1)
                            {
                                sh = 1;
                                locnbch++;
                            }
                        }
                        else
                        {
                            if (sh == 1)
                            {
                                sh = -1;
                                locnbch++;
                            }
                        }
                    }
                }
                if (locnbch > nbch)
                {
                    nbch = locnbch;
                }
            }
        }
        nbSamplesU = nbch + 5;
    }

    // Modified IFV
    /// <summary>
    /// Computes the sample parameters adaptively (Adaptor3d_TopolTool.SamplePnts), then takes the grid's origin and
    /// steps from them.
    /// </summary>
    public override void SamplePnts(double defl, int nUmin, int nVmin)
    {
        base.SamplePnts(defl, nUmin, nVmin);

        _nbSmplU = base.NbSamplesU();
        _nbSmplV = base.NbSamplesV();

        _u0 = _uPars!.Value(1);
        _v0 = _vPars!.Value(1);

        _du = (_uPars.Value(_nbSmplU) - _u0) / (_nbSmplU - 1);
        _dv = (_vPars.Value(_nbSmplV) - _v0) / (_nbSmplV - 1);
    }
}
