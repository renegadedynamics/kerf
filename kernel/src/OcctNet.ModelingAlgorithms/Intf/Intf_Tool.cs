// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Tool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Provides services to create boxes for infinite curves (lines, hyperbolas, parabolas) in a given context: the
/// part of the curve inside a bounding domain, as a box and as parameter segments on the curve.
/// </summary>
/// <remarks>
/// A working object (decision 21): the box methods are public and fill the tool's segments, read back through
/// <see cref="NbSegments"/>, <see cref="BeginParam"/> and <see cref="EndParam"/>. The output boxes are
/// <c>ref</c> parameters (OCCT's <c>Bnd_Box2d&amp;</c>/<c>Bnd_Box&amp;</c>); they are reset first.
/// </remarks>
public class Intf_Tool
{
    private int nbSeg;
    private readonly double[] beginOnCurve = new double[6];
    private readonly double[] endOnCurve = new double[6];
    private readonly int[] bord = new int[12];
    private readonly double[] xint = new double[12];
    private readonly double[] yint = new double[12];
    private readonly double[] zint = new double[12];
    private readonly double[] parint = new double[12];

    public Intf_Tool()
    {
        nbSeg = 0;
    }

    public void Lin2dBox(in Line2D L2d, in Box2d domain, ref Box2d boxLin)
    {
        nbSeg = 0;
        boxLin.SetVoid();
        if (domain.IsWhole)
        {
            boxLin.Set(L2d.Location(), L2d.Direction());
            boxLin.Add(L2d.Direction().Reversed());
            nbSeg = 1;
            beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        double xmin, xmax, ymin, ymax;
        double Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0;
        double parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
        double parcur, par1, par2;
        bool xToSet, yToSet;

        domain.Get(out xmin, out ymin, out xmax, out ymax);

        if (L2d.Direction().XY().X() > 0.0)
        {
            if (domain.IsOpenXmin)
            {
                parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmin = (xmin - L2d.Location().XY().X()) / L2d.Direction().XY().X();
            }
            if (domain.IsOpenXmax)
            {
                parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmax = (xmax - L2d.Location().XY().X()) / L2d.Direction().XY().X();
            }
            xToSet = true;
        }
        else if (L2d.Direction().XY().X() < 0.0)
        {
            if (domain.IsOpenXmax)
            {
                parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmin = (xmax - L2d.Location().XY().X()) / L2d.Direction().XY().X();
            }
            if (domain.IsOpenXmin)
            {
                parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmax = (xmin - L2d.Location().XY().X()) / L2d.Direction().XY().X();
            }
            xToSet = true;
        }
        else
        { // Parallel to axis  X
            if (L2d.Location().XY().X() < xmin || xmax < L2d.Location().XY().X())
            {
                return;
            }
            Xmin = L2d.Location().XY().X();
            Xmax = L2d.Location().XY().X();
            xToSet = false;
        }

        if (L2d.Direction().XY().Y() > 0.0)
        {
            if (domain.IsOpenYmin)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymin - L2d.Location().XY().Y()) / L2d.Direction().XY().Y();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenYmax)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymax - L2d.Location().XY().Y()) / L2d.Direction().XY().Y();
            }
            parmax = Min(parmax, parcur);
            yToSet = true;
        }
        else if (L2d.Direction().XY().Y() < 0.0)
        {
            if (domain.IsOpenYmax)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymax - L2d.Location().XY().Y()) / L2d.Direction().XY().Y();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenYmin)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymin - L2d.Location().XY().Y()) / L2d.Direction().XY().Y();
            }
            parmax = Min(parmax, parcur);
            yToSet = true;
        }
        else
        { // Parallel to axis  Y
            if (L2d.Location().XY().Y() < ymin || ymax < L2d.Location().XY().Y())
            {
                return;
            }
            Ymin = L2d.Location().XY().Y();
            Ymax = L2d.Location().XY().Y();
            yToSet = false;
        }

        nbSeg++;
        beginOnCurve[0] = parmin;
        endOnCurve[0] = parmax;

        if (xToSet)
        {
            par1 = L2d.Location().XY().X() + parmin * L2d.Direction().XY().X();
            par2 = L2d.Location().XY().X() + parmax * L2d.Direction().XY().X();
            Xmin = Min(par1, par2);
            Xmax = Max(par1, par2);
        }

        if (yToSet)
        {
            par1 = L2d.Location().XY().Y() + parmin * L2d.Direction().XY().Y();
            par2 = L2d.Location().XY().Y() + parmax * L2d.Direction().XY().Y();
            Ymin = Min(par1, par2);
            Ymax = Max(par1, par2);
        }

        boxLin.Update(Xmin, Ymin, Xmax, Ymax);
    }

    public void Hypr2dBox(in Hyperbola2D theHypr2d, in Box2d domain, ref Box2d boxHypr2d)
    {
        nbSeg = 0;
        boxHypr2d.SetVoid();
        if (domain.IsWhole)
        {
            boxHypr2d.SetWhole();
            nbSeg = 1;
            beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        int nbPi = Inters2d(theHypr2d, domain);

        if (nbPi > 0)
        {
            double Xmin, Xmax, Ymin, Ymax;

            domain.Get(out Xmin, out Ymin, out Xmax, out Ymax);

            int npi;
            for (npi = 0; npi < nbPi; npi++)
            {
                Xmin = Min(Xmin, xint[npi]);
                Xmax = Max(Xmax, xint[npi]);
                Ymin = Min(Ymin, yint[npi]);
                Ymax = Max(Ymax, yint[npi]);
            }
            boxHypr2d.Update(Xmin, Ymin, Xmax, Ymax);

            int npj, npk;
            double parmin;
            for (npi = 0; npi < nbPi; npi++)
            {
                npk = npi;
                for (npj = npi + 1; npj < nbPi; npj++)
                {
                    if (parint[npj] < parint[npk])
                    {
                        npk = npj;
                    }
                }
                if (npk != npi)
                {
                    parmin = parint[npk];
                    parint[npk] = parint[npi];
                    parint[npi] = parmin;
                    npj = bord[npk];
                    bord[npk] = bord[npi];
                    bord[npi] = npj;
                }
            }

            Point2D Pn;
            Vector2D Tan;
            double sinan = 0;
            bool out_ = true;

            for (npi = 0; npi < nbPi; npi++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(parint[npi], theHypr2d, out Pn, out Tan);
                switch (bord[npi])
                {
                    case 1:
                        sinan = new Xy(-1.0, 0.0) ^ Tan.XY();
                        break;
                    case 2:
                        sinan = new Xy(0.0, -1.0) ^ Tan.XY();
                        break;
                    case 3:
                        sinan = new Xy(1.0, 0.0) ^ Tan.XY();
                        break;
                    case 4:
                        sinan = new Xy(0.0, 1.0) ^ Tan.XY();
                        break;
                }
                if (Math.Abs(sinan) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    if (sinan > 0.0)
                    {
                        if (nbSeg < 6)
                        {
                            out_ = false;
                            beginOnCurve[nbSeg] = parint[npi];
                            nbSeg++;
                        }
                    }
                    else
                    {
                        if (out_ && nbSeg < 6)
                        {
                            beginOnCurve[nbSeg] = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            nbSeg++;
                        }
                        if (nbSeg > 0)
                        {
                            endOnCurve[nbSeg - 1] = parint[npi];
                        }
                        out_ = true;

                        int ipmin;
                        if (beginOnCurve[nbSeg - 1] < -10.0)
                        {
                            ipmin = -10;
                        }
                        else
                        {
                            ipmin = (int)beginOnCurve[nbSeg - 1];
                        }
                        int ipmax;
                        if (endOnCurve[nbSeg - 1] > 10.0)
                        {
                            ipmax = 10;
                        }
                        else
                        {
                            ipmax = (int)endOnCurve[nbSeg - 1];
                        }

                        ipmin = ipmin * 10 + 1;
                        ipmax = ipmax * 10 - 1;
                        int ip, pas = 1;
                        for (ip = ipmin; ip <= ipmax; ip += pas)
                        {
                            boxHypr2d.Add(global::Kerf.Engine.Geometry.ElCLib.Value((double)ip / 10.0, theHypr2d));
                            if (Math.Abs(ip) <= 10)
                            {
                                pas = 1;
                            }
                            else
                            {
                                pas = 10;
                            }
                        }
                    }
                }
            }
            if (!out_ && nbSeg > 0)
            {
                endOnCurve[nbSeg - 1] = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }
        else if (!domain.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, theHypr2d)))
        {
            boxHypr2d = domain;
            beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            nbSeg = 1;
        }
    }

    private int Inters2d(in Hyperbola2D theCurv, in Box2d Domain)
    {
        int nbpi = 0;
        int npi;
        double xmin, xmax, ymin, ymax;

        Domain.Get(out xmin, out ymin, out xmax, out ymax);

        if (!Domain.IsOpenYmax)
        {
            Line2D L1 = new(new Point2D(0.0, ymax), new Direction2D(Direction2D.D.NX));
            IntAna2d_AnaIntersection Inters1 = new(theCurv, new IntAna2d_Conic(L1));
            if (Inters1.IsDone())
            {
                if (!Inters1.IsEmpty())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).Value().X();
                        if (xmin < xint[nbpi] && xint[nbpi] <= xmax)
                        {
                            yint[nbpi] = ymax;
                            parint[nbpi] = Inters1.Point(npi).ParamOnFirst();
                            bord[nbpi] = 1;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenXmin)
        {
            Line2D L2 = new(new Point2D(xmin, 0.0), new Direction2D(Direction2D.D.NY));
            IntAna2d_AnaIntersection Inters2 = new(theCurv, new IntAna2d_Conic(L2));
            if (Inters2.IsDone())
            {
                if (!Inters2.IsEmpty())
                {
                    for (npi = 1; npi <= Inters2.NbPoints(); npi++)
                    {
                        yint[nbpi] = Inters2.Point(npi).Value().Y();
                        if (ymin < yint[nbpi] && yint[nbpi] <= ymax)
                        {
                            xint[nbpi] = xmin;
                            parint[nbpi] = Inters2.Point(npi).ParamOnFirst();
                            bord[nbpi] = 2;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenYmin)
        {
            Line2D L3 = new(new Point2D(0.0, ymin), new Direction2D(Direction2D.D.X));
            IntAna2d_AnaIntersection Inters3 = new(theCurv, new IntAna2d_Conic(L3));
            if (Inters3.IsDone())
            {
                if (!Inters3.IsEmpty())
                {
                    for (npi = 1; npi <= Inters3.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters3.Point(npi).Value().X();
                        if (xmin <= xint[nbpi] && xint[nbpi] < xmax)
                        {
                            yint[nbpi] = ymin;
                            parint[nbpi] = Inters3.Point(npi).ParamOnFirst();
                            bord[nbpi] = 3;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenXmax)
        {
            Line2D L4 = new(new Point2D(xmax, 0.0), new Direction2D(Direction2D.D.Y));
            IntAna2d_AnaIntersection Inters4 = new(theCurv, new IntAna2d_Conic(L4));
            if (Inters4.IsDone())
            {
                if (!Inters4.IsEmpty())
                {
                    for (npi = 1; npi <= Inters4.NbPoints(); npi++)
                    {
                        yint[nbpi] = Inters4.Point(npi).Value().Y();
                        if (ymin <= yint[nbpi] && yint[nbpi] < ymax)
                        {
                            xint[nbpi] = xmax;
                            parint[nbpi] = Inters4.Point(npi).ParamOnFirst();
                            bord[nbpi] = 4;
                            nbpi++;
                        }
                    }
                }
            }
        }
        return nbpi;
    }

    public void Parab2dBox(in Parabola2D theParab2d, in Box2d domain, ref Box2d boxParab2d)
    {
        nbSeg = 0;
        boxParab2d.SetVoid();
        if (domain.IsWhole)
        {
            boxParab2d.SetWhole();
            nbSeg = 1;
            beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        int nbPi = Inters2d(theParab2d, domain);

        if (nbPi > 0)
        {
            double Xmin, Xmax, Ymin, Ymax;

            domain.Get(out Xmin, out Ymin, out Xmax, out Ymax);

            int npi;
            for (npi = 0; npi < nbPi; npi++)
            {
                Xmin = Min(Xmin, xint[npi]);
                Xmax = Max(Xmax, xint[npi]);
                Ymin = Min(Ymin, yint[npi]);
                Ymax = Max(Ymax, yint[npi]);
            }
            boxParab2d.Update(Xmin, Ymin, Xmax, Ymax);

            int npj, npk;
            double parmin;
            for (npi = 0; npi < nbPi; npi++)
            {
                npk = npi;
                for (npj = npi + 1; npj < nbPi; npj++)
                {
                    if (parint[npj] < parint[npk])
                    {
                        npk = npj;
                    }
                }
                if (npk != npi)
                {
                    parmin = parint[npk];
                    parint[npk] = parint[npi];
                    parint[npi] = parmin;
                    npj = bord[npk];
                    bord[npk] = bord[npi];
                    bord[npi] = npj;
                }
            }

            Point2D Pn;
            Vector2D Tan;
            double sinan = 0;
            bool out_ = true;

            for (npi = 0; npi < nbPi; npi++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(parint[npi], theParab2d, out Pn, out Tan);
                switch (bord[npi])
                {
                    case 1:
                        sinan = new Xy(-1.0, 0.0) ^ Tan.XY();
                        break;
                    case 2:
                        sinan = new Xy(0.0, -1.0) ^ Tan.XY();
                        break;
                    case 3:
                        sinan = new Xy(1.0, 0.0) ^ Tan.XY();
                        break;
                    case 4:
                        sinan = new Xy(0.0, 1.0) ^ Tan.XY();
                        break;
                }
                if (Math.Abs(sinan) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    if (sinan > 0.0)
                    {
                        if (nbSeg < 6)
                        {
                            out_ = false;
                            beginOnCurve[nbSeg] = parint[npi];
                            nbSeg++;
                        }
                    }
                    else
                    {
                        if (out_ && nbSeg < 6)
                        {
                            beginOnCurve[nbSeg] = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            nbSeg++;
                        }
                        if (nbSeg > 0)
                        {
                            endOnCurve[nbSeg - 1] = parint[npi];
                        }
                        out_ = true;

                        int ipmin;
                        if (beginOnCurve[nbSeg - 1] < -10.0)
                        {
                            ipmin = -10;
                        }
                        else
                        {
                            ipmin = (int)beginOnCurve[nbSeg - 1];
                        }
                        int ipmax;
                        if (endOnCurve[nbSeg - 1] > 10.0)
                        {
                            ipmax = 10;
                        }
                        else
                        {
                            ipmax = (int)endOnCurve[nbSeg - 1];
                        }

                        ipmin = ipmin * 10 + 1;
                        ipmax = ipmax * 10 - 1;
                        int ip, pas = 1;
                        for (ip = ipmin; ip <= ipmax; ip += pas)
                        {
                            boxParab2d.Add(global::Kerf.Engine.Geometry.ElCLib.Value((double)ip / 10.0, theParab2d));
                            if (Math.Abs(ip) <= 10)
                            {
                                pas = 1;
                            }
                            else
                            {
                                pas = 10;
                            }
                        }
                    }
                }
            }
            if (!out_ && nbSeg > 0)
            {
                endOnCurve[nbSeg - 1] = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }
        else if (!domain.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, theParab2d)))
        {
            boxParab2d = domain;
            beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            nbSeg = 1;
        }
    }

    private int Inters2d(in Parabola2D theCurv, in Box2d Domain)
    {
        int nbpi = 0;
        int npi;
        double xmin, xmax, ymin, ymax;

        Domain.Get(out xmin, out ymin, out xmax, out ymax);

        if (!Domain.IsOpenYmax)
        {
            Line2D L1 = new(new Point2D(0.0, ymax), new Direction2D(Direction2D.D.NX));
            IntAna2d_AnaIntersection Inters1 = new(theCurv, new IntAna2d_Conic(L1));
            if (Inters1.IsDone())
            {
                if (!Inters1.IsEmpty())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).Value().X();
                        if (xmin < xint[nbpi] && xint[nbpi] <= xmax)
                        {
                            yint[nbpi] = ymax;
                            parint[nbpi] = Inters1.Point(npi).ParamOnFirst();
                            bord[nbpi] = 1;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenXmin)
        {
            Line2D L2 = new(new Point2D(xmin, 0.0), new Direction2D(Direction2D.D.NY));
            IntAna2d_AnaIntersection Inters2 = new(theCurv, new IntAna2d_Conic(L2));
            if (Inters2.IsDone())
            {
                if (!Inters2.IsEmpty())
                {
                    for (npi = 1; npi <= Inters2.NbPoints(); npi++)
                    {
                        yint[nbpi] = Inters2.Point(npi).Value().Y();
                        if (ymin < yint[nbpi] && yint[nbpi] <= ymax)
                        {
                            xint[nbpi] = xmin;
                            parint[nbpi] = Inters2.Point(npi).ParamOnFirst();
                            bord[nbpi] = 2;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenYmin)
        {
            Line2D L3 = new(new Point2D(0.0, ymin), new Direction2D(Direction2D.D.X));
            IntAna2d_AnaIntersection Inters3 = new(theCurv, new IntAna2d_Conic(L3));
            if (Inters3.IsDone())
            {
                if (!Inters3.IsEmpty())
                {
                    for (npi = 1; npi <= Inters3.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters3.Point(npi).Value().X();
                        if (xmin <= xint[nbpi] && xint[nbpi] < xmax)
                        {
                            yint[nbpi] = ymin;
                            parint[nbpi] = Inters3.Point(npi).ParamOnFirst();
                            bord[nbpi] = 3;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenXmax)
        {
            Line2D L4 = new(new Point2D(xmax, 0.0), new Direction2D(Direction2D.D.Y));
            IntAna2d_AnaIntersection Inters4 = new(theCurv, new IntAna2d_Conic(L4));
            if (Inters4.IsDone())
            {
                if (!Inters4.IsEmpty())
                {
                    for (npi = 1; npi <= Inters4.NbPoints(); npi++)
                    {
                        yint[nbpi] = Inters4.Point(npi).Value().Y();
                        if (ymin <= yint[nbpi] && yint[nbpi] < ymax)
                        {
                            xint[nbpi] = xmax;
                            parint[nbpi] = Inters4.Point(npi).ParamOnFirst();
                            bord[nbpi] = 4;
                            nbpi++;
                        }
                    }
                }
            }
        }
        return nbpi;
    }

    public void LinBox(in Line3D L, in Box domain, ref Box boxLin)
    {
        nbSeg = 0;
        boxLin.SetVoid();
        if (domain.IsWhole)
        {
            boxLin.Set(L.Location(), L.Direction());
            boxLin.Add(L.Direction().Reversed());
            nbSeg = 1;
            beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        double xmin, xmax, ymin, ymax, zmin, zmax;
        double Xmin = 0, Xmax = 0, Ymin = 0, Ymax = 0, Zmin = 0, Zmax = 0;
        double parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
        double parcur, par1, par2;
        bool xToSet, yToSet, zToSet;

        domain.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);

        if (L.Direction().XYZ().X() > 0.0)
        {
            if (domain.IsOpenXmin)
            {
                parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmin = (xmin - L.Location().XYZ().X()) / L.Direction().XYZ().X();
            }
            if (domain.IsOpenXmax)
            {
                parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmax = (xmax - L.Location().XYZ().X()) / L.Direction().XYZ().X();
            }
            xToSet = true;
        }
        else if (L.Direction().XYZ().X() < 0.0)
        {
            if (domain.IsOpenXmax)
            {
                parmin = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmin = (xmax - L.Location().XYZ().X()) / L.Direction().XYZ().X();
            }
            if (domain.IsOpenXmin)
            {
                parmax = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parmax = (xmin - L.Location().XYZ().X()) / L.Direction().XYZ().X();
            }
            xToSet = true;
        }
        else
        { // Perpendiculaire a l axe  X
            if (L.Location().XYZ().X() < xmin || xmax < L.Location().XYZ().X())
            {
                return;
            }
            Xmin = L.Location().XYZ().X();
            Xmax = L.Location().XYZ().X();
            xToSet = false;
        }

        if (L.Direction().XYZ().Y() > 0.0)
        {
            if (domain.IsOpenYmin)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymin - L.Location().XYZ().Y()) / L.Direction().XYZ().Y();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenYmax)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymax - L.Location().XYZ().Y()) / L.Direction().XYZ().Y();
            }
            parmax = Min(parmax, parcur);
            yToSet = true;
        }
        else if (L.Direction().XYZ().Y() < 0.0)
        {
            if (domain.IsOpenYmax)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymax - L.Location().XYZ().Y()) / L.Direction().XYZ().Y();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenYmin)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (ymin - L.Location().XYZ().Y()) / L.Direction().XYZ().Y();
            }
            parmax = Min(parmax, parcur);
            yToSet = true;
        }
        else
        { // Perpendiculaire a l axe  Y
            if (L.Location().XYZ().Y() < ymin || ymax < L.Location().XYZ().Y())
            {
                return;
            }
            Ymin = L.Location().XYZ().Y();
            Ymax = L.Location().XYZ().Y();
            yToSet = false;
        }

        if (L.Direction().XYZ().Z() > 0.0)
        {
            if (domain.IsOpenZmin)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (zmin - L.Location().XYZ().Z()) / L.Direction().XYZ().Z();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenZmax)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (zmax - L.Location().XYZ().Z()) / L.Direction().XYZ().Z();
            }
            parmax = Min(parmax, parcur);
            zToSet = true;
        }
        else if (L.Direction().XYZ().Z() < 0.0)
        {
            if (domain.IsOpenZmax)
            {
                parcur = -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (zmax - L.Location().XYZ().Z()) / L.Direction().XYZ().Z();
            }
            parmin = Max(parmin, parcur);
            if (domain.IsOpenZmin)
            {
                parcur = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                parcur = (zmin - L.Location().XYZ().Z()) / L.Direction().XYZ().Z();
            }
            parmax = Min(parmax, parcur);
            zToSet = true;
        }
        else
        { // Perpendicular to axis Z
            if (L.Location().XYZ().Z() < zmin || zmax < L.Location().XYZ().Z())
            {
                return;
            }
            Zmin = L.Location().XYZ().Z();
            Zmax = L.Location().XYZ().Z();
            zToSet = false;
        }

        nbSeg++;
        beginOnCurve[0] = parmin;
        endOnCurve[0] = parmax;

        if (xToSet)
        {
            par1 = L.Location().XYZ().X() + parmin * L.Direction().XYZ().X();
            par2 = L.Location().XYZ().X() + parmax * L.Direction().XYZ().X();
            Xmin = Min(par1, par2);
            Xmax = Max(par1, par2);
        }

        if (yToSet)
        {
            par1 = L.Location().XYZ().Y() + parmin * L.Direction().XYZ().Y();
            par2 = L.Location().XYZ().Y() + parmax * L.Direction().XYZ().Y();
            Ymin = Min(par1, par2);
            Ymax = Max(par1, par2);
        }

        if (zToSet)
        {
            par1 = L.Location().XYZ().Z() + parmin * L.Direction().XYZ().Z();
            par2 = L.Location().XYZ().Z() + parmax * L.Direction().XYZ().Z();
            Zmin = Min(par1, par2);
            Zmax = Max(par1, par2);
        }

        boxLin.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
    }

    public void HyprBox(in Hyperbola3D theHypr, in Box domain, ref Box boxHypr)
    {
        nbSeg = 0;
        boxHypr.SetVoid();

        if (domain.IsWhole)
        {
            boxHypr.SetWhole();
            nbSeg = 1;
            // beginOnCurve[0]=-Precision::Infinite();
            // endOnCurve[0]=Precision::Infinite();
            beginOnCurve[0] = -100.0;
            endOnCurve[0] = 100.0;
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        int nbPi;

        nbPi = Inters3d(theHypr, domain);
        if (nbPi > 0)
        {
            int npi;
            double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;

            domain.Get(out Xmin, out Ymin, out Zmin, out Xmax, out Ymax, out Zmax);

            for (npi = 0; npi < nbPi; npi++)
            {
                Xmin = Min(Xmin, xint[npi]);
                Xmax = Max(Xmax, xint[npi]);
                Ymin = Min(Ymin, yint[npi]);
                Ymax = Max(Ymax, yint[npi]);
                Zmin = Min(Zmin, zint[npi]);
                Zmax = Max(Zmax, zint[npi]);
            }
            boxHypr.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);

            Point3D Pn;
            Vector3D Tan;
            double sinan = 0;
            bool out_ = true;

            for (npi = 0; npi < nbPi; npi++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(parint[npi], theHypr, out Pn, out Tan);
                switch (bord[npi])
                {
                    case 1:
                        sinan = new Xyz(1.0, 0.0, 0.0) * Tan.XYZ();
                        break;
                    case 2:
                        sinan = new Xyz(0.0, 1.0, 0.0) * Tan.XYZ();
                        break;
                    case 3:
                        sinan = new Xyz(0.0, 0.0, 1.0) * Tan.XYZ();
                        break;
                    case 4:
                        sinan = new Xyz(-1.0, 0.0, 0.0) * Tan.XYZ();
                        break;
                    case 5:
                        sinan = new Xyz(0.0, -1.0, 0.0) * Tan.XYZ();
                        break;
                    case 6:
                        sinan = new Xyz(0.0, 0.0, -1.0) * Tan.XYZ();
                        break;
                }
                if (Math.Abs(sinan) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    if (sinan > 0.0)
                    {
                        out_ = false;
                        // OCCT: unlike the other boxes (fixed for bugs #75/#76), HyprBox doesn't check nbSeg < 6
                        // before writing the 6-element segment arrays; more than six entries overflow them in C++
                        // and throw IndexOutOfRangeException here.
                        beginOnCurve[nbSeg] = parint[npi];
                        //// modified by jgv, 10.11.2009 /////
                        endOnCurve[nbSeg] = 10.0;
                        //////////////////////////////////////
                        nbSeg++;
                    }
                    else
                    {
                        if (out_)
                        {
                            beginOnCurve[nbSeg] = -10.0; // OCCT: unchecked index, as above
                            nbSeg++;
                        }
                        endOnCurve[nbSeg - 1] = parint[npi];
                        out_ = true;

                        double ipmin, ipmax, ip, pas;

                        ipmin = -10.0;
                        if (beginOnCurve[nbSeg - 1] > ipmin)
                        {
                            ipmin = beginOnCurve[nbSeg - 1];
                        }
                        ipmax = 10.0;
                        if (endOnCurve[nbSeg - 1] < ipmax)
                        {
                            ipmax = endOnCurve[nbSeg - 1];
                        }
                        ipmin = ipmin * 10.0 + 1.0;
                        ipmax = ipmax * 10.0 - 1.0;

                        pas = 1.0;
                        for (ip = ipmin; ip <= ipmax; ip += pas)
                        {
                            boxHypr.Add(global::Kerf.Engine.Geometry.ElCLib.Value(ip / 10.0, theHypr));
                            pas = 10.0;
                            if (Math.Abs(ip) <= 10.0)
                            {
                                pas = 1.0;
                            }
                        }
                    }
                }
            }
        }
        else if (!domain.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, theHypr)))
        {
            boxHypr = domain;
            // beginOnCurve[0]=-Precision::Infinite();
            // endOnCurve[0]=Precision::Infinite();
            beginOnCurve[0] = -100.0;
            endOnCurve[0] = 100.0;
            nbSeg = 1;
        }
    }

    private int Inters3d(in Hyperbola3D theCurv, in Box Domain)
    {
        int nbpi = 0;
        int npi;
        double xmin, ymin, zmin, xmax, ymax, zmax;

        Domain.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);

        if (!Domain.IsOpenXmin)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(1.0, 0.0, 0.0, -xmin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        yint[nbpi] = Inters1.Point(npi).Y();
                        zint[nbpi] = Inters1.Point(npi).Z();
                        if (ymin <= yint[nbpi] && yint[nbpi] < ymax && zmin <= zint[nbpi] && zint[nbpi] < zmax)
                        {
                            xint[nbpi] = xmin;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 1;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenYmin)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(0.0, 1.0, 0.0, -ymin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).X();
                        zint[nbpi] = Inters1.Point(npi).Z();
                        if (xmin < xint[nbpi] && xint[nbpi] <= xmax && zmin <= zint[nbpi] && zint[nbpi] < zmax)
                        {
                            yint[nbpi] = ymin;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 2;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenZmin)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(0.0, 0.0, 1.0, -zmin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).X();
                        yint[nbpi] = Inters1.Point(npi).Y();
                        if (xmin < xint[nbpi] && xint[nbpi] <= xmax && ymin < yint[nbpi] && yint[nbpi] <= ymax)
                        {
                            zint[nbpi] = zmin;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 3;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenXmax)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(-1.0, 0.0, 0.0, xmax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        yint[nbpi] = Inters1.Point(npi).Y();
                        zint[nbpi] = Inters1.Point(npi).Z();
                        if (ymin < yint[nbpi] && yint[nbpi] <= ymax && zmin < zint[nbpi] && zint[nbpi] <= zmax)
                        {
                            xint[nbpi] = xmax;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 4;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenYmax)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(0.0, -1.0, 0.0, ymax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).X();
                        zint[nbpi] = Inters1.Point(npi).Z();
                        if (xmin <= xint[nbpi] && xint[nbpi] < xmax && zmin < zint[nbpi] && zint[nbpi] <= zmax)
                        {
                            yint[nbpi] = ymax;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 5;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenZmax)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(0.0, 0.0, -1.0, zmax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).X();
                        yint[nbpi] = Inters1.Point(npi).Y();
                        if (xmin <= xint[nbpi] && xint[nbpi] < xmax && ymin <= yint[nbpi] && yint[nbpi] < ymax)
                        {
                            zint[nbpi] = zmax;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 6;
                            nbpi++;
                        }
                    }
                }
            }
        }

        int aNbDiffPoints = nbpi;

        // Sort parint and check if parint contains several
        // matched values. If that is true they will be deleted.
        for (int i = nbpi - 1; i > 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (parint[i] <= parint[j])
                {
                    (parint[i], parint[j]) = (parint[j], parint[i]);
                    (zint[i], zint[j]) = (zint[j], zint[i]);
                    (yint[i], yint[j]) = (yint[j], yint[i]);
                    (xint[i], xint[j]) = (xint[j], xint[i]);
                    (bord[i], bord[j]) = (bord[j], bord[i]);
                }

                if ((i < nbpi - 1) && IsEqual(parint[i], parint[i + 1]))
                {
                    aNbDiffPoints--;
                    for (int k = i; k < aNbDiffPoints; k++)
                    {
                        parint[k] = parint[k + 1];
                        zint[k] = zint[k + 1];
                        yint[k] = yint[k + 1];
                        xint[k] = xint[k + 1];
                        bord[k] = bord[k + 1];
                    }
                }
            }
        }

        return aNbDiffPoints;
    }

    private int Inters3d(in Parabola3D theCurv, in Box Domain)
    {
        int nbpi = 0;
        int npi;
        double xmin, ymin, zmin, xmax, ymax, zmax;

        Domain.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);

        if (!Domain.IsOpenXmin)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(1.0, 0.0, 0.0, -xmin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        yint[nbpi] = Inters1.Point(npi).Y();
                        zint[nbpi] = Inters1.Point(npi).Z();
                        if (ymin <= yint[nbpi] && yint[nbpi] < ymax && zmin <= zint[nbpi] && zint[nbpi] < zmax)
                        {
                            xint[nbpi] = xmin;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 1;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenYmin)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(0.0, 1.0, 0.0, -ymin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).X();
                        zint[nbpi] = Inters1.Point(npi).Z();
                        if (xmin < xint[nbpi] && xint[nbpi] <= xmax && zmin <= zint[nbpi] && zint[nbpi] < zmax)
                        {
                            yint[nbpi] = ymin;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 2;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenZmin)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(0.0, 0.0, 1.0, -zmin), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).X();
                        yint[nbpi] = Inters1.Point(npi).Y();
                        if (xmin < xint[nbpi] && xint[nbpi] <= xmax && ymin < yint[nbpi] && yint[nbpi] <= ymax)
                        {
                            zint[nbpi] = zmin;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 3;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenXmax)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(-1.0, 0.0, 0.0, xmax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        yint[nbpi] = Inters1.Point(npi).Y();
                        zint[nbpi] = Inters1.Point(npi).Z();
                        if (ymin < yint[nbpi] && yint[nbpi] <= ymax && zmin < zint[nbpi] && zint[nbpi] <= zmax)
                        {
                            xint[nbpi] = xmax;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 4;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenYmax)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(0.0, -1.0, 0.0, ymax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).X();
                        zint[nbpi] = Inters1.Point(npi).Z();
                        if (xmin <= xint[nbpi] && xint[nbpi] < xmax && zmin < zint[nbpi] && zint[nbpi] <= zmax)
                        {
                            yint[nbpi] = ymax;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 5;
                            nbpi++;
                        }
                    }
                }
            }
        }

        if (!Domain.IsOpenZmax)
        {
            IntAna_IntConicQuad Inters1 = new(theCurv, new Plane3D(0.0, 0.0, -1.0, zmax), global::Kerf.Engine.Foundation.Precision.Angular());
            if (Inters1.IsDone())
            {
                if (!Inters1.IsInQuadric())
                {
                    for (npi = 1; npi <= Inters1.NbPoints(); npi++)
                    {
                        xint[nbpi] = Inters1.Point(npi).X();
                        yint[nbpi] = Inters1.Point(npi).Y();
                        if (xmin <= xint[nbpi] && xint[nbpi] < xmax && ymin <= yint[nbpi] && yint[nbpi] < ymax)
                        {
                            zint[nbpi] = zmax;
                            parint[nbpi] = Inters1.ParamOnConic(npi);
                            bord[nbpi] = 6;
                            nbpi++;
                        }
                    }
                }
            }
        }

        int aNbDiffPoints = nbpi;

        // Sort parint and check if parint contains several
        // matched values. If that is true they will be deleted.
        for (int i = nbpi - 1; i > 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (parint[i] <= parint[j])
                {
                    (parint[i], parint[j]) = (parint[j], parint[i]);
                    (zint[i], zint[j]) = (zint[j], zint[i]);
                    (yint[i], yint[j]) = (yint[j], yint[i]);
                    (xint[i], xint[j]) = (xint[j], xint[i]);
                    (bord[i], bord[j]) = (bord[j], bord[i]);
                }

                if ((i < nbpi - 1) && IsEqual(parint[i], parint[i + 1]))
                {
                    aNbDiffPoints--;
                    for (int k = i; k < aNbDiffPoints; k++)
                    {
                        parint[k] = parint[k + 1];
                        zint[k] = zint[k + 1];
                        yint[k] = yint[k + 1];
                        xint[k] = xint[k + 1];
                        bord[k] = bord[k + 1];
                    }
                }
            }
        }

        return aNbDiffPoints;
    }

    public void ParabBox(in Parabola3D theParab, in Box domain, ref Box boxParab)
    {
        nbSeg = 0;
        boxParab.SetVoid();

        if (domain.IsWhole)
        {
            boxParab.SetWhole();
            nbSeg = 1;
            beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            return;
        }
        else if (domain.IsVoid)
        {
            return;
        }

        int nbPi = Inters3d(theParab, domain);

        if (nbPi > 0)
        {
            int npi;
            double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
            domain.Get(out Xmin, out Ymin, out Zmin, out Xmax, out Ymax, out Zmax);
            for (npi = 0; npi < nbPi; npi++)
            {
                Xmin = Min(Xmin, xint[npi]);
                Xmax = Max(Xmax, xint[npi]);
                Ymin = Min(Ymin, yint[npi]);
                Ymax = Max(Ymax, yint[npi]);
                Zmin = Min(Zmin, zint[npi]);
                Zmax = Max(Zmax, zint[npi]);
            }
            boxParab.Update(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);

            Point3D Pn;
            Vector3D Tan;
            double sinan = 0;
            bool out_ = true;

            for (npi = 0; npi < nbPi; npi++)
            {
                global::Kerf.Engine.Geometry.ElCLib.D1(parint[npi], theParab, out Pn, out Tan);
                switch (bord[npi])
                {
                    case 1:
                        sinan = new Xyz(1.0, 0.0, 0.0) * Tan.XYZ();
                        break;
                    case 2:
                        sinan = new Xyz(0.0, 1.0, 0.0) * Tan.XYZ();
                        break;
                    case 3:
                        sinan = new Xyz(0.0, 0.0, 1.0) * Tan.XYZ();
                        break;
                    case 4:
                        sinan = new Xyz(-1.0, 0.0, 0.0) * Tan.XYZ();
                        break;
                    case 5:
                        sinan = new Xyz(0.0, -1.0, 0.0) * Tan.XYZ();
                        break;
                    case 6:
                        sinan = new Xyz(0.0, 0.0, -1.0) * Tan.XYZ();
                        break;
                }
                if (Math.Abs(sinan) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    if (sinan > 0.0)
                    {
                        if (nbSeg < 6)
                        {
                            out_ = false;
                            beginOnCurve[nbSeg] = parint[npi];
                            nbSeg++;
                        }
                    }
                    else
                    {
                        if (out_ && nbSeg < 6)
                        {
                            beginOnCurve[nbSeg] = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            nbSeg++;
                        }
                        if (nbSeg > 0)
                        {
                            endOnCurve[nbSeg - 1] = parint[npi];
                        }
                        out_ = true;

                        int ipmin;
                        if (beginOnCurve[nbSeg - 1] < -10.0)
                        {
                            ipmin = -10;
                        }
                        else
                        {
                            ipmin = (int)beginOnCurve[nbSeg - 1];
                        }
                        int ipmax;
                        if (endOnCurve[nbSeg - 1] > 10.0)
                        {
                            ipmax = 10;
                        }
                        else
                        {
                            ipmax = (int)endOnCurve[nbSeg - 1];
                        }

                        ipmin = ipmin * 10 + 1;
                        ipmax = ipmax * 10 - 1;
                        int ip, pas = 1;
                        for (ip = ipmin; ip <= ipmax; ip += pas)
                        {
                            boxParab.Add(global::Kerf.Engine.Geometry.ElCLib.Value((double)ip / 10.0, theParab));
                            if (Math.Abs(ip) <= 10)
                            {
                                pas = 1;
                            }
                            else
                            {
                                pas = 10;
                            }
                        }
                    }
                }
            }
            if (!out_ && nbSeg > 0)
            {
                endOnCurve[nbSeg - 1] = global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }
        else if (!domain.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, theParab)))
        {
            boxParab = domain;
            beginOnCurve[0] = -global::Kerf.Engine.Foundation.Precision.Infinite();
            endOnCurve[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
            nbSeg = 1;
        }
    }

    public int NbSegments() => nbSeg;

    public double BeginParam(int SegmentNum)
    {
        Standard_OutOfRange_Raise_if(SegmentNum < 1 || SegmentNum > nbSeg, "Intf_Tool::BeginParam");
        return beginOnCurve[SegmentNum - 1];
    }

    public double EndParam(int SegmentNum)
    {
        Standard_OutOfRange_Raise_if(SegmentNum < 1 || SegmentNum > nbSeg, "Intf_Tool::EndParam");
        return endOnCurve[SegmentNum - 1];
    }
}
