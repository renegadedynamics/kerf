# Third-party notices

build123d.NET is a port of build123d. Its package ships no native libraries: it runs on OCCT.NET, the managed port of
OCCT, a package dependency under its own licence. The full licence texts are in [licenses/](licenses), which the
package carries too.

## build123d

- **What:** the Python library this is a port of. The .NET code follows its design, algorithms and
  messages.
- **Source:** https://github.com/gumyr/build123d
- **Licence:** Apache License 2.0, copyright the build123d contributors (its NOTICE is in [NOTICE](NOTICE)).
  See [licenses/build123d-Apache-2.0.txt](licenses/build123d-Apache-2.0.txt).

## OCCT.NET (Open CASCADE Technology 8.0.1, ported)

- **What:** the geometry kernel, the `OcctNet.*` packages this package depends on (the repository's `kernel` folder).
  It isn't shipped in this package.
- **Licence:** GNU Lesser General Public License 2.1 with the Open CASCADE exception, as OCCT's, copyright Open CASCADE
  SAS and the OCCT.NET contributors. See [licenses/OCCT-LGPL-2.1.txt](licenses/OCCT-LGPL-2.1.txt) and
  [licenses/OCCT-LGPL-exception.txt](licenses/OCCT-LGPL-exception.txt).

Until unit 10B the package could also be built on native OCCT and FreeType, whose libraries it then shipped; that
backend is gone, and with it [licenses/FreeType-FTL.txt](licenses/FreeType-FTL.txt)'s reason to be here. It is kept
for the earlier packages that shipped FreeType.

## Schematiq.Collections

- **What:** the immutable collections under `ShapeList`. It is a NuGet dependency and isn't shipped
  in these packages.
- **Licence:** MIT.
